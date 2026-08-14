import logging
import struct
import time
import unittest
from unittest.mock import MagicMock, patch

import can

from a1z.motor_drivers import can_backend


class _DummyBus(can.BusABC):
    def __init__(self):
        super().__init__(channel="dummy")

    def send(self, msg, timeout=None):
        pass

    def _recv_internal(self, timeout):
        return None, False


class _EchoThenNoneBus(_DummyBus):
    def __init__(self):
        super().__init__()
        self.receive_count = 0

    def _recv_internal(self, timeout):
        self.receive_count += 1
        if self.receive_count == 1:
            return (
                can.Message(
                    arbitration_id=1,
                    data=[0],
                    is_extended_id=False,
                    is_rx=False,
                ),
                False,
            )
        if timeout:
            time.sleep(timeout)
        return None, False


class CanBackendTests(unittest.TestCase):
    def test_async_hhs_setup_uses_interface_recipient(self):
        handle = MagicMock()
        bus = can_backend.AsyncHhsBus.__new__(can_backend.AsyncHhsBus)
        bus._handle = handle

        bus._setup_gs_usb()

        self.assertEqual(handle.controlWrite.call_count, 3)
        self.assertTrue(
            all(call.args[0] == 0x41 for call in handle.controlWrite.call_args_list)
        )
        self.assertEqual(
            handle.controlWrite.call_args_list[0].args[4],
            struct.pack("<I", 0x0000BEEF),
        )

    def test_hhs_selection_uses_libusb1_without_pyusb_scan(self):
        hhs = MagicMock()
        hhs.getVendorID.return_value = can_backend._HHS_VID
        hhs.getProductID.return_value = can_backend._HHS_PID
        hhs.getBusNumber.return_value = 1
        hhs.getDeviceAddress.return_value = 2
        other = MagicMock()
        other.getVendorID.return_value = 0x1234
        other.getProductID.return_value = 0x5678
        other.getBusNumber.return_value = 1
        other.getDeviceAddress.return_value = 1
        context = MagicMock()
        context.__enter__.return_value = context
        context.getDeviceIterator.return_value = [other, hhs]

        with patch("usb1.USBContext", return_value=context):
            self.assertTrue(can_backend._selected_gs_usb_is_hhs(0))
            self.assertFalse(can_backend._selected_gs_usb_is_hhs(1))

    def test_hhs_send_preserves_hardware_timestamp_frame_format(self):
        from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP

        writes = []
        packed_modes = []

        class UsbDevice:
            def write(self, endpoint, payload):
                writes.append((endpoint, payload))

        class Frame:
            def pack(self, hw_timestamps):
                packed_modes.append(hw_timestamps)
                return b"24-byte-frame"

        sender = type("Sender", (), {})()
        sender.device_flags = GS_CAN_MODE_HW_TIMESTAMP
        sender.gs_usb = UsbDevice()

        result = can_backend._send_hhs_ep01(sender, Frame())

        self.assertTrue(result)
        self.assertEqual(packed_modes, [True])
        self.assertEqual(writes, [(0x01, b"24-byte-frame")])

    def test_hhs_send_uses_base_frame_without_hardware_timestamps(self):
        writes = []
        packed_modes = []

        class UsbDevice:
            def write(self, endpoint, payload):
                writes.append((endpoint, payload))

        class Frame:
            def pack(self, hw_timestamps):
                packed_modes.append(hw_timestamps)
                return b"20-byte-frame"

        sender = type("Sender", (), {})()
        sender.device_flags = 0
        sender.gs_usb = UsbDevice()

        can_backend._send_hhs_ep01(sender, Frame())

        self.assertEqual(packed_modes, [False])
        self.assertEqual(writes, [(0x01, b"20-byte-frame")])

    def test_gs_usb_string_channel_is_converted_and_wrapped(self):
        created = []

        def create_bus(**kwargs):
            created.append(kwargs)
            return _DummyBus()

        with (
            patch.dict("os.environ", {"A1Z_GS_USB_ASYNC_TX": "0"}),
            patch.object(can_backend, "_ensure_hhs_recognized"),
            patch.object(can.interface, "Bus", side_effect=create_bus),
        ):
            bus = can_backend.open_can_bus(channel="0", bustype="gs_usb")

        self.assertEqual(created[0]["channel"], 0)
        self.assertIsInstance(bus, can_backend.BufferedReceiveBus)
        self.assertIsInstance(bus._inner, can_backend.EchoFilterBus)
        bus.shutdown()

    def test_async_hhs_backend_skips_second_rx_thread(self):
        async_bus = _DummyBus()
        with (
            patch.dict("os.environ", {"A1Z_GS_USB_ASYNC_TX": "1"}),
            patch.object(can_backend, "_ensure_hhs_recognized"),
            patch.object(can_backend, "AsyncHhsBus", return_value=async_bus) as create,
            patch.object(can.interface, "Bus") as create_standard,
        ):
            bus = can_backend.open_can_bus(channel="0", bustype="gs_usb")

        create.assert_called_once_with(channel=0, bitrate=1_000_000)
        create_standard.assert_not_called()
        self.assertIsInstance(bus, can_backend.EchoFilterBus)
        self.assertIs(bus._inner, async_bus)
        self.assertNotIsInstance(bus, can_backend.BufferedReceiveBus)
        bus.shutdown()

    def test_async_hhs_backend_is_default_for_hhs_on_macos(self):
        async_bus = _DummyBus()
        with (
            patch.dict("os.environ", {}, clear=True),
            patch.object(can_backend, "_SYSTEM", "Darwin"),
            patch.object(can_backend, "_ensure_hhs_recognized"),
            patch.object(can_backend, "_selected_gs_usb_is_hhs", return_value=True),
            patch.object(can_backend, "AsyncHhsBus", return_value=async_bus) as create,
            patch.object(can.interface, "Bus") as create_standard,
        ):
            bus = can_backend.open_can_bus(channel=0, bustype="gs_usb")

        create.assert_called_once_with(channel=0, bitrate=1_000_000)
        create_standard.assert_not_called()
        bus.shutdown()

    def test_zero_timeout_stays_nonblocking_after_echo(self):
        bus = can_backend.EchoFilterBus(_EchoThenNoneBus())

        started = time.perf_counter()
        message = bus.recv(timeout=0.0)
        elapsed = time.perf_counter() - started

        self.assertIsNone(message)
        self.assertLess(elapsed, 0.005)
        bus.shutdown()

    def test_echo_filter_forwards_transport_flush(self):
        class FlushBus(_DummyBus):
            def __init__(self):
                super().__init__()
                self.timeouts = []

            def flush_tx(self, timeout=0.5):
                self.timeouts.append(timeout)
                return True

        inner = FlushBus()
        bus = can_backend.EchoFilterBus(inner)

        self.assertTrue(bus.flush_tx(timeout=0.25))
        self.assertEqual(inner.timeouts, [0.25])
        bus.shutdown()

    def test_socketcan_trace_wraps_without_filtering_echo(self):
        with (
            patch.dict("os.environ", {"A1Z_TRACE_CAN": "1"}),
            patch.object(can.interface, "Bus", return_value=_EchoThenNoneBus()),
        ):
            bus = can_backend.open_can_bus(channel="can0", bustype="socketcan")

        self.assertIsInstance(bus, can_backend.EchoFilterBus)
        message = bus.recv(timeout=0.0)
        self.assertIsNotNone(message)
        self.assertFalse(message.is_rx)
        bus.shutdown()

    def test_buffered_receive_bus_moves_receive_off_caller_thread(self):
        class OneMessageBus(_DummyBus):
            def __init__(self):
                super().__init__()
                self.sent = False

            def _recv_internal(self, timeout):
                if not self.sent:
                    self.sent = True
                    return (
                        can.Message(
                            arbitration_id=3,
                            data=[1] * 8,
                            is_extended_id=False,
                            is_rx=True,
                        ),
                        False,
                    )
                if timeout:
                    time.sleep(min(timeout, 0.001))
                return None, False

        bus = can_backend.BufferedReceiveBus(OneMessageBus())
        deadline = time.monotonic() + 0.1
        message = None
        while message is None and time.monotonic() < deadline:
            message = bus.recv(timeout=0.0)
            if message is None:
                time.sleep(0.001)

        self.assertIsNotNone(message)
        self.assertEqual(message.arbitration_id, 3)
        bus.shutdown()

    def test_can_trace_counts_tx_echo_rx_and_drain_state(self):
        class TraceBus(_DummyBus):
            def __init__(self):
                super().__init__()
                self.messages = [
                    can.Message(
                        arbitration_id=1,
                        data=[0] * 8,
                        is_extended_id=False,
                        is_rx=False,
                    ),
                    can.Message(
                        arbitration_id=0x11,
                        data=[0] * 6,
                        is_extended_id=False,
                        is_rx=True,
                    ),
                ]

            def _recv_internal(self, timeout):
                if self.messages:
                    return self.messages.pop(0), False
                return None, False

        with patch.dict("os.environ", {"A1Z_TRACE_CAN": "1"}):
            bus = can_backend.EchoFilterBus(TraceBus())
        bus.send(
            can.Message(
                arbitration_id=1,
                data=[0] * 8,
                is_extended_id=False,
            )
        )
        message = bus.recv(timeout=0.0)
        bus._trace_started -= 2.0

        with self.assertLogs(can_backend.logger, level=logging.INFO) as captured:
            bus.record_drain(1, 12)

        output = "\n".join(captured.output)
        self.assertEqual(message.arbitration_id, 0x11)
        self.assertIn("tx=1 echo=1 rx=1", output)
        self.assertIn("saturated=0 messages=1", output)
        self.assertIn("tx_call_avg/max=", output)
        self.assertIn("tx_gap_avg/max=", output)
        self.assertIn("j1_j6_span_avg/max=", output)
        self.assertIn("tx_rx_avg/max=", output)
        self.assertIn("feedback_age_avg/max=", output)
        self.assertIn("rx_queue_max=", output)
        self.assertIn("rx_queue_dropped=", output)
        self.assertIn("tx_dlc={8: 1}", output)
        self.assertIn("rx_dlc={6: 1}", output)
        self.assertIn("tx_last={1: '0000000000000000'}", output)
        self.assertIn("echo_last={1: '0000000000000000'}", output)
        self.assertIn("rx_last={17: '000000000000'}", output)
        bus.shutdown()


if __name__ == "__main__":
    unittest.main()
