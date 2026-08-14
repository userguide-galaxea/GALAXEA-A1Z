"""Cross-platform CAN backend selection.

Automatically selects the appropriate CAN backend based on the operating
system:

- **Linux**: native SocketCAN (kernel driver).
- **macOS / Windows**: gs_usb userspace backend via the HHS USB-CANFD
  adapter (VID:PID ``a8fa:8598``), with required patches and TX-echo
  filtering.

Usage::

    from a1z.motor_drivers.can_backend import open_can_bus

    bus = open_can_bus(channel="can0", bitrate=1_000_000)
"""

import logging
import os
import platform
import queue
import threading
import time
from collections import Counter
from typing import Optional, Union

import can

logger = logging.getLogger(__name__)

# HHS USB-CANFD adapter USB identifiers
_HHS_VID = 0xA8FA
_HHS_PID = 0x8598

# Linux 6.8 gs_usb auto-selected values reported by
# ``ip -details link show can0`` for this exact 80 MHz HHS adapter at 1 Mbps.
# Keeping the userspace backend bit-for-bit aligned avoids comparing two
# materially different CAN sampling configurations.
_HHS_1M_LINUX_TIMING = {
    "prop_seg": 14,
    "phase_seg1": 15,
    "phase_seg2": 10,
    "sjw": 5,
    "brp": 2,
}

_SYSTEM = platform.system()
_IS_LINUX = _SYSTEM == "Linux"

_patched = False


def _send_hhs_ep01(self, frame) -> bool:
    """Send through HHS endpoint 0x01 using the negotiated frame format.

    This mirrors ``GsUsb.send`` except for the endpoint. The frame size must
    follow the hardware-timestamp mode selected by ``GsUsb.start()``.
    """
    from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_NORMAL

    hw_timestamps = bool(self.device_flags & GS_CAN_MODE_HW_TIMESTAMP)
    self.gs_usb.write(0x01, frame.pack(hw_timestamps))
    return True


def _ensure_hhs_recognized() -> None:
    """Apply one-time patches required for the HHS adapter (gs_usb userspace).

    1. Whitelist the HHS VID/PID in ``GsUsb.is_gs_usb_device`` (the gs_usb
       library only recognises known candleLight-style IDs by default).
    2. On macOS/Windows there is no detachable kernel driver, so
       ``usb.core.Device.is_kernel_driver_active`` is forced to ``False``
       to avoid an Access-denied / NotImplemented error.
    3. The HHS adapter's OUT endpoint is ``0x01`` (gs_usb defaults to
       ``0x02``); override ``GsUsb.send`` accordingly.
    """
    global _patched
    if _patched:
        return

    import usb.core
    from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_NORMAL
    from gs_usb.gs_usb import GsUsb

    # 1. Whitelist HHS VID/PID
    _orig = GsUsb.is_gs_usb_device
    GsUsb.is_gs_usb_device = staticmethod(
        lambda dev: _orig(dev) or (dev.idVendor == _HHS_VID and dev.idProduct == _HHS_PID)
    )

    # 2. macOS/Windows have no kernel driver to detach
    if not _IS_LINUX:
        usb.core.Device.is_kernel_driver_active = lambda self, intf: False

    # 3. HHS adapter OUT endpoint is 0x01, not gs_usb's default 0x02.
    # Preserve the upstream hardware-timestamp frame-size selection.
    GsUsb.send = _send_hhs_ep01

    # 4. Keep the HHS adapter in the same classic-CAN mode used by the Linux
    # SocketCAN path. gs_usb enables hardware timestamps by default, changing
    # the USB host-frame format from 20 to 24 bytes. Timestamps are not used by
    # this SDK, and the HHS firmware behaves differently under that mode.
    original_start = GsUsb.start

    def start_without_hhs_hw_timestamp(self, flags=None):
        usb_dev = getattr(self, "gs_usb", None)
        is_hhs = (
            getattr(usb_dev, "idVendor", None) == _HHS_VID
            and getattr(usb_dev, "idProduct", None) == _HHS_PID
        )
        if flags is None:
            flags = (
                GS_CAN_MODE_NORMAL
                if is_hhs
                else GS_CAN_MODE_NORMAL | GS_CAN_MODE_HW_TIMESTAMP
            )
        if is_hhs:
            flags &= ~GS_CAN_MODE_HW_TIMESTAMP
        return original_start(self, flags)

    GsUsb.start = start_without_hhs_hw_timestamp

    _patched = True
    logger.debug(
        "gs_usb patches applied (HHS VID/PID whitelist, kernel-driver bypass, "
        "EP 0x01, hardware timestamps disabled)"
    )


class EchoFilterBus(can.BusABC):
    """Wrap a bus for optional TX-echo filtering and CAN diagnostics.

    SocketCAN suppresses echo frames by default; the gs_usb userspace
    backend delivers them.  Without filtering, the SDK control loop would
    parse its own MIT command frames as motor feedback, producing
    saturated positions and false faults.

    When ``filter_echo`` is false, messages are returned exactly as the
    underlying backend provides them.  This mode lets ``A1Z_TRACE_CAN=1``
    collect comparable SocketCAN statistics without changing Linux receive
    behaviour.
    """

    def __init__(
        self,
        inner: can.BusABC,
        channel: str = "echo-filtered",
        filter_echo: bool = True,
    ):
        self._inner = inner
        self._filter_echo = filter_echo
        self._trace_enabled = os.environ.get("A1Z_TRACE_CAN") == "1"
        self._trace_started = time.monotonic()
        self._trace_tx = Counter()
        self._trace_rx = Counter()
        self._trace_tx_dlc = Counter()
        self._trace_rx_dlc = Counter()
        self._trace_tx_last = {}
        self._trace_rx_last = {}
        self._trace_echo_last = {}
        self._trace_last_tx_time = {}
        self._trace_echo = 0
        self._trace_empty = 0
        self._trace_drains = 0
        self._trace_drain_zero = 0
        self._trace_drain_saturated = 0
        self._trace_drain_messages = 0
        self._trace_tx_call_count = 0
        self._trace_tx_call_total = 0.0
        self._trace_tx_call_max = 0.0
        self._trace_tx_gap_count = 0
        self._trace_tx_gap_total = 0.0
        self._trace_tx_gap_max = 0.0
        self._trace_cycle_count = 0
        self._trace_cycle_span_total = 0.0
        self._trace_cycle_span_max = 0.0
        self._trace_cycle_start = None
        self._trace_previous_tx_end = None
        self._trace_previous_tx_id = None
        self._trace_tx_rx_count = 0
        self._trace_tx_rx_total = 0.0
        self._trace_tx_rx_max = 0.0
        self._trace_feedback_age_count = 0
        self._trace_feedback_age_total = 0.0
        self._trace_feedback_age_max = 0.0
        self._trace_rx_queue_max = 0
        self._trace_rx_queue_dropped = 0
        super().__init__(channel=channel)

    def send(self, msg: can.Message, timeout=None) -> None:
        send_started = time.perf_counter()
        if self._trace_enabled:
            self._trace_tx[msg.arbitration_id] += 1
            self._trace_tx_dlc[msg.dlc] += 1
            self._trace_tx_last[msg.arbitration_id] = bytes(msg.data).hex()
            self._trace_last_tx_time[msg.arbitration_id] = send_started
            if msg.arbitration_id == 1:
                self._trace_cycle_start = send_started
            if (
                self._trace_previous_tx_end is not None
                and self._trace_previous_tx_id is not None
                and msg.arbitration_id == self._trace_previous_tx_id + 1
            ):
                gap = max(0.0, send_started - self._trace_previous_tx_end)
                self._trace_tx_gap_count += 1
                self._trace_tx_gap_total += gap
                self._trace_tx_gap_max = max(self._trace_tx_gap_max, gap)
        self._inner.send(msg, timeout)
        if self._trace_enabled:
            send_ended = time.perf_counter()
            duration = send_ended - send_started
            self._trace_tx_call_count += 1
            self._trace_tx_call_total += duration
            self._trace_tx_call_max = max(self._trace_tx_call_max, duration)
            self._trace_previous_tx_end = send_ended
            self._trace_previous_tx_id = msg.arbitration_id
            if msg.arbitration_id == 6 and self._trace_cycle_start is not None:
                span = send_ended - self._trace_cycle_start
                self._trace_cycle_count += 1
                self._trace_cycle_span_total += span
                self._trace_cycle_span_max = max(self._trace_cycle_span_max, span)
                self._trace_cycle_start = None

    def _recv_internal(self, timeout):
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if deadline is None:
                remaining = None
            else:
                remaining = max(0.0, deadline - time.monotonic())

            msg, filtered = self._inner._recv_internal(remaining)
            if msg is None:
                if self._trace_enabled:
                    self._trace_empty += 1
                return None, filtered
            if msg.is_rx:
                if self._trace_enabled:
                    received_at = time.perf_counter()
                    self._trace_rx[msg.arbitration_id] += 1
                    self._trace_rx_dlc[msg.dlc] += 1
                    self._trace_rx_last[msg.arbitration_id] = bytes(msg.data).hex()
                    sent_at = self._trace_last_tx_time.get(msg.arbitration_id)
                    if sent_at is not None:
                        latency = max(0.0, received_at - sent_at)
                        self._trace_tx_rx_count += 1
                        self._trace_tx_rx_total += latency
                        self._trace_tx_rx_max = max(
                            self._trace_tx_rx_max, latency
                        )
                return msg, filtered
            if self._trace_enabled:
                self._trace_echo += 1
                self._trace_echo_last[msg.arbitration_id] = bytes(msg.data).hex()
            if not self._filter_echo:
                return msg, filtered
            # Echo frame: drop it and keep polling within the caller's
            # original timeout budget. In particular, recv(timeout=0)
            # must remain non-blocking.

    def record_feedback_age(self, age: float, queue_depth: int) -> None:
        """Record how old a buffered feedback frame is when consumed."""
        if not self._trace_enabled:
            return
        self._trace_feedback_age_count += 1
        self._trace_feedback_age_total += age
        self._trace_feedback_age_max = max(self._trace_feedback_age_max, age)
        self._trace_rx_queue_max = max(self._trace_rx_queue_max, queue_depth)

    def record_buffer_drops(self, count: int) -> None:
        """Record frames discarded because the receive queue was full."""
        if self._trace_enabled:
            self._trace_rx_queue_dropped += count

    def record_drain(self, count: int, limit: int) -> None:
        """Record one feedback-drain result and periodically log CAN stats.

        This is a diagnostic-only hook used when ``A1Z_TRACE_CAN=1``.
        It deliberately does not perform any additional bus reads.
        """
        if not self._trace_enabled:
            return
        self._trace_drains += 1
        self._trace_drain_messages += count
        if count == 0:
            self._trace_drain_zero += 1
        if count >= limit:
            self._trace_drain_saturated += 1

        now = time.monotonic()
        elapsed = now - self._trace_started
        if elapsed < 1.0:
            return

        tx_call_avg_ms = (
            1000.0 * self._trace_tx_call_total / self._trace_tx_call_count
            if self._trace_tx_call_count
            else 0.0
        )
        tx_gap_avg_ms = (
            1000.0 * self._trace_tx_gap_total / self._trace_tx_gap_count
            if self._trace_tx_gap_count
            else 0.0
        )
        cycle_span_avg_ms = (
            1000.0 * self._trace_cycle_span_total / self._trace_cycle_count
            if self._trace_cycle_count
            else 0.0
        )
        tx_rx_avg_ms = (
            1000.0 * self._trace_tx_rx_total / self._trace_tx_rx_count
            if self._trace_tx_rx_count
            else 0.0
        )
        feedback_age_avg_ms = (
            1000.0
            * self._trace_feedback_age_total
            / self._trace_feedback_age_count
            if self._trace_feedback_age_count
            else 0.0
        )
        logger.info(
            "CAN trace %.2fs: tx=%d echo=%d rx=%d empty=%d "
            "drains=%d zero=%d saturated=%d messages=%d "
            "tx_call_avg/max=%.3f/%.3fms tx_gap_avg/max=%.3f/%.3fms "
            "j1_j6_span_avg/max=%.3f/%.3fms "
            "tx_rx_avg/max=%.3f/%.3fms feedback_age_avg/max=%.3f/%.3fms "
            "rx_queue_max=%d rx_queue_dropped=%d "
            "tx_ids=%s rx_ids=%s tx_dlc=%s rx_dlc=%s "
            "tx_last=%s echo_last=%s rx_last=%s",
            elapsed,
            sum(self._trace_tx.values()),
            self._trace_echo,
            sum(self._trace_rx.values()),
            self._trace_empty,
            self._trace_drains,
            self._trace_drain_zero,
            self._trace_drain_saturated,
            self._trace_drain_messages,
            tx_call_avg_ms,
            1000.0 * self._trace_tx_call_max,
            tx_gap_avg_ms,
            1000.0 * self._trace_tx_gap_max,
            cycle_span_avg_ms,
            1000.0 * self._trace_cycle_span_max,
            tx_rx_avg_ms,
            1000.0 * self._trace_tx_rx_max,
            feedback_age_avg_ms,
            1000.0 * self._trace_feedback_age_max,
            self._trace_rx_queue_max,
            self._trace_rx_queue_dropped,
            dict(sorted(self._trace_tx.items())),
            dict(sorted(self._trace_rx.items())),
            dict(sorted(self._trace_tx_dlc.items())),
            dict(sorted(self._trace_rx_dlc.items())),
            dict(sorted(self._trace_tx_last.items())),
            dict(sorted(self._trace_echo_last.items())),
            dict(sorted(self._trace_rx_last.items())),
        )
        self._reset_trace(now)

    def _reset_trace(self, now: float) -> None:
        self._trace_started = now
        self._trace_tx.clear()
        self._trace_rx.clear()
        self._trace_tx_dlc.clear()
        self._trace_rx_dlc.clear()
        self._trace_tx_last.clear()
        self._trace_rx_last.clear()
        self._trace_echo_last.clear()
        self._trace_last_tx_time.clear()
        self._trace_echo = 0
        self._trace_empty = 0
        self._trace_drains = 0
        self._trace_drain_zero = 0
        self._trace_drain_saturated = 0
        self._trace_drain_messages = 0
        self._trace_tx_call_count = 0
        self._trace_tx_call_total = 0.0
        self._trace_tx_call_max = 0.0
        self._trace_tx_gap_count = 0
        self._trace_tx_gap_total = 0.0
        self._trace_tx_gap_max = 0.0
        self._trace_cycle_count = 0
        self._trace_cycle_span_total = 0.0
        self._trace_cycle_span_max = 0.0
        self._trace_cycle_start = None
        self._trace_previous_tx_end = None
        self._trace_previous_tx_id = None
        self._trace_tx_rx_count = 0
        self._trace_tx_rx_total = 0.0
        self._trace_tx_rx_max = 0.0
        self._trace_feedback_age_count = 0
        self._trace_feedback_age_total = 0.0
        self._trace_feedback_age_max = 0.0
        self._trace_rx_queue_max = 0
        self._trace_rx_queue_dropped = 0

    def shutdown(self) -> None:
        self._inner.shutdown()
        super().shutdown()

    def flush_tx(self, timeout: float = 0.5) -> bool:
        """Forward an optional transport-level TX flush to the inner bus."""
        flush_tx = getattr(self._inner, "flush_tx", None)
        if flush_tx is None:
            return True
        return bool(flush_tx(timeout=timeout))

    @property
    def state(self):
        return self._inner.state


class BufferedReceiveBus(can.BusABC):
    """Continuously receive CAN frames into a thread-safe in-memory queue.

    python-can's gs_usb backend converts ``recv(timeout=0)`` to a blocking
    1 ms USB read. The control loop only has about 1 ms available for its
    complete receive drain, so TX echoes can consume that budget before the
    corresponding motor feedback is read. A dedicated receiver keeps the USB
    IN endpoint drained while the control thread sends commands and processes
    already-buffered feedback without performing USB I/O.
    """

    def __init__(
        self,
        inner: can.BusABC,
        channel: str = "buffered-receive",
        max_queue_size: int = 4096,
    ):
        self._inner = inner
        self._queue = queue.Queue(maxsize=max_queue_size)
        self._stop_event = threading.Event()
        self._dropped = 0
        self._reported_drops = 0
        super().__init__(channel=channel)
        self._thread = threading.Thread(
            target=self._receive_loop,
            name="a1z_can_rx",
            daemon=True,
        )
        self._thread.start()

    def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self._inner.recv(timeout=0.01)
            except Exception:
                if not self._stop_event.is_set():
                    logger.exception("CAN receive thread failed")
                break
            if msg is None:
                continue
            item = (time.perf_counter(), msg)
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                # Keep the newest feedback. This should never happen during
                # normal operation because the control loop can drain up to
                # 3000 messages/s, but bounded memory is safer on shutdown or
                # when the control thread stalls.
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                self._dropped += 1
                try:
                    self._queue.put_nowait(item)
                except queue.Full:
                    self._dropped += 1

    def send(self, msg: can.Message, timeout=None) -> None:
        self._inner.send(msg, timeout)

    def _recv_internal(self, timeout):
        try:
            if timeout is None:
                received_at, msg = self._queue.get()
            elif timeout <= 0:
                received_at, msg = self._queue.get_nowait()
            else:
                received_at, msg = self._queue.get(timeout=timeout)
            record_age = getattr(self._inner, "record_feedback_age", None)
            if record_age is not None:
                record_age(
                    max(0.0, time.perf_counter() - received_at),
                    self._queue.qsize(),
                )
            return msg, False
        except queue.Empty:
            return None, False

    def record_drain(self, count: int, limit: int) -> None:
        new_drops = self._dropped - self._reported_drops
        if new_drops:
            record_drops = getattr(self._inner, "record_buffer_drops", None)
            if record_drops is not None:
                record_drops(new_drops)
            self._reported_drops = self._dropped
        record_drain = getattr(self._inner, "record_drain", None)
        if record_drain is not None:
            record_drain(count, limit)

    def shutdown(self) -> None:
        if self._is_shutdown:
            return
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=0.1)
        self._inner.shutdown()
        super().shutdown()

    @property
    def state(self):
        return self._inner.state


class AsyncHhsBus(can.BusABC):
    """Experimental HHS backend using concurrent libusb TX/RX transfers.

    The Linux gs_usb kernel driver keeps up to ten TX URBs in flight. The
    python-can gs_usb backend instead performs one synchronous USB write per
    CAN frame. This backend follows the kernel model: each outgoing frame gets
    its own echo ID and USB transfer, while a pool of IN transfers continuously
    receives motor feedback and TX completion echoes.

    This is the default HHS backend on macOS and Windows. macOS passed
    empty-bus, low-torque, full-gravity and 300-second stability validation;
    on Windows the CAN link and 6/6 motor scan have been verified on
    hardware. Set ``A1Z_GS_USB_ASYNC_TX=0`` to fall back to python-can's
    synchronous path.
    """

    _MAX_TX_TRANSFERS = 10
    _MAX_RX_TRANSFERS = 30
    _FRAME_SIZE = 20
    _ENDPOINT_OUT = 0x01
    _ENDPOINT_IN = 0x81
    _RX_ECHO_ID = 0xFFFFFFFF

    def __init__(self, channel: Union[str, int] = 0, bitrate: int = 1_000_000):
        import usb1
        from gs_usb.constants import CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_RTR_FLAG

        if bitrate != 1_000_000:
            raise can.CanInitializationError(
                "AsyncHhsBus only supports 1 Mbps on the HHS adapter "
                f"(requested {bitrate})"
            )

        index = int(channel)
        self._usb1 = usb1
        self._can_eff_flag = CAN_EFF_FLAG
        self._can_rtr_flag = CAN_RTR_FLAG
        self._can_err_flag = CAN_ERR_FLAG
        self._context = usb1.USBContext()

        # Enumerate HHS adapters directly through libusb1. On Windows this
        # avoids the PyUSB -> libusb1 "handover" (open via PyUSB, dispose, then
        # re-open via libusb1) that failed on hardware: disposing the PyUSB
        # handle can tear down the device on the WinUSB stack so the later
        # libusb1 re-enumeration can no longer match it.
        candidates = []
        for device in self._context.getDeviceIterator(skip_on_error=True):
            try:
                vid = device.getVendorID()
                pid = device.getProductID()
            except Exception:
                continue
            if (vid, pid) == (_HHS_VID, _HHS_PID):
                candidates.append(device)
        candidates.sort(
            key=lambda d: (d.getBusNumber(), d.getDeviceAddress())
        )
        if index < 0 or index >= len(candidates):
            self._context.close()
            raise can.CanInitializationError(
                f"Cannot find HHS gs_usb device {index}. "
                f"HHS adapters found: {len(candidates)}"
            )

        self._handle = candidates[index].open()
        try:
            if _IS_LINUX:
                try:
                    if self._handle.kernelDriverActive(0):
                        self._handle.detachKernelDriver(0)
                except usb1.USBErrorNotSupported:
                    pass
            self._handle.claimInterface(0)
            self._setup_gs_usb()
        except Exception:
            try:
                self._handle.close()
            except Exception:
                pass
            self._context.close()
            raise

        logger.info(
            "HHS CAN setup: clock=%dHz bitrate=%d sample_point=%.2f%% "
            "brp=%d prop_seg=%d phase_seg1=%d phase_seg2=%d sjw=%d "
            "mode=normal frame_size=%d",
            80_000_000,
            bitrate,
            75.0,
            _HHS_1M_LINUX_TIMING["brp"],
            _HHS_1M_LINUX_TIMING["prop_seg"],
            _HHS_1M_LINUX_TIMING["phase_seg1"],
            _HHS_1M_LINUX_TIMING["phase_seg2"],
            _HHS_1M_LINUX_TIMING["sjw"],
            self._FRAME_SIZE,
        )

        self._rx_queue = queue.Queue(maxsize=4096)
        self._tx_slots = queue.Queue(maxsize=self._MAX_TX_TRANSFERS)
        for slot in range(self._MAX_TX_TRANSFERS):
            self._tx_slots.put_nowait(slot)
        self._tx_transfer_pool = [
            self._handle.getTransfer() for _ in range(self._MAX_TX_TRANSFERS)
        ]
        self._tx_transfers = {}
        self._rx_transfers = []
        self._transfer_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._shutdown_deadline = None
        self._fatal_error = None
        self._event_thread = threading.Thread(
            target=self._event_loop,
            name="a1z_hhs_usb_events",
            daemon=True,
        )

        super().__init__(channel=f"hhs-async-{index}")
        try:
            for _ in range(self._MAX_RX_TRANSFERS):
                transfer = self._handle.getTransfer()
                transfer.setBulk(
                    self._ENDPOINT_IN,
                    self._FRAME_SIZE,
                    callback=self._on_rx_complete,
                    timeout=0,
                )
                self._rx_transfers.append(transfer)
                transfer.submit()
            self._event_thread.start()
        except Exception:
            self.shutdown()
            raise

    def _setup_gs_usb(self) -> None:
        """Configure the HHS CAN controller through libusb1 control transfers.

        Mirrors the Linux gs_usb kernel handshake (matching kernel/drivers/
        net/can/usb/gs_usb.c):
          - HOST_FORMAT handshake with the little-endian magic 0x0000BEEF
          - BIT_TIMING with the Linux-aligned 1 Mbps parameters (80 MHz clock)
          - MODE_START for classic CAN (no hardware-timestamp mode flag)
        """
        import struct

        # USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE.  The interface
        # recipient bit (0x01) is required by the gs_usb protocol.
        request_type_out = 0x41
        # 1) host format handshake
        self._handle.controlWrite(
            request_type_out,
            0,  # GS_USB_BREQ_HOST_FORMAT
            0,
            0,
            struct.pack("<I", 0x0000BEEF),
            timeout=100,
        )
        # 2) Linux-aligned 1 Mbps bit timing (80 MHz HHS controller)
        timing = _HHS_1M_LINUX_TIMING
        self._handle.controlWrite(
            request_type_out,
            1,  # GS_USB_BREQ_BITTIMING
            0,
            0,
            struct.pack(
                "<IIIII",
                timing["prop_seg"],
                timing["phase_seg1"],
                timing["phase_seg2"],
                timing["sjw"],
                timing["brp"],
            ),
            timeout=100,
        )
        # 3) start classic CAN controller (GS_CAN_MODE_START = 1, flags = 0)
        self._handle.controlWrite(
            request_type_out,
            2,  # GS_USB_BREQ_MODE
            0,
            0,
            struct.pack("<II", 1, 0),
            timeout=100,
        )

    def _event_loop(self) -> None:
        while True:
            if self._stop_event.is_set():
                transfers = list(self._rx_transfers)
                with self._transfer_lock:
                    transfers += list(self._tx_transfers.values())
                if not any(transfer.isSubmitted() for transfer in transfers):
                    break
                if (
                    self._shutdown_deadline is not None
                    and time.monotonic() >= self._shutdown_deadline
                ):
                    break
            try:
                self._context.handleEventsTimeout(0.01)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._fatal_error = exc
                    logger.exception("Async HHS USB event loop failed")
                break

    def _on_tx_complete(self, transfer) -> None:
        status = transfer.getStatus()
        if status != self._usb1.TRANSFER_COMPLETED and not self._stop_event.is_set():
            slot = transfer.getUserData()
            self._fatal_error = can.CanOperationError(
                f"Async HHS TX transfer failed with libusb status {status}"
            )
            self._release_tx_slot(slot)

    def _release_tx_slot(self, slot: int) -> None:
        with self._transfer_lock:
            transfer = self._tx_transfers.pop(slot, None)
        if transfer is not None:
            try:
                self._tx_slots.put_nowait(slot)
            except queue.Full:
                pass

    def _on_rx_complete(self, transfer) -> None:
        status = transfer.getStatus()
        if status == self._usb1.TRANSFER_COMPLETED:
            actual_length = transfer.getActualLength()
            if actual_length == self._FRAME_SIZE:
                payload = bytes(transfer.getBuffer()[:actual_length])
                self._process_usb_frame(payload)
            elif actual_length and not self._stop_event.is_set():
                logger.warning(
                    "Ignoring HHS USB frame with unexpected length %d",
                    actual_length,
                )
        elif status not in (
            self._usb1.TRANSFER_CANCELLED,
            self._usb1.TRANSFER_NO_DEVICE,
        ) and not self._stop_event.is_set():
            self._fatal_error = can.CanOperationError(
                f"Async HHS RX transfer failed with libusb status {status}"
            )

        if not self._stop_event.is_set() and self._fatal_error is None:
            try:
                transfer.submit()
            except Exception as exc:
                self._fatal_error = exc

    def _process_usb_frame(self, payload: bytes) -> None:
        from gs_usb.gs_usb_frame import GsUsbFrame

        frame = GsUsbFrame()
        GsUsbFrame.unpack_into(frame, payload, False)
        is_rx = frame.echo_id == self._RX_ECHO_ID
        if not is_rx and frame.echo_id < self._MAX_TX_TRANSFERS:
            self._release_tx_slot(frame.echo_id)

        msg = can.Message(
            timestamp=frame.timestamp,
            arbitration_id=frame.arbitration_id,
            is_extended_id=frame.is_extended_id,
            is_remote_frame=frame.is_remote_frame,
            is_error_frame=frame.is_error_frame,
            channel=self.channel_info,
            dlc=frame.can_dlc,
            data=bytearray(frame.data[: frame.can_dlc]),
            is_rx=is_rx,
        )
        try:
            self._rx_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._rx_queue.get_nowait()
                self._rx_queue.put_nowait(msg)
            except (queue.Empty, queue.Full):
                pass

    def send(self, msg: can.Message, timeout=None) -> None:
        from gs_usb.gs_usb_frame import GsUsbFrame

        if self._fatal_error is not None:
            raise can.CanOperationError(
                f"Async HHS backend is unhealthy: {self._fatal_error}"
            )
        wait_timeout = 0.02 if timeout is None else timeout
        try:
            slot = self._tx_slots.get(timeout=wait_timeout)
        except queue.Empty as exc:
            raise can.CanOperationError(
                "Async HHS TX queue exhausted while waiting for echo"
            ) from exc

        can_id = msg.arbitration_id
        if msg.is_extended_id:
            can_id |= self._can_eff_flag
        if msg.is_remote_frame:
            can_id |= self._can_rtr_flag
        if msg.is_error_frame:
            can_id |= self._can_err_flag

        frame = GsUsbFrame(can_id=can_id, data=list(msg.data))
        frame.echo_id = slot
        payload = frame.pack(False)
        transfer = self._tx_transfer_pool[slot]
        transfer.setBulk(
            self._ENDPOINT_OUT,
            payload,
            callback=self._on_tx_complete,
            user_data=slot,
            timeout=20,
        )
        with self._transfer_lock:
            self._tx_transfers[slot] = transfer
        try:
            transfer.submit()
        except Exception:
            self._release_tx_slot(slot)
            raise

    def _recv_internal(self, timeout):
        if self._fatal_error is not None:
            raise can.CanOperationError(
                f"Async HHS backend is unhealthy: {self._fatal_error}"
            )
        try:
            if timeout is None:
                return self._rx_queue.get(), False
            if timeout <= 0:
                return self._rx_queue.get_nowait(), False
            return self._rx_queue.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def shutdown(self) -> None:
        if getattr(self, "_is_shutdown", False):
            return
        self._stop_event.set()
        self._shutdown_deadline = time.monotonic() + 0.2
        transfers = list(getattr(self, "_rx_transfers", []))
        transfers += list(getattr(self, "_tx_transfers", {}).values())
        for transfer in transfers:
            try:
                if transfer.isSubmitted():
                    transfer.cancel()
            except Exception:
                pass
        event_thread = getattr(self, "_event_thread", None)
        if event_thread is not None and event_thread.is_alive():
            event_thread.join(timeout=0.2)
        handle = getattr(self, "_handle", None)
        if handle is not None:
            try:
                # GS_CAN_MODE_RESET (=0), flags=0, via a vendor OUT transfer
                # matching the gs_usb protocol (request 2 = GS_USB_BREQ_MODE).
                import struct

                handle.controlWrite(
                    0x40, 2, 0, 0, struct.pack("<II", 0, 0), timeout=100
                )
            except Exception:
                pass
            try:
                handle.releaseInterface(0)
            except Exception:
                pass
            handle.close()
        context = getattr(self, "_context", None)
        if context is not None:
            context.close()
        super().shutdown()

    def flush_tx(self, timeout: float = 0.5) -> bool:
        """Wait until every submitted TX frame has received its hardware echo.

        This is used by the robot shutdown path after sending motor-disable
        frames. Closing the USB device while those frames are still queued can
        leave the motors enabled even though the host program has exited.
        """
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            if self._fatal_error is not None:
                raise can.CanOperationError(
                    f"Cannot flush unhealthy Async HHS backend: "
                    f"{self._fatal_error}"
                )
            with self._transfer_lock:
                pending = len(self._tx_transfers)
            if pending == 0:
                return True
            if time.monotonic() >= deadline:
                logger.error(
                    "Timed out waiting for %d HHS TX frame(s) to complete",
                    pending,
                )
                return False
            time.sleep(0.001)

    @property
    def state(self):
        return can.BusState.ERROR if self._fatal_error is not None else can.BusState.ACTIVE


def default_bustype() -> str:
    """Return the default python-can backend for the current OS."""
    return "socketcan" if _IS_LINUX else "gs_usb"


def default_channel() -> Union[str, int]:
    """Return the default CAN channel for the current OS."""
    return "can0" if _IS_LINUX else 0


def _selected_gs_usb_is_hhs(channel: Union[str, int]) -> bool:
    """Return whether the selected gs_usb device is the validated HHS model.

    Use libusb1 directly so selecting the async backend does not first depend
    on PyUSB discovering a separate libusb backend.
    """
    try:
        import usb1

        index = int(channel)
        if index < 0:
            return False
        with usb1.USBContext() as context:
            devices = []
            for device in context.getDeviceIterator(skip_on_error=True):
                try:
                    vid = device.getVendorID()
                    pid = device.getProductID()
                    bus = device.getBusNumber()
                    address = device.getDeviceAddress()
                except Exception:
                    continue
                if (vid, pid) == (_HHS_VID, _HHS_PID):
                    devices.append((bus, address))
            devices.sort(key=lambda item: (item[0], item[1]))
            return index < len(devices)
    except Exception:
        logger.debug("Unable to identify selected gs_usb device", exc_info=True)
        return False


def open_can_bus(
    channel: Union[str, int, None] = None,
    bitrate: int = 1_000_000,
    bustype: Optional[str] = None,
    echo_filter: Optional[bool] = None,
) -> can.BusABC:
    """Open a CAN bus appropriate for the current platform.

    Args:
        channel: CAN channel name.  ``"can0"``/``"can1"`` for Linux
            SocketCAN; integer device index for gs_usb; e.g.
            ``"PCAN_USBBUS1"`` for PEAK.  When ``None`` the OS default
            is used.
        bitrate: CAN bus bitrate (default 1 Mbps).
        bustype: Force a specific python-can backend (``socketcan``,
            ``gs_usb``, ``pcan``, ``slcan``...).  When ``None``
            (default), the backend is chosen automatically:
            ``"socketcan"`` on Linux, ``"gs_usb"`` on macOS/Windows.
        echo_filter: Whether to filter TX-echo frames. When ``None``
            (default), echo filtering is enabled for gs_usb and disabled
            otherwise. Setting ``A1Z_TRACE_CAN=1`` still installs the
            diagnostic wrapper on other backends, but does not change their
            receive behaviour.

    Returns:
        A ``can.BusABC`` instance ready for use.
    """
    if bustype is None:
        bustype = default_bustype()
    if channel is None:
        channel = default_channel()

    if bustype == "gs_usb":
        _ensure_hhs_recognized()
        # gs_usb backend expects an integer channel (device index)
        if isinstance(channel, str):
            if channel.isdigit():
                channel = int(channel)
            elif channel.startswith("can") and channel[3:].isdigit():
                channel = int(channel[3:])

    async_setting = os.environ.get("A1Z_GS_USB_ASYNC_TX")
    # macOS and Windows both default to the async HHS backend when the
    # selected gs_usb device is the validated HHS adapter. This avoids the
    # per-frame synchronous USB round trip that starves the 250 Hz control
    # loop (macOS has passed empty-bus, low-torque, full-gravity and
    # 300-second stability validation; Windows CAN link and 6/6 motor scan
    # have been verified on hardware).
    async_hhs = bustype == "gs_usb" and (
        async_setting == "1"
        or (
            async_setting is None
            and _SYSTEM in ("Darwin", "Windows")
            and _selected_gs_usb_is_hhs(channel)
        )
    )
    if async_hhs:
        bus = AsyncHhsBus(channel=channel, bitrate=bitrate)
        logger.info("Async HHS gs_usb backend enabled")
    else:
        bus = can.interface.Bus(interface=bustype, channel=channel, bitrate=bitrate)

    # Decide echo filtering
    if echo_filter is None:
        echo_filter = bustype == "gs_usb"
    trace_can = os.environ.get("A1Z_TRACE_CAN") == "1"
    if echo_filter or trace_can:
        bus = EchoFilterBus(bus, filter_echo=echo_filter)
    if (
        bustype == "gs_usb"
        and not async_hhs
        and os.environ.get("A1Z_GS_USB_RX_THREAD", "1") != "0"
    ):
        bus = BufferedReceiveBus(bus)
        logger.info("gs_usb continuous receive thread enabled")

    if bustype == "gs_usb":
        desc = f"gs_usb idx={channel} (HHS a8fa:8598) @ {bitrate} bps"
    else:
        desc = f"{bustype}/{channel} @ {bitrate} bps"
    logger.info(f"Opening CAN bus: {desc}")
    return bus
