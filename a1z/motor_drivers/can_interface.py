"""CAN bus abstraction layer."""

import logging
import time
from typing import List, Optional

import can

from a1z.motor_drivers.utils import ReceiveMode


class CanInterface:
    """CAN bus send/receive with retry and optional buffered reader."""

    def __init__(
        self,
        channel: str = "can0",
        bustype: str = "socketcan",
        bitrate: int = 1_000_000,
        name: str = "default_can_interface",
        receive_mode: ReceiveMode = ReceiveMode.same,
        use_buffered_reader: bool = False,
    ):
        self.bus = can.interface.Bus(bustype=bustype, channel=channel, bitrate=bitrate)
        self.busstate = self.bus.state
        self.name = name
        self.receive_mode = receive_mode
        self.use_buffered_reader = use_buffered_reader
        logging.info(f"CAN interface {self.name} opened on {channel} (buffered_reader={use_buffered_reader})")
        if use_buffered_reader:
            self.buffered_reader = can.BufferedReader()
            self.notifier = can.Notifier(self.bus, [self.buffered_reader])

    def close(self) -> None:
        """Shut down the CAN bus."""
        if self.use_buffered_reader:
            self.notifier.stop()
        self.bus.shutdown()

    def _send_message_get_response(
        self,
        id: int,
        motor_id: int,
        data: List[int],
        max_retry: int = 5,
        expected_id: Optional[int] = None,
    ) -> can.Message:
        """Send a CAN message and wait for the expected response."""
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        for _ in range(max_retry):
            try:
                self.bus.send(message)
                response = self._receive_message(motor_id, timeout=0.2)
                if expected_id is None:
                    expected_id = self.receive_mode.get_receive_id(motor_id)
                if response and (expected_id == response.arbitration_id):
                    return response
                self.try_receive_message(id)
            except (can.CanError, AssertionError) as e:
                logging.warning(f"CAN Error {self.name}: {e}. Retrying...")
            time.sleep(0.001)
        raise AssertionError(
            f"Failed to communicate with motor {id} on {self.name} at {self.bus.channel_info}"
        )

    def try_receive_message(self, motor_id: Optional[int] = None, timeout: float = 0.009) -> Optional[can.Message]:
        """Try to receive a message, returning None on timeout."""
        try:
            return self._receive_message(motor_id, timeout, supress_warning=True)
        except AssertionError:
            return None

    def _receive_message(
        self,
        motor_id: Optional[int] = None,
        timeout: float = 0.009,
        supress_warning: bool = False,
    ) -> Optional[can.Message]:
        """Receive a message from the CAN bus within timeout."""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.use_buffered_reader:
                message = self.buffered_reader.get_message(timeout=0.002)
            else:
                message = self.bus.recv(timeout=0.002)
            if message:
                return message
            else:
                if self.use_buffered_reader:
                    message = self.buffered_reader.get_message(timeout=0.0008)
                else:
                    message = self.bus.recv(timeout=0.0008)
                if message:
                    return message
        if not supress_warning:
            logging.warning(
                f"CAN {self.name}: motor {motor_id} receive timeout. "
                "Check motor power and ID."
            )
        return None
