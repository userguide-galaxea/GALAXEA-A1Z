"""G4 SPI transport bus for the lemo main board.

Talks to the lemo main board's G4 MCU over Linux spidev and presents the
same ``send(msg)`` / ``recv(timeout)`` surface as a python-can ``BusABC``, so
``MotorA`` / ``MotorB`` / ``MixedMotorChain`` / ``Gripper`` work unchanged.

Wire protocol (mirrors lemo_main_board ``src/g4spi.cpp``, keep in sync)::

    Header[2]  Length[2]  CMD_ID[1]  Param[N]  Index[2]  CRC16[2]
    FF FD      little-end  command    payload   little-end little-end

Length = CMD_ID(1) + Param(N) + Index(2) + CRC(2) = N + 5.  CRC is
CRC-16/CCITT-FALSE over every byte except the two CRC bytes themselves.

The MCU bridges one 56-byte Param to the arm's CAN bus: slots 0..5 are the
raw 8-byte CAN payloads of motors 1..6, slot 6 (offset 48) is the claw
payload (sent on CAN ID 0x300+7, feedback arriving on CAN ID 7).  CMD_ID
selects the arm: 0x11 = left, 0x12 = right, same value both directions.

What does NOT fit this channel: management frames on other CAN IDs — the
MotorA enable/disable/set-zero broadcast (0x7FF) and the MotorB register
write (0x7FF + 0x55, e.g. the gripper's mode-4 switch).  Those are dropped
here with a warning; the G4 firmware must handle motor enable and gripper
mode setup itself.  MotorB special frames (0xFC/0xFD/0xFB on the motor's own
ID) DO pass through, since they are ordinary 8-byte payloads on IDs 4..6.

Requires the ``spidev`` package (target board only); imported lazily so the
SDK stays importable off-board.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

import can

from a1z.motor_drivers.command_image import PAYLOAD_SIZE, CommandImage

logger = logging.getLogger(__name__)

# --- G4 frame protocol constants (mirror lemo_main_board src/g4spi.h) ---
_HEADER0 = 0xFF
_HEADER1 = 0xFD
_FRAME_MAX_SIZE = 1024
_FIXED_OVERHEAD = 9  # header(2) + length(2) + cmd(1) + index(2) + crc(2)
_POLL_CHUNK_SIZE = 256  # dummy-clock bytes per poll transfer (as g4spi.cpp)

CMD_LEFT_A1Z_DATA = 0x11
CMD_RIGHT_A1Z_DATA = 0x12


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, identical to ``G4Spi::crc16`` in g4spi.cpp."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd_id: int, param: bytes, index: int) -> bytes:
    """Pack one G4 frame (Header/Length/CMD/Param/Index/CRC)."""
    if len(param) > _FRAME_MAX_SIZE - _FIXED_OVERHEAD:
        raise ValueError(f"param of {len(param)} bytes exceeds protocol limit")
    frame = bytearray(_FIXED_OVERHEAD + len(param))
    frame[0] = _HEADER0
    frame[1] = _HEADER1
    struct.pack_into("<H", frame, 2, len(param) + 5)
    frame[4] = cmd_id
    frame[5 : 5 + len(param)] = param
    struct.pack_into("<H", frame, 5 + len(param), index & 0xFFFF)
    struct.pack_into("<H", frame, len(frame) - 2, crc16_ccitt_false(bytes(frame[:-2])))
    return bytes(frame)


class G4FrameParser:
    """Byte-stream parser for G4 frames; tolerates splits, noise and CRC errors.

    Equivalent to the ``G4Spi`` parser state machine in g4spi.cpp: on any
    framing or CRC error it drops one byte and resynchronises on the header.
    Parsed frames are appended to ``frames`` as ``(cmd_id, index, param)``.
    """

    def __init__(self) -> None:
        self.frames: Deque[Tuple[int, int, bytes]] = deque()
        self.error_count = 0
        self._buf = bytearray()

    def feed(self, data: bytes) -> None:
        self._buf += data
        while True:
            # Resynchronise on the 0xFF 0xFD header.
            start = self._buf.find(b"\xff\xfd")
            if start < 0:
                # Keep a trailing 0xFF — it may be the first header byte.
                keep = 1 if self._buf.endswith(b"\xff") else 0
                del self._buf[: len(self._buf) - keep]
                return
            if start > 0:
                self.error_count += 1
                del self._buf[:start]
            if len(self._buf) < 4:
                return
            (length_field,) = struct.unpack_from("<H", self._buf, 2)
            if length_field < 5 or length_field > _FRAME_MAX_SIZE - 4:
                self.error_count += 1
                logger.warning("[g4spi] invalid Length=%d, resyncing", length_field)
                del self._buf[0]
                continue
            frame_size = 4 + length_field
            if len(self._buf) < frame_size:
                return
            frame = bytes(self._buf[:frame_size])
            del self._buf[:frame_size]
            (received_crc,) = struct.unpack_from("<H", frame, frame_size - 2)
            if received_crc != crc16_ccitt_false(frame[:-2]):
                self.error_count += 1
                logger.warning(
                    "[g4spi] CRC mismatch cmd=0x%02x, resyncing", frame[4]
                )
                # The dropped bytes may contain a valid header; rescan them.
                self._buf = bytearray(frame[1:]) + self._buf
                continue
            param_length = length_field - 5
            (index,) = struct.unpack_from("<H", frame, 5 + param_length)
            self.frames.append((frame[4], index, frame[5 : 5 + param_length]))


class G4SpiBus:
    """python-can BusABC-compatible facade over the lemo board G4 SPI link.

    ``send()`` writes the message payload into the matching slot of the
    56-byte command image; once all six arm-motor slots have been written the
    image is transmitted as one G4 frame (matching the SDK's per-tick burst of
    six motor frames followed by an optional claw frame — the claw slot
    simply carries its most recent value).  ``recv()`` returns feedback
    decoded from incoming frames as ``can.Message`` objects with
    ``arbitration_id`` 1..6 (arm motors) and 7 (claw), ``is_rx=True`` and a
    Linux-side timestamp.

    A daemon poller thread clocks dummy bytes at ``poll_period_s`` so MCU
    feedback flows independently of the caller's recv() pattern (same role as
    the 2 kHz receive loop in the board's ROS node).

    Args:
        spi_device: spidev node, e.g. ``/dev/spidev0.0``.
        spi_speed_hz: SPI clock; the board's ROS node defaults to 10 MHz.
        arm_side: ``"left"`` (CMD 0x11) or ``"right"`` (CMD 0x12).
        poll_period_s: dummy-clock poll period of the background thread.
        spi: optional already-configured spidev-like object (test hook);
            when given, ``spi_device`` is not opened.
    """

    def __init__(
        self,
        spi_device: str = "/dev/spidev0.0",
        spi_speed_hz: int = 10_000_000,
        arm_side: str = "left",
        poll_period_s: float = 0.0005,
        spi=None,
    ) -> None:
        if arm_side == "left":
            self._cmd_id = CMD_LEFT_A1Z_DATA
        elif arm_side == "right":
            self._cmd_id = CMD_RIGHT_A1Z_DATA
        else:
            raise ValueError(f"arm_side must be 'left' or 'right', got {arm_side!r}")

        if spi is None:
            import spidev  # target board only

            spi = spidev.SpiDev()
            spi.open(*self._parse_spi_device(spi_device))
            spi.mode = 0
            spi.bits_per_word = 8
            spi.max_speed_hz = spi_speed_hz
        self._spi = spi

        self._lock = threading.Lock()  # serialises SPI transfers + parser
        self._rx_cv = threading.Condition()
        self._rx_queue: Deque[can.Message] = deque()
        self._parser = G4FrameParser()

        self._tx = CommandImage()
        self._send_index = 0

        self._stop = threading.Event()
        self._poller = threading.Thread(
            target=self._poll_loop, args=(poll_period_s,), name="g4spi-poller", daemon=True
        )
        self._poller.start()

    # -- construction helpers -------------------------------------------

    @staticmethod
    def _parse_spi_device(device: str) -> Tuple[int, int]:
        """``/dev/spidev<bus>.<cs>`` -> ``(bus, cs)`` for ``SpiDev.open``."""
        node = device.rsplit("/", 1)[-1]
        if not node.startswith("spidev"):
            raise ValueError(f"spi_device must look like /dev/spidev<bus>.<cs>, got {device!r}")
        bus_str, _, cs_str = node[len("spidev") :].partition(".")
        return int(bus_str), int(cs_str)

    # -- python-can compatible surface -----------------------------------

    def send(self, msg: can.Message, timeout: Optional[float] = None) -> None:
        """Buffer one CAN payload into the command image; flush when full."""
        with self._lock:
            if self._tx.write(msg.arbitration_id, bytes(msg.data)):
                self._flush_locked()

    def recv(self, timeout: Optional[float] = 0.0) -> Optional[can.Message]:
        """Return the next decoded feedback message, or None on timeout."""
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._rx_cv:
            while not self._rx_queue:
                if timeout is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._rx_cv.wait(remaining)
                else:
                    self._rx_cv.wait()
            return self._rx_queue.popleft()

    def shutdown(self) -> None:
        """Stop the poller thread and close the SPI device."""
        self._stop.set()
        self._poller.join(timeout=1.0)
        with self._rx_cv:
            self._rx_cv.notify_all()
        try:
            self._spi.close()
        except Exception:  # noqa: BLE001 - close must not raise
            logger.exception("[g4spi] error closing SPI device")

    @property
    def parse_error_count(self) -> int:
        return self._parser.error_count

    # -- internals ---------------------------------------------------------

    def _poll_loop(self, poll_period_s: float) -> None:
        dummy = [0x00] * _POLL_CHUNK_SIZE
        errors = 0
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                with self._lock:
                    rx = self._spi.xfer2(dummy)
                self._ingest_rx(bytes(rx))
            except Exception:  # noqa: BLE001 - keep clocking; device may recover
                errors += 1
                if errors == 1 or errors % 1000 == 0:
                    logger.exception("[g4spi] poll transfer failed (count=%d)", errors)
                time.sleep(0.01)
                continue
            elapsed = time.monotonic() - started
            if elapsed < poll_period_s:
                time.sleep(poll_period_s - elapsed)

    def _flush_locked(self) -> None:
        """Transmit the 56-byte command image as one G4 frame. Caller holds lock."""
        frame = build_frame(self._cmd_id, bytes(self._tx.payload), self._send_index)
        self._send_index += 1
        self._tx.reset()
        try:
            rx = self._spi.xfer2(list(frame))
        except OSError as exc:
            raise can.CanOperationError(f"g4spi command frame transfer failed: {exc}") from exc
        # Full duplex: bytes clocked in while sending may carry feedback.
        self._ingest_rx_locked(bytes(rx))

    def _ingest_rx(self, data: bytes) -> None:
        with self._lock:
            self._ingest_rx_locked(data)

    def _ingest_rx_locked(self, data: bytes) -> None:
        self._parser.feed(data)
        messages: List[can.Message] = []
        while self._parser.frames:
            cmd_id, _index, param = self._parser.frames.popleft()
            if cmd_id != self._cmd_id or len(param) != PAYLOAD_SIZE:
                # Servo-state / leader / other-arm traffic is not ours.
                continue
            stamp = time.time()
            for slot in range(PAYLOAD_SIZE // 8):
                arb = slot + 1  # slots 0..5 -> IDs 1..6, slot 6 (claw) -> ID 7
                messages.append(
                    can.Message(
                        timestamp=stamp,
                        arbitration_id=arb,
                        data=param[slot * 8 : slot * 8 + 8],
                        is_extended_id=False,
                        is_rx=True,
                    )
                )
        if messages:
            with self._rx_cv:
                self._rx_queue.extend(messages)
                self._rx_cv.notify_all()

    def __enter__(self) -> "G4SpiBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.shutdown()
