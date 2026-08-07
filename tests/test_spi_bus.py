"""Tests for the G4 SPI transport bus (a1z.motor_drivers.spi_bus).

No hardware involved: a FakeSpi stands in for spidev, and frame bytes are
built/parsed with the same pure helpers the bus uses.
"""

import struct
import threading
from collections import deque

import can
import pytest

from a1z.motor_drivers.command_image import CommandImage
from a1z.motor_drivers.spi_bus import (
    CMD_LEFT_A1Z_DATA,
    CMD_RIGHT_A1Z_DATA,
    G4FrameParser,
    G4SpiBus,
    build_frame,
    crc16_ccitt_false,
)


class FakeSpi:
    """spidev stand-in: records TX, replays queued RX blobs (zeros otherwise)."""

    def __init__(self):
        self.sent = []
        self.rx_queue = deque()
        self.lock = threading.Lock()

    def xfer2(self, data):
        with self.lock:
            self.sent.append(bytes(data))
            if self.rx_queue:
                blob = self.rx_queue.popleft()
                return list(blob)
        return [0x00] * len(data)

    def push_rx(self, blob: bytes):
        with self.lock:
            self.rx_queue.append(blob)

    def close(self):
        pass


@pytest.fixture
def fake_spi():
    return FakeSpi()


@pytest.fixture
def bus(fake_spi):
    b = G4SpiBus(spi=fake_spi, arm_side="left", poll_period_s=0.001)
    yield b
    b.shutdown()


def make_payload56(seed: int = 0) -> bytes:
    return bytes((seed + i) & 0xFF for i in range(56))


def command_frames(fake_spi: FakeSpi):
    """Only command flushes carry the FF FD header; poller TX is all zeros."""
    return [f for f in fake_spi.sent if f[:2] == b"\xff\xfd"]


# --- pure helpers -----------------------------------------------------------


def test_crc16_ccitt_false_known_vector():
    # CRC-16/CCITT-FALSE of ASCII "123456789" is 0x29B1 by definition.
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_build_frame_layout():
    param = bytes(range(56))
    frame = build_frame(CMD_LEFT_A1Z_DATA, param, 0x1234)
    assert frame[0] == 0xFF and frame[1] == 0xFD
    (length_field,) = struct.unpack_from("<H", frame, 2)
    assert length_field == 56 + 5
    assert frame[4] == CMD_LEFT_A1Z_DATA
    assert frame[5:61] == param
    (index,) = struct.unpack_from("<H", frame, 61)
    assert index == 0x1234
    (crc,) = struct.unpack_from("<H", frame, 63)
    assert crc == crc16_ccitt_false(frame[:-2])
    assert len(frame) == 65


def test_parser_roundtrip_and_split():
    frame = build_frame(CMD_LEFT_A1Z_DATA, make_payload56(), 7)
    parser = G4FrameParser()
    # Feed byte-by-byte: frames may arrive split across SPI transfers.
    for i in range(len(frame)):
        parser.feed(frame[i : i + 1])
    assert len(parser.frames) == 1
    cmd_id, index, param = parser.frames.popleft()
    assert cmd_id == CMD_LEFT_A1Z_DATA
    assert index == 7
    assert param == make_payload56()
    assert parser.error_count == 0


def test_parser_resyncs_after_noise_and_crc_error():
    good = build_frame(CMD_RIGHT_A1Z_DATA, make_payload56(1), 3)
    bad_crc = bytearray(build_frame(CMD_RIGHT_A1Z_DATA, make_payload56(2), 4))
    bad_crc[10] ^= 0xFF  # corrupt payload -> CRC mismatch
    bad_len = b"\xff\xfd\x01\x00"  # Length < 5, invalid
    parser = G4FrameParser()
    parser.feed(b"\x00\x11\x22" + bytes(bad_crc) + bad_len + good)
    assert len(parser.frames) == 1
    cmd_id, index, _ = parser.frames.popleft()
    assert (cmd_id, index) == (CMD_RIGHT_A1Z_DATA, 3)
    assert parser.error_count >= 2


# --- bus behaviour ------------------------------------------------------------


def test_send_buffers_then_flushes_full_image(bus, fake_spi):
    for motor_id in range(1, 7):
        bus.send(can.Message(arbitration_id=motor_id, data=[motor_id] * 8, is_extended_id=False))
    # Flush happens exactly once, when the 6th arm slot is written.
    frames = command_frames(fake_spi)
    assert len(frames) == 1
    frame = frames[0]
    assert frame[4] == CMD_LEFT_A1Z_DATA
    for motor_id in range(1, 7):
        off = 5 + (motor_id - 1) * 8
        assert frame[off : off + 8] == bytes([motor_id] * 8)


def test_send_claw_slot_via_hybrid_and_plain_id(bus, fake_spi):
    bus.send(can.Message(arbitration_id=0x307, data=[0xAA] * 8, is_extended_id=False))
    bus.send(can.Message(arbitration_id=7, data=[0xBB] * 8, is_extended_id=False))
    for motor_id in range(1, 7):
        bus.send(can.Message(arbitration_id=motor_id, data=[motor_id] * 8, is_extended_id=False))
    frames = command_frames(fake_spi)
    assert len(frames) == 1
    frame = frames[0]
    # Plain ID 7 overwrote the earlier 0x307 write in the shared claw slot.
    assert frame[53:61] == bytes([0xBB] * 8)


def test_send_drops_untransportable_management_ids(bus, fake_spi):
    # 0x7FF broadcast (MotorA enable / register writes) has no slot.
    bus.send(can.Message(arbitration_id=0x7FF, data=[0, 1, 0, 1], is_extended_id=False))
    for motor_id in range(1, 7):
        bus.send(can.Message(arbitration_id=motor_id, data=[motor_id] * 8, is_extended_id=False))
    assert len(command_frames(fake_spi)) == 1  # only the real flush, no extra traffic


def test_send_right_arm_uses_cmd_0x12(fake_spi):
    b = G4SpiBus(spi=fake_spi, arm_side="right", poll_period_s=0.001)
    try:
        for motor_id in range(1, 7):
            bus_msg = can.Message(arbitration_id=motor_id, data=[motor_id] * 8, is_extended_id=False)
            b.send(bus_msg)
        assert command_frames(fake_spi)[0][4] == CMD_RIGHT_A1Z_DATA
    finally:
        b.shutdown()


def test_recv_decodes_incoming_frame(bus, fake_spi):
    payload = make_payload56(9)
    fake_spi.push_rx(build_frame(CMD_LEFT_A1Z_DATA, payload, 0))
    msgs = [bus.recv(timeout=1.0) for _ in range(7)]
    assert all(m is not None for m in msgs)
    assert [m.arbitration_id for m in msgs] == [1, 2, 3, 4, 5, 6, 7]
    assert all(m.is_rx for m in msgs)
    for slot, m in enumerate(msgs):
        assert bytes(m.data) == payload[slot * 8 : slot * 8 + 8]


def test_recv_ignores_other_cmd_ids(bus, fake_spi):
    # e.g. servo-state traffic (0x03) and the other arm's data (0x12).
    fake_spi.push_rx(build_frame(0x03, make_payload56(1), 0))
    fake_spi.push_rx(build_frame(CMD_RIGHT_A1Z_DATA, make_payload56(2), 0))
    assert bus.recv(timeout=0.05) is None


def test_recv_timeout_zero_is_nonblocking(bus):
    assert bus.recv(timeout=0.0) is None



# --- CommandImage (shared 56-byte command buffering) ------------------------------


def test_full_arm_burst_triggers_flush():
    img = CommandImage()
    for motor_id in range(1, 6):
        assert img.write(motor_id, bytes([motor_id] * 8)) is False
    assert img.write(6, bytes([6] * 8)) is True
    for motor_id in range(1, 7):
        off = (motor_id - 1) * 8
        assert img.payload[off : off + 8] == bytes([motor_id] * 8)


def test_claw_slots_and_seen_flag():
    img = CommandImage()
    assert img.claw_seen is False
    assert img.write(0x307, bytes([0xAA] * 8)) is False  # hybrid command ID
    assert img.claw_seen is True
    assert img.write(7, bytes([0xBB] * 8)) is False  # plain gripper ID, same slot
    assert img.payload[48:56] == bytes([0xBB] * 8)


def test_reset_keeps_payload_clears_trigger():
    img = CommandImage()
    for motor_id in range(1, 7):
        img.write(motor_id, bytes([motor_id] * 8))
    img.reset()
    # After reset, one more write alone must not retrigger a flush...
    assert img.write(1, bytes([0xFF] * 8)) is False
    # ...but the payload keeps the previously latched values.
    assert img.payload[8:16] == bytes([2] * 8)


def test_untransportable_and_malformed_frames_dropped():
    img = CommandImage()
    assert img.write(0x7FF, bytes([0, 1, 0, 1])) is False  # management broadcast
    assert img.write(1, bytes([1, 2, 3])) is False  # wrong length
    assert not any(img.payload)  # nothing landed in the image
