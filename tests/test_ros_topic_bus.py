"""Tests for the shared CommandImage and the ROS topic bus decode helpers.

No ROS involved: decode helpers take duck-typed messages (same attributes as
lemo_main_board/msg A1zFrame / Claw), and RosTopicBus construction is only
checked for its ImportError guidance when rclpy is unavailable.
"""

from types import SimpleNamespace

import pytest

from a1z.motor_drivers.command_image import PAYLOAD_SIZE, CommandImage
from a1z.motor_drivers.ros_topic_bus import decode_a1z_frame, decode_claw


# --- CommandImage -------------------------------------------------------------


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


# --- ROS decode helpers ---------------------------------------------------------


def fake_a1z_frame(seed: int = 0):
    motors = [
        SimpleNamespace(id=i + 1, data=[(seed + i) & 0xFF] * 8)
        for i in range(6)
    ]
    return SimpleNamespace(motors=motors)


def test_decode_a1z_frame():
    msgs = decode_a1z_frame(fake_a1z_frame(seed=10))
    assert [m.arbitration_id for m in msgs] == [1, 2, 3, 4, 5, 6]
    assert all(m.is_rx for m in msgs)
    for i, m in enumerate(msgs):
        assert bytes(m.data) == bytes([(10 + i) & 0xFF] * 8)


def test_decode_claw():
    msg = SimpleNamespace(data=[0x5A] * 8)
    m = decode_claw(msg)
    assert m.arbitration_id == 7
    assert m.is_rx is True
    assert bytes(m.data) == bytes([0x5A] * 8)


# --- RosTopicBus import guard ---------------------------------------------------


def test_ros_topic_bus_import_error_is_actionable():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        from a1z.motor_drivers.ros_topic_bus import RosTopicBus

        with pytest.raises(ImportError, match="lemo_main_board"):
            RosTopicBus(arm_side="left")
    else:
        pytest.skip("rclpy available; import-guard path not exercised")


def test_ros_topic_bus_validates_arm_side():
    pytest.importorskip("rclpy")
    pytest.importorskip("lemo_main_board.msg")
    from a1z.motor_drivers.ros_topic_bus import RosTopicBus

    with pytest.raises(ValueError, match="arm_side"):
        RosTopicBus(arm_side="middle")
