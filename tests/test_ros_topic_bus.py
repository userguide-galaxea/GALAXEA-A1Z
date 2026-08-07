"""Tests for the ROS topic bus (a1z.motor_drivers.ros_topic_bus) helpers.

No ROS involved: decode_motor_frame takes a duck-typed message (same
attributes as lemo_main_board/msg CanFrame: header/id/data), and RosTopicBus
construction is only checked for its ImportError guidance when rclpy is
unavailable.
"""

from types import SimpleNamespace

import can
import pytest

from a1z.motor_drivers.ros_topic_bus import decode_motor_frame, to_fixed8


# --- payload padding ------------------------------------------------------------


def test_to_fixed8_pads_short_payload():
    # MotorA 0x7FF management frames carry only 4 meaningful bytes.
    assert to_fixed8(bytes([0, 1, 0, 1])) == bytes([0, 1, 0, 1, 0, 0, 0, 0])


def test_to_fixed8_passes_full_payload():
    payload = bytes(range(8))
    assert to_fixed8(payload) == payload


def test_to_fixed8_rejects_oversize():
    with pytest.raises(can.CanOperationError):
        to_fixed8(bytes(9))


# --- decode helper ---------------------------------------------------------------


def test_decode_motor_frame_arm_id():
    msg = SimpleNamespace(id=3, data=[0x11] * 8)
    m = decode_motor_frame(msg)
    assert m.arbitration_id == 3
    assert m.is_rx is True
    assert bytes(m.data) == bytes([0x11] * 8)


def test_decode_motor_frame_claw_and_management_ids():
    for arb_id in (7, 0x307, 0x7FF):
        msg = SimpleNamespace(id=arb_id, data=[0x5A] * 8)
        m = decode_motor_frame(msg)
        assert m.arbitration_id == arb_id
        assert m.is_rx is True
        assert bytes(m.data) == bytes([0x5A] * 8)


# --- RosTopicBus import guard -----------------------------------------------------


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
