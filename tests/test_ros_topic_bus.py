"""Tests for the ROS topic bus (a1z.motor_drivers.ros_topic_bus) helpers.

No ROS involved: decode_motor_frame takes a duck-typed message (same
attributes as lemo_main_board/msg MotorData: header/can_id/arm_id/data), and
RosTopicBus construction is only checked for its ImportError guidance when
rclpy is unavailable.
"""

import threading
from collections import deque
from types import SimpleNamespace

import can
import pytest

from a1z.motor_drivers.ros_topic_bus import (
    ARM_ID_BY_SIDE,
    RosTopicBus,
    check_payload,
    decode_motor_frame,
)


# --- payload validation ---------------------------------------------------------


def test_check_payload_passes_short_payload_unchanged():
    # MotorA 0x7FF management frames carry only 4 meaningful bytes; no padding.
    assert check_payload(bytes([0, 1, 0, 1])) == bytes([0, 1, 0, 1])


def test_check_payload_passes_full_payload():
    payload = bytes(range(8))
    assert check_payload(payload) == payload


def test_check_payload_rejects_oversize():
    with pytest.raises(can.CanOperationError):
        check_payload(bytes(9))


# --- decode helper ---------------------------------------------------------------


def test_decode_motor_frame_arm_id():
    msg = SimpleNamespace(can_id=3, data=[0x11] * 8)
    m = decode_motor_frame(msg)
    assert m.arbitration_id == 3
    assert m.is_rx is True
    assert bytes(m.data) == bytes([0x11] * 8)


def test_decode_motor_frame_claw_and_management_ids():
    for arb_id in (7, 0x307, 0x7FF):
        msg = SimpleNamespace(can_id=arb_id, data=[0x5A] * 8)
        m = decode_motor_frame(msg)
        assert m.arbitration_id == arb_id
        assert m.is_rx is True
        assert bytes(m.data) == bytes([0x5A] * 8)


# --- arm_id mapping and filtering ------------------------------------------


def test_arm_id_by_side():
    assert ARM_ID_BY_SIDE == {"left": 1, "right": 2}


def _bus_for_side(side: str) -> RosTopicBus:
    """Bare instance (no ROS) wired for _on_motor_frame testing."""
    bus = object.__new__(RosTopicBus)
    bus._side = side
    bus._arm_id = ARM_ID_BY_SIDE[side]
    bus._rx_cv = threading.Condition()
    bus._rx_queue = deque()
    return bus


def test_on_motor_frame_enqueues_own_arm():
    bus = _bus_for_side("left")
    bus._on_motor_frame(SimpleNamespace(arm_id=1, can_id=3, data=[0x11] * 8))
    msg = bus.recv(timeout=0.0)
    assert msg is not None
    assert msg.arbitration_id == 3


def test_on_motor_frame_drops_other_arm():
    bus = _bus_for_side("left")
    bus._on_motor_frame(SimpleNamespace(arm_id=2, can_id=3, data=[0x11] * 8))
    assert bus.recv(timeout=0.0) is None


def test_on_motor_frame_accepts_missing_arm_id():
    # Old firmware without the arm_id field: keep accepting feedback.
    bus = _bus_for_side("right")
    bus._on_motor_frame(SimpleNamespace(can_id=7, data=[0x5A] * 8))
    msg = bus.recv(timeout=0.0)
    assert msg is not None
    assert msg.arbitration_id == 7


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
