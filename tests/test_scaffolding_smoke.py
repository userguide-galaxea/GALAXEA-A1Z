"""Smoke tests for the shared scaffolding in tests/conftest.py.

Each fixture/helper is exercised the way a real test would use it, so a
broken double fails here first instead of inside an unrelated suite. Keep
this file small — behaviour coverage lives in the dedicated test modules.
"""

import numpy as np
import pytest

from tests.conftest import (
    DEFAULT_JOINT_LIMITS,
    can_msg,
    make_command_robot,
    make_mixed_chain,
    motor_a_frame,
    motor_b_frame,
)


def test_fake_bus_is_silent(fake_bus):
    fake_bus.send(can_msg(0x01, b"\x00" * 8))
    assert fake_bus.recv() is None


def test_mixed_chain_layout_and_health(mixed_chain):
    seen, age = mixed_chain.get_feedback_health()
    assert seen.shape == (6,)
    assert not seen.any()
    assert np.all(np.isinf(age))
    # Healthy error codes: MotorA 0x0 = no error, MotorB 0x1 = enabled.
    assert not mixed_chain.classify_error_codes(np.array([0, 0, 0, 1, 1, 1])).any()


def test_frame_builders_round_trip_through_drivers(fake_bus):
    from a1z.motor_drivers.motor_a_driver import MotorA
    from a1z.motor_drivers.motor_b_driver import MotorB

    fb_a = MotorA(motor_id=1, bus=fake_bus).parse_feedback(
        can_msg(1, motor_a_frame(0x1, 0x0, 32768))
    )
    assert fb_a is not None and abs(fb_a.position) < 1e-3

    fb_b = MotorB(motor_id=4, bus=fake_bus).parse_feedback(
        can_msg(4, motor_b_frame(0x4, 0x1, 32768))
    )
    assert fb_b is not None and abs(fb_b.position) < 1e-3


def test_echo_frame_is_dropped(mixed_chain):
    echo = can_msg(1, motor_a_frame(0x0, 0x0, 32768), is_rx=False)
    mixed_chain._dispatch_feedback(echo)
    assert mixed_chain._motor_a_list[0].last_feedback is None


def test_command_robot_clips_to_default_limits(command_robot):
    pos = np.zeros(6)
    pos[1] = -0.5  # J2 lower limit is 0.0
    assert command_robot.command_joint_pos(pos) is True
    assert command_robot.get_command_state()["pos"][1] == 0.0


def test_command_robot_factory_accepts_overrides():
    robot = make_command_robot(joint_limits=None)
    pos = np.array([10.0, -10.0, 5.0, -5.0, 3.0, -3.0])
    np.testing.assert_array_equal(robot._clip_joint_pos(pos), pos)


def test_default_limits_match_arm_geometry():
    assert len(DEFAULT_JOINT_LIMITS) == 6
    lo, hi = zip(*DEFAULT_JOINT_LIMITS)
    assert all(l < h for l, h in zip(lo, hi))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
