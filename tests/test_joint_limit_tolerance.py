"""Offline tests for physical limits and per-joint command tolerance."""

import numpy as np
import pytest

from a1z.robots.arm_robot import ArmRobot


LIMITS = [
    (-2.094, 2.094),
    (0.0, 3.142),
    (-3.142, 0.0),
    (-1.484, 1.484),
    (-1.484, 1.484),
    (-2.007, 2.007),
]
LOWER_TOLERANCE = np.full(6, 0.05)
UPPER_TOLERANCE = np.full(6, 0.05)


def _robot():
    robot = ArmRobot(
        motor_chain=None,
        bus=None,
        gravity_model=None,
        joint_limits=LIMITS,
        joint_limit_lower_tolerance_rad=LOWER_TOLERANCE,
        joint_limit_upper_tolerance_rad=UPPER_TOLERANCE,
    )
    robot._commands_blocked.clear()
    return robot


def test_j2_negative_value_inside_tolerance_is_accepted_and_clipped_to_zero():
    robot = _robot()
    target = np.zeros(6)
    target[1] = -0.049

    assert robot.command_joint_pos(target) is True
    assert robot.get_command_state()["pos"][1] == 0.0


def test_j2_negative_value_beyond_tolerance_is_clipped_not_rejected():
    robot = _robot()
    baseline = np.zeros(6)
    baseline[0] = 0.2
    assert robot.command_joint_pos(baseline) is True

    # Beyond tolerance: now clipped instead of rejected.
    # J2=-0.051 is beyond lower tolerance (lo=0.0, tol=0.05 → allowed_lo=-0.05),
    # so it is clipped to 0.0. Other joints track freely.
    beyond = baseline.copy()
    beyond[0] = 0.8
    beyond[1] = -0.051
    assert robot.command_joint_state(
        {"pos": beyond, "vel": np.zeros(6)}
    ) is True
    cmd = robot.get_command_state()
    assert cmd["pos"][0] == 0.8    # J1 accepted as-is (valid joint)
    assert cmd["pos"][1] == 0.0    # J2 clipped to lower boundary


def test_non_streaming_validation_uses_the_same_per_joint_tolerance():
    robot = _robot()
    inside = np.zeros(6)
    inside[1] = -0.05
    assert robot._validate_joint_pos(inside)[1] == 0.0

    outside = np.zeros(6)
    outside[1] = -0.0501
    with pytest.raises(ValueError, match="joint2"):
        robot._validate_joint_pos(outside)


def test_all_boundaries_have_uniform_tolerance():
    info = _robot().get_robot_info()
    assert info["joint_limit_lower_tolerance_rad"] == [0.05] * 6
    assert info["joint_limit_upper_tolerance_rad"] == [0.05] * 6

    # Within tolerance of a boundary: accepted and clipped onto the limit.
    robot = _robot()
    j1_below = np.zeros(6)
    j1_below[0] = LIMITS[0][0] - 0.049
    assert robot.command_joint_pos(j1_below) is True
    assert robot.get_command_state()["pos"][0] == LIMITS[0][0]

    robot = _robot()
    robot._command.pos[1] = 3.0  # seed q_current near the bound so unwrap doesn't trigger
    j2_above = np.zeros(6)
    j2_above[1] = LIMITS[1][1] + 0.049
    assert robot.command_joint_pos(j2_above) is True
    assert robot.get_command_state()["pos"][1] == LIMITS[1][1]

    # Beyond tolerance: the frame is accepted and the value clipped to limit.
    robot = _robot()
    j1_too_far = np.zeros(6)
    j1_too_far[0] = LIMITS[0][0] - 0.051
    assert robot.command_joint_pos(j1_too_far) is True
    assert robot.get_command_state()["pos"][0] == LIMITS[0][0]

    robot = _robot()
    robot._command.pos[1] = 3.0  # seed so unwrap doesn't trigger on first frame
    j2_too_far = np.zeros(6)
    j2_too_far[1] = LIMITS[1][1] + 0.051
    assert robot.command_joint_pos(j2_too_far) is True
    assert robot.get_command_state()["pos"][1] == LIMITS[1][1]


@pytest.mark.parametrize("bad", [np.zeros(5), np.full(6, -0.01), np.full(6, np.nan)])
def test_invalid_tolerance_configuration_is_rejected(bad):
    with pytest.raises(ValueError, match="joint_limit_lower_tolerance_rad"):
        ArmRobot(
            motor_chain=None,
            bus=None,
            gravity_model=None,
            joint_limits=LIMITS,
            joint_limit_lower_tolerance_rad=bad,
        )
