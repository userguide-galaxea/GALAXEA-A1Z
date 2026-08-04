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
LOWER_TOLERANCE = np.array([0.0, 0.05, 0.0, 0.0, 0.0, 0.0])
UPPER_TOLERANCE = np.zeros(6)


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


def test_j2_negative_value_beyond_tolerance_rejects_entire_frame():
    robot = _robot()
    baseline = np.zeros(6)
    baseline[0] = 0.2
    assert robot.command_joint_pos(baseline) is True

    rejected = baseline.copy()
    rejected[0] = 0.8
    rejected[1] = -0.051
    assert robot.command_joint_state(
        {"pos": rejected, "vel": np.zeros(6)}
    ) is False
    np.testing.assert_array_equal(robot.get_command_state()["pos"], baseline)


def test_non_streaming_validation_uses_the_same_per_joint_tolerance():
    robot = _robot()
    inside = np.zeros(6)
    inside[1] = -0.05
    assert robot._validate_joint_pos(inside)[1] == 0.0

    outside = np.zeros(6)
    outside[1] = -0.0501
    with pytest.raises(ValueError, match="joint2"):
        robot._validate_joint_pos(outside)


def test_only_j2_lower_boundary_has_tolerance():
    info = _robot().get_robot_info()
    assert info["joint_limit_lower_tolerance_rad"] == [
        0.0, 0.05, 0.0, 0.0, 0.0, 0.0
    ]
    assert info["joint_limit_upper_tolerance_rad"] == [0.0] * 6

    robot = _robot()
    j1_below = np.zeros(6)
    j1_below[0] = LIMITS[0][0] - 0.001
    assert robot.command_joint_pos(j1_below) is False

    j2_above = np.zeros(6)
    j2_above[1] = LIMITS[1][1] + 0.001
    assert robot.command_joint_pos(j2_above) is False


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
