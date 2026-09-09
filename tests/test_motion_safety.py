"""Tests for streaming motion-safety checks (P1 always-clip behaviour)."""

import logging
import time

import numpy as np
import pytest

from a1z.robots.arm_robot import ArmRobot

# Imported verbatim from the GitLab source repo (feat/circle-clip @ 598df97)
# for port stage P4. The file exercises the always-clip / S¹-unwrap clip
# policy, per-joint limit tolerances and bool-returning command entries,
# all of which landed with stage P5 (clip strategy).

LIMITS = [
    (-2.094, 2.094),   # J1
    (0.0, 3.142),       # J2
    (-3.142, 0.0),      # J3
    (-1.484, 1.484),    # J4
    (-1.484, 1.484),    # J5
    (-2.007, 2.007),    # J6
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


class TestClipJointPos:
    """Always-clip behaviour for _clip_joint_pos (P1)."""

    def test_clip_never_returns_none(self):
        """_clip_joint_pos always returns an ndarray, never None."""
        robot = _robot()
        # Valid within limits
        assert robot._clip_joint_pos(np.zeros(6)) is not None
        # Large violations
        way_out = np.array([10.0, -10.0, 10.0, 10.0, 10.0, 10.0])
        assert robot._clip_joint_pos(way_out) is not None
        # Small overshoot
        edge = np.zeros(6)
        edge[1] = -0.02
        assert robot._clip_joint_pos(edge) is not None

    def test_clip_at_boundary(self):
        """J2 negative value is clipped to lower boundary 0.0."""
        robot = _robot()
        pos = np.zeros(6)
        pos[1] = -0.5  # well past J2 lower limit 0.0
        result = robot._clip_joint_pos(pos)
        assert result[1] == 0.0

    def test_clip_preserves_valid_joints(self):
        """Only the violating joint is clipped; other joints are unchanged."""
        robot = _robot()
        pos = np.array([0.5, -1.0, -1.5, 0.3, -0.3, 1.0])
        result = robot._clip_joint_pos(pos)
        # J1 in [-2.094, 2.094], 0.5 is valid
        assert result[0] == 0.5
        # J2 in [0, 3.142], -1.0 clipped to 0.0
        assert result[1] == 0.0
        # J3 in [-3.142, 0], -1.5 is valid
        assert result[2] == -1.5
        # J4 in [-1.484, 1.484], 0.3 is valid
        assert result[3] == 0.3
        # J5 in [-1.484, 1.484], -0.3 is valid
        assert result[4] == -0.3
        # J6 in [-2.007, 2.007], 1.0 is valid
        assert result[5] == 1.0

    def test_clip_large_violation_logs_error(self, caplog):
        """Large violations (>tol_rad) produce ERROR log on every frame."""
        robot = _robot()
        pos = np.zeros(6)
        pos[1] = -0.5  # beyond lower tolerance (allowed_lo = 0.0 - 0.05 = -0.05)

        with caplog.at_level(logging.ERROR):
            robot._clip_joint_pos(pos)

        assert any("large limit violation" in r.message.lower() for r in caplog.records)

        # Second call also logs (no rate limiting for large violations)
        caplog.clear()
        with caplog.at_level(logging.ERROR):
            robot._clip_joint_pos(pos)
        assert any("large limit violation" in r.message.lower() for r in caplog.records)

    def test_clip_small_violation_rate_limited(self, caplog):
        """Small overshoots (≤tol_rad) produce 1 Hz rate-limited WARNING."""
        robot = _robot()
        pos = np.zeros(6)
        pos[1] = -0.02  # within tolerance (allowed_lo = -0.05)

        # First call should log
        with caplog.at_level(logging.WARNING):
            robot._clip_joint_pos(pos)
        assert any("minor overshoot" in r.message.lower() for r in caplog.records)

        # Immediate second call should NOT log (rate limited)
        caplog.clear()
        with caplog.at_level(logging.WARNING):
            robot._clip_joint_pos(pos)
        assert not any("minor overshoot" in r.message.lower() for r in caplog.records)

    def test_no_limits_returns_unchanged(self):
        """When joint_limits is None, positions pass through unchanged."""
        robot = ArmRobot(
            motor_chain=None,
            bus=None,
            gravity_model=None,
            joint_limits=None,
        )
        robot._commands_blocked.clear()
        pos = np.array([10.0, -10.0, 5.0, -5.0, 3.0, -3.0])
        result = robot._clip_joint_pos(pos)
        np.testing.assert_array_equal(result, pos)

    def test_command_joint_pos_accepts_previously_rejected_frame(self):
        """Frames that used to be rejected are now accepted and clipped."""
        robot = _robot()
        j2_beyond = np.zeros(6)
        j2_beyond[1] = -1.0  # previously rejected, now clipped to 0.0
        assert robot.command_joint_pos(j2_beyond) is True
        assert robot.get_command_state()["pos"][1] == 0.0

    def test_streaming_end_to_end_clip_and_recover(self):
        """Simulate teleop: push past limit → hold → return → resume tracking."""
        robot = _robot()

        # Normal tracking
        normal = np.zeros(6)
        normal[1] = 1.5  # J2=1.5 rad
        assert robot.command_joint_pos(normal) is True
        assert robot.get_command_state()["pos"][1] == 1.5

        # Leader pushed J2 past lower limit
        past_limit = np.zeros(6)
        past_limit[1] = -0.8  # clipped to 0.0
        assert robot.command_joint_pos(past_limit) is True
        assert robot.get_command_state()["pos"][1] == 0.0  # parked at boundary

        # Leader returns to valid range — resume from boundary
        recovered = np.zeros(6)
        recovered[1] = 0.3
        assert robot.command_joint_pos(recovered) is True
        assert robot.get_command_state()["pos"][1] == 0.3
        # Δpos = 0.3 (from boundary 0.0), not 1.8 (from frozen 1.5)


class TestCircleClipJointPos:
    """S¹-topology aware clipping (circle-clip) for _clip_joint_pos."""

    def test_no_q_current_falls_back_to_linear_clip(self):
        """Without q_current, behaves identically to plain np.clip."""
        robot = _robot()
        result = robot._clip_joint_pos(np.array([0.0, -1.0, 0.0, 0.0, 0.0, 0.0]))
        assert result[1] == 0.0  # J2 clipped to lower boundary

    def test_wrap_protection_stays_at_boundary(self):
        """Leader wrap across ±π → unwrap finds nearest equivalent, stays at π."""
        robot = _robot()
        q_current = np.array([0.0, 3.14, 0.0, 0.0, 0.0, 0.0])
        target = np.array([0.0, -3.106, 0.0, 0.0, 0.0, 0.0])
        # -3.106 is 3.177 when unwrapped (k=-1), clipped to π
        result = robot._clip_joint_pos(target, q_current)
        assert result[1] == pytest.approx(np.pi, abs=0.01)

    def test_normal_tracking_unchanged(self):
        """Normal tracking passes through without distortion."""
        robot = _robot()
        q_current = np.array([0.0, 1.5, 0.0, 0.0, 0.0, 0.0])
        target = np.array([0.0, 1.6, 0.0, 0.0, 0.0, 0.0])
        result = robot._clip_joint_pos(target, q_current)
        assert result[1] == 1.6

    def test_genuine_boundary_violation_still_clips(self):
        """Real overshoot across 0 boundary still clips to 0."""
        robot = _robot()
        q_current = np.array([0.0, 0.175, 0.0, 0.0, 0.0, 0.0])
        target = np.array([0.0, -0.175, 0.0, 0.0, 0.0, 0.0])
        result = robot._clip_joint_pos(target, q_current)
        assert result[1] == 0.0  # Leader crossed 0 for real → clip

    def test_recovery_from_boundary(self):
        """Recovery from boundary is smooth."""
        robot = _robot()
        q_current = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        target = np.array([0.0, 0.3, 0.0, 0.0, 0.0, 0.0])
        result = robot._clip_joint_pos(target, q_current)
        assert result[1] == 0.3

    def test_multi_turn_unwrap(self):
        """Multiple full turns unwrap correctly."""
        robot = _robot()
        q_current = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        target = np.array([0.0, -10.0, 0.0, 0.0, 0.0, 0.0])
        # diff = -11.0, k = round(-11.0/6.283) = round(-1.751) = -2
        # unwrapped = -10.0 - (-2)*6.283 = 2.566, clip(2.566, 0, π) = 2.566
        result = robot._clip_joint_pos(target, q_current)
        assert result[1] == pytest.approx(2.566, abs=0.01)

    def test_symmetric_limits_j1_unwrap(self):
        """J1 symmetric limits [-2.094, 2.094] with wrap from + side."""
        robot = _robot()
        q_current = np.array([-2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        target = np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # diff = 5.0, k = round(5.0/6.283) = round(0.796) = 1
        # unwrapped = 3.0 - 6.283 = -3.283
        # clip(-3.283, -2.094, 2.094) = -2.094
        result = robot._clip_joint_pos(target, q_current)
        assert result[0] == pytest.approx(-2.094, abs=0.01)

    def test_unwrap_finds_nearest_equivalent_on_circle(self):
        """Always unwrap to the nearest S¹ equivalent, even for |diff| < 2π."""
        robot = _robot()
        # q_current=3.0, target=-0.5, diff=-3.5, k=-1
        # unwrapped = -0.5+2π = 5.783, clip(0,π) = π
        q_current = np.array([0.0, 3.0, 0.0, 0.0, 0.0, 0.0])
        target = np.array([0.0, -0.5, 0.0, 0.0, 0.0, 0.0])
        result = robot._clip_joint_pos(target, q_current)
        assert result[1] == pytest.approx(np.pi, abs=0.01)
