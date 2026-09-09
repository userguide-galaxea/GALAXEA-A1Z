"""Offline L0 unit tests for feedforward clear-semantics at command entries.

Imported from the GitLab source repo (feat/circle-clip @ 598df97) for port
stage P5, with the error-integrator cases (SOP-09 W4 reset paths, tau_i
observability) removed — they exercise ``a1z.robots.integrator`` and the
``integral_config`` constructor parameter, internal algorithm details that
are not part of this version.

Builds an ``ArmRobot`` with no motor chain / bus / gravity model (the command
entry points touch none of them) to pin the SOP-09 P0-1 guarantees:

  * every feedforward field not supplied by an entry point is explicitly zeroed
    (no stale acc / torque_ff / vel carry-over — devlog 2026-07-22 Q9 hazard),
  * torque_ff / acc are accepted through command_joint_state with reject-frame
    bounds.

Run: ``pytest tests/test_command_clear_semantics.py -v``.
"""

import numpy as np
import pytest

from a1z.robots.arm_robot import ArmRobot, ControlState

_TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])


def _robot():
    r = ArmRobot(
        motor_chain=None, bus=None, gravity_model=None, num_joints=6,
        torque_clip=_TORQUE_CLIP, joint_limits=None,
    )
    r._commands_blocked.clear()
    r._control_state = ControlState.RUNNING
    return r


# --- P0-1 clear semantics ---------------------------------------------------
def test_command_joint_state_zeroes_unset_feedforward():
    r = _robot()
    r._command.acc = np.full(6, 0.5)        # stale from a prior move_joints
    r._command.torque_ff = np.full(6, 0.3)  # stale from a prior raw command
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6)})
    assert np.allclose(r._command.acc, 0.0)
    assert np.allclose(r._command.torque_ff, 0.0)


def test_command_joint_pos_zeroes_vel_and_acc():
    r = _robot()
    r._command.vel = np.full(6, 0.7)
    r._command.acc = np.full(6, 0.4)
    r.command_joint_pos(np.zeros(6))
    assert np.allclose(r._command.vel, 0.0)
    assert np.allclose(r._command.acc, 0.0)
    assert np.allclose(r._command.torque_ff, 0.0)


def test_command_joint_state_accepts_valid_feedforward():
    r = _robot()
    tff = np.full(6, 1.0)  # within torque_clip
    acc = np.full(6, 2.0)  # within _MAX_CMD_ACC_RAD_S2
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6),
                           "acc": acc, "torque_ff": tff})
    assert np.allclose(r._command.torque_ff, 1.0)
    assert np.allclose(r._command.acc, 2.0)


def test_command_joint_state_uses_per_joint_velocity_limits():
    r = _robot()
    # This is an entry-validation unit test without hardware/startup. Current
    # ArmRobot lifecycle correctly blocks all public commands while STOPPED,
    # so open only the command gate used by this isolated boundary check.
    r._commands_blocked.clear()
    boundary = np.array([8.0, -8.0, 8.0, -7.0, 8.0, -8.0])
    accepted = r.command_joint_state({
        "pos": np.full(6, 0.1),
        "vel": boundary,
    })
    assert accepted is True
    np.testing.assert_array_equal(r._command.vel, boundary)

    rejected = r.command_joint_state({
        "pos": np.full(6, 0.2),
        "vel": np.array([8.0, 8.0, 8.0, 7.01, 8.0, 8.0]),
    })
    assert rejected is False
    np.testing.assert_array_equal(r._command.pos, np.full(6, 0.1))

    rejected = r.command_joint_state({
        "pos": np.full(6, 0.3),
        "vel": np.array([8.01, 8.0, 8.0, 7.0, 8.0, 8.0]),
    })
    assert rejected is False
    np.testing.assert_array_equal(r._command.pos, np.full(6, 0.1))


def test_command_joint_state_rejects_out_of_range_torque_ff():
    r = _robot()
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6)})  # baseline
    bad = np.zeros(6); bad[4] = 11.0  # J5 clip is 10 Nm
    r.command_joint_state({"pos": np.full(6, 0.1), "vel": np.zeros(6),
                           "torque_ff": bad})
    # frame rejected → previous command held (pos unchanged, torque_ff still 0)
    assert np.allclose(r._command.pos, 0.0)
    assert np.allclose(r._command.torque_ff, 0.0)


def test_command_joint_state_rejects_out_of_range_acc():
    r = _robot()
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6)})
    bad = np.zeros(6); bad[0] = 999.0
    r.command_joint_state({"pos": np.full(6, 0.1), "vel": np.zeros(6), "acc": bad})
    assert np.allclose(r._command.pos, 0.0)


def test_cross_entry_no_torque_ff_residual():
    r = _robot()
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6),
                           "torque_ff": np.full(6, 1.0)})
    assert np.allclose(r._command.torque_ff, 1.0)
    r.command_joint_pos(np.zeros(6))                 # clears torque_ff
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6)})  # no key
    assert np.allclose(r._command.torque_ff, 0.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
