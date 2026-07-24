"""Offline L0 unit tests for P0-1 feedforward clear-semantics + integrator reset.

Builds an ``ArmRobot`` with no motor chain / bus / gravity model (the command
entry points touch none of them) to pin the SOP-09 P0-1 guarantees:

  * every feedforward field not supplied by an entry point is explicitly zeroed
    (no stale acc / torque_ff / vel carry-over — devlog 2026-07-22 Q9 hazard),
  * torque_ff / acc are accepted through command_joint_state with reject-frame
    bounds, and
  * the error integrator is reset by every non-streaming entry / fault path
    (SOP-09 W4) while the streaming command_joint_state leaves it running.

Run: ``pytest tests/test_command_clear_semantics.py -v``.
"""

import numpy as np
import pytest

from a1z.robots.arm_robot import ArmRobot
from a1z.robots.integrator import IntegralConfig

_TAU_C = np.array([np.nan] * 5 + [0.13])
_TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])


def _robot(integral_config=None):
    return ArmRobot(
        motor_chain=None, bus=None, gravity_model=None, num_joints=6,
        torque_clip=_TORQUE_CLIP, joint_limits=None,
        integral_config=integral_config,
    )


def _k2_j6():
    return IntegralConfig.from_level("K2", _TAU_C, joints=[6])


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


# --- observability ----------------------------------------------------------
def test_get_joint_state_has_tau_i():
    r = _robot()
    assert "tau_i" in r.get_joint_state()
    assert np.allclose(r.get_joint_state()["tau_i"], 0.0)


def test_get_robot_info_integral_block():
    assert _robot().get_robot_info()["integral"]["level"] == "K0"
    r = _robot(_k2_j6())
    info = r.get_robot_info()["integral"]
    assert info["level"] == "K2"
    assert info["enable_mask"] == [False, False, False, False, False, True]


# --- W4 integrator reset paths ---------------------------------------------
def _wind_up(r):
    e = np.zeros(6); e[5] = np.deg2rad(0.5)
    for _ in range(50):
        r._integrator.step(e, np.zeros(6))
    r._last_tau_i = r._integrator.tau_i
    assert abs(r._integrator.tau_i[5]) > 0


def _estop_then_release(r):
    r.estop()
    r.release()


@pytest.mark.parametrize("act", [
    lambda r: r.command_joint_pos(np.zeros(6)),
    lambda r: r.estop(),
    _estop_then_release,
    lambda r: r.set_gravity_mode(True),
    lambda r: r.reset_integral(),
])
def test_non_streaming_entries_reset_integrator(act):
    r = _robot(_k2_j6())
    _wind_up(r)
    act(r)
    assert np.allclose(r._integrator.tau_i, 0.0)
    assert np.allclose(r.get_joint_state()["tau_i"], 0.0)


def test_streaming_command_does_not_reset_integrator():
    r = _robot(_k2_j6())
    _wind_up(r)
    r.command_joint_state({"pos": np.zeros(6), "vel": np.zeros(6)})
    assert abs(r._integrator.tau_i[5]) > 0  # streaming path leaves it running


def test_set_integral_config_swaps_and_resets():
    r = _robot(_k2_j6())
    _wind_up(r)
    r.set_integral_config(None)
    assert r._integrator is None
    assert r.get_robot_info()["integral"]["level"] == "K0"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
