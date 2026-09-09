"""Offline tests for ArmRobot control-loop fault classification.

Imported verbatim from the GitLab source repo (feat/circle-clip @ 598df97)
for port stage P4, with one exception marked xfail(strict=True):

- test_mixed_chain_reports_motor_whose_disable_frames_both_failed encodes
  the GitLab chain contract (disable_all raises); the target chain
  deliberately keeps its "try every motor, return bool" contract (P1
  deviation), and ``ArmRobot._disable_outputs`` translates a False return
  into an error-list entry instead.

The P5 clip-policy stage landed ``_coerce_joint_vector`` /
``_require_gain_vector`` in the command/start/move entry points, so the
former strict-xfail cases for NaN stream targets and gain validation now
run unmarked.
"""

import errno
import threading
import time

import can
import numpy as np
import pytest

from a1z.motor_drivers.motor_b_driver import MixedMotorChain
from a1z.robots.arm_robot import (
    ArmRobot,
    ControlState,
    HardSafetyFault,
    RecoverableControlFault,
    _STARTUP_FEEDBACK_PROBE_ATTEMPTS,
)


class _FakeChain:
    def __init__(self):
        self.disable_count = 0
        self.enable_count = 0
        self.feedback_seen = np.ones(6, dtype=bool)
        self.feedback_age = np.zeros(6, dtype=float)

    def disable_all(self):
        self.disable_count += 1

    def enable_all(self):
        self.enable_count += 1

    def get_feedback_health(self, now=None):
        return self.feedback_seen.copy(), self.feedback_age.copy()

    def classify_error_codes(self, codes):
        # This generic test double models a DaMiao-only chain.
        return np.asarray(codes) != 0x1

    def describe_error_code(self, joint_idx, code):
        return f"error_code=0x{int(code):X} (fake-damiao)"


class _FakeGravity:
    def compute_inverse_dynamics(self, q, vel, acc):
        return np.zeros_like(q)


class _ReadyStartupChain(_FakeChain):
    """Chain double that supplies one complete, healthy startup probe."""

    inter_cmd_gap_s = 0.0

    def __init__(self):
        super().__init__()
        self.sent = []
        self.positions = np.full(6, 0.4)
        self.velocities = np.zeros(6)
        self.error_codes = np.ones(6, dtype=int)
        self.temp_mos = np.zeros(6)
        self.temp_rotor = np.zeros(6)

    def reset_feedback_health(self):
        self.feedback_seen[:] = False
        self.feedback_age[:] = np.inf

    def send_commands(self, pos, vel, kp, kd, torque):
        self.sent.append({
            "pos": pos.copy(),
            "vel": vel.copy(),
            "kp": kp.copy(),
            "kd": kd.copy(),
            "torque": torque.copy(),
        })

    def drain_and_update(self, _bus):
        self.feedback_seen[:] = True
        self.feedback_age[:] = 0.0
        return 6

    def get_positions(self):
        return self.positions.copy()

    def get_velocities(self):
        return self.velocities.copy()

    def get_efforts(self):
        return np.zeros(6)

    def get_error_codes(self):
        return self.error_codes.copy()

    def get_temperatures(self):
        return self.temp_mos.copy(), self.temp_rotor.copy()


class _RoutingMotor:
    """Minimal motor double for MixedMotorChain feedback-routing tests."""

    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.last_feedback = None

    def parse_feedback(self, _message):
        return object()


class _PartialSendMotor:
    """Motor double used to expose MixedMotorChain's per-axis send order."""

    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.last_feedback = None
        self.fail = False
        self.positions = []

    def send_mit_command(self, pos, vel, kp, kd, torque, mode=0):
        if self.fail:
            raise can.CanOperationError(f"joint{self.motor_id} TX failed")
        self.positions.append(pos)


def _make_robot(chain=None, **robot_kwargs):
    chain = chain or _FakeChain()
    robot = ArmRobot(
        motor_chain=chain,
        bus=object(),
        gravity_model=_FakeGravity(),
        **robot_kwargs,
    )
    robot._running = True
    robot._last_feedback_t = time.monotonic()
    robot._last_successful_update_t = time.monotonic()
    robot._control_state = ControlState.RUNNING
    robot._commands_blocked.clear()
    robot._command.pos = np.arange(6, dtype=float)
    robot._command.vel = np.ones(6)
    robot._command.acc = np.ones(6)
    return robot, chain


def _enter_soft_estop(
    robot,
    fault_code="LEADER_COMMUNICATION",
    *,
    mark_complete_update=True,
):
    assert robot.estop(
        reason="test soft fault",
        fault_code=fault_code,
    )
    assert robot.get_fault_status()["state"] == "SOFT_ESTOP"
    if mark_complete_update:
        # Simulate one complete SDK cycle after the fault was latched. Release
        # separately verifies feedback health and this post-fault barrier.
        robot._last_successful_update_t = time.monotonic()


def _tx_buffer_full():
    try:
        return can.CanOperationError(
            "Transmit buffer full",
            error_code=errno.ENOBUFS,
        )
    except TypeError:
        return can.CanOperationError("Transmit buffer full")


def test_transient_can_error_retries_with_last_command_without_latching():
    robot, chain = _make_robot()

    keep_running, since = robot._handle_control_exception(
        _tx_buffer_full(),
        None,
    )

    assert keep_running
    assert since is not None
    assert chain.disable_count == 0
    assert robot.get_fault_status()["state"] == "RUNNING"
    assert robot.get_fault_status()["code"] == ""
    assert np.allclose(robot._command.vel, 0)
    assert np.allclose(robot._command.acc, 0)
    assert not robot.is_estopped


def test_confirmed_hard_fault_disables():
    robot, chain = _make_robot()

    keep_running, _ = robot._handle_control_exception(
        HardSafetyFault("motor over-temperature"),
        None,
    )

    assert not keep_running
    assert chain.disable_count == 1
    assert robot.get_fault_status()["state"] == "HARD_DISABLED"


def test_non_finite_stream_target_is_rejected_and_last_command_is_held():
    robot, _ = _make_robot()
    previous = robot._command.pos.copy()

    robot.command_joint_state({
        "pos": np.array([0.0, 0.0, np.nan, 0.0, 0.0, 0.0]),
        "vel": np.zeros(6),
    })

    assert np.array_equal(robot._command.pos, previous)
    assert robot.get_fault_status()["state"] == "RUNNING"


def test_seven_element_position_remains_compatible_without_gripper():
    robot, _ = _make_robot()

    robot.command_joint_pos(np.arange(7, dtype=float))

    np.testing.assert_array_equal(
        robot.get_command_state()["pos"],
        np.arange(6, dtype=float),
    )


def test_hold_last_command_preserves_position_and_zeros_feedforward():
    robot, _ = _make_robot()
    previous = robot._command.pos.copy()

    assert robot.hold_last_command()

    assert np.array_equal(robot._command.pos, previous)
    assert np.allclose(robot._command.vel, 0)
    assert np.allclose(robot._command.acc, 0)
    assert not robot.is_estopped


def test_latch_last_command_preserves_target_and_blocks_new_commands():
    robot, _ = _make_robot()
    previous_pos = robot._command.pos.copy()
    previous_kp = robot._command.kp.copy()
    previous_kd = robot._command.kd.copy()
    robot._command.torque_ff = np.ones(6)

    assert robot.latch_last_command(
        reason="Leader offline",
        fault_code="LEADER_COMMUNICATION",
    )

    status = robot.get_fault_status()
    assert status["state"] == "COMMAND_HOLD"
    assert status["code"] == "LEADER_COMMUNICATION"
    np.testing.assert_array_equal(robot._command.pos, previous_pos)
    np.testing.assert_array_equal(robot._command.kp, previous_kp)
    np.testing.assert_array_equal(robot._command.kd, previous_kd)
    np.testing.assert_array_equal(robot._command.vel, np.zeros(6))
    np.testing.assert_array_equal(robot._command.acc, np.zeros(6))
    np.testing.assert_array_equal(robot._command.torque_ff, np.zeros(6))

    robot.command_joint_pos(np.full(6, -1.0))
    np.testing.assert_array_equal(robot._command.pos, previous_pos)


def test_one_stale_joint_is_recoverable_communication_fault():
    robot, chain = _make_robot(stale_feedback_estop_s=0.2)
    chain.feedback_age[4] = 0.201

    with pytest.raises(RecoverableControlFault, match="joint5") as caught:
        robot._check_feedback_stale()

    assert caught.value.fault_code == "CAN_FEEDBACK_STALE"
    assert chain.disable_count == 0


def test_stale_feedback_sends_last_real_mit_target_before_faulting():
    class _UpdateChain(_FakeChain):
        inter_cmd_gap_s = 0.0

        def __init__(self):
            super().__init__()
            self.sent = []

        def drain_and_update(self, _bus):
            return 0

        def get_positions(self):
            return np.zeros(6)

        def get_velocities(self):
            return np.zeros(6)

        def get_efforts(self):
            return np.zeros(6)

        def get_error_codes(self):
            return np.ones(6, dtype=int)

        def get_temperatures(self):
            return np.zeros(6), np.zeros(6)

        def send_commands(self, pos, vel, kp, kd, torque):
            self.sent.append({
                "pos": pos.copy(),
                "vel": vel.copy(),
                "kp": kp.copy(),
                "kd": kd.copy(),
                "torque": torque.copy(),
            })

    chain = _UpdateChain()
    robot, _ = _make_robot(chain=chain, stale_feedback_estop_s=0.2)
    robot._command.pos = np.full(6, 0.25)
    robot._command.vel = np.zeros(6)
    robot._command.acc = np.zeros(6)
    robot._command.kp = np.full(6, 12.0)
    robot._command.kd = np.full(6, 0.8)
    robot._update()
    chain.sent.clear()

    class _ExplodingGravity:
        def compute_inverse_dynamics(self, *_args):
            pytest.fail("stale feedback path must not recompute dynamics")

        def compute_gravity_torque(self, *_args):
            pytest.fail("stale feedback path must not recompute gravity")

    robot._gravity_model = _ExplodingGravity()
    chain.feedback_age[5] = 0.201
    robot._command.pos = np.full(6, 0.9)  # newer, not actually sent
    complete_update_t = robot._last_successful_update_t

    with pytest.raises(RecoverableControlFault, match="joint6"):
        robot._update()

    assert len(chain.sent) == 1
    np.testing.assert_array_equal(chain.sent[0]["pos"], np.full(6, 0.25))
    np.testing.assert_array_equal(chain.sent[0]["vel"], np.zeros(6))
    np.testing.assert_array_equal(chain.sent[0]["kp"], np.full(6, 12.0))
    np.testing.assert_array_equal(chain.sent[0]["kd"], np.full(6, 0.8))
    np.testing.assert_array_equal(chain.sent[0]["torque"], np.zeros(6))
    assert time.monotonic() - robot._last_successful_arm_tx_t < 0.1
    assert robot._last_successful_update_t == complete_update_t


def test_latch_waits_for_inflight_send_and_holds_the_command_that_won():
    send_started = threading.Event()
    allow_send = threading.Event()

    class _BarrierChain(_FakeChain):
        inter_cmd_gap_s = 0.0

        def drain_and_update(self, _bus):
            return 0

        def get_positions(self):
            return np.zeros(6)

        def get_velocities(self):
            return np.zeros(6)

        def get_efforts(self):
            return np.zeros(6)

        def get_error_codes(self):
            return np.ones(6, dtype=int)

        def get_temperatures(self):
            return np.zeros(6), np.zeros(6)

        def send_commands(self, *_args, **_kwargs):
            send_started.set()
            assert allow_send.wait(timeout=1.0)

    robot, _ = _make_robot(chain=_BarrierChain())
    robot._command.pos = np.full(6, 0.9)
    robot._command.vel = np.zeros(6)
    robot._command.acc = np.zeros(6)

    update_thread = threading.Thread(target=robot._update)
    update_thread.start()
    assert send_started.wait(timeout=1.0)

    latch_result = []
    latch_thread = threading.Thread(
        target=lambda: latch_result.append(
            robot.latch_last_command(
                reason="Leader offline",
                fault_code="LEADER_COMMUNICATION",
            )
        )
    )
    latch_thread.start()
    latch_thread.join(timeout=0.02)
    assert latch_thread.is_alive()

    allow_send.set()
    update_thread.join(timeout=1.0)
    latch_thread.join(timeout=1.0)

    assert latch_result == [True]
    assert not update_thread.is_alive()
    assert not latch_thread.is_alive()
    np.testing.assert_array_equal(
        robot.get_command_state()["pos"],
        np.full(6, 0.9),
    )
    assert robot.get_fault_status()["state"] == "COMMAND_HOLD"


def test_fault_generation_prevents_pre_fault_cycle_late_commit():
    chain = _ReadyStartupChain()
    robot, _ = _make_robot(chain=chain)
    cycle_reached_commit_window = threading.Event()
    allow_cycle_to_commit = threading.Event()

    def gated_gripper_step():
        cycle_reached_commit_window.set()
        assert allow_cycle_to_commit.wait(timeout=1.0)

    robot._step_integrated_gripper = gated_gripper_step
    prior_complete_t = robot._last_successful_update_t

    update_thread = threading.Thread(target=robot._update)
    update_thread.start()
    assert cycle_reached_commit_window.wait(timeout=1.0)

    assert robot.latch_last_command(
        reason="Leader offline",
        fault_code="LEADER_COMMUNICATION",
    )
    allow_cycle_to_commit.set()
    update_thread.join(timeout=1.0)

    assert not update_thread.is_alive()
    assert robot.get_fault_status()["state"] == "COMMAND_HOLD"
    assert robot._last_successful_update_t == prior_complete_t


def test_dynamics_failure_resends_cached_gravity_hold_frame():
    class _UpdateChain(_FakeChain):
        inter_cmd_gap_s = 0.0

        def __init__(self):
            super().__init__()
            self.sent = []

        def drain_and_update(self, _bus):
            return 0

        def get_positions(self):
            return np.zeros(6)

        def get_velocities(self):
            return np.zeros(6)

        def get_efforts(self):
            return np.zeros(6)

        def get_error_codes(self):
            return np.ones(6, dtype=int)

        def get_temperatures(self):
            return np.zeros(6), np.zeros(6)

        def send_commands(self, pos, vel, kp, kd, torque):
            self.sent.append({
                "pos": pos.copy(),
                "vel": vel.copy(),
                "kp": kp.copy(),
                "kd": kd.copy(),
                "torque": torque.copy(),
            })

    class _FailingGravity:
        def compute_inverse_dynamics(self, *_args):
            raise RuntimeError("model unavailable")

    chain = _UpdateChain()
    robot, _ = _make_robot(chain=chain)
    robot._command.pos = np.full(6, 0.3)
    robot._command.vel = np.zeros(6)
    robot._command.acc = np.zeros(6)
    robot._update()
    chain.sent.clear()
    robot._gravity_model = _FailingGravity()
    robot._command.pos = np.full(6, 0.8)

    with pytest.raises(RecoverableControlFault) as caught:
        robot._update()
    keep_running, since = robot._handle_control_exception(
        caught.value,
        None,
    )

    assert keep_running
    assert since is not None
    assert len(chain.sent) == 1
    np.testing.assert_array_equal(chain.sent[0]["pos"], np.full(6, 0.3))
    np.testing.assert_array_equal(chain.sent[0]["vel"], np.zeros(6))
    np.testing.assert_array_equal(chain.sent[0]["torque"], np.zeros(6))


def test_partial_six_axis_send_does_not_commit_new_safe_cache():
    motors = [_PartialSendMotor(index + 1) for index in range(6)]
    chain = MixedMotorChain(
        motor_a_list=motors,
        motor_b_list=[],
        motor_a_joint_indices=list(range(6)),
        motor_b_joint_indices=[],
    )
    robot, _ = _make_robot(chain=chain)
    robot._command.pos = np.full(6, 0.2)
    robot._command.vel = np.zeros(6)
    robot._command.acc = np.zeros(6)

    with robot._send_lock:
        robot._send_command_and_cache_hold_locked(
            robot._snapshot_command(),
            np.zeros(6),
        )

    motors[2].fail = True
    robot._command.pos = np.full(6, 0.9)
    with robot._send_lock:
        with pytest.raises(can.CanOperationError, match="joint3"):
            robot._send_command_and_cache_hold_locked(
                robot._snapshot_command(),
                np.zeros(6),
            )

    np.testing.assert_array_equal(
        robot._last_sent_command.pos,
        np.full(6, 0.2),
    )
    np.testing.assert_array_equal(
        robot._last_safe_hold_frame.pos,
        np.full(6, 0.2),
    )

    motors[2].fail = False
    for motor in motors:
        motor.positions.clear()
    robot._resend_last_safe_hold()
    assert [motor.positions[-1] for motor in motors] == [0.2] * 6


def test_cached_hold_removes_motion_feedforward_but_keeps_gravity():
    class _DistinctGravity:
        def compute_inverse_dynamics(self, q, vel, acc):
            return np.full(6, 3.0)

        def compute_gravity_torque(self, q):
            return np.full(6, 1.0)

    chain = _ReadyStartupChain()
    joint_sign = np.array([1, -1, 1, -1, 1, -1], dtype=float)
    torque_scale = np.array([1, 2, 3, 4, 5, 6], dtype=float)
    robot = ArmRobot(
        motor_chain=chain,
        bus=object(),
        gravity_model=_DistinctGravity(),
        joint_sign=joint_sign,
        gravity_torque_scale=torque_scale,
        torque_clip=np.full(6, 100.0),
        max_gravity_torque=np.full(6, 10.0),
    )
    robot._running = True
    robot._control_state = ControlState.RUNNING
    robot._commands_blocked.clear()
    robot._control_started_t = time.monotonic()
    robot._command.pos = np.full(6, 0.25)
    robot._command.vel = np.full(6, 0.3)
    robot._command.acc = np.full(6, 0.4)
    robot._command.kp = np.full(6, 12.0)
    robot._command.kd = np.full(6, 0.8)
    robot._command.torque_ff = np.full(6, 0.5)

    robot._update()

    dynamic_torque = (0.5 + 3.0 * torque_scale) * joint_sign
    gravity_hold_torque = torque_scale * joint_sign
    np.testing.assert_allclose(chain.sent[-1]["torque"], dynamic_torque)
    np.testing.assert_array_equal(
        robot._last_safe_hold_frame.vel,
        np.zeros(6),
    )
    np.testing.assert_allclose(
        robot._last_safe_hold_frame.torque,
        gravity_hold_torque,
    )
    assert not np.allclose(dynamic_torque, gravity_hold_torque)


def test_fresh_disabled_motor_report_is_confirmed_hard_fault():
    robot, _ = _make_robot()
    robot._state.error_codes[:] = 0x1
    robot._state.error_codes[2] = 0x0

    with pytest.raises(HardSafetyFault, match="joint3"):
        robot._check_motor_errors()


def test_failed_hard_disable_transmission_is_reported_unconfirmed():
    class _FailDisableChain(_FakeChain):
        def disable_all(self):
            self.disable_count += 1
            raise RuntimeError("CAN interface down")

    robot, chain = _make_robot(chain=_FailDisableChain())

    keep_running, _ = robot._handle_control_exception(
        HardSafetyFault("motor over-temperature"),
        None,
    )

    assert not keep_running
    assert chain.disable_count == 1
    status = robot.get_fault_status()
    assert status["state"] == "HARD_DISABLE_UNCONFIRMED"
    assert status["disable_transmission_confirmed"] is False
    assert "physical emergency stop" in status["reason"]


def test_hard_fault_cannot_be_cleared_by_calling_start_again():
    robot, chain = _make_robot()
    robot._handle_control_exception(
        HardSafetyFault("motor over-temperature"),
        None,
    )
    chain.enable_count = 0

    with pytest.raises(RuntimeError, match="requires STOPPED"):
        robot.start()

    assert chain.enable_count == 0
    assert robot.get_fault_status()["state"] == "HARD_DISABLED"


@pytest.mark.xfail(
    strict=True,
    reason="target MixedMotorChain.disable_all keeps the GitHub bool "
    "contract (P1 deviation); GitLab raise semantics not ported",
)
def test_mixed_chain_reports_motor_whose_disable_frames_both_failed():
    class _DisableMotor:
        def __init__(self, motor_id, fail=False):
            self.motor_id = motor_id
            self.fail = fail
            self.calls = 0

        def disable(self):
            self.calls += 1
            if self.fail:
                raise can.CanOperationError("bus down")

    good = _DisableMotor(1)
    failed = _DisableMotor(2, fail=True)
    chain = MixedMotorChain(
        motor_a_list=[good, failed],
        motor_b_list=[],
        motor_a_joint_indices=[0, 1],
        motor_b_joint_indices=[],
    )

    with pytest.raises(RuntimeError, match=r"MotorA\[2\]"):
        chain.disable_all()

    assert good.calls == 2
    assert failed.calls == 2


def test_persistent_transient_escalates_to_latched_fault_hold(monkeypatch):
    robot, chain = _make_robot()
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.monotonic",
        lambda: 10.25,
    )

    keep_running, _ = robot._handle_control_exception(
        _tx_buffer_full(),
        10.0,
    )

    assert keep_running
    assert chain.disable_count == 0
    assert robot.get_fault_status()["state"] == "FAULT_HOLD"
    assert robot.get_fault_status()["code"] == "CAN_TX_TRANSIENT_PERSISTENT"
    assert robot.is_estopped


def test_fault_wins_race_with_release_and_keeps_commands_blocked():
    robot, _ = _make_robot()
    robot._control_state = ControlState.SOFT_ESTOP
    robot._fault_code = "LEADER_COMMUNICATION"
    robot._commands_blocked.set()

    release_entered_state = threading.Event()
    allow_release_to_finish = threading.Event()

    class _GatedStateLock:
        def __enter__(self):
            if not release_entered_state.is_set():
                release_entered_state.set()
                assert allow_release_to_finish.wait(timeout=1.0)
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

    robot._state_lock = _GatedStateLock()
    release_result = []

    release_thread = threading.Thread(
        target=lambda: release_result.append(
            robot.release(expected_fault_code="LEADER_COMMUNICATION")
        )
    )
    release_thread.start()
    assert release_entered_state.wait(timeout=1.0)

    fault_thread = threading.Thread(
        target=lambda: robot.hold_position(
            reason="new SDK fault",
            fault_code="CAN_TX_TRANSIENT",
        )
    )
    fault_thread.start()
    allow_release_to_finish.set()
    release_thread.join(timeout=1.0)
    fault_thread.join(timeout=1.0)

    assert release_result == [True]
    assert robot.get_fault_status()["state"] == "FAULT_HOLD"
    assert robot.get_fault_status()["code"] == "CAN_TX_TRANSIENT"
    assert robot._commands_blocked.is_set()


@pytest.mark.parametrize("joint_index", range(6), ids=lambda index: f"joint-{index + 1}")
def test_release_rejects_any_never_seen_joint(joint_index):
    robot, chain = _make_robot()
    _enter_soft_estop(robot)
    chain.feedback_seen[joint_index] = False
    chain.feedback_age[joint_index] = np.inf

    assert not robot.release(expected_fault_code="LEADER_COMMUNICATION")

    assert robot.get_fault_status()["state"] == "SOFT_ESTOP"
    assert robot.is_estopped


@pytest.mark.parametrize("joint_index", range(6), ids=lambda index: f"joint-{index + 1}")
def test_release_rejects_any_stale_joint_when_other_five_are_fresh(joint_index):
    robot, chain = _make_robot(stale_feedback_estop_s=0.2)
    _enter_soft_estop(robot)
    chain.feedback_age[:] = 0.0
    chain.feedback_age[joint_index] = 0.201

    assert not robot.release(expected_fault_code="LEADER_COMMUNICATION")

    assert robot.get_fault_status()["state"] == "SOFT_ESTOP"
    assert robot.is_estopped


def test_release_accepts_only_when_all_six_joints_are_seen_and_fresh():
    robot, chain = _make_robot(stale_feedback_estop_s=0.2)
    _enter_soft_estop(robot)
    chain.feedback_seen[:] = True
    chain.feedback_age[:] = 0.199

    assert robot.release(expected_fault_code="LEADER_COMMUNICATION")

    assert robot.get_fault_status()["state"] == "RUNNING"
    assert not robot.is_estopped


def test_release_rejects_when_safe_control_update_has_not_succeeded_recently():
    robot, chain = _make_robot(stale_feedback_estop_s=0.2)
    _enter_soft_estop(robot)
    chain.feedback_seen[:] = True
    chain.feedback_age[:] = 0.0
    robot._last_successful_update_t = time.monotonic() - 0.201

    assert not robot.release(expected_fault_code="LEADER_COMMUNICATION")

    assert robot.get_fault_status()["state"] == "SOFT_ESTOP"
    assert robot.is_estopped


def test_release_requires_a_complete_cycle_after_the_active_fault():
    robot, chain = _make_robot(stale_feedback_estop_s=0.2)
    _enter_soft_estop(robot, mark_complete_update=False)
    chain.feedback_seen[:] = True
    chain.feedback_age[:] = 0.0

    assert not robot.release(expected_fault_code="LEADER_COMMUNICATION")

    assert robot.get_fault_status()["state"] == "SOFT_ESTOP"
    assert robot.is_estopped


def test_gripper_and_unknown_frames_cannot_replace_missing_arm_feedback():
    arm_motors = [_RoutingMotor(motor_id) for motor_id in range(1, 7)]
    chain = MixedMotorChain(
        motor_a_list=arm_motors,
        motor_b_list=[],
        motor_a_joint_indices=list(range(6)),
        motor_b_joint_indices=[],
    )
    gripper = _RoutingMotor(0x07)
    chain.register_external_motor(gripper)

    # Five real joints are fresh; neither the registered external gripper nor
    # arbitrary traffic may fill the missing joint-6 health slot.
    for motor_id in range(1, 6):
        chain._dispatch_feedback(can.Message(arbitration_id=motor_id))
    chain._dispatch_feedback(can.Message(arbitration_id=gripper.motor_id))
    chain._dispatch_feedback(can.Message(arbitration_id=0x55))

    seen, age = chain.get_feedback_health()
    assert seen.tolist() == [True, True, True, True, True, False]
    assert np.all(np.isfinite(age[:5]))
    assert np.isinf(age[5])

    robot, _ = _make_robot(chain=chain, stale_feedback_estop_s=60.0)
    _enter_soft_estop(robot)
    assert not robot.release(expected_fault_code="LEADER_COMMUNICATION")
    assert robot.is_estopped


@pytest.mark.parametrize(
    "missing_joint",
    range(6),
    ids=lambda index: f"joint-{index + 1}",
)
def test_startup_refuses_position_gains_without_new_feedback_from_every_joint(
    monkeypatch,
    missing_joint,
):
    class _StartupChain(_FakeChain):
        inter_cmd_gap_s = 0.0

        def __init__(self):
            super().__init__()
            self.reset_count = 0
            self.sent = []
            self.positions = np.full(6, 7.0)

        def reset_feedback_health(self):
            self.reset_count += 1
            self.feedback_seen[:] = False
            self.feedback_age[:] = np.inf
            self.positions[:] = 0.0

        def send_commands(self, pos, vel, kp, kd, torque):
            self.sent.append({
                "pos": pos.copy(),
                "vel": vel.copy(),
                "kp": kp.copy(),
                "kd": kd.copy(),
                "torque": torque.copy(),
            })

        def drain_and_update(self, _bus):
            self.feedback_seen[:] = True
            self.feedback_seen[missing_joint] = False
            self.feedback_age[:] = 0.0
            self.feedback_age[missing_joint] = np.inf
            self.positions[:] = 0.4
            self.positions[missing_joint] = 0.0
            return 5

        def get_positions(self):
            return self.positions.copy()

        def get_velocities(self):
            return np.zeros(6)

        def get_efforts(self):
            return np.zeros(6)

        def get_error_codes(self):
            return np.ones(6, dtype=int)

        def get_temperatures(self):
            return np.zeros(6), np.zeros(6)

    chain = _StartupChain()
    robot, _ = _make_robot(chain=chain)
    robot._running = False
    robot._control_state = ControlState.STOPPED
    robot._last_sent_command.pos = np.full(6, 9.0)
    robot._has_sent_command = True
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.sleep",
        lambda _seconds: None,
    )

    with pytest.raises(RuntimeError, match="Startup probe"):
        robot.start()

    assert chain.reset_count == 1
    assert chain.enable_count == 1
    assert chain.disable_count == 1
    assert len(chain.sent) == _STARTUP_FEEDBACK_PROBE_ATTEMPTS
    for command in chain.sent:
        np.testing.assert_array_equal(command["kp"], np.zeros(6))
    assert not robot._has_sent_command
    assert robot._thread is None
    assert not robot.is_running
    assert robot.get_fault_status()["state"] == "STOPPED"


def test_startup_retries_zero_gain_probe_until_missing_joint_recovers(
    monkeypatch,
):
    class _TransientMissingChain(_ReadyStartupChain):
        def __init__(self):
            super().__init__()
            self.drain_count = 0

        def drain_and_update(self, _bus):
            self.drain_count += 1
            self.feedback_seen[:] = True
            self.feedback_age[:] = 0.0
            if self.drain_count == 1:
                self.feedback_seen[2] = False
                self.feedback_age[2] = np.inf
                return 5
            return 6

    class _NoopThread:
        def __init__(self, *args, **kwargs):
            self.started = False

        def start(self):
            self.started = True

        def is_alive(self):
            return self.started

        def join(self, timeout=None):
            self.started = False

    chain = _TransientMissingChain()
    robot, _ = _make_robot(chain=chain, zero_gravity_mode=False)
    robot._running = False
    robot._control_state = ControlState.STOPPED
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.sleep",
        lambda _seconds: None,
    )
    monkeypatch.setattr(
        "a1z.robots.arm_robot.threading.Thread",
        _NoopThread,
    )

    robot.start()

    assert chain.drain_count == 2
    assert len(chain.sent) == 3
    for command in chain.sent[:2]:
        np.testing.assert_array_equal(command["kp"], np.zeros(6))
    np.testing.assert_array_equal(chain.sent[2]["kp"], robot._default_kp)
    assert robot.get_fault_status()["state"] == "RUNNING"


def test_startup_queues_current_pose_only_after_complete_new_feedback(
    monkeypatch,
):
    class _CompleteStartupChain(_FakeChain):
        inter_cmd_gap_s = 0.0

        def __init__(self):
            super().__init__()
            self.sent = []
            self.positions = np.full(6, 7.0)

        def reset_feedback_health(self):
            self.feedback_seen[:] = False
            self.feedback_age[:] = np.inf
            self.positions[:] = 0.0

        def send_commands(self, pos, vel, kp, kd, torque):
            self.sent.append({
                "pos": pos.copy(),
                "vel": vel.copy(),
                "kp": kp.copy(),
                "kd": kd.copy(),
                "torque": torque.copy(),
            })

        def drain_and_update(self, _bus):
            self.feedback_seen[:] = True
            self.feedback_age[:] = 0.0
            self.positions[:] = 0.4
            return 6

        def get_positions(self):
            return self.positions.copy()

        def get_velocities(self):
            return np.zeros(6)

        def get_efforts(self):
            return np.zeros(6)

        def get_error_codes(self):
            return np.ones(6, dtype=int)

        def get_temperatures(self):
            return np.zeros(6), np.zeros(6)

    class _NoopThread:
        def __init__(self, *args, **kwargs):
            self.started = False

        def start(self):
            self.started = True

        def is_alive(self):
            return self.started

        def join(self, timeout=None):
            self.started = False

    chain = _CompleteStartupChain()
    robot, _ = _make_robot(
        chain=chain,
        zero_gravity_mode=False,
    )
    robot._running = False
    robot._control_state = ControlState.STOPPED
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.sleep",
        lambda _seconds: None,
    )
    monkeypatch.setattr(
        "a1z.robots.arm_robot.threading.Thread",
        _NoopThread,
    )

    robot.start()

    assert chain.enable_count == 1
    assert chain.disable_count == 0
    assert len(chain.sent) == 2
    np.testing.assert_array_equal(chain.sent[0]["kp"], np.zeros(6))
    np.testing.assert_array_equal(chain.sent[1]["pos"], np.full(6, 0.4))
    np.testing.assert_array_equal(chain.sent[1]["kp"], robot._default_kp)
    assert robot._has_sent_command
    assert robot._last_safe_hold_frame is not None
    assert robot._last_successful_arm_tx_t > 0
    assert robot._last_successful_update_t == 0
    assert robot.get_fault_status()["state"] == "RUNNING"


def test_startup_thread_failure_rolls_back_enabled_motors(monkeypatch):
    class _StartFailsThread:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            raise RuntimeError("thread launch failed")

        def is_alive(self):
            return False

    chain = _ReadyStartupChain()
    robot, _ = _make_robot(chain=chain, zero_gravity_mode=False)
    robot._running = False
    robot._control_state = ControlState.STOPPED
    robot._commands_blocked.set()
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.sleep",
        lambda _seconds: None,
    )
    monkeypatch.setattr(
        "a1z.robots.arm_robot.threading.Thread",
        _StartFailsThread,
    )

    with pytest.raises(RuntimeError, match="thread launch failed"):
        robot.start()

    assert chain.enable_count == 1
    assert chain.disable_count == 1
    assert not robot.is_running
    assert robot.is_estopped
    assert robot.get_fault_status()["state"] == "STOPPED"


def test_startup_thread_that_exits_immediately_is_rolled_back(monkeypatch):
    class _AlreadyExitedThread:
        def __init__(self, *args, **kwargs):
            self.started = False

        def start(self):
            self.started = True

        def is_alive(self):
            return False

    chain = _ReadyStartupChain()
    robot, _ = _make_robot(chain=chain, zero_gravity_mode=False)
    robot._running = False
    robot._control_state = ControlState.STOPPED
    robot._commands_blocked.set()
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.sleep",
        lambda _seconds: None,
    )
    monkeypatch.setattr(
        "a1z.robots.arm_robot.threading.Thread",
        _AlreadyExitedThread,
    )

    with pytest.raises(RuntimeError, match="control thread faulted"):
        robot.start()

    assert chain.enable_count == 1
    assert chain.disable_count == 1
    assert not robot.is_running
    assert robot.get_fault_status()["state"] == "STOPPED"


def test_stop_timeout_is_non_restartable(monkeypatch):
    class _StuckThread:
        def __init__(self):
            self.join_count = 0

        def is_alive(self):
            return True

        def join(self, timeout=None):
            self.join_count += 1
            assert timeout == 2.0

    robot, chain = _make_robot()
    stuck_thread = _StuckThread()
    robot._thread = stuck_thread

    robot.stop()

    status = robot.get_fault_status()
    assert stuck_thread.join_count == 1
    assert chain.disable_count == 1
    assert robot.is_running
    assert robot.is_estopped
    assert status["state"] == "HARD_DISABLE_UNCONFIRMED"
    assert status["code"] == "CONTROL_THREAD_STOP_TIMEOUT"
    assert status["restart_allowed"] is False

    chain.enable_count = 0
    with pytest.raises(RuntimeError, match="requires STOPPED"):
        robot.start()
    assert chain.enable_count == 0


def test_late_control_thread_cleanup_cannot_clear_timeout_latch():
    robot, chain = _make_robot()
    robot._restart_forbidden.set()
    robot._set_fault_state(
        ControlState.HARD_DISABLE_UNCONFIRMED,
        "CONTROL_THREAD_STOP_TIMEOUT",
        "control thread timed out",
    )

    state = robot._finalize_control_thread_shutdown([])

    assert chain.disable_count == 0
    assert state == ControlState.HARD_DISABLED
    assert not robot.is_running
    assert robot.get_fault_status()["code"] == "CONTROL_THREAD_STOP_TIMEOUT"
    with pytest.raises(RuntimeError, match="requires STOPPED"):
        robot.start()


def test_startup_rejects_fresh_motor_fault_before_position_gain(monkeypatch):
    chain = _ReadyStartupChain()
    chain.error_codes[2] = 0x0
    robot, _ = _make_robot(chain=chain, zero_gravity_mode=False)
    robot._running = False
    robot._control_state = ControlState.STOPPED
    robot._commands_blocked.set()
    monkeypatch.setattr(
        "a1z.robots.arm_robot.time.sleep",
        lambda _seconds: None,
    )

    with pytest.raises(HardSafetyFault, match="joint3"):
        robot.start()

    assert chain.enable_count == 1
    assert chain.disable_count == 1
    assert len(chain.sent) == 1
    np.testing.assert_array_equal(chain.sent[0]["kp"], np.zeros(6))
    assert robot.get_fault_status()["state"] == "STOPPED"


_INVALID_GAINS = [
    pytest.param("kp", np.ones(5), id="kp-wrong-size"),
    pytest.param("kp", np.full(6, np.nan), id="kp-nan"),
    pytest.param("kp", np.full(6, -0.1), id="kp-negative"),
    pytest.param("kp", np.full(6, 200.1), id="kp-above-max"),
    pytest.param("kd", np.ones(5), id="kd-wrong-size"),
    pytest.param("kd", np.full(6, np.inf), id="kd-inf"),
    pytest.param("kd", np.full(6, -0.1), id="kd-negative"),
    pytest.param("kd", np.full(6, 5.1), id="kd-above-max"),
]


@pytest.mark.parametrize("gain_name,bad_gain", _INVALID_GAINS)
def test_start_rejects_invalid_gains_before_hardware_or_control_thread(
    monkeypatch,
    gain_name,
    bad_gain,
):
    robot, chain = _make_robot()
    robot._running = False
    robot._control_state = ControlState.STOPPED

    def fail_if_enabled():
        chain.enable_count += 1
        pytest.fail("invalid startup gains reached motor enable")

    monkeypatch.setattr(chain, "enable_all", fail_if_enabled)

    with pytest.raises(ValueError):
        robot.start(**{f"initial_{gain_name}": bad_gain})

    assert chain.enable_count == 0
    assert robot._thread is None
    assert not robot.is_running


@pytest.mark.parametrize("gain_name,bad_gain", _INVALID_GAINS)
def test_move_joints_rejects_invalid_gains_before_command_mutation(
    gain_name,
    bad_gain,
):
    robot, _ = _make_robot()
    robot._command.pos = np.zeros(6)
    expected = {
        name: getattr(robot._command, name).copy()
        for name in ("pos", "vel", "acc", "kp", "kd", "torque_ff")
    }

    # A no-op move must still validate gains instead of bypassing validation
    # through the "already at target" early return.
    with pytest.raises(ValueError):
        robot.move_joints(
            np.zeros(6),
            **{gain_name: bad_gain},
        )

    for name, value in expected.items():
        np.testing.assert_array_equal(getattr(robot._command, name), value)


def test_get_command_state_returns_defensive_copies():
    robot, _ = _make_robot()
    expected = {
        "pos": np.arange(6, dtype=float),
        "vel": np.arange(6, dtype=float) + 10.0,
        "acc": np.arange(6, dtype=float) + 20.0,
        "kp": np.arange(6, dtype=float) + 30.0,
        "kd": np.arange(6, dtype=float) + 40.0,
        "torque_ff": np.arange(6, dtype=float) + 50.0,
    }
    with robot._command_lock:
        for name, value in expected.items():
            setattr(robot._command, name, value.copy())

    first = robot.get_command_state()
    assert set(first) == set(expected)
    for name, value in expected.items():
        np.testing.assert_array_equal(first[name], value)
        first[name][:] = -999.0

    second = robot.get_command_state()
    for name, value in expected.items():
        np.testing.assert_array_equal(second[name], value)
        assert second[name] is not first[name]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
