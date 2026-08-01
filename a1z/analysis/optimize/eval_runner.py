"""Persistent-robot trial evaluator for the BO loop (SOP-11 §3.2, §10.3).

``OptimizeRunner`` wraps ``JointUnitTestRunner`` with a context-manager
lifecycle and exposes ``eval_trial(theta, joint1)`` that:

1. injects (kp, kd) from the reparameterised search point,
2. runs L0 fast-evaluation (triangle + step),
3. collects v2 metrics,
4. computes J_joint cost,
5. returns a ``TrialResult`` with cost, breakdown, watchdog verdict and raw data.

The robot stays alive across trials — only gains change between calls.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from a1z.analysis.optimize.cost_spec import (
    DEFAULT_KD,
    DEFAULT_KP,
    I_HAT,
    L0,
    PENALTY_COST,
    compute_joint_cost,
)
from a1z.analysis.optimize.search_space import theta_to_gains_6, to_gains
from a1z.analysis.optimize.watchdog import (
    TickWatchdog,
    TrialChecker,
    WatchdogViolation,
)
from a1z.analysis.runner import JointUnitTestRunner, _ZERO6
from a1z.analysis.signals import WaveTrajectory


DEG = 180.0 / math.pi


@dataclass
class TrialResult:
    cost: float
    breakdown: Dict[str, float]
    watchdog_verdict: str = "ok"
    tick_wd_reason: str = ""
    trial_checker_reasons: List[str] = field(default_factory=list)
    temp_mos: Optional[np.ndarray] = None
    temp_rotor: Optional[np.ndarray] = None
    raw: Dict = field(default_factory=dict)
    duration_s: float = 0.0


class RobotHardFaultError(RuntimeError):
    """Robot has entered HARD_DISABLED and cannot be released.

    The BO session must abort and the hardware needs power-cycle / restart.
    """


class OptimizeRunner:
    """Context-managed persistent robot for BO trials (SOP-11 §10.3)."""

    def __init__(
        self,
        can_channel: str,
        *,
        vel_ff: bool = True,
        sample_hz: int = 100,
        transit_speed_deg_s: float = 15.0,
        integral_level: str = "K0",
        integral_joints: Optional[list] = None,
        integral_overrides: Optional[dict] = None,
        inter_cmd_gap_us: Optional[float] = None,
    ):
        self._runner = JointUnitTestRunner(
            can_channel,
            amp_deg=L0.tri_amp_deg,
            period=L0.tri_period,
            cycles=L0.tri_cycles,
            hold_pre=L0.hold_pre,
            hold_post=L0.hold_post,
            waves=("triangle",),
            vel_ff=vel_ff,
            sample_hz=sample_hz,
            transit_speed_deg_s=transit_speed_deg_s,
            integral_level=integral_level,
            integral_joints=integral_joints,
            integral_overrides=integral_overrides,
            inter_cmd_gap_us=inter_cmd_gap_us,
        )
        self._step_runner = JointUnitTestRunner(
            can_channel,
            amp_deg=L0.step_amp_deg,
            period=L0.step_period,
            cycles=L0.step_cycles,
            hold_pre=L0.hold_pre,
            hold_post=L0.hold_post,
            waves=("square",),
            vel_ff=False,
            sample_hz=sample_hz,
            transit_speed_deg_s=transit_speed_deg_s,
            integral_level=integral_level,
            integral_joints=integral_joints,
            integral_overrides=integral_overrides,
            inter_cmd_gap_us=inter_cmd_gap_us,
        )
        self._trial_checker = TrialChecker()

    def __enter__(self):
        try:
            self._runner.start()
        except Exception:
            # start() may fail after opening the CAN bus (e.g. incomplete
            # startup feedback). Shut the bus down so the process does not
            # leak "SocketcanBus was not properly shut down" and leave the
            # interface in a bad state for the next attempt.
            try:
                self._runner.shutdown()
            finally:
                raise
        # The step runner shares the same robot instance (same CAN bus) —
        # we just reuse the already-started robot.
        self._step_runner.robot = self._runner.robot
        self._step_runner._started = True
        return self

    def __exit__(self, *exc):
        if self._runner._started:
            try:
                self._runner.kp = None
                self._runner.kd = None
                self._runner.return_to_zero()
            finally:
                self._runner.shutdown()

    @property
    def robot(self):
        return self._runner.robot

    def _robot_state(self) -> str:
        """Return the SDK control-state string, or UNKNOWN on error."""
        try:
            return str(self.robot.get_fault_status().get("state", "UNKNOWN"))
        except Exception:
            return "UNKNOWN"

    def _is_hard_disabled(self) -> bool:
        return self._robot_state() == "HARD_DISABLED"

    def _maybe_release_estop(self) -> Optional[str]:
        """If the robot is in a recoverable fault, try to release it.

        Returns ``None`` on success, or a short reason string on failure.
        """
        try:
            fs = self.robot.get_fault_status()
        except Exception as exc:
            return f"fault_status_error:{exc}"
        state = fs.get("state")
        if state == "RUNNING":
            return None
        if state == "HARD_DISABLED":
            return f"HARD_DISABLED:{fs.get('code')}:{fs.get('reason')}"
        if state not in ("SOFT_ESTOP", "FAULT_HOLD", "COMMAND_HOLD"):
            return f"unrecoverable_state:{state}"
        code = fs.get("code") or None
        if code == "":
            code = None
        try:
            released = self.robot.release(expected_fault_code=code)
        except Exception as exc:
            return f"release_exception:{exc}"
        if not released:
            # Release may have failed because the state became HARD_DISABLED.
            if self._is_hard_disabled():
                fs2 = self.robot.get_fault_status()
                return f"HARD_DISABLED:{fs2.get('code')}:{fs2.get('reason')}"
            return f"release_failed:{fs.get('code')}"
        return None

    def eval_trial(
        self,
        theta: Dict[str, float],
        joint1: int,
        *,
        tick_watchdog: Optional[TickWatchdog] = None,
    ) -> TrialResult:
        """Run one L0 fast-evaluation and return cost + diagnostics."""
        t_start = time.monotonic()
        j = joint1 - 1

        # --- Pre-trial robot-state recovery ---
        if self._is_hard_disabled():
            fs = self.robot.get_fault_status()
            raise RobotHardFaultError(
                f"robot HARD_DISABLED ({fs.get('code')}): {fs.get('reason')}"
            )
        release_fail = self._maybe_release_estop()
        if release_fail is not None:
            if release_fail.startswith("HARD_DISABLED"):
                raise RobotHardFaultError(f"robot {release_fail}")
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"fault_hold:{release_fail}",
                duration_s=time.monotonic() - t_start,
            )

        kp_j = theta["kp"]
        kp_6, kd_6 = theta_to_gains_6(
            j, kp_j, theta.get("zeta_hat"), kd_j=theta.get("kd"))

        self._runner.kp = kp_6
        self._runner.kd = kd_6
        self._step_runner.kp = kp_6
        self._step_runner.kd = kd_6

        raw: Dict = {}
        tick_wd_tripped = False
        tick_wd_reason = ""

        # --- Triangle leg (vel-ff) ---
        if tick_watchdog is not None:
            tick_watchdog.reset()

        tri_traj = WaveTrajectory(
            _ZERO6, j, "triangle",
            amp_deg=L0.tri_amp_deg,
            period=L0.tri_period,
            cycles=L0.tri_cycles,
            hold_pre=L0.hold_pre,
            hold_post=L0.hold_post,
        )
        q_start = tri_traj.q_start

        try:
            self._runner.transit_to(q_start)
        except Exception as exc:
            if self._is_hard_disabled():
                fs = self.robot.get_fault_status()
                raise RobotHardFaultError(
                    f"robot HARD_DISABLED during transit ({fs.get('code')}): {fs.get('reason')}"
                )
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"transit_fail:{exc}",
                duration_s=time.monotonic() - t_start,
            )

        vel_fn = tri_traj.sample_vel if self._runner.vel_ff else None
        t_tri, ref_tri, resp_tri, eff_tri = self._runner.stream(
            tri_traj.sample, tri_traj.duration,
            kp_6, kd_6, vel_fn=vel_fn,
            watchdog=tick_watchdog,
        )
        raw["triangle"] = {"t": t_tri, "ref": ref_tri, "resp": resp_tri, "eff": eff_tri}

        if self._is_hard_disabled():
            fs = self.robot.get_fault_status()
            raise RobotHardFaultError(
                f"robot HARD_DISABLED during triangle stream ({fs.get('code')}): {fs.get('reason')}"
            )
        if self.robot.is_estopped:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict="violated:estop_during_triangle_stream",
                duration_s=time.monotonic() - t_start,
            )
        if len(t_tri) < 10:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict="violated:stream_too_short",
                duration_s=time.monotonic() - t_start,
            )

        if tick_watchdog is not None and tick_watchdog.tripped:
            tick_wd_tripped = True
            tick_wd_reason = tick_watchdog.reason

        # --- Step leg ---
        if not tick_wd_tripped:
            if tick_watchdog is not None:
                tick_watchdog.reset()

            step_traj = WaveTrajectory(
                _ZERO6, j, "square",
                amp_deg=L0.step_amp_deg,
                period=L0.step_period,
                cycles=L0.step_cycles,
                hold_pre=L0.hold_pre,
                hold_post=L0.hold_post,
                edge_rate_deg_s=240.0,
            )
            try:
                self._runner.transit_to(step_traj.q_start)
            except Exception:
                pass
            else:
                t_sq, ref_sq, resp_sq, eff_sq = self._runner.stream(
                    step_traj.sample, step_traj.duration,
                    kp_6, kd_6,
                    watchdog=tick_watchdog,
                )
                raw["square"] = {"t": t_sq, "ref": ref_sq, "resp": resp_sq, "eff": eff_sq}

                if self._is_hard_disabled():
                    fs = self.robot.get_fault_status()
                    raise RobotHardFaultError(
                        f"robot HARD_DISABLED during step stream ({fs.get('code')}): {fs.get('reason')}"
                    )
                if self.robot.is_estopped:
                    return TrialResult(
                        cost=PENALTY_COST,
                        breakdown={},
                        watchdog_verdict="violated:estop_during_step_stream",
                        duration_s=time.monotonic() - t_start,
                    )

                if tick_watchdog is not None and tick_watchdog.tripped:
                    tick_wd_tripped = True
                    tick_wd_reason = tick_watchdog.reason

        # --- Return to safe config ---
        try:
            self._runner.return_to_zero()
        except Exception:
            pass

        # --- If tick watchdog tripped, return penalty ---
        if tick_wd_tripped:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"violated:{tick_wd_reason}",
                tick_wd_reason=tick_wd_reason,
                raw=raw,
                duration_s=time.monotonic() - t_start,
            )

        # --- Compute metrics ---
        from a1z.analysis.metrics import (
            hold_noise_floor_from_trace,
            step_metrics_v2,
            summarize_steps_v2,
            triangle_metrics_v2,
        )

        tri_data = raw.get("triangle", {})
        lag_s = 0.0
        resid_std = 0.0

        if "t" in tri_data and len(tri_data["t"]) > 10:
            tri_m = triangle_metrics_v2(
                tri_data["t"], tri_data["ref"][:, j], tri_data["resp"][:, j],
                eff=tri_data["eff"][:, j],
            )
            lag_s = tri_m.get("lag_at_rate_deg", 0.0) or 0.0
            resid_std = tri_m.get("resid_std_deg", 0.0) or 0.0
            raw["tri_metrics"] = tri_m

        ts_s = 0.0
        ess_p95 = 0.0
        step_summary = None

        sq_data = raw.get("square", {})
        if "t" in sq_data and len(sq_data["t"]) > 10:
            noise_floor = hold_noise_floor_from_trace(
                sq_data["t"], sq_data["ref"][:, j], sq_data["resp"][:, j])
            steps = step_metrics_v2(
                sq_data["t"], sq_data["ref"][:, j], sq_data["resp"][:, j])
            if steps:
                step_summary = summarize_steps_v2(steps, noise_floor_deg=noise_floor)
                # If every step never settles (ts_ms is None), penalise with a
                # large-but-bounded proxy instead of silently returning 0.
                ts_s = step_summary.get("ts_max_ms")
                if ts_s is None:
                    ts_s = 2000.0  # ms — worse than any plausible settle time
                ts_s = ts_s or 0.0
                ess_p95 = step_summary.get("ess_max_deg", 0.0) or 0.0
                raw["step_metrics"] = step_summary

        os_pct = (
            step_summary.get("overshoot_max_pct", 0.0)
            if step_summary is not None else 0.0
        )
        cost, breakdown = compute_joint_cost(
            j, lag_s, ts_s, resid_std, ess_p95, os_pct)

        # --- Trial-level checks ---
        temp_mos = None
        temp_rotor = None
        try:
            st = self._runner.robot.get_joint_state()
            temp_mos = st.get("temp_mos")
            temp_rotor = st.get("temp_rotor")
        except Exception:
            pass

        trial_verdict = self._trial_checker.check(
            step_summary, temp_mos, tick_wd_tripped)

        watchdog_verdict = "ok"
        if not trial_verdict.ok:
            watchdog_verdict = "violated:" + ";".join(trial_verdict.reasons)
            cost = PENALTY_COST

        if trial_verdict.temp_pause:
            watchdog_verdict = "temp_pause:" + ";".join(trial_verdict.reasons)

        return TrialResult(
            cost=cost,
            breakdown=breakdown,
            watchdog_verdict=watchdog_verdict,
            trial_checker_reasons=trial_verdict.reasons,
            temp_mos=temp_mos,
            temp_rotor=temp_rotor,
            raw=raw,
            duration_s=time.monotonic() - t_start,
        )
