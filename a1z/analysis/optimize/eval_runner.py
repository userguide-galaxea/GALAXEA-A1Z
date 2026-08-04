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
    L0EE,
    PENALTY_COST,
    compute_ee_cost,
    compute_joint_cost,
    compute_total_cost,
)
from a1z.analysis.optimize.search_space import (
    refine_theta_to_gains_6,
    theta_to_gains_6,
    to_gains,
)
from a1z.analysis.optimize.watchdog import (
    TickWatchdog,
    TrialChecker,
    WatchdogViolation,
)
from a1z.analysis.runner import EETrackingRunner, JointUnitTestRunner, _ZERO6
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
        ee_leg: bool = False,
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
        # E-segment EE leg (SOP-11 §12.1): small circle + terminal hold, only
        # constructed for refine sessions.  The dry-run solve + gate run here,
        # BEFORE any hardware contact — a failed gate is gains-independent,
        # so every trial would fail identically (devlog 2026-08-01 E3).
        self._ee_runner: Optional[EETrackingRunner] = None
        self._ee_offline: Optional[dict] = None
        if ee_leg:
            self._ee_runner = EETrackingRunner(
                can_channel,
                q_nom_deg=L0EE.q_nom_deg,
                ee_kind="circle",
                plane=L0EE.plane,
                radius=L0EE.radius,
                period=L0EE.period,
                cycles=L0EE.cycles,
                hold_s=L0EE.hold_s,
                settle_s=L0EE.settle_s,
                sample_hz=sample_hz,
                transit_speed_deg_s=transit_speed_deg_s,
                integral_level=integral_level,
                integral_joints=integral_joints,
                integral_overrides=integral_overrides,
                inter_cmd_gap_us=inter_cmd_gap_us,
            )
            self._ee_offline = self._ee_runner.solve_offline()
            if not self._ee_offline["gate"].passed:
                raise RuntimeError(
                    "EE dry-run gate failed — refusing to start the session: "
                    + self._ee_offline["gate"].summary())

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
        if self._ee_runner is not None:
            self._ee_runner.robot = self._runner.robot
            self._ee_runner._started = True
        return self

    def __exit__(self, *exc):
        if self._runner._started:
            try:
                self._runner.kp = None
                self._runner.kd = None
                if self._ee_runner is not None:
                    self._ee_runner.kp = None
                    self._ee_runner.kd = None
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

    # ------------------------------------------------------------------
    # Shared trial primitives
    # ------------------------------------------------------------------
    def _recover_or_penalty(self, t_start: float) -> Optional[TrialResult]:
        """Pre-trial robot-state recovery.  ``None`` = OK to proceed.

        Raises RobotHardFaultError on HARD_DISABLED; returns a penalty
        TrialResult when a fault cannot be released.
        """
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
        return None

    def _set_gains(self, kp_6: np.ndarray, kd_6: np.ndarray) -> None:
        """Whole-arm 6-vector gains on every leg runner (E3)."""
        for r in (self._runner, self._step_runner, self._ee_runner):
            if r is not None:
                r.kp = kp_6
                r.kd = kd_6

    def _run_joint_legs(
        self,
        joint1: int,
        kp_6: np.ndarray,
        kd_6: np.ndarray,
        tick_watchdog: Optional[TickWatchdog],
        raw: Dict,
        t_start: float,
    ) -> Tuple[Optional[TrialResult], bool, str]:
        """L0 triangle + step legs for ONE joint (Phase A semantics unchanged).

        Returns ``(early_exit, tick_wd_tripped, tick_wd_reason)`` —
        ``early_exit`` is a penalty TrialResult for transit/estop/short-stream
        failures, ``None`` when the legs completed.  ``raw`` receives the
        ``"triangle"`` / ``"square"`` traces.
        """
        j = joint1 - 1
        tripped = False
        reason = ""

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

        try:
            self._runner.transit_to(tri_traj.q_start)
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
            ), False, ""

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
            ), False, ""
        if len(t_tri) < 10:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict="violated:stream_too_short",
                duration_s=time.monotonic() - t_start,
            ), False, ""

        if tick_watchdog is not None and tick_watchdog.tripped:
            tripped = True
            reason = tick_watchdog.reason

        # --- Step leg ---
        if not tripped:
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
                    ), tripped, reason

                if tick_watchdog is not None and tick_watchdog.tripped:
                    tripped = True
                    reason = tick_watchdog.reason

        # --- Return to safe config ---
        try:
            self._runner.return_to_zero()
        except Exception:
            pass

        return None, tripped, reason

    @staticmethod
    def _joint_leg_metrics(j: int, raw: Dict):
        """v2 metrics of one joint's recorded L0 legs.

        Returns ``(lag_deg, ts_ms, resid_std_deg, ess_deg, overshoot_pct,
        step_summary)`` — the exact inputs of ``compute_joint_cost``.
        """
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
        return lag_s, ts_s, resid_std, ess_p95, os_pct, step_summary

    def _temps(self):
        """(temp_mos, temp_rotor) best-effort readout."""
        temp_mos = None
        temp_rotor = None
        try:
            st = self._runner.robot.get_joint_state()
            temp_mos = st.get("temp_mos")
            temp_rotor = st.get("temp_rotor")
        except Exception:
            pass
        return temp_mos, temp_rotor

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
        early = self._recover_or_penalty(t_start)
        if early is not None:
            return early

        kp_j = theta["kp"]
        kp_6, kd_6 = theta_to_gains_6(
            j, kp_j, theta.get("zeta_hat"), kd_j=theta.get("kd"))
        self._set_gains(kp_6, kd_6)

        raw: Dict = {}
        early, tick_wd_tripped, tick_wd_reason = self._run_joint_legs(
            joint1, kp_6, kd_6, tick_watchdog, raw, t_start)
        if early is not None:
            return early

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
        lag_s, ts_s, resid_std, ess_p95, os_pct, step_summary = (
            self._joint_leg_metrics(j, raw))
        cost, breakdown = compute_joint_cost(
            j, lag_s, ts_s, resid_std, ess_p95, os_pct)

        # --- Trial-level checks ---
        temp_mos, temp_rotor = self._temps()

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

    def _run_ee_leg(
        self,
        ee_watchdog,
        raw: Dict,
        t_start: float,
    ) -> Tuple[Optional[TrialResult], Optional[Dict]]:
        """EE small-circle leg + metrics (shared by refine / Phase B trials).

        Returns ``(early_exit, ee_metrics)`` — ``early_exit`` is a penalty
        TrialResult on leg failure (RobotHardFaultError is raised through),
        ``ee_metrics`` the ee_refine_metrics dict on success.  ``raw["ee"]``
        / ``raw["ee_metrics"]`` are filled on success.
        """
        if ee_watchdog is not None:
            ee_watchdog.reset()
        try:
            live = self._ee_runner.run(self._ee_offline, watchdog=ee_watchdog)
        except Exception as exc:
            if self._is_hard_disabled():
                fs = self.robot.get_fault_status()
                raise RobotHardFaultError(
                    f"robot HARD_DISABLED during EE leg ({fs.get('code')}): {fs.get('reason')}"
                )
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"ee_leg_fail:{exc}",
                raw=raw,
                duration_s=time.monotonic() - t_start,
            ), None
        if self._is_hard_disabled():
            fs = self.robot.get_fault_status()
            raise RobotHardFaultError(
                f"robot HARD_DISABLED during EE leg ({fs.get('code')}): {fs.get('reason')}"
            )
        if self.robot.is_estopped:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict="violated:estop_during_ee_leg",
                raw=raw,
                duration_s=time.monotonic() - t_start,
            ), None

        raw["ee"] = {"t": live["t"], "T_ref": live["T_ref"],
                     "T_resp": live["T_resp"], "q_ref": live["q_ref"],
                     "q_resp": live["q_resp"]}

        from a1z.analysis.metrics import ee_refine_metrics
        ee_metrics = ee_refine_metrics(
            live["T_ref"], live["T_resp"],
            float(self._runner.sample_hz),
            np.asarray(L0EE.normal, dtype=float),
            tail_frac=L0EE.tail_frac)
        raw["ee_metrics"] = ee_metrics

        if ee_watchdog is not None and ee_watchdog.tripped:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"violated:{ee_watchdog.reason}",
                tick_wd_reason=ee_watchdog.reason,
                raw=raw,
                duration_s=time.monotonic() - t_start,
            ), None
        return None, ee_metrics

    # ------------------------------------------------------------------
    # E-segment refine trial (multi-joint subset + EE leg, SOP-11 §6.3/§12.1)
    # ------------------------------------------------------------------
    def eval_trial_refine(
        self,
        theta: Dict[str, float],
        joints1: List[int],
        *,
        tick_watchdogs: Optional[Dict[int, TickWatchdog]] = None,
        ee_watchdog=None,
    ) -> TrialResult:
        """E-segment refine trial (devlog 2026-08-01 E3).

        Whole-arm 6-vector gains from the subset theta; every searched joint
        gets the unchanged L0 triangle + step legs (its own preconditioned
        posture, Phase A semantics), then one EE small-circle leg with the
        terminal hold.  Cost = J_total (§2.3): mean per-joint J_joint
        combined with J_ee from the EE leg.
        """
        t_start = time.monotonic()

        if self._ee_runner is None or self._ee_offline is None:
            raise RuntimeError("eval_trial_refine requires ee_leg=True")

        # --- Pre-trial robot-state recovery ---
        early = self._recover_or_penalty(t_start)
        if early is not None:
            return early

        kp_6, kd_6 = refine_theta_to_gains_6(theta, joints1)
        self._set_gains(kp_6, kd_6)

        # --- Per-joint L0 legs ---
        joint_legs: Dict[int, Dict] = {}
        tick_wd_tripped = False
        tick_wd_reason = ""
        for j1 in joints1:
            leg_raw: Dict = {}
            wd = (tick_watchdogs or {}).get(j1 - 1)
            early, tick_wd_tripped, tick_wd_reason = self._run_joint_legs(
                j1, kp_6, kd_6, wd, leg_raw, t_start)
            joint_legs[j1] = leg_raw
            if early is not None:
                early.raw = {"joint_legs": joint_legs}
                return early
            if tick_wd_tripped:
                tick_wd_reason = f"J{j1}:{tick_wd_reason}"
                break

        raw: Dict = {"joint_legs": joint_legs}

        if tick_wd_tripped:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"violated:{tick_wd_reason}",
                tick_wd_reason=tick_wd_reason,
                raw=raw,
                duration_s=time.monotonic() - t_start,
            )

        # --- EE leg (small circle + terminal hold; multi-joint backstop wd) ---
        early, ee_metrics = self._run_ee_leg(ee_watchdog, raw, t_start)
        if early is not None:
            return early

        # --- Costs: mean per-joint J_joint + J_ee → J_total (§2.3) ---
        joint_costs: Dict[str, float] = {}
        joint_metrics: Dict[str, Dict] = {}
        step_summaries: List[Optional[dict]] = []
        for j1 in joints1:
            j = j1 - 1
            lag_s, ts_s, resid_std, ess_p95, os_pct, step_summary = (
                self._joint_leg_metrics(j, joint_legs[j1]))
            c_j, _ = compute_joint_cost(j, lag_s, ts_s, resid_std, ess_p95, os_pct)
            joint_costs[f"J{j1}"] = c_j
            joint_metrics[f"J{j1}"] = {
                "lag_deg": lag_s, "ts_ms": ts_s, "resid_std_deg": resid_std,
                "ess_deg": ess_p95, "overshoot_pct": os_pct,
            }
            step_summaries.append(step_summary)
        j_joint = float(np.mean(list(joint_costs.values())))
        j_ee, ee_bd = compute_ee_cost(ee_metrics)
        total, total_bd = compute_total_cost(j_joint, j_ee)
        raw["joint_costs"] = joint_costs
        raw["joint_metrics"] = joint_metrics
        raw["ee_breakdown"] = ee_bd
        breakdown = {
            "ee": total_bd["ee"],
            "joint": total_bd["joint"],
            "j_ee": j_ee,
            "j_joint": j_joint,
        }

        # --- Trial-level checks (per-joint step summaries; temp once) ---
        temp_mos, temp_rotor = self._temps()
        ok_all = True
        temp_pause = False
        reasons_all: List[str] = []
        for k, j1 in enumerate(joints1):
            v = self._trial_checker.check(
                step_summaries[k], temp_mos if k == 0 else None, False)
            if not v.ok:
                ok_all = False
            if v.temp_pause:
                temp_pause = True
            reasons_all.extend(f"J{j1}:{r}" for r in v.reasons)

        cost = total
        watchdog_verdict = "ok"
        if not ok_all:
            watchdog_verdict = "violated:" + ";".join(reasons_all)
            cost = PENALTY_COST
        if temp_pause:
            watchdog_verdict = "temp_pause:" + ";".join(reasons_all)

        return TrialResult(
            cost=cost,
            breakdown=breakdown,
            watchdog_verdict=watchdog_verdict,
            trial_checker_reasons=reasons_all,
            temp_mos=temp_mos,
            temp_rotor=temp_rotor,
            raw=raw,
            duration_s=time.monotonic() - t_start,
        )

    # ------------------------------------------------------------------
    # Phase B trial (kp/kd frozen; per-trial coulomb_ff / integral, SOP-11 §7)
    # ------------------------------------------------------------------
    def eval_trial_phase_b(
        self,
        theta: Dict[str, float],
        joint1: int,
        *,
        tick_watchdog: Optional[TickWatchdog] = None,
        ee_watchdog=None,
    ) -> TrialResult:
        """Phase B trial (devlog 2026-08-03 B2).

        PD 冻结（kp/kd = 冻结默认 6-vector）；逐 trial 机制注入：
        ``coulomb_ff``（绝对 Nm，tanh 平滑路径，单关节使能）+ 积分维度
        （theta 含 ``t_wind_s`` 时按 §1.4 裁剪开启，否则关断）。trial 流程
        = 该关节 L0 两腿 + EE 小圆腿，代价 = J_total。
        """
        from a1z.analysis.optimize.friction import CoulombConfig
        from a1z.robots.integrator import IntegralConfig

        t_start = time.monotonic()

        if self._ee_runner is None or self._ee_offline is None:
            raise RuntimeError("eval_trial_phase_b requires ee_leg=True")

        # --- Pre-trial robot-state recovery ---
        early = self._recover_or_penalty(t_start)
        if early is not None:
            return early

        j = joint1 - 1
        kp_6 = DEFAULT_KP.copy()
        kd_6 = DEFAULT_KD.copy()
        self._set_gains(kp_6, kd_6)

        # --- Per-trial feedforward mechanisms (runtime swap, arm_robot
        # setters hold the command lock) ---
        c_ff = float(theta.get("coulomb_ff", 0.0))
        if c_ff > 0.0:
            tau_c = np.zeros(6)
            tau_c[j] = c_ff
            self.robot.set_coulomb_config(CoulombConfig(tau_c=tau_c))
        else:
            self.robot.set_coulomb_config(None)
        if "t_wind_s" in theta:
            self.robot.set_integral_config(IntegralConfig.from_level(
                "K1", joints=[joint1],
                t_wind_s=float(theta["t_wind_s"]),
                clamp_scale=float(theta.get("clamp_scale", 1.2)),
                t_leak_s=float(theta.get("t_leak_s", 1.0))))
        else:
            self.robot.set_integral_config(None)

        # --- Joint L0 legs (Phase A semantics, gains frozen) ---
        raw: Dict = {}
        early, tick_wd_tripped, tick_wd_reason = self._run_joint_legs(
            joint1, kp_6, kd_6, tick_watchdog, raw, t_start)
        if early is not None:
            return early
        if tick_wd_tripped:
            return TrialResult(
                cost=PENALTY_COST,
                breakdown={},
                watchdog_verdict=f"violated:{tick_wd_reason}",
                tick_wd_reason=tick_wd_reason,
                raw=raw,
                duration_s=time.monotonic() - t_start,
            )

        # --- EE leg ---
        early, ee_metrics = self._run_ee_leg(ee_watchdog, raw, t_start)
        if early is not None:
            return early

        # --- Costs: single-joint J_joint + J_ee → J_total (§2.3) ---
        lag_s, ts_s, resid_std, ess_p95, os_pct, step_summary = (
            self._joint_leg_metrics(j, raw))
        j_joint, _ = compute_joint_cost(j, lag_s, ts_s, resid_std, ess_p95, os_pct)
        j_ee, ee_bd = compute_ee_cost(ee_metrics)
        total, total_bd = compute_total_cost(j_joint, j_ee)
        raw["joint_metrics"] = {
            "lag_deg": lag_s, "ts_ms": ts_s, "resid_std_deg": resid_std,
            "ess_deg": ess_p95, "overshoot_pct": os_pct,
        }
        raw["ee_breakdown"] = ee_bd
        breakdown = {
            "ee": total_bd["ee"],
            "joint": total_bd["joint"],
            "j_ee": j_ee,
            "j_joint": j_joint,
        }

        # --- Trial-level checks ---
        temp_mos, temp_rotor = self._temps()
        trial_verdict = self._trial_checker.check(step_summary, temp_mos, False)

        cost = total
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
