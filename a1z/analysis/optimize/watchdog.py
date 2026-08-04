"""Three-layer watchdog: tick, trial, session (SOP-11 §4).

Tick-level ``TickWatchdog`` is fed into ``_Base.stream(watchdog=...)`` and runs
at ~100 Hz inside the control loop.  Trial-level ``TrialChecker`` runs once
after each eval.  Session-level ``AnchorMonitor`` tracks anchor-config drift.
"""
from __future__ import annotations

import csv
import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from a1z.analysis.optimize.cost_spec import (
    TORQUE_CLIP,
    VEL_LIMIT_RAD_S,
    WATCHDOG_VEL_ABS_FACTOR,
)

DEG = 180.0 / math.pi

# Far-field position-divergence default.  This is meant as a safety net for
# unstable runaway, not as a tracking-accuracy gate.  B1-calibrated
# theta_pos = 2 * norm_max |ref-resp| is often too tight for BO exploration:
# with high kp the response can legitimately sit 3–5 deg off the reference at
# triangle turnarounds.  15 deg gives headroom while still catching true
# divergence long before joint limits are reached.
DEFAULT_THETA_POS_DEG = 15.0


def _deg_to_rad(v: Optional[float]) -> Optional[float]:
    return None if v is None else math.radians(v)


# ---------------------------------------------------------------------------
# Exception
# ---------------------------------------------------------------------------
class WatchdogViolation(Exception):
    """Raised (or stored) when any watchdog layer trips."""

    def __init__(self, reason: str, *, tick: Optional[float] = None,
                 snapshot: Optional[Dict] = None):
        super().__init__(reason)
        self.reason = reason
        self.tick = tick
        self.snapshot = snapshot or {}


# ---------------------------------------------------------------------------
# Layer 1 — Tick-level  (SOP-11 §4.1)
# ---------------------------------------------------------------------------
class TickWatchdog:
    """Per-tick safety checks, designed to plug into ``_Base.stream(watchdog=...)``.

    Protocol: ``ok, reason = wd.check(t, ref, resp, eff, vel=None)``
    * ok=True  → proceed
    * ok=False → stream must break immediately

    ``vel`` is the measured joint velocity from ``get_joint_state()``; when
    omitted the watchdog finite-differences ``resp`` itself.  Existing callers
    that pass only ``(t, ref, resp, eff)`` continue to work unchanged.

    After stream returns, inspect ``wd.tripped`` and ``wd.reason``.
    """

    def __init__(
        self,
        joint: int,
        *,
        theta_pos_deg: float = DEFAULT_THETA_POS_DEG,
        theta_eff_scale: float = 0.9,
        theta_hf_scale: float = 4.0,
        hf_window: int = 50,
        theta_vel_deg_s: Optional[float] = None,
        theta_acc_deg_s2: Optional[float] = None,
        theta_vel_abs_deg_s: Optional[float] = None,
        kin_window: int = 5,
        hf_confirm_ticks: int = 3,
        hf_step_exempt_deg: Optional[float] = 1.0,
        hf_step_exempt_s: float = 0.6,
    ):
        self.j = int(joint)
        self.theta_pos = math.radians(theta_pos_deg)
        self.theta_eff = theta_eff_scale * TORQUE_CLIP[self.j]
        self.theta_hf_scale = theta_hf_scale
        self.hf_window = hf_window
        # HF hysteresis (devlog 2026-07-31 Q12-D): a single over-threshold
        # window is often a statistical fluke near the cliff edge; require
        # this many consecutive over-threshold evaluations (≈ sustained
        # oscillation) before tripping.  1 = legacy single-window behaviour.
        self.hf_confirm_ticks = max(1, int(hf_confirm_ticks))
        self._hf_over_ticks = 0

        # HF step-edge exemption (devlog 2026-08-01 归因分析): the eff-diff
        # RMS channel cannot distinguish a sharp-but-safe torque transient at
        # a square-wave ref edge from genuine sustained oscillation — all four
        # hf_osc violations in session 2026-08-01-run-opt-phaseA-J6 were edge
        # transients.  When |Δref_j| exceeds ``hf_step_exempt_deg``, skip HF
        # over-threshold counting for ``hf_step_exempt_s`` seconds (must be
        # ≥ hf_window × tick period = 0.5 s at 100 Hz so the transient fully
        # flushes out of the sliding window before judging resumes).
        # ``hf_step_exempt_deg=None`` disables the exemption (legacy).
        self.hf_step_exempt = (math.radians(hf_step_exempt_deg)
                               if hf_step_exempt_deg is not None else None)
        self.hf_step_exempt_s = float(hf_step_exempt_s)
        self._prev_ref_j: Optional[float] = None
        self._hf_exempt_until: float = 0.0

        # Kinematic channels (SOP-11 §4.1 / devlog 2026-07-29)
        self.theta_vel = _deg_to_rad(theta_vel_deg_s)
        self.theta_acc = _deg_to_rad(theta_acc_deg_s2)
        self.theta_vel_abs = _deg_to_rad(theta_vel_abs_deg_s)
        self.kin_window = max(2, int(kin_window))

        # HF ring buffer (eff differences)
        self._eff_buf: List[float] = []
        self._prev_eff: Optional[float] = None

        # PROVISIONAL HF-RMS baseline — replaced after B1 calibration (R1).
        self._hf_baseline_rms: Optional[float] = None

        # Kinematic ring buffers: (t, ref_j, resp_j, vel_j)
        self._kin_buf: Deque[Tuple[float, float, float, Optional[float]]] = deque(
            maxlen=self.kin_window)
        self._vel_err_buf: Deque[float] = deque(maxlen=self.kin_window)
        self._resp_acc_buf: Deque[float] = deque(maxlen=self.kin_window)

        self.tripped = False
        self.reason = ""
        self._last_snapshot: Dict = {}

    # -----------------------------------------------------------------------
    # Threshold setters (allow runtime injection from calibration JSON)
    # -----------------------------------------------------------------------
    def set_hf_baseline(self, rms: float) -> None:
        """Set the normal-operation HF-RMS baseline from B1 calibration data."""
        self._hf_baseline_rms = rms

    def set_vel_threshold_deg_s(self, deg_s: Optional[float]) -> None:
        """Set velocity-error threshold; ``None`` disables the channel."""
        self.theta_vel = _deg_to_rad(deg_s)

    def set_acc_threshold_deg_s2(self, deg_s2: Optional[float]) -> None:
        """Set response-acceleration threshold; ``None`` disables the channel."""
        self.theta_acc = _deg_to_rad(deg_s2)

    def set_vel_abs_threshold_deg_s(self, deg_s: Optional[float]) -> None:
        """Set absolute response-velocity threshold; ``None`` disables the channel."""
        self.theta_vel_abs = _deg_to_rad(deg_s)

    # -----------------------------------------------------------------------
    def check(
        self,
        t: float,
        ref: np.ndarray,
        resp: np.ndarray,
        eff: np.ndarray,
        *,
        vel: Optional[np.ndarray] = None,
    ) -> Tuple[bool, str]:
        j = self.j
        snapshot = {"t": t, "ref_j": float(ref[j]), "resp_j": float(resp[j]),
                    "eff_j": float(eff[j])}

        # 1) Position divergence
        pos_err = abs(resp[j] - ref[j])
        if pos_err > self.theta_pos:
            return self._trip(
                f"pos_diverge: |err|={pos_err * DEG:.1f}deg > {self.theta_pos * DEG:.0f}deg",
                t, snapshot)

        # 2) Effort spike
        if abs(eff[j]) > self.theta_eff:
            return self._trip(
                f"eff_spike: |eff|={abs(eff[j]):.2f}Nm > {self.theta_eff:.1f}Nm",
                t, snapshot)

        # 3) Kinematic violence (vel / acc overshoot at turnarounds)
        measured_vel = None if vel is None else float(vel[j])
        resp_j = float(resp[j])
        self._update_kinematics(t, float(ref[j]), resp_j, measured_vel)

        # 3a) Instantaneous absolute velocity guard (hardware-limit-aware).
        # This is independent of the B1-calibrated vel/acc RMS channels and is
        # intended to catch runaway velocity *before* the robot's own hard fault.
        if self.theta_vel_abs is not None and self.theta_vel_abs > 0:
            vel_abs = 0.0
            if measured_vel is not None:
                vel_abs = abs(measured_vel)
            elif len(self._kin_buf) >= 2:
                t_prev, _, resp_prev, _ = self._kin_buf[-2]
                dt = t - t_prev
                if dt > 0:
                    vel_abs = abs((resp_j - resp_prev) / dt)
            if vel_abs > self.theta_vel_abs:
                snapshot["vel_abs"] = vel_abs
                snapshot["theta_vel_abs"] = self.theta_vel_abs
                return self._trip(
                    f"vel_abs: |qdot|={vel_abs * DEG:.1f}deg/s > "
                    f"{self.theta_vel_abs * DEG:.1f}deg/s",
                    t, snapshot)

        kin_ok, kin_reason = self._check_kinematics(snapshot)
        if not kin_ok:
            return self._trip(kin_reason, t, snapshot)

        # 4) HF oscillation (difference-RMS over sliding window)
        # Step-edge exemption: a ref jump marks the start of a transient;
        # suppress HF judging until the spike has flushed out of the window.
        ref_j = float(ref[j])
        if (self.hf_step_exempt is not None
                and self._prev_ref_j is not None
                and abs(ref_j - self._prev_ref_j) > self.hf_step_exempt):
            self._hf_exempt_until = t + self.hf_step_exempt_s
        self._prev_ref_j = ref_j

        eff_j = float(eff[j])
        if self._prev_eff is not None:
            self._eff_buf.append(eff_j - self._prev_eff)
            if len(self._eff_buf) > self.hf_window:
                self._eff_buf.pop(0)
        self._prev_eff = eff_j

        if (len(self._eff_buf) >= self.hf_window
                and self._hf_baseline_rms is not None
                and self._hf_baseline_rms > 0):
            hf_rms = math.sqrt(sum(d * d for d in self._eff_buf) / len(self._eff_buf))
            theta_hf = self.theta_hf_scale * self._hf_baseline_rms
            if t < self._hf_exempt_until:
                self._hf_over_ticks = 0
            elif hf_rms > theta_hf:
                self._hf_over_ticks += 1
            else:
                self._hf_over_ticks = 0
            if self._hf_over_ticks >= self.hf_confirm_ticks:
                snapshot["hf_rms"] = hf_rms
                snapshot["theta_hf"] = theta_hf
                snapshot["hf_confirm_ticks"] = self.hf_confirm_ticks
                return self._trip(
                    f"hf_osc: RMS={hf_rms:.3f} > {theta_hf:.3f} "
                    f"({self._hf_over_ticks} consecutive)",
                    t, snapshot)

        return True, ""

    def _update_kinematics(
        self,
        t: float,
        ref_j: float,
        resp_j: float,
        measured_vel: Optional[float],
    ) -> None:
        """Append newest sample and derive vel_err / resp_acc for this tick."""
        self._kin_buf.append((t, ref_j, resp_j, measured_vel))

        if len(self._kin_buf) < 2:
            return

        # Response velocity: prefer measured, else backward difference.
        t_prev, _, resp_prev, _ = self._kin_buf[-2]
        dt = t - t_prev
        if dt <= 0:
            return
        if measured_vel is not None:
            resp_vel = measured_vel
        else:
            resp_vel = (resp_j - resp_prev) / dt

        # Reference velocity: backward difference.
        _, ref_prev, _, _ = self._kin_buf[-2]
        ref_vel = (ref_j - ref_prev) / dt

        # Response acceleration: second-order divided difference once we have
        # three samples; skip the acc buffer otherwise (avoids noisy one-sided
        # estimate on the first tick).
        if len(self._kin_buf) >= 3:
            t_m2, _, resp_m2, _ = self._kin_buf[-3]
            dt_m2 = t - t_m2
            if dt_m2 > 0:
                resp_acc = 2.0 * ((resp_j - resp_prev) / dt -
                                  (resp_prev - resp_m2) / (t_prev - t_m2)) / dt_m2
                self._resp_acc_buf.append(abs(resp_acc))

        self._vel_err_buf.append(abs(resp_vel - ref_vel))

    def _check_kinematics(self, snapshot: Dict) -> Tuple[bool, str]:
        # Velocity-error channel
        if (self.theta_vel is not None and self.theta_vel > 0
                and len(self._vel_err_buf) >= self.kin_window):
            vel_rms = math.sqrt(sum(v * v for v in self._vel_err_buf) /
                                len(self._vel_err_buf))
            if vel_rms > self.theta_vel:
                snapshot["vel_err_rms"] = vel_rms
                snapshot["theta_vel"] = self.theta_vel
                return False, (f"vel_spike: RMS={vel_rms * DEG:.2f}deg/s > "
                               f"{self.theta_vel * DEG:.2f}deg/s")

        # Response-acceleration channel
        if (self.theta_acc is not None and self.theta_acc > 0
                and len(self._resp_acc_buf) >= self.kin_window):
            acc_rms = math.sqrt(sum(a * a for a in self._resp_acc_buf) /
                                len(self._resp_acc_buf))
            if acc_rms > self.theta_acc:
                snapshot["resp_acc_rms"] = acc_rms
                snapshot["theta_acc"] = self.theta_acc
                return False, (f"acc_spike: RMS={acc_rms * DEG:.1f}deg/s2 > "
                               f"{self.theta_acc * DEG:.1f}deg/s2")

        return True, ""

    def _trip(self, reason: str, t: float, snapshot: Dict) -> Tuple[bool, str]:
        self.tripped = True
        self.reason = reason
        self._last_snapshot = snapshot
        return False, reason

    def as_violation(self) -> WatchdogViolation:
        return WatchdogViolation(self.reason, tick=self._last_snapshot.get("t"),
                                 snapshot=self._last_snapshot)

    def reset(self) -> None:
        self.tripped = False
        self.reason = ""
        self._eff_buf.clear()
        self._prev_eff = None
        self._prev_ref_j = None
        self._hf_exempt_until = 0.0
        self._kin_buf.clear()
        self._vel_err_buf.clear()
        self._resp_acc_buf.clear()


# ---------------------------------------------------------------------------
# Multi-joint fan-out (E-segment EE leg, SOP-11 §12.1)
# ---------------------------------------------------------------------------
class MultiTickWatchdog:
    """Check every child ``TickWatchdog`` per tick; trip on the first violation.

    The EE leg streams all six joints at once, so the single-joint
    ``TickWatchdog`` cannot monitor it directly.  This wrapper fans one
    ``check()`` call out to per-joint children (built via
    ``make_tick_watchdog`` so the B1 v4 active-table per-joint channel
    enables/thresholds apply verbatim) and exposes the same
    ``tripped`` / ``reason`` / ``reset()`` surface ``stream()`` and the
    eval loop already use.
    """

    def __init__(self, watchdogs: List[TickWatchdog]):
        self._wds = list(watchdogs)
        self.tripped = False
        self.reason = ""

    def check(
        self,
        t: float,
        ref: np.ndarray,
        resp: np.ndarray,
        eff: np.ndarray,
        *,
        vel: Optional[np.ndarray] = None,
    ) -> Tuple[bool, str]:
        for wd in self._wds:
            ok, reason = wd.check(t, ref, resp, eff, vel=vel)
            if not ok:
                self.tripped = True
                self.reason = f"J{wd.j + 1}:{reason}"
                return False, self.reason
        return True, ""

    def reset(self) -> None:
        self.tripped = False
        self.reason = ""
        for wd in self._wds:
            wd.reset()


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------
def make_tick_watchdog(
    joint: int,
    calib_path: Optional[Path] = None,
    *,
    theta_pos_deg: Optional[float] = None,
    theta_eff_scale: float = 0.9,
    theta_hf_scale: float = 4.0,
    hf_window: int = 50,
    theta_vel_deg_s: Optional[float] = None,
    theta_acc_deg_s2: Optional[float] = None,
    theta_vel_abs_deg_s: Optional[float] = None,
    kin_window: int = 5,
    hf_confirm_ticks: int = 3,
    hf_step_exempt_deg: Optional[float] = 1.0,
    hf_step_exempt_s: float = 0.6,
) -> TickWatchdog:
    """Build a ``TickWatchdog`` with optional per-joint B1 calibration JSON.

    The JSON is the v2 output of ``calibrate_watchdog.py``.  Missing fields
    fall back to the supplied defaults; this keeps v1 JSON files valid and
    lets callers opt into new channels incrementally.

    If ``theta_vel_abs_deg_s`` is not provided and not present in the JSON, it
    defaults to ``WATCHDOG_VEL_ABS_FACTOR`` of the SDK velocity limit for the
    joint (e.g. J6 = 10 rad/s).  This guard does *not* depend on B1 separation.

    ``theta_pos_deg`` defaults to ``DEFAULT_THETA_POS_DEG`` (far-field safety
    net).  A B1-calibrated ``theta_pos_deg`` in the JSON is honored when
    present, but it is intended to be the loose norm-based bound, not a
    tracking-accuracy gate.
    """
    jkey = f"J{int(joint) + 1}"
    jcal: Dict = {}
    if calib_path is not None:
        doc = json.loads(Path(calib_path).read_text())
        jcal = doc.get("joints", {}).get(jkey, {})

    theta_pos_deg = jcal.get("theta_pos_deg", theta_pos_deg)
    if theta_pos_deg is None:
        theta_pos_deg = DEFAULT_THETA_POS_DEG
    theta_eff_scale = jcal.get("theta_eff_scale", theta_eff_scale)
    theta_vel_deg_s = jcal.get("theta_vel_deg_s", theta_vel_deg_s)
    theta_acc_deg_s2 = jcal.get("theta_acc_deg_s2", theta_acc_deg_s2)
    theta_vel_abs_deg_s = jcal.get(
        "theta_vel_abs_deg_s",
        theta_vel_abs_deg_s
        if theta_vel_abs_deg_s is not None
        else float(math.degrees(VEL_LIMIT_RAD_S[joint] * WATCHDOG_VEL_ABS_FACTOR)),
    )
    kin_window = jcal.get("kin_window", kin_window)
    hf_window = jcal.get("window_ticks", hf_window)
    hf_confirm_ticks = jcal.get("hf_confirm_ticks", hf_confirm_ticks)
    hf_step_exempt_deg = jcal.get("hf_step_exempt_deg", hf_step_exempt_deg)
    hf_step_exempt_s = jcal.get("hf_step_exempt_s", hf_step_exempt_s)

    wd = TickWatchdog(
        joint,
        theta_pos_deg=theta_pos_deg,
        theta_eff_scale=theta_eff_scale,
        theta_hf_scale=theta_hf_scale,
        hf_window=hf_window,
        theta_vel_deg_s=theta_vel_deg_s,
        theta_acc_deg_s2=theta_acc_deg_s2,
        theta_vel_abs_deg_s=theta_vel_abs_deg_s,
        kin_window=kin_window,
        hf_confirm_ticks=hf_confirm_ticks,
        hf_step_exempt_deg=hf_step_exempt_deg,
        hf_step_exempt_s=hf_step_exempt_s,
    )

    # HF channel: calibration gives absolute theta_hf + baseline; convert to
    # the scale the online code expects.
    hf_base = jcal.get("hf_baseline_rms")
    theta_hf = jcal.get("theta_hf")
    if hf_base is not None and hf_base > 0:
        wd.set_hf_baseline(float(hf_base))
        if theta_hf is not None and theta_hf > 0:
            wd.theta_hf_scale = float(theta_hf) / float(hf_base)

    return wd


# ---------------------------------------------------------------------------
# Layer 2 — Trial-level  (SOP-11 §4.2)
# ---------------------------------------------------------------------------
@dataclass
class TrialVerdict:
    ok: bool = True
    reasons: List[str] = field(default_factory=list)
    temp_pause: bool = False


class TrialChecker:
    """Post-trial checks (SOP-11 §4.2)."""

    def __init__(
        self,
        *,
        overshoot_limit: float = 0.30,
        jump_p95_baseline: float = 1.0,
        jump_regression_factor: float = 1.2,
        temp_mos_pause_c: float = 65.0,
        consec_wd_limit: int = 3,
    ):
        self.overshoot_limit = overshoot_limit
        self.jump_p95_baseline = jump_p95_baseline
        self.jump_regression_factor = jump_regression_factor
        self.temp_mos_pause_c = temp_mos_pause_c
        self.consec_wd_limit = consec_wd_limit
        self._consec_wd_trips = 0

    def check(
        self,
        step_metrics: Optional[dict],
        temp_mos: Optional[np.ndarray],
        tick_wd_tripped: bool,
    ) -> TrialVerdict:
        v = TrialVerdict()

        # Consecutive tick-watchdog trips
        if tick_wd_tripped:
            self._consec_wd_trips += 1
            if self._consec_wd_trips >= self.consec_wd_limit:
                v.ok = False
                v.reasons.append(
                    f"consec_wd: {self._consec_wd_trips} consecutive tick-watchdog trips")
        else:
            self._consec_wd_trips = 0

        # Overshoot (checked against the worst step in the trial summary)
        if step_metrics is not None:
            os_pct = step_metrics.get("overshoot_max_pct", 0.0)
            if os_pct is not None and os_pct > self.overshoot_limit * 100:
                v.ok = False
                v.reasons.append(f"overshoot: {os_pct:.1f}% > {self.overshoot_limit * 100:.0f}%")

            jp95 = step_metrics.get("jump_p95_star")
            if jp95 is not None and self.jump_p95_baseline > 0:
                if jp95 > self.jump_p95_baseline * self.jump_regression_factor:
                    v.ok = False
                    v.reasons.append(
                        f"jump_p95_regression: {jp95:.4f} > "
                        f"{self.jump_p95_baseline * self.jump_regression_factor:.4f}")

        # Temperature
        if temp_mos is not None:
            max_t = float(np.max(temp_mos))
            if max_t > self.temp_mos_pause_c:
                v.temp_pause = True
                v.reasons.append(f"temp_mos: {max_t:.0f}C > {self.temp_mos_pause_c:.0f}C")

        return v


# ---------------------------------------------------------------------------
# Layer 3 — Session-level anchor drift  (SOP-11 §4.3)
# ---------------------------------------------------------------------------
class AnchorMonitor:
    """Track anchor-config cost drift across the session."""

    def __init__(
        self,
        session_dir: Path,
        *,
        anchor_interval: int = 25,
        drift_sigma_factor: float = 2.0,
    ):
        self.anchor_interval = anchor_interval
        self.drift_sigma_factor = drift_sigma_factor
        self._costs: List[float] = []
        self._temps: List[Tuple[float, List[float], List[float]]] = []
        self._csv_path = Path(session_dir) / "session_drift.csv"
        self._drift_flag = False

    def due(self, trial_idx: int) -> bool:
        return trial_idx % self.anchor_interval == 0

    def record(self, cost: float, temp_mos: Optional[np.ndarray] = None,
               temp_rotor: Optional[np.ndarray] = None) -> None:
        self._costs.append(cost)
        t_now = time.monotonic()
        self._temps.append((
            t_now,
            temp_mos.tolist() if temp_mos is not None else [],
            temp_rotor.tolist() if temp_rotor is not None else [],
        ))
        self._check_drift()
        self._flush_csv()

    @property
    def drift_flag(self) -> bool:
        return self._drift_flag

    def _check_drift(self) -> None:
        if len(self._costs) < 3:
            return
        mean_c = sum(self._costs) / len(self._costs)
        var_c = sum((c - mean_c) ** 2 for c in self._costs) / len(self._costs)
        sigma = math.sqrt(var_c) if var_c > 0 else 0.0
        if sigma > 0 and len(self._costs) >= 2:
            latest = self._costs[-1]
            if abs(latest - mean_c) > self.drift_sigma_factor * sigma:
                self._drift_flag = True

    def _flush_csv(self) -> None:
        with open(self._csv_path, "w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(["anchor_idx", "cost", "time_s", "temp_mos", "temp_rotor"])
            for idx, ((ts, tm, tr), c) in enumerate(zip(self._temps, self._costs)):
                w.writerow([idx, f"{c:.6f}", f"{ts:.3f}",
                            ";".join(f"{v:.1f}" for v in tm),
                            ";".join(f"{v:.1f}" for v in tr)])
