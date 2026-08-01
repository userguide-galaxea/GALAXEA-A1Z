"""Cost-function specification and search-space constants for BO (SOP-11 §1–§2).

All hyper-parameters that affect cost *values* or search *behaviour* live in
``COST_SPEC_V1``.  Every trial meta records ``cost_spec_version``; every
``study.json`` snapshots the full spec dict.  **Changing a weight = bumping the
version string** so old studies are never contaminated.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Hardware constants (mirrored from get_robot.py / safety.py — kept literal
# to avoid importing CAN-level modules from an offline optimiser context).
# ---------------------------------------------------------------------------
DEFAULT_KP = np.array([100.0, 60.0, 40.0, 30.0, 10.0, 25.0])
DEFAULT_KD = np.array([4.9, 4.5, 5.0, 2.0, 0.5, 4.0])
TAU_C_HAT = np.array([0.3442, 0.3665, 0.6371, 0.66, 0.143, 0.13])
TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])

# Equivalent diagonal inertia at the J6-precondition posture
# q = [0, 20, -20, 0, 0, 0] deg — computed via pinocchio.crba on
# A1Z_Flange.urdf (see devlog 2026-07-28 §3-2).
I_HAT = np.array([0.060028, 0.124217, 0.133801, 0.006245, 0.000137, 0.000016])

# ---------------------------------------------------------------------------
# Phase A search space  (SOP-11 §1.3)
# ---------------------------------------------------------------------------
KD_CAP = 5.0

# Per-joint kd floor: do not let the (kp, zeta_hat) reparameterisation drive
# low-inertia joints into near-zero damping.  J6 in particular can map
# zeta=0.40, kp=87 -> kd≈0.03, which caused a velocity-limit hard fault.
KD_MIN_FLOOR_FACTOR = 0.25   # keep at least 25% of default kd
KD_MIN_ABS = 0.3             # Nm/(rad/s); absolute lower bound
KD_MIN = np.maximum(DEFAULT_KD * KD_MIN_FLOOR_FACTOR, KD_MIN_ABS)

# Hardware velocity limits mirrored from ArmRobot._vel_limit (rad/s).
# Watchdog uses a fraction of these as an instantaneous absolute-velocity guard.
VEL_LIMIT_RAD_S = np.array([12.0, 12.0, 12.0, 7.0, 20.0, 20.0])
WATCHDOG_VEL_ABS_FACTOR = 0.5

def _phase_a_kp_range(j: int) -> Tuple[float, float]:
    kp0 = DEFAULT_KP[j]
    return (kp0 / 2.0, min(4.0 * kp0, 200.0))

def _phase_a_zeta_range() -> Tuple[float, float]:
    return (0.4, 1.2)

# ---------------------------------------------------------------------------
# Phase B search space  (SOP-11 §1.4)
# ---------------------------------------------------------------------------
def _phase_b_coulomb_range(j: int) -> Tuple[float, float]:
    c_ss = 1.5 * TAU_C_HAT[j]
    return (0.0, 1.5 * c_ss)

PHASE_B_T_WIND_RANGE = (0.3, 3.0)      # log scale
PHASE_B_CLAMP_SCALE_RANGE = (0.8, 2.0)  # linear
PHASE_B_T_LEAK_RANGE = (0.5, 3.0)       # log scale

# ---------------------------------------------------------------------------
# Cost function  (SOP-11 §2.1)
# ---------------------------------------------------------------------------
WEIGHTS_JOINT = np.array([0.30, 0.20, 0.20, 0.10, 0.20])

# Baselines: default-tuning metric values used for non-dimensionalisation.
# PROVISIONAL — populated from A2 归档 data; replace with B1 calibration
# data once available.  Per-joint baselines stored as shape-(6,) arrays;
# the active joint is selected at evaluation time.
# NOTE: keys are matched to metrics_v2 units: lag in deg, ts in ms,
# resid_std in deg, ess in deg, overshoot in percent.
BASELINE_LAG_DEG = np.full(6, 0.5)         # ~0.5 deg equivalent lag
BASELINE_TS_MS = np.full(6, 500.0)         # ~0.5 s settle time
BASELINE_RESID_STD_DEG = np.full(6, 0.5)   # ~0.5 deg residual std
BASELINE_ESS_DEG = np.full(6, 0.3)         # ~0.3 deg steady-state error
BASELINE_OVERSHOOT_PCT = np.full(6, 10.0)  # ~10% overshoot (2 deg step)

# --- Normalisation floors (devlog 2026-07-31 Q1) ---------------------------
# Denominator = max(baseline, floor), floor = NORM_FLOOR_K x cross-session
# sigma.  When a baseline measures ~0 (e.g. overshoot of a critically-damped
# joint), the term silently switches from a relative to an absolute scale
# instead of dividing by a near-zero number and drowning the weighted sum.
NORM_FLOOR_K = 3.0

# lag / resid_std: vf-口径 sigma(run) measured in A2
# (02-test-log/02-a1z/04-a2-velff-gate/a2-report.md §A2-2:
#  J6 lag 0.0037 deg / resid 0.0005 deg, J2 lag 0.0120 deg / resid 0.0021 deg),
# scaled by NORM_FLOOR_K per the A2-6 cross-session rule (cross ≈ 3x same).
# Joints without a measurement take the conservative max of the measured pair.
NORM_FLOOR_LAG_DEG = np.array([0.036, 0.036, 0.036, 0.036, 0.036, 0.011])
NORM_FLOOR_RESID_STD_DEG = np.array([0.0063, 0.0063, 0.0063, 0.0063, 0.0063, 0.0015])

# ts / ess / overshoot: no vf-口径 sigma measured yet -- PROVISIONAL absolute
# floors; backfill from A2/G0 archived step data at stage P2 (SOP-11 §14).
NORM_FLOOR_TS_MS = np.full(6, 50.0)
NORM_FLOOR_ESS_DEG = np.full(6, 0.03)
NORM_FLOOR_OVERSHOOT_PCT = np.full(6, 3.0)

# Per-term saturation: feasible-region terms need not be unbounded -- truly
# dangerous regions already pay PENALTY_COST via the watchdog.
TERM_CLIP = 20.0

PENALTY_COST = 1e4


def violation_surrogate(feasible_costs) -> float:
    """Objective value told to the GP for a watchdog-violated trial (v7).

    Feasibility travels via the constraint channel (GPSampler
    ``constraints_func`` → ConstrainedLogEI); the objective value only
    needs to keep the objective GP's fit on scale.  Policy comparison in
    devlog 2026-07-31 Q11: worst-observed-feasible preserves the valley
    structure best (3x max over-suppresses exploration; absolute 1e4 or
    raw partial costs destroy the standardised landscape).
    """
    import numpy as _np
    feas = [c for c in feasible_costs if c is not None and _np.isfinite(c)
            and c < PENALTY_COST]
    if not feas:
        return PENALTY_COST
    return min(PENALTY_COST, max(feas))


# GP observation noise reference — cross-session sigma (SOP-11 §2.5).
# DOCUMENTARY ONLY: Optuna 4.9 GPSampler exposes no noise-prior parameter
# (its internal Gamma noise prior is not configurable), so this value is
# recorded for traceability but is NOT wired into the sampler
# (devlog 2026-07-31 Q9 finding ②).
GP_NOISE_PRIOR = 0.05

# ---------------------------------------------------------------------------
# L0 fast-evaluation preset  (SOP-11 §6.2)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class L0Preset:
    tri_period: float = 4.0
    tri_cycles: int = 1
    tri_vel_ff: bool = True
    tri_amp_deg: float = 15.0
    step_amp_deg: float = 2.0
    step_events: int = 2
    step_period: float = 4.0
    step_cycles: int = 1
    hold_pre: float = 0.5
    hold_post: float = 0.5

L0 = L0Preset()

# ---------------------------------------------------------------------------
# Aggregate spec (versioned, snapshotted into study.json)
# ---------------------------------------------------------------------------
COST_SPEC_VERSION = "v8"


def cost_spec_snapshot() -> dict:
    """Full serialisable snapshot for study.json reproducibility."""
    return {
        "version": COST_SPEC_VERSION,
        "weights_joint": WEIGHTS_JOINT.tolist(),
        "baselines": {
            "lag_deg": BASELINE_LAG_DEG.tolist(),
            "ts_ms": BASELINE_TS_MS.tolist(),
            "resid_std_deg": BASELINE_RESID_STD_DEG.tolist(),
            "ess_deg": BASELINE_ESS_DEG.tolist(),
            "overshoot_pct": BASELINE_OVERSHOOT_PCT.tolist(),
        },
        "norm_floors": {
            "k": NORM_FLOOR_K,
            "lag_deg": NORM_FLOOR_LAG_DEG.tolist(),
            "ts_ms": NORM_FLOOR_TS_MS.tolist(),
            "resid_std_deg": NORM_FLOOR_RESID_STD_DEG.tolist(),
            "ess_deg": NORM_FLOOR_ESS_DEG.tolist(),
            "overshoot_pct": NORM_FLOOR_OVERSHOOT_PCT.tolist(),
        },
        "term_clip": TERM_CLIP,
        "penalty_cost": PENALTY_COST,
        "violation_surrogate": "max(feasible_costs), capped at PENALTY_COST",
        # Joints whose (kp, zeta_hat) -> kd mapping collapses onto KD_MIN
        # for the whole space (devlog 2026-07-31 Q16): these search
        # (kp, kd) directly instead of (kp, zeta_hat).
        "kd_direct_search_joints": [
            j + 1 for j in range(6)
            if 2.0 * _phase_a_zeta_range()[1]
            * math.sqrt(_phase_a_kp_range(j)[1] * I_HAT[j]) < KD_MIN[j]
        ],
        "gp_noise_prior": GP_NOISE_PRIOR,
        "kd_cap": KD_CAP,
        "kd_min": KD_MIN.tolist(),
        "kd_min_floor_factor": KD_MIN_FLOOR_FACTOR,
        "kd_min_abs": KD_MIN_ABS,
        "vel_limit_rad_s": VEL_LIMIT_RAD_S.tolist(),
        "watchdog_vel_abs_factor": WATCHDOG_VEL_ABS_FACTOR,
        "I_hat": I_HAT.tolist(),
        "default_kp": DEFAULT_KP.tolist(),
        "default_kd": DEFAULT_KD.tolist(),
        "tau_c_hat": TAU_C_HAT.tolist(),
        "torque_clip": TORQUE_CLIP.tolist(),
        "l0": {
            "tri_period": L0.tri_period,
            "tri_cycles": L0.tri_cycles,
            "tri_vel_ff": L0.tri_vel_ff,
            "tri_amp_deg": L0.tri_amp_deg,
            "step_amp_deg": L0.step_amp_deg,
            "step_events": L0.step_events,
            "step_period": L0.step_period,
            "step_cycles": L0.step_cycles,
        },
    }


def compute_joint_cost(
    j: int,
    lag_deg: float,
    ts_ms: float,
    resid_std_deg: float,
    ess_deg: float,
    overshoot_pct: float = 0.0,
) -> Tuple[float, Dict[str, float]]:
    """Weighted sum J_joint (SOP-11 §2.1).  Returns (cost, breakdown).

    Inputs are in metrics_v2 native units (deg / ms / percent), matching the
    corresponding baselines.  Each term is normalised by
    ``max(baseline, norm_floor)`` and saturated at ``TERM_CLIP``
    (devlog 2026-07-31 Q1).
    """
    w = WEIGHTS_JOINT
    terms = np.array([
        lag_deg / max(BASELINE_LAG_DEG[j], NORM_FLOOR_LAG_DEG[j]),
        ts_ms / max(BASELINE_TS_MS[j], NORM_FLOOR_TS_MS[j]),
        resid_std_deg / max(BASELINE_RESID_STD_DEG[j], NORM_FLOOR_RESID_STD_DEG[j]),
        ess_deg / max(BASELINE_ESS_DEG[j], NORM_FLOOR_ESS_DEG[j]),
        overshoot_pct / max(BASELINE_OVERSHOOT_PCT[j], NORM_FLOOR_OVERSHOOT_PCT[j]),
    ])
    terms = np.clip(terms, 0.0, TERM_CLIP)
    cost = float(w @ terms)
    breakdown = {
        "lag": float(w[0] * terms[0]),
        "ts": float(w[1] * terms[1]),
        "resid": float(w[2] * terms[2]),
        "ess": float(w[3] * terms[3]),
        "overshoot": float(w[4] * terms[4]),
    }
    return cost, breakdown
