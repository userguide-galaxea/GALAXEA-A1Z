"""(kp, zeta_hat) <-> kd reparameterisation + Optuna space builder (SOP-11 §1.2).

Pure math — no hardware, no Optuna at import time (Optuna is imported lazily
inside ``build_optuna_space`` so offline tests work without the dependency).
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

from a1z.analysis.optimize.cost_spec import (
    DEFAULT_KD,
    DEFAULT_KP,
    I_HAT,
    KD_CAP,
    KD_MIN,
    PHASE_B_CLAMP_SCALE_RANGE,
    PHASE_B_T_LEAK_RANGE,
    PHASE_B_T_WIND_RANGE,
    TAU_C_HAT,
    _phase_a_kp_range,
    _phase_a_zeta_range,
    _phase_b_coulomb_range,
)


# ---------------------------------------------------------------------------
# Degenerate-joint detection (devlog 2026-07-31 Q16)
# ---------------------------------------------------------------------------
def is_kd_degenerate(j: int) -> bool:
    """True if the (kp, zeta_hat) -> kd mapping collapses onto KD_MIN for the
    whole Phase-A space: even the space corner (kp_hi, zeta_hi) maps below
    KD_MIN[j].  On such joints (friction-dominated small-inertia wrists —
    measured J5 and J6) the nominal 2-D search degenerates to 1-D kp with
    kd pinned at KD_MIN, and reasonable damping is unreachable.  These
    joints search (kp, kd) directly instead.
    """
    _, kp_hi = _phase_a_kp_range(j)
    _, zeta_hi = _phase_a_zeta_range()
    return 2.0 * zeta_hi * math.sqrt(kp_hi * I_HAT[j]) < KD_MIN[j]


def _phase_a_kd_range(j: int) -> Tuple[float, float]:
    """Direct-kd search range for degenerate joints:
    [KD_MIN[j], min(max(4*kd0_j, 1.2), KD_CAP)]."""
    hi = min(max(4.0 * float(DEFAULT_KD[j]), 1.2), KD_CAP)
    return (float(KD_MIN[j]), hi)


# ---------------------------------------------------------------------------
# Core mapping  (SOP-11 §1.2)
# ---------------------------------------------------------------------------
def to_kd(kp: float, zeta_hat: float, I_hat_j: float,
          kd_cap: float = KD_CAP, kd_min: float = 0.0) -> float:
    """kd = clip(2 * zeta_hat * sqrt(kp * I_hat_j), kd_min, kd_cap)."""
    kd = 2.0 * zeta_hat * math.sqrt(max(kp * I_hat_j, 0.0))
    return min(max(kd, kd_min), kd_cap)


def to_gains(
    kp: float,
    zeta_hat: float,
    I_hat_j: float,
    kd_cap: float = KD_CAP,
    kd_min: float = 0.0,
) -> Tuple[float, float]:
    """Return (kp, kd) from the reparameterised search coordinates."""
    return kp, to_kd(kp, zeta_hat, I_hat_j, kd_cap, kd_min)


def from_gains(
    kp: float,
    kd: float,
    I_hat_j: float,
) -> float:
    """Inverse: recover zeta_hat from (kp, kd).  Used for warm-start / anchor."""
    denom = 2.0 * math.sqrt(max(kp * I_hat_j, 1e-30))
    return kd / denom


# ---------------------------------------------------------------------------
# Full 6-vector helpers
# ---------------------------------------------------------------------------
def theta_to_gains_6(
    j: int,
    kp_j: float,
    zeta_hat_j: float | None = None,
    base_kp: np.ndarray | None = None,
    base_kd: np.ndarray | None = None,
    kd_j: float | None = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build full 6-vector gains with only joint ``j`` (0-based) overridden.

    Pass either ``zeta_hat_j`` (reparameterised joints) or ``kd_j``
    (degenerate joints searching kd directly, Q16); ``kd_j`` wins when
    both are given and is clamped to [KD_MIN[j], KD_CAP].
    """
    kp = (base_kp.copy() if base_kp is not None else DEFAULT_KP.copy())
    kd = (base_kd.copy() if base_kd is not None else np.array([
        to_kd(DEFAULT_KP[i], from_gains(DEFAULT_KP[i], float(DEFAULT_KP[i]) * 0.0 + float(
            np.array([4.9, 4.5, 5.0, 2.0, 0.5, 4.0])[i]), I_HAT[i]), I_HAT[i])
        for i in range(6)]))
    if base_kd is None:
        kd = np.array([4.9, 4.5, 5.0, 2.0, 0.5, 4.0])
    kp[j] = kp_j
    if kd_j is not None:
        kd[j] = min(max(float(kd_j), KD_MIN[j]), KD_CAP)
    else:
        kd[j] = to_kd(kp_j, zeta_hat_j, I_HAT[j], kd_min=KD_MIN[j])
    return kp, kd


# ---------------------------------------------------------------------------
# Optuna search-space builder  (lazy import)
# ---------------------------------------------------------------------------
def build_optuna_space(
    joint: int,
    phase: str = "A",
) -> Dict:
    """Return dict of Optuna distribution kwargs for ``study.ask()``.

    ``joint`` is 0-based.  ``phase`` is ``"A"`` or ``"B"``.
    """
    import optuna

    j = int(joint)
    space: Dict = {}

    kp_lo, kp_hi = _phase_a_kp_range(j)
    space["kp"] = optuna.distributions.FloatDistribution(kp_lo, kp_hi, log=True)

    if is_kd_degenerate(j):
        # Friction-dominated joint (Q16): the zeta_hat mapping would pin
        # kd at KD_MIN for the whole space — search kd directly instead.
        kd_lo, kd_hi = _phase_a_kd_range(j)
        space["kd"] = optuna.distributions.FloatDistribution(kd_lo, kd_hi, log=True)
    else:
        zeta_lo, zeta_hi = _phase_a_zeta_range()
        space["zeta_hat"] = optuna.distributions.FloatDistribution(zeta_lo, zeta_hi)

    if phase == "B":
        c_lo, c_hi = _phase_b_coulomb_range(j)
        if c_hi > c_lo + 1e-6:
            space["coulomb_ff"] = optuna.distributions.FloatDistribution(c_lo, c_hi)

        tw_lo, tw_hi = PHASE_B_T_WIND_RANGE
        space["t_wind_s"] = optuna.distributions.FloatDistribution(tw_lo, tw_hi, log=True)

        cs_lo, cs_hi = PHASE_B_CLAMP_SCALE_RANGE
        space["clamp_scale"] = optuna.distributions.FloatDistribution(cs_lo, cs_hi)

        tl_lo, tl_hi = PHASE_B_T_LEAK_RANGE
        space["t_leak_s"] = optuna.distributions.FloatDistribution(tl_lo, tl_hi, log=True)

    return space
