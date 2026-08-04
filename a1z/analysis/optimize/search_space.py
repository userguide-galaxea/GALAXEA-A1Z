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
    PHASE_B_INTEGRAL_JOINTS,
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
    kd = (base_kd.copy() if base_kd is not None else DEFAULT_KD.copy())
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

    Phase A (SOP-11 §1.3): ``(kp, zeta_hat)``, or ``(kp, kd)`` direct for
    KD_MIN-degenerate joints.  Phase B (v13, devlog 2026-08-03 B2/B3 — PD
    frozen after E6, kp NOT searched): ``coulomb_ff`` always (when the
    joint's τ̂_c gives a nonzero range), integral dims only for
    ``PHASE_B_INTEGRAL_JOINTS`` (§1.4 逐关节裁剪).
    """
    import optuna

    j = int(joint)
    space: Dict = {}

    if phase == "B":
        c_lo, c_hi = _phase_b_coulomb_range(j)
        if c_hi > c_lo + 1e-6:
            space["coulomb_ff"] = optuna.distributions.FloatDistribution(c_lo, c_hi)
        if (j + 1) in PHASE_B_INTEGRAL_JOINTS:
            tw_lo, tw_hi = PHASE_B_T_WIND_RANGE
            space["t_wind_s"] = optuna.distributions.FloatDistribution(tw_lo, tw_hi, log=True)
            cs_lo, cs_hi = PHASE_B_CLAMP_SCALE_RANGE
            space["clamp_scale"] = optuna.distributions.FloatDistribution(cs_lo, cs_hi)
        return space

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

    return space


def phase_b_default_theta(joint: int) -> Dict[str, float]:
    """Phase B 种子 = 空间内最小干预点（devlog 2026-08-03 B2）。

    ``coulomb_ff=0``（无新增前馈）+ 积分最弱档（t_wind 上界 = 最慢爬满、
    clamp_scale 下界 = 最小钳位）。兼作 session 首个 anchor 记录与
    「种子失败即中止」硬门槛（与 E 段种子同构）。
    """
    j = int(joint)
    theta: Dict[str, float] = {}
    c_lo, _ = _phase_b_coulomb_range(j)
    theta["coulomb_ff"] = float(c_lo)
    if (j + 1) in PHASE_B_INTEGRAL_JOINTS:
        theta["t_wind_s"] = float(PHASE_B_T_WIND_RANGE[1])
        theta["clamp_scale"] = float(PHASE_B_CLAMP_SCALE_RANGE[0])
    return theta


# ---------------------------------------------------------------------------
# E-segment refine space — multi-joint joint search (SOP-11 §6.3,
# devlog 2026-08-01 E4).  Parameterisation follows the §6 unified-view
# principle (devlog 2026-08-01): inertia-dominated non-degenerate joints
# (J1–J4) search (kp, zeta_hat) so the "high kp + low kd" oscillation corner
# (zeta < 0.4) is structurally absent; friction-dominated degenerate joints
# (J5/J6) search (kp, kd) directly.  (v12 fix: the v10/v11 uniform
# direct-kd form re-admitted the oscillation corner for J1–J4 — the
# eeRefine-allJoints session's consecutive overshoot violations, implied
# zeta 0.18–0.33, were exactly that.)  kp ranges are anchored on the frozen
# defaults with 2x upward headroom so the Phase A boundary hits (J4 kp=120 /
# J5 kp=40 top of space) get their pending expansion review inside the
# joint space.
# ---------------------------------------------------------------------------
def _refine_kp_range(j: int) -> Tuple[float, float]:
    return (float(DEFAULT_KP[j]) / 2.0, min(2.0 * float(DEFAULT_KP[j]), 200.0))


def _refine_kd_range(j: int) -> Tuple[float, float]:
    hi = min(max(4.0 * float(DEFAULT_KD[j]), 1.2), KD_CAP)
    return (float(KD_MIN[j]), hi)


def build_refine_space(joints1) -> Dict:
    """Optuna distributions for the E-segment joint subset (1-based).

    Param keys are per-joint: ``"kp4", "zeta_hat4", "kp5", "kd5", ...`` so
    one study carries the whole coupled subset.  kp log-scale
    (multiplicative), zeta_hat linear (bounded O(1) ratio), kd log-scale —
    same scale semantics as Phase A (SOP-11 §1.2).
    """
    import optuna

    space: Dict = {}
    for j1 in joints1:
        j = int(j1) - 1
        kp_lo, kp_hi = _refine_kp_range(j)
        space[f"kp{j1}"] = optuna.distributions.FloatDistribution(kp_lo, kp_hi, log=True)
        if is_kd_degenerate(j):
            kd_lo, kd_hi = _refine_kd_range(j)
            space[f"kd{j1}"] = optuna.distributions.FloatDistribution(kd_lo, kd_hi, log=True)
        else:
            z_lo, z_hi = _phase_a_zeta_range()
            space[f"zeta_hat{j1}"] = optuna.distributions.FloatDistribution(z_lo, z_hi)
    return space


def refine_default_theta(joints1) -> Dict[str, float]:
    """The frozen-default point of the refine space — the warm-start seed.

    Degenerate joints carry their frozen kd; non-degenerate joints carry
    the zeta_hat implied by the frozen (kp, kd) pair.
    """
    theta: Dict[str, float] = {}
    for j1 in joints1:
        j = int(j1) - 1
        theta[f"kp{j1}"] = float(DEFAULT_KP[j])
        if is_kd_degenerate(j):
            theta[f"kd{j1}"] = float(DEFAULT_KD[j])
        else:
            theta[f"zeta_hat{j1}"] = from_gains(
                float(DEFAULT_KP[j]), float(DEFAULT_KD[j]), I_HAT[j])
    return theta


def refine_theta_to_gains_6(
    theta: Dict[str, float],
    joints1,
    base_kp: np.ndarray | None = None,
    base_kd: np.ndarray | None = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Full 6-vector gains with every joint of the subset overridden.

    Non-searched joints keep the (frozen) defaults.  Direct kd values are
    clamped to [KD_MIN[j], KD_CAP]; zeta_hat is mapped through the same
    KD_MIN-clamped to_kd as the online application path.
    """
    kp = (base_kp.copy() if base_kp is not None else DEFAULT_KP.copy())
    kd = (base_kd.copy() if base_kd is not None else DEFAULT_KD.copy())
    for j1 in joints1:
        j = int(j1) - 1
        kp[j] = float(theta[f"kp{j1}"])
        if f"kd{j1}" in theta:
            kd[j] = min(max(float(theta[f"kd{j1}"]), KD_MIN[j]), KD_CAP)
        else:
            kd[j] = to_kd(kp[j], float(theta[f"zeta_hat{j1}"]), I_HAT[j],
                          kd_min=KD_MIN[j])
    return kp, kd
