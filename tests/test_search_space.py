"""Offline tests for search_space.py reparameterisation (SOP-11 §1.2).

Pure numpy — no Optuna, no hardware.
Run with ``pytest tests/test_search_space.py -v``.
"""

import math

import numpy as np
import pytest

from a1z.analysis.optimize.cost_spec import (
    DEFAULT_KP, DEFAULT_KD, I_HAT, KD_CAP, KD_MIN,
)
from a1z.analysis.optimize.search_space import (
    _phase_a_kd_range,
    build_optuna_space,
    build_refine_space,
    from_gains,
    is_kd_degenerate,
    phase_b_default_theta,
    refine_default_theta,
    refine_theta_to_gains_6,
    theta_to_gains_6,
    to_gains,
    to_kd,
)


# --- to_kd / to_gains --------------------------------------------------------

@pytest.mark.parametrize("j", range(6))
def test_to_kd_matches_formula(j):
    kp = float(DEFAULT_KP[j])
    zeta = 0.7
    kd = to_kd(kp, zeta, I_HAT[j])
    expected = min(2.0 * zeta * math.sqrt(kp * I_HAT[j]), KD_CAP)
    assert kd == pytest.approx(expected, rel=1e-12)


def test_to_kd_cap_clips():
    kd = to_kd(200.0, 1.2, I_HAT[0])
    assert kd <= KD_CAP


def test_to_gains_returns_pair():
    kp_in = 50.0
    zeta = 0.8
    kp_out, kd_out = to_gains(kp_in, zeta, I_HAT[2])
    assert kp_out == kp_in
    assert kd_out == to_kd(kp_in, zeta, I_HAT[2])


# --- from_gains (inverse) ----------------------------------------------------

@pytest.mark.parametrize("j", range(6))
def test_roundtrip_uncapped(j):
    """from_gains(to_gains(kp, z, I)) ≈ z when kd is below cap."""
    kp = float(DEFAULT_KP[j]) * 0.6
    zeta = 0.5
    _, kd = to_gains(kp, zeta, I_HAT[j])
    if kd < KD_CAP - 0.01:
        recovered = from_gains(kp, kd, I_HAT[j])
        assert recovered == pytest.approx(zeta, rel=1e-9)


def test_roundtrip_capped_yields_lower_zeta():
    """When kd hits the cap, recovered zeta < original zeta."""
    kp = 200.0
    zeta_in = 1.2
    _, kd = to_gains(kp, zeta_in, I_HAT[0])
    assert kd == KD_CAP
    zeta_out = from_gains(kp, kd, I_HAT[0])
    assert zeta_out < zeta_in


# --- theta_to_gains_6 --------------------------------------------------------

def test_theta_to_gains_6_overrides_single_joint():
    j = 3
    kp_j, zeta_j = 45.0, 0.9
    kp_vec, kd_vec = theta_to_gains_6(j, kp_j, zeta_j)
    assert kp_vec.shape == (6,)
    assert kd_vec.shape == (6,)
    assert kp_vec[j] == kp_j
    assert kd_vec[j] == to_kd(kp_j, zeta_j, I_HAT[j])
    for i in range(6):
        if i != j:
            assert kp_vec[i] == DEFAULT_KP[i]


def test_theta_to_gains_6_with_custom_base():
    base_kp = np.ones(6) * 42.0
    base_kd = np.ones(6) * 2.0
    j = 0
    kp_vec, kd_vec = theta_to_gains_6(j, 80.0, 0.6, base_kp=base_kp, base_kd=base_kd)
    assert kp_vec[1] == 42.0
    assert kd_vec[1] == 2.0
    assert kp_vec[j] == 80.0


# --- edge cases ---------------------------------------------------------------

def test_to_kd_zero_kp():
    kd = to_kd(0.0, 1.0, I_HAT[0])
    assert kd == 0.0


def test_from_gains_near_zero_kp():
    zeta = from_gains(1e-15, 0.001, I_HAT[5])
    assert math.isfinite(zeta)


# --- Degenerate joints: direct kd search (devlog 2026-07-31 Q16) ---------

def test_kd_degenerate_joints_are_j5_and_j6():
    """The zeta->kd mapping collapses onto KD_MIN exactly for the
    friction-dominated wrist joints J5/J6 (measured in Q16)."""
    degenerate = {j + 1 for j in range(6) if is_kd_degenerate(j)}
    assert degenerate == {5, 6}


def test_kd_range_for_degenerate_joint():
    lo, hi = _phase_a_kd_range(4)  # J5: kd0=0.5 -> [0.3, max(2.0, 1.2)]
    assert lo == pytest.approx(float(KD_MIN[4]))
    assert hi == pytest.approx(min(max(4 * DEFAULT_KD[4], 1.2), KD_CAP))
    assert hi > 1.0  # manual-tuned kd=1.0 must be reachable
    lo6, hi6 = _phase_a_kd_range(5)  # J6: kd0=4.0 -> capped at KD_CAP
    assert hi6 == pytest.approx(KD_CAP)


def test_build_optuna_space_degenerate_uses_kd():
    space = build_optuna_space(4, phase="A")
    assert set(space.keys()) == {"kp", "kd"}
    space6 = build_optuna_space(5, phase="A")
    assert set(space6.keys()) == {"kp", "kd"}


def test_build_optuna_space_normal_keeps_zeta():
    for j in (0, 1, 2, 3):
        space = build_optuna_space(j, phase="A")
        assert set(space.keys()) == {"kp", "zeta_hat"}


def test_theta_to_gains_6_with_direct_kd():
    kp_vec, kd_vec = theta_to_gains_6(4, 30.0, kd_j=1.0)
    assert kp_vec[4] == 30.0
    assert kd_vec[4] == pytest.approx(1.0)
    # kd_j clamped to [KD_MIN, KD_CAP]
    _, kd_lo = theta_to_gains_6(4, 30.0, kd_j=0.01)
    assert kd_lo[4] == pytest.approx(float(KD_MIN[4]))
    _, kd_hi = theta_to_gains_6(5, 30.0, kd_j=99.0)
    assert kd_hi[5] == pytest.approx(KD_CAP)


# --- E-segment refine space (SOP-11 §6.3, devlog 2026-08-01 E4) -------------

def test_build_refine_space_keys_and_ranges():
    """v12: per-joint parameterisation — non-degenerate joints (J4) get
    (kp, zeta_hat), degenerate joints (J5/J6) get (kp, kd)."""
    space = build_refine_space([4, 5, 6])
    assert set(space.keys()) == {"kp4", "zeta_hat4", "kp5", "kd5", "kp6", "kd6"}
    for j1 in (4, 5, 6):
        j = j1 - 1
        kp_d = space[f"kp{j1}"]
        # kp anchored on the frozen default with 2x headroom (boundary-hit
        # review for the Phase A top-of-space values J4=120 / J5=40)
        assert kp_d.low == pytest.approx(DEFAULT_KP[j] / 2.0)
        assert kp_d.high == pytest.approx(min(2.0 * DEFAULT_KP[j], 200.0))
        assert kp_d.high > DEFAULT_KP[j]      # headroom above the frozen value
        assert kp_d.log is True
    # J4: zeta_hat in the Phase A structural range (oscillation corner
    # zeta<0.4 is absent BY CONSTRUCTION — the v12 fix)
    assert space["zeta_hat4"].low == pytest.approx(0.4)
    assert space["zeta_hat4"].high == pytest.approx(1.2)
    assert space["zeta_hat4"].log is False
    # J5/J6: direct kd, frozen kd reachable + headroom
    for j1 in (5, 6):
        j = j1 - 1
        kd_d = space[f"kd{j1}"]
        assert kd_d.low == pytest.approx(float(KD_MIN[j]))
        assert kd_d.high <= KD_CAP
        assert kd_d.high > DEFAULT_KD[j]
        assert kd_d.log is True


def test_build_refine_space_all_joints_parameterisation():
    """v12 motivation: for inertia-dominated J1–J4 the bare (kp, kd) box
    re-admits the high-kp/low-kd oscillation corner; the zeta_hat form
    excludes it structurally (zeta >= 0.4)."""
    space = build_refine_space([1, 2, 3, 4, 5, 6])
    for j1 in (1, 2, 3, 4):
        assert f"zeta_hat{j1}" in space
        assert f"kd{j1}" not in space
        assert space[f"zeta_hat{j1}"].low == pytest.approx(0.4)
    for j1 in (5, 6):
        assert f"kd{j1}" in space
        assert f"zeta_hat{j1}" not in space


def test_refine_default_theta_is_frozen_defaults_inside_space():
    space = build_refine_space([4, 5, 6])
    theta = refine_default_theta([4, 5, 6])
    assert theta["kp4"] == pytest.approx(float(DEFAULT_KP[3]))
    # J4 frozen gains imply zeta_hat ≈ 1.2 (top of the structural range —
    # the Phase A boundary review proceeds via the kp headroom)
    assert theta["zeta_hat4"] == pytest.approx(1.2, abs=1e-3)
    for j1 in (5, 6):
        assert theta[f"kd{j1}"] == pytest.approx(float(DEFAULT_KD[j1 - 1]))
    for key, val in theta.items():
        d = space[key]
        assert d.low <= val <= d.high, f"{key}={val} outside {d}"


def test_refine_theta_to_gains_6_overrides_subset_only():
    theta = {"kp4": 150.0, "zeta_hat4": 0.8, "kp6": 120.0, "kd6": 2.0}
    kp, kd = refine_theta_to_gains_6(theta, [4, 6])
    assert kp[3] == 150.0
    assert kd[3] == pytest.approx(to_kd(150.0, 0.8, I_HAT[3], kd_min=float(KD_MIN[3])))
    assert kp[5] == 120.0
    assert kd[5] == 2.0
    # non-searched joints keep the frozen defaults
    for i in (0, 1, 2, 4):
        assert kp[i] == DEFAULT_KP[i]
        assert kd[i] == DEFAULT_KD[i]


def test_refine_theta_to_gains_6_kd_clamped():
    theta = {"kp5": 40.0, "kd5": 99.0, "kp6": 100.0, "kd6": 0.0}
    _, kd = refine_theta_to_gains_6(theta, [5, 6])
    assert kd[4] == pytest.approx(KD_CAP)
    assert kd[5] == pytest.approx(float(KD_MIN[5]))


# --- Phase B space (v13, SOP-11 §1.4/§7 — kp frozen after E6) --------------

def test_phase_b_space_kp_frozen_coulomb_only_for_non_integral_joint():
    """J5 (not in PHASE_B_INTEGRAL_JOINTS): coulomb_ff only — no kp, no
    zeta/kd, no integral dims (§1.4 cropping)."""
    from a1z.analysis.optimize.cost_spec import TAU_C_HAT
    space = build_optuna_space(4, phase="B")
    assert set(space.keys()) == {"coulomb_ff"}
    c = space["coulomb_ff"]
    assert c.low == pytest.approx(0.0)
    assert c.high == pytest.approx(2.25 * TAU_C_HAT[4])
    assert c.log is False  # 有物理零点 → 线性（devlog 2026-08-03 约束 3）


def test_phase_b_space_no_integral_dims_after_b1_verdict():
    """v14: B1 G0-ext 6/6 积分对 FAIL（ess 已压进死区 e_db=0.3°，唯一超死区
    的 J3 副作用 +261.7%/+314.6%）——积分维度整体淘汰，全关节仅剩
    coulomb_ff（devlog 2026-08-03「B1 判定归因」）。"""
    for j in range(6):
        space = build_optuna_space(j, phase="B")
        assert "t_wind_s" not in space
        assert "clamp_scale" not in space
        assert "kp" not in space and "zeta_hat" not in space and "kd" not in space


def test_phase_b_default_theta_is_minimal_intervention_inside_space():
    for j in (2, 4):  # J3, J5
        space = build_optuna_space(j, phase="B")
        theta = phase_b_default_theta(j)
        # v14: 种子 = 纯 coulomb=0（积分维度已淘汰）
        assert theta == {"coulomb_ff": 0.0}
        for key, val in theta.items():
            d = space[key]
            assert d.low <= val <= d.high


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
