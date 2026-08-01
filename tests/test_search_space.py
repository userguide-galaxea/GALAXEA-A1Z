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
    from_gains,
    is_kd_degenerate,
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


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
