"""Offline tests for cost_spec.py cost function and constants (SOP-11 §1–§2).

Pure numpy — no CAN bus, no hardware.
Run with ``pytest tests/test_cost_spec.py -v``.
"""

import math

import numpy as np
import pytest

from a1z.analysis.optimize.cost_spec import (
    BASELINE_ESS_DEG,
    BASELINE_LAG_DEG,
    BASELINE_OVERSHOOT_PCT,
    BASELINE_RESID_STD_DEG,
    BASELINE_TS_MS,
    COST_SPEC_VERSION,
    DEFAULT_KD,
    DEFAULT_KP,
    I_HAT,
    KD_CAP,
    L0,
    NORM_FLOOR_ESS_DEG,
    NORM_FLOOR_K,
    NORM_FLOOR_LAG_DEG,
    NORM_FLOOR_OVERSHOOT_PCT,
    NORM_FLOOR_RESID_STD_DEG,
    NORM_FLOOR_TS_MS,
    PENALTY_COST,
    TAU_C_HAT,
    TERM_CLIP,
    TORQUE_CLIP,
    WEIGHTS_JOINT,
    _phase_a_kp_range,
    _phase_a_zeta_range,
    _phase_b_coulomb_range,
    compute_joint_cost,
    cost_spec_snapshot,
    violation_surrogate,
)


# --- Constants sanity ---------------------------------------------------------

def test_weights_sum_to_one():
    assert float(np.sum(WEIGHTS_JOINT)) == pytest.approx(1.0)


def test_arrays_are_length_6():
    for arr in (DEFAULT_KP, DEFAULT_KD, TAU_C_HAT, TORQUE_CLIP, I_HAT,
                BASELINE_LAG_DEG, BASELINE_TS_MS, BASELINE_RESID_STD_DEG,
                BASELINE_ESS_DEG, BASELINE_OVERSHOOT_PCT,
                NORM_FLOOR_LAG_DEG, NORM_FLOOR_TS_MS, NORM_FLOOR_RESID_STD_DEG,
                NORM_FLOOR_ESS_DEG, NORM_FLOOR_OVERSHOOT_PCT):
        assert arr.shape == (6,), f"Expected (6,), got {arr.shape}"


def test_norm_floors_positive_and_below_baselines():
    """Floors must be positive (no division blow-up) and below the current
    baselines (so the baseline-normalised cost at default tuning stays 1.0)."""
    for base, floor in (
        (BASELINE_LAG_DEG, NORM_FLOOR_LAG_DEG),
        (BASELINE_TS_MS, NORM_FLOOR_TS_MS),
        (BASELINE_RESID_STD_DEG, NORM_FLOOR_RESID_STD_DEG),
        (BASELINE_ESS_DEG, NORM_FLOOR_ESS_DEG),
        (BASELINE_OVERSHOOT_PCT, NORM_FLOOR_OVERSHOOT_PCT),
    ):
        assert np.all(floor > 0)
        assert np.all(floor < base)


def test_i_hat_positive_finite():
    assert np.all(I_HAT > 0)
    assert np.all(np.isfinite(I_HAT))


def test_i_hat_decreasing_trend():
    """Proximal joints have larger equivalent inertia than distal ones."""
    assert I_HAT[0] > I_HAT[4]
    assert I_HAT[1] > I_HAT[5]


def test_penalty_cost_is_large():
    assert PENALTY_COST > 100


def test_violation_surrogate_is_worst_feasible():
    """Violated trials are told the worst observed feasible cost (v7, Q11):
    keeps the objective GP on scale while feasibility travels via the
    constraint channel.  Values >= PENALTY_COST are not feasible
    references; PENALTY_COST when no reference exists."""
    assert violation_surrogate([1.2, 1.5]) == pytest.approx(1.5)
    assert violation_surrogate([1.0, 1e9]) == pytest.approx(1.0)
    assert violation_surrogate([2e4]) == PENALTY_COST   # capped
    assert violation_surrogate([]) == PENALTY_COST      # no reference
    assert violation_surrogate([None, float("nan")]) == PENALTY_COST


# --- Phase A search space -----------------------------------------------------

@pytest.mark.parametrize("j", range(6))
def test_phase_a_kp_range_contains_default(j):
    lo, hi = _phase_a_kp_range(j)
    assert lo <= DEFAULT_KP[j] <= hi


@pytest.mark.parametrize("j", range(6))
def test_phase_a_kp_range_bounded(j):
    lo, hi = _phase_a_kp_range(j)
    assert lo > 0
    assert hi <= 200.0


def test_phase_a_zeta_range():
    lo, hi = _phase_a_zeta_range()
    assert lo == 0.4
    assert hi == 1.2


@pytest.mark.parametrize("j", range(6))
def test_phase_b_coulomb_range(j):
    lo, hi = _phase_b_coulomb_range(j)
    assert lo == 0.0
    assert hi > 0
    assert hi == pytest.approx(1.5 * 1.5 * TAU_C_HAT[j])


# --- compute_joint_cost -------------------------------------------------------

def test_baseline_inputs_give_cost_one():
    """When all metrics match baselines, cost should be sum(weights) = 1.0."""
    for j in range(6):
        cost, bd = compute_joint_cost(
            j,
            float(BASELINE_LAG_DEG[j]),
            float(BASELINE_TS_MS[j]),
            float(BASELINE_RESID_STD_DEG[j]),
            float(BASELINE_ESS_DEG[j]),
            float(BASELINE_OVERSHOOT_PCT[j]),
        )
        assert cost == pytest.approx(1.0, rel=1e-9)


def test_worse_metrics_give_higher_cost():
    cost_good, _ = compute_joint_cost(0, 0.020, 0.3, 0.3, 0.2, 2.0)
    cost_bad, _ = compute_joint_cost(0, 0.100, 1.0, 2.0, 1.0, 25.0)
    assert cost_bad > cost_good


def test_cost_breakdown_keys():
    _, bd = compute_joint_cost(0, 0.04, 0.5, 0.01, 0.005, 1.0)
    assert set(bd.keys()) == {"lag", "ts", "resid", "ess", "overshoot"}


def test_cost_breakdown_sums_to_cost():
    cost, bd = compute_joint_cost(3, 0.035, 0.4, 0.008, 0.004, 0.5)
    assert sum(bd.values()) == pytest.approx(cost, rel=1e-9)


def test_cost_zero_inputs():
    cost, _ = compute_joint_cost(5, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert cost == 0.0


def test_cost_finite_for_all_joints():
    for j in range(6):
        cost, _ = compute_joint_cost(j, 0.05, 0.6, 0.01, 0.005, 1.0)
        assert math.isfinite(cost)
        assert cost > 0


def test_overshoot_increases_cost():
    base, _ = compute_joint_cost(0, 0.04, 0.5, 0.01, 0.005, 0.0)
    with_os, _ = compute_joint_cost(0, 0.04, 0.5, 0.01, 0.005, 20.0)
    assert with_os > base


# --- Normalisation floors & term clip (devlog 2026-07-31 Q1) ------------------

def test_zero_baseline_falls_back_to_floor(monkeypatch):
    """A zero (or tiny) baseline must not blow up the ratio: the denominator
    silently becomes the noise floor, i.e. an absolute scale."""
    import a1z.analysis.optimize.cost_spec as cs

    monkeypatch.setattr(cs, "BASELINE_OVERSHOOT_PCT", np.zeros(6))
    # overshoot = 2x floor -> term = 2, contribution = w5 * 2
    cost, bd = compute_joint_cost(
        0, 0.0, 0.0, 0.0, 0.0, 2.0 * float(NORM_FLOOR_OVERSHOOT_PCT[0])
    )
    assert bd["overshoot"] == pytest.approx(WEIGHTS_JOINT[4] * 2.0)
    assert math.isfinite(cost)


def test_term_clip_bounds_single_term():
    """No single feasible-region term may exceed TERM_CLIP, so one extreme
    metric cannot drown the weighted sum (dangerous regions pay PENALTY_COST
    via the watchdog instead)."""
    _, bd = compute_joint_cost(0, 1e6, 0.0, 0.0, 0.0, 0.0)
    assert bd["lag"] == pytest.approx(WEIGHTS_JOINT[0] * TERM_CLIP)
    cost, bd = compute_joint_cost(5, 1e6, 1e6, 1e6, 1e6, 1e6)
    assert cost == pytest.approx(float(np.sum(WEIGHTS_JOINT)) * TERM_CLIP)
    for v in bd.values():
        assert v <= max(WEIGHTS_JOINT) * TERM_CLIP


# --- cost_spec_snapshot -------------------------------------------------------

def test_snapshot_has_version():
    snap = cost_spec_snapshot()
    assert snap["version"] == COST_SPEC_VERSION


def test_snapshot_round_trips():
    snap = cost_spec_snapshot()
    assert snap["weights_joint"] == WEIGHTS_JOINT.tolist()
    assert snap["I_hat"] == I_HAT.tolist()
    assert snap["kd_cap"] == KD_CAP
    assert "l0" in snap
    assert snap["norm_floors"]["overshoot_pct"] == NORM_FLOOR_OVERSHOOT_PCT.tolist()
    assert snap["norm_floors"]["k"] == NORM_FLOOR_K
    assert snap["term_clip"] == TERM_CLIP
    assert "violation_surrogate" in snap


# --- L0 preset ----------------------------------------------------------------

def test_l0_preset_values():
    assert L0.tri_vel_ff is True
    assert L0.tri_period > 0
    assert L0.step_amp_deg > 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
