"""Offline L0 unit tests for the error-integral feedforward (SOP-09 §3).

Pure numpy — no CAN bus, no hardware. Covers IntegralConfig.from_level's ki
calibration + enable-mask, and JointErrorIntegrator's four anti-hunting pieces
(clamp / leak / dead-band / schedule). Run with
``pytest tests/test_integrator.py -v`` or ``python tests/test_integrator.py``.
"""

import numpy as np
import pytest

from a1z.robots.integrator import (
    E_TYP_RAD,
    LEVELS,
    IntegralConfig,
    JointErrorIntegrator,
)

DT = 1.0 / 250.0
TAU_C = np.array([np.nan] * 5 + [0.13])  # only J6 calibrated (SOP-09 _TAU_C_HAT)


# --- IntegralConfig.from_level ---------------------------------------------
def test_k0_is_all_zero():
    c = IntegralConfig.from_level("K0", TAU_C)
    assert np.allclose(c.ki, 0.0)
    assert np.allclose(c.tau_i_max, 0.0)  # no live clamp next to a dead ki
    assert c.as_info()["enable_mask"] == [False] * 6


def test_uncalibrated_joints_disabled():
    """NaN τ̂_c joints get ki=0 and tau_i_max=0 regardless of level."""
    c = IntegralConfig.from_level("K2", TAU_C)
    assert np.allclose(c.ki[:5], 0.0) and c.ki[5] > 0.0
    assert np.allclose(c.tau_i_max[:5], 0.0)
    assert abs(c.tau_i_max[5] - 1.2 * 0.13) < 1e-12


def test_joints_filter_restricts_enable_mask():
    c = IntegralConfig.from_level("K2", np.full(6, 0.1), joints=[6])
    assert c.as_info()["enable_mask"] == [False, False, False, False, False, True]


def test_ki_calibration_matches_t_wind():
    """ki = tau_i_max / (E_TYP · t_wind), per level (SOP-09 §3 table)."""
    for level in ("K1", "K2", "K3"):
        c = IntegralConfig.from_level(level, TAU_C, joints=[6])
        expected = (1.2 * 0.13) / (E_TYP_RAD * LEVELS[level])
        assert c.ki[5] == pytest.approx(expected, rel=1e-9)


def test_unknown_level_raises():
    with pytest.raises(ValueError):
        IntegralConfig.from_level("K9", TAU_C)


def test_post_init_rejects_nonfinite_clamp():
    with pytest.raises(ValueError):
        IntegralConfig(ki=np.zeros(6), tau_i_max=np.full(6, np.nan),
                       e_db_rad=np.zeros(6))


# --- JointErrorIntegrator ---------------------------------------------------
def _integrator(level="K2", **ov):
    return JointErrorIntegrator(
        IntegralConfig.from_level(level, TAU_C, joints=[6], **ov), DT)


def test_k0_integrator_emits_zeros():
    g = JointErrorIntegrator(IntegralConfig.from_level("K0", TAU_C), DT)
    out = np.zeros(6)
    for _ in range(100):
        out = g.step(np.full(6, 0.02), np.zeros(6))
    assert np.allclose(out, 0.0)


def test_leak_coefficient():
    g = _integrator(t_leak_s=1.0)
    assert g._lam == pytest.approx(1.0 - DT / 1.0)


def test_windup_saturates_at_clamp():
    g = _integrator()
    e = np.zeros(6); e[5] = np.deg2rad(0.5)  # above the 0.3° dead-band
    out = np.zeros(6)
    for _ in range(4000):
        out = g.step(e, np.zeros(6))
    assert out[5] == pytest.approx(g.cfg.tau_i_max[5], rel=1e-6)


def test_dead_band_blocks_accumulation():
    g = _integrator()
    e = np.zeros(6); e[5] = np.deg2rad(0.2)  # below 0.3° dead-band
    for _ in range(500):
        out = g.step(e, np.zeros(6))
    assert abs(out[5]) < 1e-12


def test_freeze_blocks_accumulation():
    g = _integrator(qd_freeze=0.15)
    e = np.zeros(6); e[5] = np.deg2rad(0.5)
    qd = np.zeros(6); qd[5] = 0.2  # above freeze threshold
    for _ in range(500):
        out = g.step(e, qd)
    assert abs(out[5]) < 1e-12


def test_reversal_halves_accumulator():
    g = _integrator()
    e = np.zeros(6); e[5] = np.deg2rad(0.5)
    qd_pos = np.zeros(6); qd_pos[5] = 0.05  # below freeze, positive
    for _ in range(200):
        before = g.step(e, qd_pos)
    after = g.step(e, np.array([0, 0, 0, 0, 0, -0.05]))  # sign flip
    # halved (×0.5) then one leak+integrate tick → strictly below `before`.
    assert after[5] < before[5]


def test_reset_zeroes_state():
    g = _integrator()
    e = np.zeros(6); e[5] = np.deg2rad(0.5)
    for _ in range(50):
        g.step(e, np.zeros(6))
    assert abs(g.tau_i[5]) > 0
    g.reset()
    assert np.allclose(g.tau_i, 0.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
