"""Offline tests for friction.py CoulombConfig (SOP-11 §7.2).

Pure numpy — no CAN bus, no hardware.
Run with ``pytest tests/test_friction.py -v``.
"""

import numpy as np
import pytest

from a1z.analysis.optimize.friction import CoulombConfig
from a1z.analysis.optimize.cost_spec import TAU_C_HAT


# --- Construction -------------------------------------------------------------

def test_basic_construction():
    cc = CoulombConfig(tau_c=np.array([0.1, 0.2, 0.3]), qd_eps=0.1)
    assert cc.tau_c.shape == (3,)
    assert cc.qd_eps == 0.1


def test_from_tau_c_hat_default():
    cc = CoulombConfig.from_tau_c_hat(TAU_C_HAT)
    assert cc.tau_c.shape == (6,)
    np.testing.assert_allclose(cc.tau_c, TAU_C_HAT)


def test_from_tau_c_hat_scale():
    cc = CoulombConfig.from_tau_c_hat(TAU_C_HAT, scale=1.5)
    np.testing.assert_allclose(cc.tau_c, TAU_C_HAT * 1.5)


def test_from_tau_c_hat_joints_filter():
    cc = CoulombConfig.from_tau_c_hat(TAU_C_HAT, joints=[1, 6])
    assert cc.tau_c[0] > 0  # J1
    assert cc.tau_c[5] > 0  # J6
    assert cc.tau_c[2] == 0.0  # J3 masked out
    assert cc.tau_c[3] == 0.0  # J4 masked out


def test_enable_mask_zeros_disabled():
    mask = np.array([True, False, True, False, False, False])
    cc = CoulombConfig(tau_c=np.ones(6) * 0.5, enable_mask=mask)
    assert cc.tau_c[0] == 0.5
    assert cc.tau_c[1] == 0.0
    assert cc.tau_c[2] == 0.5
    assert cc.tau_c[3] == 0.0


def test_mask_length_mismatch_raises():
    with pytest.raises(ValueError, match="mismatch"):
        CoulombConfig(tau_c=np.ones(6), enable_mask=np.ones(4, dtype=bool))


def test_negative_qd_eps_raises():
    with pytest.raises(ValueError, match="positive"):
        CoulombConfig(tau_c=np.ones(3), qd_eps=-0.01)


# --- compute_tau --------------------------------------------------------------

def test_compute_tau_zero_velocity():
    cc = CoulombConfig(tau_c=np.array([0.3, 0.5]), qd_eps=0.05)
    tau = cc.compute_tau(np.array([0.0, 0.0]))
    np.testing.assert_allclose(tau, 0.0, atol=1e-15)


def test_compute_tau_large_positive_velocity():
    cc = CoulombConfig(tau_c=np.array([0.3, 0.5]), qd_eps=0.05)
    tau = cc.compute_tau(np.array([10.0, 10.0]))
    np.testing.assert_allclose(tau, [0.3, 0.5], atol=1e-6)


def test_compute_tau_large_negative_velocity():
    cc = CoulombConfig(tau_c=np.array([0.3, 0.5]), qd_eps=0.05)
    tau = cc.compute_tau(np.array([-10.0, -10.0]))
    np.testing.assert_allclose(tau, [-0.3, -0.5], atol=1e-6)


def test_compute_tau_antisymmetric():
    cc = CoulombConfig(tau_c=np.array([0.4]), qd_eps=0.1)
    v = 0.05
    t_pos = cc.compute_tau(np.array([v]))[0]
    t_neg = cc.compute_tau(np.array([-v]))[0]
    assert t_pos == pytest.approx(-t_neg, abs=1e-15)


def test_compute_tau_smooth_near_zero():
    """tanh smoothing: derivative at v=0 is tau_c / qd_eps (finite, not discontinuous)."""
    cc = CoulombConfig(tau_c=np.array([0.3]), qd_eps=0.05)
    eps = 1e-6
    t1 = cc.compute_tau(np.array([eps]))[0]
    t2 = cc.compute_tau(np.array([-eps]))[0]
    numerical_deriv = (t1 - t2) / (2 * eps)
    expected_deriv = 0.3 / 0.05  # tau_c / qd_eps
    assert numerical_deriv == pytest.approx(expected_deriv, rel=1e-4)


# --- as_info ------------------------------------------------------------------

def test_as_info_keys():
    cc = CoulombConfig.from_tau_c_hat(TAU_C_HAT)
    info = cc.as_info()
    assert "tau_c" in info
    assert "qd_eps" in info
    assert "enable_mask" in info
    assert len(info["tau_c"]) == 6


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
