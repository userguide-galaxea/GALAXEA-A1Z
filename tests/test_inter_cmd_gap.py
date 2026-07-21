"""Offline L0 unit tests for SOP-06 CAN command pacing resolution.

Covers the ``_resolve_inter_cmd_gap_us`` parameter/default precedence and range
check, plus the ``MixedMotorChain`` neutral default — all runnable without a CAN
bus or hardware (no ``can.interface.Bus`` is opened; motor drivers only store
attributes at construction). The gap is hard-coded/parameter-only: the SDK reads
no environment variable, and one test pins that ``A1Z_INTER_CMD_GAP_US`` is
ignored even when set. Run with ``pytest tests/test_inter_cmd_gap.py -v`` or
directly with ``python tests/test_inter_cmd_gap.py``.
"""

import pytest

from a1z.motor_drivers.motor_a_driver import MotorA
from a1z.motor_drivers.motor_b_driver import MotorB, MixedMotorChain
from a1z.robots.get_robot import (
    _DEFAULT_INTER_CMD_GAP_US,
    _MAX_INTER_CMD_GAP_US,
    _resolve_inter_cmd_gap_us,
)

_ENV = "A1Z_INTER_CMD_GAP_US"


def test_default_when_no_param():
    assert _resolve_inter_cmd_gap_us(None) == _DEFAULT_INTER_CMD_GAP_US == 250.0


def test_param_overrides_default():
    assert _resolve_inter_cmd_gap_us(310) == 310.0


def test_param_zero_disables_pacing():
    assert _resolve_inter_cmd_gap_us(0) == 0.0


def test_env_is_ignored(monkeypatch):
    """The SDK reads no environment variable — A1Z_INTER_CMD_GAP_US has no effect."""
    monkeypatch.setenv(_ENV, "42")
    assert _resolve_inter_cmd_gap_us(None) == 250.0
    assert _resolve_inter_cmd_gap_us(310) == 310.0


def test_negative_raises():
    with pytest.raises(ValueError):
        _resolve_inter_cmd_gap_us(-1)


def test_above_max_raises():
    with pytest.raises(ValueError):
        _resolve_inter_cmd_gap_us(600)


def test_boundaries_inclusive():
    assert _resolve_inter_cmd_gap_us(0) == 0.0
    assert _resolve_inter_cmd_gap_us(_MAX_INTER_CMD_GAP_US) == _MAX_INTER_CMD_GAP_US == 500.0


def _make_chain(inter_cmd_gap_s=0.0):
    """Build a MixedMotorChain without touching the bus (bus=None is never used)."""
    motor_a = [MotorA(motor_id=0x01, bus=None)]
    motor_b = [MotorB(motor_id=0x04, bus=None)]
    return MixedMotorChain(
        motor_a_list=motor_a,
        motor_b_list=motor_b,
        motor_a_joint_indices=[0],
        motor_b_joint_indices=[1],
        inter_cmd_gap_s=inter_cmd_gap_s,
    )


def test_chain_default_gap_is_neutral():
    """MixedMotorChain default stays 0.0 — policy lives in the factory (SOP-06 §1.1)."""
    chain = _make_chain()
    assert chain.inter_cmd_gap_s == 0.0


def test_chain_property_reflects_constructor():
    chain = _make_chain(inter_cmd_gap_s=250e-6)
    assert chain.inter_cmd_gap_s == pytest.approx(250e-6)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
