"""Offline tests for watchdog.py three-layer safety (SOP-11 §4).

Pure numpy — no CAN bus, no hardware.
Run with ``pytest tests/test_watchdog.py -v``.
"""

import math
import tempfile
from pathlib import Path

import numpy as np
import pytest

from a1z.analysis.optimize.watchdog import (
    AnchorMonitor,
    MultiTickWatchdog,
    TickWatchdog,
    TrialChecker,
    TrialVerdict,
    WatchdogViolation,
)


# --- MultiTickWatchdog (E-segment EE leg fan-out, SOP-11 §12.1) ---------------

def test_multi_ok_when_all_children_ok():
    mwd = MultiTickWatchdog([TickWatchdog(j) for j in range(6)])
    ok, reason = mwd.check(0.0, _zeros6(), _zeros6(), _zeros6())
    assert ok is True
    assert reason == ""
    assert mwd.tripped is False


def test_multi_trips_on_first_child_violation():
    mwd = MultiTickWatchdog([TickWatchdog(j, theta_pos_deg=10.0) for j in range(6)])
    resp = _zeros6()
    resp[3] = math.radians(20.0)   # J4 diverges
    ok, reason = mwd.check(0.0, _zeros6(), resp, _zeros6())
    assert ok is False
    assert mwd.tripped is True
    assert reason.startswith("J4:")
    assert "pos_diverge" in reason


def test_multi_reset_clears_children():
    mwd = MultiTickWatchdog([TickWatchdog(j, theta_pos_deg=10.0) for j in range(6)])
    resp = _zeros6()
    resp[5] = math.radians(30.0)
    mwd.check(0.0, _zeros6(), resp, _zeros6())
    assert mwd.tripped is True
    mwd.reset()
    assert mwd.tripped is False
    assert mwd.reason == ""
    assert all(not wd.tripped for wd in mwd._wds)
    ok, _ = mwd.check(0.0, _zeros6(), _zeros6(), _zeros6())
    assert ok is True


# --- TickWatchdog -------------------------------------------------------------

def _make_wd(joint=5, **kw):
    return TickWatchdog(joint, **kw)


def _zeros6():
    return np.zeros(6)


def test_tick_normal_passes():
    wd = _make_wd()
    ref = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.1])
    resp = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.105])
    eff = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    ok, reason = wd.check(0.0, ref, resp, eff)
    assert ok is True
    assert reason == ""
    assert wd.tripped is False


def test_tick_pos_divergence_trips():
    wd = _make_wd(theta_pos_deg=10.0)
    ref = _zeros6()
    resp = _zeros6()
    resp[5] = math.radians(15.0)
    ok, reason = wd.check(0.0, ref, resp, _zeros6())
    assert ok is False
    assert "pos_diverge" in reason
    assert wd.tripped is True


def test_tick_eff_spike_trips():
    wd = _make_wd()
    ref = _zeros6()
    resp = _zeros6()
    eff = _zeros6()
    eff[5] = 100.0  # well above 0.9 * TORQUE_CLIP[5]=10.0
    ok, reason = wd.check(0.0, ref, resp, eff)
    assert ok is False
    assert "eff_spike" in reason


def test_tick_hf_osc_trips():
    wd = _make_wd(hf_window=10)
    wd.set_hf_baseline(0.01)
    ref = _zeros6()
    resp = _zeros6()
    for i in range(30):
        eff = _zeros6()
        eff[5] = 2.0 * ((-1) ** i)  # rapid alternation → high diff-RMS
        ok, reason = wd.check(float(i), ref, resp, eff)
        if not ok:
            assert "hf_osc" in reason
            return
    pytest.fail("HF oscillation watchdog did not trip")


def test_tick_hf_no_baseline_skips():
    """Without set_hf_baseline, the HF check is silently skipped."""
    wd = _make_wd(hf_window=5)
    ref = _zeros6()
    resp = _zeros6()
    for i in range(20):
        eff = _zeros6()
        eff[5] = 5.0 * ((-1) ** i)
        ok, _ = wd.check(float(i), ref, resp, eff)
        assert ok is True


def test_tick_hf_hysteresis_single_window_no_trip():
    """Q12-D: fewer than N consecutive over-threshold evaluations must NOT
    trip; N consecutive must; a quiet stretch flushes the window and resets
    the counter."""
    wd = _make_wd(hf_window=10, hf_confirm_ticks=3)
    wd.set_hf_baseline(0.01)
    ref = _zeros6()
    resp = _zeros6()

    def drive(n, amp, t0=0.0):
        last = None
        for k in range(n):
            eff = _zeros6()
            eff[5] = amp * ((-1) ** k)
            last = wd.check(t0 + float(k), ref, resp, eff)
        return last

    # 2 consecutive over-threshold evaluations → no trip yet
    drive(12, 2.0)  # 10 ticks fill the window; evals #1..#2 over threshold
    assert wd._hf_over_ticks == 2
    assert wd.tripped is False
    # 3rd consecutive over-threshold evaluation → trip
    drive(1, 2.0)
    assert wd.tripped is True
    assert "hf_osc" in wd.reason

    # Fresh watchdog: 1 marginally over-threshold eval (RMS just above
    # theta), then quiet ticks pull the window RMS back below within one
    # evaluation → counter resets, no trip.
    wd2 = _make_wd(hf_window=10, hf_confirm_ticks=3)
    wd2.set_hf_baseline(0.01)  # theta = 4.0 * 0.01 = 0.04
    for k in range(11):  # window full + 1st over-threshold eval (RMS≈0.042)
        eff = _zeros6()
        eff[5] = 0.021 * ((-1) ** k)
        wd2.check(float(k), ref, resp, eff)
    assert wd2._hf_over_ticks == 1
    for k in range(3):  # quiet: RMS drops below theta on the first eval
        wd2.check(20.0 + float(k), ref, resp, _zeros6())
    assert wd2._hf_over_ticks == 0
    assert wd2.tripped is False


def test_tick_reset_clears_state():
    wd = _make_wd(theta_pos_deg=10.0)
    ref = _zeros6()
    resp = _zeros6()
    resp[5] = math.radians(15.0)
    wd.check(0.0, ref, resp, _zeros6())
    assert wd.tripped is True
    wd.reset()
    assert wd.tripped is False
    assert wd.reason == ""


def test_tick_hf_step_edge_exempt():
    """2026-08-01 v9: a square-edge torque transient must NOT trip hf_osc;
    oscillation sustained past the exemption window still must."""
    dt = 0.01

    def fresh():
        wd = _make_wd(hf_window=10, hf_confirm_ticks=3,
                      hf_step_exempt_deg=1.0, hf_step_exempt_s=0.5)
        wd.set_hf_baseline(0.01)  # theta_hf = 4.0 * 0.01 = 0.04
        ref = _zeros6()
        resp = _zeros6()
        for k in range(12):  # fill the window quietly
            wd.check(k * dt, ref, resp, _zeros6())
        ref[5] = math.radians(2.0)  # square edge at t_edge
        return wd, ref, resp, 12 * dt

    # Case 1: hard ringing for 0.2 s after the edge, then quiet → no trip.
    wd, ref, resp, t_edge = fresh()
    tripped = False
    for k in range(20):
        eff = _zeros6()
        eff[5] = 2.0 * ((-1) ** k)
        ok, _ = wd.check(t_edge + k * dt, ref, resp, eff)
        tripped |= not ok
    for k in range(60):  # quiet, well past the 0.5 s exemption
        ok, _ = wd.check(t_edge + (20 + k) * dt, ref, resp, _zeros6())
        tripped |= not ok
    assert not tripped

    # Case 2: ringing persists past the exemption → must trip.
    wd, ref, resp, t_edge = fresh()
    for k in range(200):
        eff = _zeros6()
        eff[5] = 2.0 * ((-1) ** k)
        ok, _ = wd.check(t_edge + k * dt, ref, resp, eff)
        if not ok:
            assert "hf_osc" in wd.reason
            break
    else:
        pytest.fail("sustained post-edge oscillation did not trip hf_osc")


def test_tick_hf_step_exempt_disabled_legacy():
    """hf_step_exempt_deg=None restores legacy behaviour: an edge transient
    trips hf_osc immediately (3 consecutive confirms)."""
    wd = _make_wd(hf_window=10, hf_confirm_ticks=3, hf_step_exempt_deg=None)
    wd.set_hf_baseline(0.01)
    ref = _zeros6()
    resp = _zeros6()
    dt = 0.01
    for k in range(12):
        wd.check(k * dt, ref, resp, _zeros6())
    ref[5] = math.radians(2.0)
    for k in range(30):
        eff = _zeros6()
        eff[5] = 2.0 * ((-1) ** k)
        ok, _ = wd.check(0.12 + k * dt, ref, resp, eff)
        if not ok:
            assert "hf_osc" in wd.reason
            return
    pytest.fail("legacy (exemption disabled) hf_osc did not trip")


def test_as_violation_returns_exception():
    wd = _make_wd(theta_pos_deg=5.0)
    ref = _zeros6()
    resp = _zeros6()
    resp[5] = math.radians(10.0)
    wd.check(1.5, ref, resp, _zeros6())
    v = wd.as_violation()
    assert isinstance(v, WatchdogViolation)
    assert "pos_diverge" in v.reason


def test_tick_vel_spike_trips():
    """Response velocity much larger than reference velocity trips vel_spike."""
    wd = _make_wd(joint=5, theta_vel_deg_s=1.0, kin_window=5)
    ref = _zeros6()
    resp = _zeros6()
    eff = _zeros6()
    dt = 0.01
    for i in range(20):
        t = i * dt
        # reference stationary, response sweeping at ~5 deg/s on J6
        ref[5] = 0.0
        resp[5] = math.radians(5.0 * t)
        ok, reason = wd.check(t, ref, resp, eff)
        if not ok:
            assert "vel_spike" in reason
            return
    pytest.fail("vel_spike watchdog did not trip")


def test_tick_acc_spike_trips():
    """Sharp response acceleration at a turnaround trips acc_spike."""
    wd = _make_wd(joint=5, theta_acc_deg_s2=100.0, kin_window=5)
    ref = _zeros6()
    resp = _zeros6()
    eff = _zeros6()
    dt = 0.01
    for i in range(40):
        t = i * dt
        # reference: slow ramp; response: sudden direction reversal
        ref[5] = math.radians(1.0 * t)
        if i < 20:
            resp[5] = math.radians(5.0 * t)
        else:
            resp[5] = math.radians(-5.0 * (t - 0.20) + 1.0)
        ok, reason = wd.check(t, ref, resp, eff)
        if not ok:
            assert "acc_spike" in reason
            return
    pytest.fail("acc_spike watchdog did not trip")


def test_tick_kin_disabled_by_default():
    """Without theta_vel/theta_acc, extreme kinematics are ignored."""
    wd = _make_wd(joint=5)
    ref = _zeros6()
    resp = _zeros6()
    eff = _zeros6()
    for i in range(20):
        t = i * 0.01
        ref[5] = 0.0
        resp[5] = math.radians(50.0 * t)  # very fast
        ok, _ = wd.check(t, ref, resp, eff)
        assert ok is True


def test_tick_kin_reset_clears_state():
    wd = _make_wd(joint=5, theta_vel_deg_s=1.0, kin_window=5)
    ref = _zeros6()
    resp = _zeros6()
    eff = _zeros6()
    dt = 0.01
    for i in range(10):
        t = i * dt
        resp[5] = math.radians(5.0 * t)
        wd.check(t, ref, resp, eff)
    assert wd.tripped is True
    assert "vel_spike" in wd.reason
    wd.reset()
    assert wd.tripped is False
    assert wd.reason == ""


def test_tick_vel_abs_trips():
    """Instantaneous absolute response velocity above the hardware-margin limit."""
    wd = _make_wd(joint=5, theta_vel_abs_deg_s=100.0)
    ref = _zeros6()
    resp = _zeros6()
    eff = _zeros6()
    # measured velocity on J6 = 200 deg/s, well above 100 deg/s threshold
    vel = _zeros6()
    vel[5] = math.radians(200.0)
    ok, reason = wd.check(0.0, ref, resp, eff, vel=vel)
    assert ok is False
    assert "vel_abs" in reason


def test_tick_vel_abs_default_from_factory():
    """make_tick_watchdog supplies a default absolute-velocity guard."""
    from a1z.analysis.optimize.watchdog import make_tick_watchdog
    wd = make_tick_watchdog(5)
    assert wd.theta_vel_abs is not None
    assert wd.theta_vel_abs > 0


# --- TrialChecker -------------------------------------------------------------

def test_trial_normal_passes():
    tc = TrialChecker()
    v = tc.check({"overshoot_max_pct": 10.0, "jump_p95_star": 0.5}, None, False)
    assert v.ok is True
    assert v.temp_pause is False


def test_trial_overshoot_fails():
    tc = TrialChecker(overshoot_limit=0.20)
    v = tc.check({"overshoot_max_pct": 25.0}, None, False)
    assert v.ok is False
    assert any("overshoot" in r for r in v.reasons)


def test_trial_jump_regression_fails():
    tc = TrialChecker(jump_p95_baseline=0.01, jump_regression_factor=1.2)
    v = tc.check({"overshoot_max_pct": 5.0, "jump_p95_star": 0.02}, None, False)
    assert v.ok is False
    assert any("jump_p95" in r for r in v.reasons)


def test_trial_temp_pause():
    tc = TrialChecker(temp_mos_pause_c=60.0)
    temps = np.array([55.0, 62.0, 50.0, 45.0, 40.0, 38.0])
    v = tc.check(None, temps, False)
    assert v.temp_pause is True


def test_trial_consecutive_wd_trips():
    tc = TrialChecker(consec_wd_limit=3)
    for _ in range(2):
        v = tc.check(None, None, True)
        assert v.ok is True
    v = tc.check(None, None, True)
    assert v.ok is False
    assert any("consec_wd" in r for r in v.reasons)


def test_trial_consec_wd_resets_on_clean():
    tc = TrialChecker(consec_wd_limit=2)
    tc.check(None, None, True)
    tc.check(None, None, False)  # resets counter
    v = tc.check(None, None, True)
    assert v.ok is True  # only 1 consecutive, not 2


# --- AnchorMonitor -----------------------------------------------------------

def test_anchor_due():
    with tempfile.TemporaryDirectory() as d:
        am = AnchorMonitor(Path(d), anchor_interval=10)
        assert am.due(0)
        assert not am.due(1)
        assert am.due(10)
        assert am.due(20)


def test_anchor_no_drift_under_3():
    with tempfile.TemporaryDirectory() as d:
        am = AnchorMonitor(Path(d))
        am.record(1.0)
        am.record(1.0)
        assert am.drift_flag is False


def test_anchor_drift_detection():
    with tempfile.TemporaryDirectory() as d:
        am = AnchorMonitor(Path(d), drift_sigma_factor=1.5)
        for _ in range(5):
            am.record(1.0)
        am.record(10.0)  # far outlier
        assert am.drift_flag is True


def test_anchor_csv_written():
    with tempfile.TemporaryDirectory() as d:
        am = AnchorMonitor(Path(d))
        am.record(1.0, temp_mos=np.array([40.0, 41.0]))
        csv_path = Path(d) / "session_drift.csv"
        assert csv_path.exists()
        lines = csv_path.read_text().strip().split("\n")
        assert len(lines) == 2  # header + 1 data row


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
