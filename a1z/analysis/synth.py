"""Synthetic-signal rig for the v2-metrics self-check (SOP-08 P1 / gate G0-6).

Generates the same waveform family as :class:`a1z.analysis.signals.WaveTrajectory`
(piecewise-linear triangle / slew-limited square between lo/hi with leading and
trailing holds) plus a response with KNOWN injected structure:

    e[k] = k_v·q̇_ref[k] + c·sign(q̇_ref[k]) + b        (lag model, SOP-08 §1.1)
         + Σ teeth  (triangular error pulses at known times/amplitudes)
         + gaussian noise
    resp = quantize(ref + e, q_step)

so tests can assert that ``decompose_lag`` / ``triangle_metrics_v2`` /
``step_metrics_v2`` recover the injected ground truth (G0-6: parameter error
< 5 %, injected-tooth detection 100 %, zero false positives at zero injection).

Deliberately independent: this module re-derives the waveform and its analytic
q̇_ref itself and imports nothing from ``signals``/``metrics`` — a bug in the
code under test cannot cancel out in the round trip. Pure numpy, deterministic
via ``seed``. Angles are degrees at the API (like the CLI), radians in the
returned arrays (like the CSVs).
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

import numpy as np

DEG = 180.0 / math.pi


def make_triangle_ref(
    *,
    fs: float = 100.0,
    period: float = 4.0,
    cycles: int = 3,
    lo_deg: float = -15.0,
    hi_deg: float = 15.0,
    hold_pre: float = 1.0,
    hold_post: float = 1.5,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangle sweep lo→hi→lo per period, holds at lo on both ends.

    Returns (t (N,) s, ref (N,) rad, qdot_ref (N,) rad/s) with qdot the
    ANALYTIC per-sample slope (±(hi−lo)/(period/2), 0 in holds) — the ground
    truth the estimator in metrics.ref_speed_profile is checked against.
    """
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    dur = hold_pre + cycles * period + hold_post
    t = np.arange(0.0, dur, 1.0 / fs)
    ref = np.full_like(t, lo)
    qdot = np.zeros_like(t)
    slope = (hi - lo) / (period / 2.0)
    tt = t - hold_pre
    inwave = (tt >= 0.0) & (tt < cycles * period)
    ph = (tt[inwave] % period) / period
    up = ph < 0.5
    frac = np.where(up, 2.0 * ph, 2.0 * (1.0 - ph))
    ref[inwave] = lo + (hi - lo) * frac
    qdot[inwave] = np.where(up, slope, -slope)
    return t, ref, qdot


def make_square_ref(
    *,
    fs: float = 100.0,
    period: float = 4.0,
    cycles: int = 3,
    lo_deg: float = -15.0,
    hi_deg: float = 15.0,
    hold_pre: float = 1.0,
    hold_post: float = 1.5,
    edge_rate_deg_s: float = 240.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Slew-limited square lo↔hi (flip每半周期, ramp at edge_rate), holds at lo.

    Returns (t, ref rad, qdot rad/s); qdot = ±edge_rate during a ramp, else 0.
    """
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)
    rate = math.radians(edge_rate_deg_s)
    half = period / 2.0
    dur = hold_pre + cycles * period + hold_post
    t = np.arange(0.0, dur, 1.0 / fs)
    ref = np.full_like(t, lo)
    qdot = np.zeros_like(t)
    for i, ti in enumerate(t):
        tt = ti - hold_pre
        if tt < 0.0 or tt >= cycles * period:
            continue
        k = int(tt // half)
        going_up = (k % 2 == 0)
        start_val = lo if going_up else hi
        target = hi if going_up else lo
        span = rate * (tt - k * half)
        if going_up:
            v = min(start_val + span, target)
            ref[i] = v
            qdot[i] = rate if v < target else 0.0
        else:
            v = max(start_val - span, target)
            ref[i] = v
            qdot[i] = -rate if v > target else 0.0
    return t, ref, qdot


def lag_teeth_response(
    t: np.ndarray,
    ref: np.ndarray,
    qdot: np.ndarray,
    *,
    k_v_s: float = -0.16,
    c_deg: float = -0.30,
    b_deg: float = 0.05,
    noise_std_deg: float = 0.02,
    quant_step_deg: float = 0.0,
    teeth: Iterable[Sequence[float]] = (),
    tooth_width_s: float = 0.08,
    tau_s: float = 0.0,
    seed: int = 0,
) -> np.ndarray:
    """Response = ref + known lag + injected teeth + noise, then quantized.

    ``k_v_s``/``c_deg`` default NEGATIVE — a physically lagging joint
    (e = resp − ref trails the sweep; |k_v| ≈ kd/kp, |c| ≈ τ_c/kp). ``teeth``
    is an iterable of (t_center_s, amp_deg): each adds a symmetric triangular
    error pulse of the given peak amplitude and ``tooth_width_s`` base width —
    one ZigZag peak with leg amplitudes ≈ amp, i.e. one significant tooth.
    ``tau_s`` > 0 passes the noiseless error through a first-order lag with
    that time constant, modelling the closed-loop settling dynamics
    (τ ≈ kd/kp; J6 at kd=4/kp=25 → 0.16 s): the apex error flip then takes
    ~3τ instead of one sample, which is what makes ``apex_excl_s`` a real
    tuning knob (SOP-08 §3 G0-4 failure branch). Quantization rounds the
    RESPONSE (as an encoder would), so the hold segments expose
    ``quant_step_deg`` to ``estimate_quant_step``.
    """
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    qdot = np.asarray(qdot, dtype=float)
    rng = np.random.default_rng(seed)
    e = (k_v_s * qdot
         + math.radians(c_deg) * np.sign(qdot)
         + math.radians(b_deg))
    if tau_s > 0.0:
        # First-order dynamics apply to the tracking-lag state only: stick-slip
        # teeth are state JUMPS (re-latch/slip events) and stay sharp — they
        # are injected after the filter, like the noise and the quantizer.
        ef = np.empty_like(e)
        ef[0] = e[0]
        for i in range(1, len(t)):
            dt = t[i] - t[i - 1]
            ef[i] = ef[i - 1] + (e[i - 1] - ef[i - 1]) * dt / tau_s
        e = ef
    for tc, amp_deg in teeth:
        w = tooth_width_s / 2.0
        e += math.radians(amp_deg) * np.clip(1.0 - np.abs(t - tc) / w, 0.0, None)
    if noise_std_deg > 0.0:
        e = e + rng.normal(0.0, math.radians(noise_std_deg), len(t))
    resp = ref + e
    if quant_step_deg > 0.0:
        q = math.radians(quant_step_deg)
        resp = np.round(resp / q) * q
    return resp


def first_order_response(t: np.ndarray, ref: np.ndarray, *, tau: float = 0.05,
                         noise_std_deg: float = 0.0, seed: int = 0) -> np.ndarray:
    """Discrete first-order tracker resp' = (ref − resp)/tau — the step-test
    plant for ``ts_ms_v2`` checks (same recursion as the metrics __main__)."""
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    resp = np.empty_like(ref)
    resp[0] = ref[0]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        resp[i] = resp[i - 1] + (ref[i - 1] - resp[i - 1]) * dt / tau
    if noise_std_deg > 0.0:
        rng = np.random.default_rng(seed)
        resp = resp + rng.normal(0.0, math.radians(noise_std_deg), len(t))
    return resp


if __name__ == "__main__":  # tiny smoke print
    t, ref, qd = make_triangle_ref()
    resp = lag_teeth_response(t, ref, qd, teeth=((6.2, 1.0),))
    print(f"triangle: n={len(t)} ref[deg] in [{ref.min() * DEG:.1f},"
          f" {ref.max() * DEG:.1f}] rate={np.max(np.abs(qd)) * DEG:.1f}°/s "
          f"err ptp={np.ptp(resp - ref) * DEG:.2f}°")
    t, ref, qd = make_square_ref()
    print(f"square:   n={len(t)} slew samples={(qd != 0).sum()} "
          f"(expect ≈ cycles×2×125 ms×fs = 75)")
