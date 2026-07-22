"""G0-6 synthetic self-check for the v2 metrics (SOP-08 §3 G0-6 / §4 P1+P2).

Hard-gate criteria encoded as tests, on synthetic trajectories with KNOWN
injected structure (a1z.analysis.synth — an independent re-derivation of the
waveform family, so a bug in the code under test cannot cancel out):

  * lag recovery: the identifiable lag observable is recovered within 5 % on
    noisy/quantized data (<0.5 % clean); b within 0.05°. On symmetric
    single-rate triangles only L = k_v·s + c is identifiable (exact q̇/sign
    collinearity — found BY this rig, see decompose_lag docstring), so
    single-rate tests assert on lag_at_rate_deg and the kv_c_separable flag,
    and a multi-rate test asserts the full (k_v, c) split.
  * tooth detection: every injected significant tooth found (count exact,
    amplitude in range); zero false positives at zero injection.
  * apex handling: reversal count recovered; the big geometric error-flip
    legs land in apex_removed_big_teeth (G0-4: 2×cycles ± 1), not in the
    jump statistics.
  * ts_ms_v2 counts from ref plateau arrival: ts_v1 − ts_v2 = slew duration.
  * ε_adapt floors: quantization-dominated, noise-dominated, override.

Run with ``pytest tests/test_metrics_v2.py -v`` or directly with
``python tests/test_metrics_v2.py`` (pytest not required — plain asserts).
"""
import math

import numpy as np

from a1z.analysis import synth
from a1z.analysis.metrics import (
    decompose_lag,
    ess_ratio,
    ref_speed_profile,
    step_metrics_v2,
    summarize_steps_v2,
    triangle_metrics_v2,
)

# Injected ground truth for the G0-6 rig (J6-flavored: kd/kp=4/25=0.16 s,
# τ_c/kp≈0.3°, 15°/s sweep, 0.13° quantization — SOP-08 §1.1/§1.3 numbers).
KV, C_DEG, B_DEG = -0.16, -0.30, 0.05
RATE = math.radians(15.0)                    # (hi−lo)/(period/2) = 30°/2 s
L_DEG = abs(KV * RATE + math.radians(C_DEG)) * synth.DEG   # ≈ 2.70°
TEETH = ((2.0, 1.2), (6.2, 0.9), (10.0, 1.5))


def _tri(**kw):
    t, ref, qd = synth.make_triangle_ref()
    return t, ref, qd, synth.lag_teeth_response(
        t, ref, qd, k_v_s=KV, c_deg=C_DEG, b_deg=B_DEG, **kw)


def test_ref_speed_profile_matches_analytic():
    t, ref, qd = synth.make_triangle_ref()
    prof = ref_speed_profile(t, ref)
    # ±1 sample skew allowed at regime boundaries; interiors must be exact.
    bad = np.abs(prof["qdot"] - qd) > 1e-9
    assert bad.mean() < 0.02, f"qdot mismatch on {bad.mean():.1%} of samples"
    assert len(prof["apex_idx"]) == 5            # 3 cycles → 5 interior flips
    assert abs(prof["sweep_rate"] - RATE) / RATE < 1e-6


def test_lag_recovery_single_rate_clean():
    t, ref, _qd, resp = _tri(noise_std_deg=0.0, quant_step_deg=0.0)
    d = decompose_lag(t, ref, resp)
    assert d["fit_ok"] and not d["kv_c_separable"]
    assert math.isnan(d["k_v_s"]) and math.isnan(d["c_deg"])  # honest NaN, not min-norm garbage
    assert abs(d["lag_at_rate_deg"] - L_DEG) / L_DEG < 0.005
    assert d["lag_signed_deg"] < 0               # lagging joint trails the sweep
    assert abs(d["b_deg"] - B_DEG) < 0.01
    assert d["resid_std_deg"] < 0.02


def test_g06_lag_recovery_noisy_quant_teeth():
    t, ref, _qd, resp = _tri(noise_std_deg=0.02, quant_step_deg=0.13,
                             teeth=TEETH, seed=0)
    m = triangle_metrics_v2(t, ref, resp)
    assert abs(m["lag_at_rate_deg"] - L_DEG) / L_DEG < 0.05    # G0-6: <5 %
    assert abs(m["b_deg"] - B_DEG) < 0.05
    assert abs(m["rate_deg_s"] - 15.0) < 0.1
    # calipers recovered from the holds
    assert 0.10 <= m["q_step_deg"] <= 0.17                     # 0.13 ± ~25 %
    assert m["sigma_floor_deg"] < 0.08
    assert 0.20 <= m["eps_adapt_deg"] <= 0.40                  # 2·q_step regime
    assert m["eps_source"] == "adaptive"
    # residual smoothness stays at the noise/quant scale, not the lag scale
    assert m["resid_std_deg"] < 0.15


def test_g06_teeth_detected_100pct():
    t, ref, _qd, resp = _tri(noise_std_deg=0.02, quant_step_deg=0.13,
                             teeth=TEETH, seed=0)
    m = triangle_metrics_v2(t, ref, resp)
    assert m["jump_count_v2"] == len(TEETH)                    # 100 % detection
    assert 1.30 <= m["jump_max_deg_v2"] <= 1.75                # 1.5 ± tol
    assert 1.00 <= m["jump_p95_deg"] <= 1.75
    assert 0.90 <= m["jump_mean_deg_v2"] <= 1.45


def test_g06_zero_injection_zero_false_positives():
    t, ref, _qd, resp = _tri(noise_std_deg=0.02, quant_step_deg=0.13, seed=0)
    m = triangle_metrics_v2(t, ref, resp)
    assert m["jump_count_v2"] == 0 and m["n_legs"] == 0        # G0-6: 误检 0
    # G0-4 sanity on the same run: the ±2.7° apex flips are big legs of the
    # UNcut trace and every one lands in the removed-diagnostic, giving
    # 2×cycles ± 1 (5 interior flips + up to 2 excitation-edge transitions).
    assert m["n_apex"] == 5
    assert 5 - 1 <= m["apex_removed_big_teeth"] <= 2 * 3 + 1


def test_multi_rate_separates_kv_c():
    # Two sweep rates in one trace (the R-D/G0-8 configuration, pooled):
    # 15 °/s (period 4) then 7.5 °/s (period 8) — q̇ and sign(q̇) decorrelate
    # and the full 3-parameter model becomes identifiable.
    t1, r1, q1 = synth.make_triangle_ref(period=4.0, cycles=2)
    t2, r2, q2 = synth.make_triangle_ref(period=8.0, cycles=1)
    dt = t1[1] - t1[0]
    t = np.concatenate([t1, t2 + t1[-1] + dt])
    ref = np.concatenate([r1, r2])
    qd = np.concatenate([q1, q2])
    # clean → near-exact split
    resp = synth.lag_teeth_response(t, ref, qd, k_v_s=KV, c_deg=C_DEG,
                                    b_deg=B_DEG, noise_std_deg=0.0)
    d = decompose_lag(t, ref, resp)
    assert d["kv_c_separable"] and d["fit_ok"]
    assert abs(d["k_v_s"] - KV) / abs(KV) < 0.005
    assert abs(d["c_deg"] - C_DEG) / abs(C_DEG) < 0.005
    # noisy + quantized → G0-6 tolerance
    resp = synth.lag_teeth_response(t, ref, qd, k_v_s=KV, c_deg=C_DEG,
                                    b_deg=B_DEG, noise_std_deg=0.02,
                                    quant_step_deg=0.13, seed=1)
    d = decompose_lag(t, ref, resp)
    assert abs(d["k_v_s"] - KV) / abs(KV) < 0.05
    assert abs(d["c_deg"] - C_DEG) / abs(C_DEG) < 0.05


def test_ts_v2_counts_from_ref_arrival():
    t, ref, _qd = synth.make_square_ref()
    resp = synth.first_order_response(t, ref, tau=0.05)
    steps = step_metrics_v2(t, ref, resp)
    assert len(steps) == 6                       # 3 cycles × 2 edges
    for s in steps:
        # 30° span at 240 °/s → 125 ms commanded ramp
        assert 110.0 <= s["ref_arrival_ms"] <= 140.0
        assert s["ts_ms"] is not None and s["ts_ms_v2"] is not None
        assert s["ts_ms_v2"] < s["ts_ms"]        # v1 carries the slew floor
        assert abs(s["ts_ms"] - s["ts_ms_v2"] - s["ref_arrival_ms"]) < 1e-6
        assert 40.0 <= s["ts_ms_v2"] <= 180.0    # ≈ τ·ln(rate·τ/band) ≈ 104 ms
    summ = summarize_steps_v2(steps, noise_floor_deg=0.05)
    assert summ["metrics_version"] == 2
    assert summ["ts_v2_max_ms"] < summ["ts_max_ms"]
    assert summ["ess_ratio"] == summ["ess_max_deg"] / 0.05


def test_eps_override_and_min_floor():
    t, ref, _qd, resp = _tri(noise_std_deg=0.0, quant_step_deg=0.0)
    m = triangle_metrics_v2(t, ref, resp)
    assert m["eps_adapt_deg"] == m["eps_min_deg"] == 0.05      # clean → floor
    m = triangle_metrics_v2(t, ref, resp, eps_override_deg=0.5)
    assert m["eps_adapt_deg"] == 0.5 and m["eps_source"] == "override"


def test_noise_dominated_eps_boundary():
    # UNBOUNDED Gaussian noise at σ=0.10° with no quantization: ε lands in the
    # k_sigma·σ regime, but a 4σ dead-band cannot promise zero reversals
    # against unbounded tails over ~10³ samples — the design guarantee of
    # "误检 0" (G0-6) applies to BOUNDED floors (quantization-dominated, the
    # real encoder regime, tested above). Here we pin the boundary behavior:
    # tail-driven legs stay at noise scale (≪ fault teeth), never tooth scale.
    t, ref, _qd, resp = _tri(noise_std_deg=0.10, quant_step_deg=0.0, seed=2)
    m = triangle_metrics_v2(t, ref, resp)
    assert 0.35 <= m["eps_adapt_deg"] <= 0.55          # ≈ 4σ = 0.40°
    assert m["jump_count_v2"] <= 15                    # rare tail reversals
    assert m["jump_max_deg_v2"] < 0.70                 # ≈7σ ≪ B0-era teeth (2.6°)


def test_apex_window_vs_settle_dynamics():
    # With real closed-loop dynamics (tau = kd/kp; J6 at kd=4 -> 0.16 s) the
    # apex/hold-onset error transitions take ~3*tau ~= 0.5 s -- a +-0.2 s
    # window leaves their decaying shoulders inside the kept chunks AND
    # inside the trimmed trailing hold, corrupting the caliper chain end to
    # end (observed on the real S3 J6 CSV: jump_p95 2.1 deg > the 1 deg G0-4
    # line at w=0.2, clean at 0.4/0.55). On this synthetic (tau=0.16, 0.45
    # deg ripple teeth, one per sweep leg): the small window inflates eps via
    # the sigma_floor decay-tail contamination -- losing the teeth -- and
    # leaks the shoulders into resid_std; +-0.55 s (~3.4*tau) restores both.
    # Per R3 this fixture is where the window value for kd=4-era data must
    # be re-derived before any real-data retune.
    ripple = tuple((float(c), 0.45) for c in (2, 4, 6, 8, 10, 12))  # mid-leg
    t, ref, qd = synth.make_triangle_ref()
    resp = synth.lag_teeth_response(t, ref, qd, k_v_s=KV, c_deg=C_DEG,
                                    b_deg=B_DEG, noise_std_deg=0.02,
                                    quant_step_deg=0.13, tau_s=0.16,
                                    teeth=ripple, seed=0)
    m_default = triangle_metrics_v2(t, ref, resp)                    # +-0.2 s
    m_wide = triangle_metrics_v2(t, ref, resp, apex_excl_s=0.55)     # ~3.4*tau
    # default window at tau=0.16: eps inflated by the hold decay tail ->
    # ripple teeth lost, residual carries the shoulders
    assert m_default["eps_adapt_deg"] > 0.4
    assert m_default["jump_count_v2"] < len(ripple)
    assert m_default["resid_std_deg"] > 2.5 * m_wide["resid_std_deg"]
    # wide window: calipers and teeth recover, lag stays honest
    assert 0.20 <= m_wide["eps_adapt_deg"] <= 0.35
    assert m_wide["jump_count_v2"] == len(ripple)
    assert m_wide["jump_p95_deg"] < 1.0           # G0-4 form
    assert abs(m_wide["lag_at_rate_deg"] - L_DEG) / L_DEG < 0.03
    # kd=0.5-era dynamics (tau = 20 ms): the default window already suffices.
    resp = synth.lag_teeth_response(t, ref, qd, k_v_s=-0.02, c_deg=C_DEG,
                                    b_deg=B_DEG, noise_std_deg=0.02,
                                    quant_step_deg=0.13, tau_s=0.02, seed=0)
    m = triangle_metrics_v2(t, ref, resp)
    assert m["jump_p95_deg"] < 1.0 and m["jump_count_v2"] == 0


def test_ess_ratio_edges():
    assert ess_ratio(0.5, 0.1) == 5.0
    assert ess_ratio(0.5, 0.0) is None
    assert ess_ratio(None, 0.1) is None
    assert ess_ratio(0.5, None) is None


def test_eff_sentinel_tolerates_missing():
    t, ref, _qd, resp = _tri(noise_std_deg=0.02, quant_step_deg=0.13, seed=0)
    m = triangle_metrics_v2(t, ref, resp, eff=None)            # K2: no eff column
    assert m["eff_std_nm"] is None and m["eff_hf_ratio"] is None
    eff = np.full(len(t), 0.1) + 0.02 * np.sin(2 * math.pi * 8.0 * t)
    m = triangle_metrics_v2(t, ref, resp, eff=eff)
    assert m["eff_std_nm"] is not None
    assert m["eff_hf_ratio"] > 0.5               # 8 Hz tone sits above the 5 Hz cut


if __name__ == "__main__":
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {name}: {e}")
    print(f"{len(fns) - failed}/{len(fns)} passed")
    raise SystemExit(1 if failed else 0)
