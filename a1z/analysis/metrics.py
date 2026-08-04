"""Metric computation (pure numpy, no hardware).

Two joint-unit-test metric families plus Cartesian RMSE, all matching the
口径 in SOP-01 §5 / SOP-03 §5:

* :func:`step_metrics` — per-event overshoot / settling-time / steady-state
  error from a square-wave excitation. Event onsets and amplitudes come from
  the *reference* signal (script-generated, so no edge-detection error).
* :func:`triangle_metrics` — error-trajectory range / std / jump statistics
  from a triangle sweep (the stick-slip signature, ODE B0).
* :func:`rmse` / :func:`ee_rmse` — joint and end-effector RMSE.
* :func:`ee_terminal_error` / :func:`ee_normal_jitter_std` /
  :func:`ee_phase_lag` — the three J_ee terms of SOP-11 §2.2 (到位误差 /
  法向抖动 / 相位滞后) for the EE-refine composite cost.

Everything is computed in rad/m internally; callers convert to deg/mm for
reporting.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

DEG = 180.0 / math.pi


def rmse(err: np.ndarray) -> float:
    err = np.asarray(err, dtype=float)
    if err.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(err ** 2)))


# ---------------------------------------------------------------------------
# Square-wave step metrics
# ---------------------------------------------------------------------------
def _step_events(ref: np.ndarray, dt: float, min_delta_rad: float):
    """Detect step onsets robust to slew-limited (ramped) edges.

    Returns list of (i_onset, i_end, delta) where the reference moves from one
    plateau to the next: ``i_onset`` is the first moving sample, ``i_end`` the
    next onset (or the last sample), and ``delta`` the plateau-to-plateau change.
    A hard step (single-tick jump) and a 125 ms slew ramp both collapse to one
    event, so the metric口径 is identical whether or not edge-rate is applied.
    """
    d = np.diff(ref)
    # "moving" if the per-tick change is an appreciable fraction of min_delta;
    # ref is script-generated (noise-free) so a small floor cleanly separates
    # ramps/steps from flat plateaus.
    move_eps = max(min_delta_rad * 0.05, 1e-4)
    moving = np.abs(d) > move_eps
    onsets = []
    i = 0
    n = len(d)
    while i < n:
        if moving[i]:
            j = i + 1
            while j < n and moving[j]:
                j += 1
            # ref goes from ref[i] to ref[j] over the moving run [i, j).
            delta = ref[j] - ref[i]
            if abs(delta) >= min_delta_rad:
                onsets.append((i, j, delta))
            i = j
        else:
            i += 1
    events = []
    for k, (i0, _run_end, delta) in enumerate(onsets):
        i_next = onsets[k + 1][0] if k + 1 < len(onsets) else len(ref) - 1
        events.append((i0, i_next, delta))
    return events


def step_metrics(
    t: np.ndarray,
    ref: np.ndarray,
    resp: np.ndarray,
    *,
    band: float = 0.05,
    ess_win_s: float = 0.3,
    min_delta_deg: float = 2.0,
) -> List[dict]:
    """Per-step overshoot / settling / steady-state error for one joint.

    Args:
        t: (N,) timestamps (s).
        ref: (N,) reference joint angle (rad) — the commanded square wave.
        resp: (N,) measured joint angle (rad).
        band: settling band as a fraction of |Δ| (default 5%).
        ess_win_s: steady-state averaging window before the next edge (s).
        min_delta_deg: ignore steps smaller than this.

    Returns:
        List of dicts (one per detected edge), each with dir/delta_deg/
        overshoot_pct/ts_ms/ess_deg. ``ts_ms`` is None if it never settles
        before the next edge, measured from the step onset.
    """
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    resp = np.asarray(resp, dtype=float)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    events = _step_events(ref, dt, math.radians(min_delta_deg))
    out: List[dict] = []
    for i0, i_next, delta in events:
        if i_next - i0 < 2:
            continue
        r_after = ref[i_next]
        seg = resp[i0:i_next + 1]
        seg_t = t[i0:i_next + 1]
        # Steady-state: mean |resp - ref| over the last ess_win_s before edge.
        win_mask = seg_t >= (seg_t[-1] - ess_win_s)
        ess = float(np.mean(np.abs(seg[win_mask] - r_after)))
        resp_final = float(np.mean(seg[win_mask]))
        # Overshoot: peak beyond target in the step direction.
        if delta > 0:
            q_peak = float(np.max(seg))
        else:
            q_peak = float(np.min(seg))
        overshoot = (q_peak - resp_final) / delta * 100.0
        overshoot = max(overshoot, 0.0)
        # Settling time: first entry into [r_after ± band|Δ|] that holds to end.
        lo = r_after - abs(band * delta)
        hi = r_after + abs(band * delta)
        inside = (seg >= lo) & (seg <= hi)
        ts_ms: Optional[float] = None
        for m in range(len(seg)):
            if inside[m] and inside[m:].all():
                ts_ms = float((seg_t[m] - seg_t[0]) * 1000.0)
                break
        out.append(dict(
            dir="up" if delta > 0 else "down",
            delta_deg=delta * DEG,
            overshoot_pct=overshoot,
            ts_ms=ts_ms,
            ess_deg=ess * DEG,
        ))
    return out


def summarize_steps(steps: List[dict]) -> dict:
    """Reduce per-step dicts to the worst-case values reported in result.json."""
    if not steps:
        return dict(n_steps=0, overshoot_max_pct=None, ess_max_deg=None,
                    ts_max_ms=None, per_step=[])
    ts_vals = [s["ts_ms"] for s in steps if s["ts_ms"] is not None]
    return dict(
        n_steps=len(steps),
        overshoot_max_pct=max(s["overshoot_pct"] for s in steps),
        ess_max_deg=max(s["ess_deg"] for s in steps),
        ts_max_ms=(max(ts_vals) if ts_vals else None),
        per_step=steps,
    )


# ---------------------------------------------------------------------------
# Triangle-wave error-trajectory metrics (stick-slip signature)
# ---------------------------------------------------------------------------
def _zigzag_pivots(x: np.ndarray, eps: float):
    """Turning points of x via a retracement (ZigZag) filter with dead-band eps.

    Tracks the running max/min since the last confirmed pivot; a pivot is
    confirmed only when x retraces more than ``eps`` from that running extreme.
    Returns (indices, values, is_peak) for each pivot in order. Robust to a slow
    monotonic drift (a tracking-lag ramp yields no pivots until it reverses),
    which is exactly why it separates stick-slip teeth from the sweep lag —
    unlike a per-sample |Δ| threshold that fires on every sample of the ramp.
    """
    n = len(x)
    idx: List[int] = []
    val: List[float] = []
    is_peak: List[bool] = []
    if n < 2:
        return idx, val, is_peak
    rmax = rmin = x[0]
    rmax_i = rmin_i = 0
    trend = 0  # +1 rising, -1 falling, 0 unknown
    for i in range(n):
        v = x[i]
        if v > rmax:
            rmax, rmax_i = v, i
        if v < rmin:
            rmin, rmin_i = v, i
        if trend <= 0 and (v - rmin) > eps:      # confirmed upturn at the min
            idx.append(rmin_i); val.append(rmin); is_peak.append(False)
            trend = 1
            rmax, rmax_i = v, i
        elif trend >= 0 and (rmax - v) > eps:     # confirmed downturn at the max
            idx.append(rmax_i); val.append(rmax); is_peak.append(True)
            trend = -1
            rmin, rmin_i = v, i
    return idx, val, is_peak


def triangle_metrics(
    t: np.ndarray,
    ref: np.ndarray,
    resp: np.ndarray,
    *,
    jump_eps_deg: float = 0.1,
) -> dict:
    """Error-trajectory range / std / stick-slip jump stats over a triangle sweep.

    ``t/ref/resp`` should already be sliced to the excitation segment (holds
    removed). Jumps are the stick-slip teeth of ``err = resp - ref``: detected as
    direction reversals via :func:`_zigzag_pivots` with dead-band ``jump_eps_deg``
    (NOT per-sample |Δerr| — under a triangle sweep err carries a slow tracking-
    lag ramp that exceeds any small per-sample threshold on nearly every sample,
    which collapses the count; see SOP-03 §5.3 / §10.12). One stick-slip cycle =
    one peak + one valley, so ``jump_count`` counts peaks (= cycles/teeth); jump
    amplitudes are the swings between consecutive pivots (≈ tooth height / slip).
    """
    t = np.asarray(t, dtype=float)
    err = np.asarray(resp, dtype=float) - np.asarray(ref, dtype=float)
    if err.size < 2:
        return dict(err_range_deg=float("nan"), err_std_deg=float("nan"),
                    jump_count=0, jump_rate_hz=0.0, jump_mean_deg=0.0,
                    jump_max_deg=0.0, jump_eps_deg=jump_eps_deg)
    eps = math.radians(jump_eps_deg)
    _idx, vals, is_peak = _zigzag_pivots(err, eps)
    # Leg amplitudes = swing between consecutive pivots (each ≈ one tooth height).
    amps = np.abs(np.diff(np.asarray(vals))) if len(vals) > 1 else np.array([])
    n_teeth = int(sum(is_peak))               # one peak per stick-slip cycle
    duration = float(t[-1] - t[0]) if t[-1] > t[0] else float("nan")
    jump_rate = (n_teeth / duration) if duration and not math.isnan(duration) else 0.0
    return dict(
        err_range_deg=float(np.ptp(err)) * DEG,
        err_std_deg=float(np.std(err)) * DEG,
        jump_count=n_teeth,
        jump_rate_hz=jump_rate,
        jump_mean_deg=(float(np.mean(amps)) * DEG if amps.size else 0.0),
        jump_max_deg=(float(np.max(amps)) * DEG if amps.size else 0.0),
        jump_eps_deg=jump_eps_deg,
    )


def noise_floor_std_deg(err_hold: np.ndarray) -> float:
    """Std (deg) of the error during a static hold, for jump-eps calibration."""
    err_hold = np.asarray(err_hold, dtype=float)
    if err_hold.size == 0:
        return 0.0
    return float(np.std(err_hold)) * DEG


def hold_noise_floor_from_trace(t: np.ndarray, ref: np.ndarray, resp: np.ndarray,
                                *, trim_s: float = 0.3) -> Optional[float]:
    """Noise-floor σ (deg) from the LEADING+TRAILING holds of a full trace.

    Recovers hold segments from the scripted ref (``ref_speed_profile`` sign==0)
    and takes only the first/last, trimming ``trim_s`` off the inner edge (the
    side adjoining a slew) so the post-edge settle transient of a middle square
    plateau can't inflate the floor above the steady-state error (SOP-08 §1.5).
    Each segment is de-meaned before pooling. Returns None if no usable hold.
    """
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    resp = np.asarray(resp, dtype=float)
    prof = ref_speed_profile(t, ref)
    segs = prof["segments"]
    if not segs:
        return None
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    trim = int(round(trim_s / dt))
    errs = []
    if segs[0][2] == 0:                       # leading hold: trim right (→ slew)
        i0, i1 = segs[0][0], max(segs[0][0], segs[0][1] - trim)
        if i1 - i0 >= 5:
            e = resp[i0:i1] - ref[i0:i1]
            errs.append(e - np.mean(e))
    if len(segs) > 1 and segs[-1][2] == 0:    # trailing hold: trim left (slew →)
        i0, i1 = min(segs[-1][1], segs[-1][0] + trim), segs[-1][1]
        if i1 - i0 >= 5:
            e = resp[i0:i1] - ref[i0:i1]
            errs.append(e - np.mean(e))
    return noise_floor_std_deg(np.concatenate(errs)) if errs else None


# ---------------------------------------------------------------------------
# v2 metrics (SOP-08 · G0 gate): lag decomposition, apex-excluded adaptive-ε
# jump statistics, ts from ref arrival, ess/noise-floor ratio.
#
# Additive by hard constraint (SOP-08 §1.7): the v1 functions above are
# untouched and their result.json fields keep byte-identical semantics; every
# v2 entry point returns its full 口径 (k_sigma, apex_excl_s, realized ε,
# quantization step, fit-mask counts) alongside the numbers, because cross-run
# comparability requires the calipers to be auditable per run. All functions
# are pure numpy over (t, ref, resp[, eff]) — hold segments and the excitation
# window are recovered from the noise-free scripted ref itself, so archived
# CSVs recompute without any runtime state.
# ---------------------------------------------------------------------------
def estimate_quant_step(x: np.ndarray) -> float:
    """Effective quantization step of a hold-segment response trace.

    Median of the nonzero |Δx| (same unit as ``x``): under a static hold the
    response only moves in encoder/protocol quanta, so the nonzero increments
    cluster at the effective step (J6 measured ≈0.13°, SOP-08 §1.3). Returns
    0.0 if the trace never moves.
    """
    x = np.asarray(x, dtype=float)
    if x.size < 2:
        return 0.0
    d = np.abs(np.diff(x))
    nz = d[d > 0]
    return float(np.median(nz)) if nz.size else 0.0


def ref_speed_profile(t: np.ndarray, ref: np.ndarray, *, slope_frac: float = 0.05) -> dict:
    """Piecewise-constant q̇_ref recovered from the scripted (noise-free) ref.

    Splits the trace into hold / up / down regimes by per-sample slope sign
    (threshold = ``slope_frac`` × max |slope|), then assigns each regime run
    the endpoint-derived constant slope — NOT a per-sample finite difference,
    whose 100 Hz staircase would pollute the lag regression (SOP-08 §1.1).

    Returns dict with:
        qdot          (N,) piecewise-constant reference velocity (unit/s)
        segments      list of (i0, i1, sign, slope), samples [i0, i1)
        apex_idx      indices where q̇_ref flips sign (±↔∓ reversals, §1.2)
        boundary_idx  every regime-change index (reversals + hold on/offsets)
        sweep_rate    median |slope| over moving segments (0.0 if none)
    """
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    n = len(ref)
    empty = dict(qdot=np.zeros(n), segments=[(0, n, 0, 0.0)] if n else [],
                 apex_idx=[], boundary_idx=[], sweep_rate=0.0)
    if n < 3:
        return empty
    dt = np.diff(t)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(dt > 0, np.diff(ref) / np.where(dt > 0, dt, 1.0), 0.0)
    vmax = float(np.max(np.abs(s))) if s.size else 0.0
    if vmax <= 0.0:
        return empty
    tol = slope_frac * vmax
    sgn = np.zeros(n - 1, dtype=int)
    sgn[s > tol] = 1
    sgn[s < -tol] = -1
    segments = []
    i = 0
    while i < n - 1:
        j = i
        while j < n - 1 and sgn[j] == sgn[i]:
            j += 1
        slope = 0.0 if sgn[i] == 0 else float((ref[j] - ref[i]) / (t[j] - t[i]))
        segments.append((i, j, int(sgn[i]), slope))
        i = j
    qdot = np.zeros(n)
    for i0, i1, _sg, sl in segments:
        qdot[i0:i1] = sl
    qdot[-1] = segments[-1][3]
    apex_idx, boundary_idx = [], []
    for k in range(1, len(segments)):
        idx = segments[k][0]
        boundary_idx.append(idx)
        if segments[k - 1][2] != 0 and segments[k][2] != 0:
            apex_idx.append(idx)
    rates = [abs(sl) for _i0, _i1, sg, sl in segments if sg != 0]
    return dict(qdot=qdot, segments=segments, apex_idx=apex_idx,
                boundary_idx=boundary_idx, sweep_rate=float(np.median(rates)))


def _boundary_excl_mask(t: np.ndarray, boundary_idx, apex_excl_s: float) -> np.ndarray:
    """True where a sample falls within ±apex_excl_s of any regime boundary."""
    excl = np.zeros(len(t), dtype=bool)
    for idx in boundary_idx:
        excl |= np.abs(t - t[idx]) <= apex_excl_s
    return excl


def decompose_lag(t: np.ndarray, ref: np.ndarray, resp: np.ndarray, *,
                  apex_excl_s: float = 0.2, profile: Optional[dict] = None) -> dict:
    """LS decomposition of the tracking error into lag terms + residual.

    Target model (SOP-08 §1.1, steady-sweep PD physics per SOP-05 §6.3:
    lag = (kd·q̇ + τ_c)/kp, verified to 5 %):

        e ≈ k_v·q̇_ref + c·sign(q̇_ref) + b

    Fitted on constant-velocity samples only: holds and ±apex_excl_s around
    every regime boundary (apex reversals and excitation start/end) are
    excluded. For a lagging joint k_v and c come out NEGATIVE (e = resp − ref
    trails the sweep); |k_v| ≈ kd/kp (s) and |c| ≈ τ_c/kp. The residual after
    removing the fitted lag is the smoothness signal → ``resid_std_deg``.

    IDENTIFIABILITY (found by the P1 synthetic self-check, G0-6): on a
    symmetric single-rate triangle |q̇_ref| is one constant s, so the q̇ and
    sign(q̇) columns are exactly collinear — only the combination
    L = k_v·s + c is observable, and a naive 3-param lstsq returns a stable
    but meaningless min-norm split of (k_v, c). Therefore:

    * single-rate data (|q̇| spread < 5 %): fit the identifiable reduced model
      e = L·sign(q̇) + b; report |L| as ``lag_at_rate_deg`` (signed copy in
      ``lag_signed_deg``), leave k_v_s/c_deg NaN, ``kv_c_separable=False``;
    * multi-rate data (pooled R-D periods / mixed-rate excitation): fit the
      full 3-param model, ``kv_c_separable=True``.

    Gate impact: G0-5/G0-7 hold verbatim on lag_at_rate (kp→kp/2 doubles L
    exactly, so the ordering and the ≈2 ratio sanity carry over); the G0-3
    "k_v CV" must be read as lag_at_rate CV on single-rate runs.
    """
    t = np.asarray(t, dtype=float)
    e = np.asarray(resp, dtype=float) - np.asarray(ref, dtype=float)
    prof = profile if profile is not None else ref_speed_profile(t, ref)
    qdot = prof["qdot"]
    mask = (qdot != 0.0) & ~_boundary_excl_mask(t, prof["boundary_idx"], apex_excl_s)
    n_fit = int(mask.sum())
    n_dir = int(len(set(np.sign(qdot[mask])))) if n_fit else 0
    rate = prof["sweep_rate"]
    out = dict(k_v_s=float("nan"), c_deg=float("nan"), b_deg=float("nan"),
               resid_std_deg=float("nan"), lag_at_rate_deg=float("nan"),
               lag_signed_deg=float("nan"), rate_deg_s=rate * DEG,
               n_fit=n_fit, n_total=len(t), n_directions=n_dir,
               kv_c_separable=False, apex_excl_s=apex_excl_s, fit_ok=False)
    if n_fit < 12 or rate <= 0.0:
        return out
    speeds = np.abs(qdot[mask])
    multi_rate = float((speeds.max() - speeds.min()) / np.median(speeds)) > 0.05
    sgn = np.sign(qdot[mask])
    if multi_rate:
        A = np.column_stack([qdot[mask], sgn, np.ones(n_fit)])
        coef, *_ = np.linalg.lstsq(A, e[mask], rcond=None)
        k_v, c, b = (float(v) for v in coef)
        L = k_v * rate + c
        out.update(k_v_s=k_v, c_deg=c * DEG, kv_c_separable=True)
    else:
        A = np.column_stack([sgn, np.ones(n_fit)])
        coef, *_ = np.linalg.lstsq(A, e[mask], rcond=None)
        L, b = (float(v) for v in coef)
    resid = e[mask] - A @ coef
    out.update(
        b_deg=b * DEG,
        resid_std_deg=float(np.std(resid)) * DEG,
        lag_at_rate_deg=abs(L) * DEG,
        lag_signed_deg=L * DEG,
        # L·sign(q̇) and the intercept b separate only when both sweep
        # directions are present; single-direction fits stay flagged not-ok.
        fit_ok=(n_dir == 2),
    )
    return out


def ess_ratio(ess_deg: Optional[float], noise_floor_deg: Optional[float]) -> Optional[float]:
    """ess / noise-floor (SOP-08 §1.5): ≲1 means ess has hit the sensor floor
    and further "improvement" of ess is not credible. None if undefined."""
    if ess_deg is None or noise_floor_deg is None:
        return None
    if not math.isfinite(ess_deg) or not math.isfinite(noise_floor_deg) \
            or noise_floor_deg <= 0.0:
        return None
    return float(ess_deg / noise_floor_deg)


def triangle_metrics_v2(
    t: np.ndarray,
    ref: np.ndarray,
    resp: np.ndarray,
    *,
    k_sigma: float = 4.0,
    apex_excl_s: float = 0.2,
    eps_min_deg: float = 0.05,
    eps_override_deg: Optional[float] = None,
    eff: Optional[np.ndarray] = None,
    hf_cut_hz: float = 5.0,
) -> dict:
    """v2 triangle metrics over the FULL capture, holds included (SOP-08 §1).

    Differences vs v1 ``triangle_metrics`` (which expects a pre-sliced
    excitation segment and a fixed ε):

    * the leading/trailing holds are recovered from ref and provide the noise
      floor σ and quantization step for the adaptive dead-band
      ε = max(k_sigma·σ_floor, 2·q_step, eps_min_deg)  (§1.3);
    * error decomposition (``decompose_lag``) separates tracking lag from the
      residual — ``resid_std_deg`` replaces v1 ``err_std_deg`` in the
      smoothness role (§1.1);
    * jump statistics run per contiguous chunk after cutting ±apex_excl_s
      around every ref regime boundary, so neither the apex error flip nor a
      splice discontinuity can fake a tooth (§1.2); severity is ``jump_p95``
      (P95 口径, ODE-aligned) with max kept for diagnosis;
    * optional eff sentinel quantities (§1.6, non-gating; ``eff=None`` for
      the 07-20-era CSVs without an eff column, K2).

    Pass ``eps_override_deg`` (e.g. a full-run ε) when slicing single cycles
    for cycle-to-cycle σ so every slice shares one 口径 (§2.3-2).
    """
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    resp = np.asarray(resp, dtype=float)
    err = resp - ref
    prof = ref_speed_profile(t, ref)
    qdot = prof["qdot"]
    segs = prof["segments"]
    dt_med = float(np.median(np.diff(t))) if len(t) > 1 else 0.01

    # --- hold segments (noise floor + quantization step), inner edge trimmed
    hold_err, hold_resp_diffs, n_hold = [], [], 0
    if segs and segs[0][2] == 0:
        i0, i1 = segs[0][0], segs[0][1]
        i1 = max(i0, i1 - int(round(apex_excl_s / dt_med)))
        if i1 - i0 >= 5:
            hold_err.append(err[i0:i1] - np.mean(err[i0:i1]))
            hold_resp_diffs.append(np.abs(np.diff(resp[i0:i1])))
            n_hold += i1 - i0
    if len(segs) > 1 and segs[-1][2] == 0:
        i0, i1 = segs[-1][0], segs[-1][1] + 1
        i0 = min(i1, i0 + int(round(apex_excl_s / dt_med)))
        if i1 - i0 >= 5:
            hold_err.append(err[i0:i1] - np.mean(err[i0:i1]))
            hold_resp_diffs.append(np.abs(np.diff(resp[i0:i1])))
            n_hold += i1 - i0
    sigma_floor_deg = (noise_floor_std_deg(np.concatenate(hold_err))
                       if hold_err else 0.0)
    q_step_source = "hold"
    if hold_resp_diffs:
        alld = np.concatenate(hold_resp_diffs)
        nz = alld[alld > 0]
        q_step_deg = (float(np.median(nz)) * DEG) if nz.size else 0.0
    else:
        q_step_deg = 0.0
    if q_step_deg == 0.0:
        # Hold-silent quantization (found by the P1 synthetic rig): a hold
        # parked mid-cell with sub-step noise rounds to ONE level — the hold
        # shows neither σ nor steps while the sweep still carries a ±q/2
        # sawtooth that would flood the tooth count. Fall back to a
        # sweep-based estimate: median of the dust-filtered
        # |Δresp − median Δresp| within each uniform-velocity segment
        # (segment-interior diffs only — a boundary-crossing diff carries the
        # apex error flip, not quantization). Motion cancels in the deviation:
        # a quantized sweep yields ≈ q (double/missing steps), a smooth noisy
        # sweep yields ≈1.3σ_noise — harmless, dominated by k_sigma·σ_floor.
        devs, steps_all = [], []
        for i0, i1, sg, _sl in segs:
            if sg == 0 or i1 - i0 < 8:
                continue
            d = np.diff(resp[i0:i1])
            devs.append(np.abs(d - np.median(d)))
            steps_all.append(np.abs(d))
        if devs:
            alld = np.concatenate(devs)
            med_step = float(np.median(np.concatenate(steps_all)))
            alld = alld[alld > max(1e-12, 0.02 * med_step)]   # drop float dust
            if alld.size:
                q_step_deg = float(np.median(alld)) * DEG
                q_step_source = "sweep-med-dev"

    # --- adaptive dead-band (§1.3)
    if eps_override_deg is not None:
        eps_deg, eps_source = float(eps_override_deg), "override"
    else:
        eps_deg = max(k_sigma * sigma_floor_deg, 2.0 * q_step_deg, eps_min_deg)
        eps_source = "adaptive"
    eps = math.radians(eps_deg)

    # --- excitation samples, boundary windows excluded, per-chunk ZigZag
    excl = _boundary_excl_mask(t, prof["boundary_idx"], apex_excl_s)
    moving = qdot != 0.0
    keep = moving & ~excl
    n_kept = int(keep.sum())
    amps, n_teeth = [], 0
    idx = np.flatnonzero(keep)
    if idx.size:
        splits = np.where(np.diff(idx) > 1)[0] + 1
        for chunk in np.split(idx, splits):
            if chunk.size < 3:
                continue
            _pi, vals, is_peak = _zigzag_pivots(err[chunk[0]:chunk[-1] + 1], eps)
            n_teeth += int(sum(is_peak))
            if len(vals) > 1:
                amps.extend(np.abs(np.diff(np.asarray(vals))).tolist())
    amps = np.asarray(amps)
    kept_dur = n_kept * dt_med

    # --- apex diagnostics (G0-4): big legs of a full-region ZigZag that touch
    # an excluded window ≈ the geometric error flips the exclusion removed.
    n_apex = len(prof["apex_idx"])
    apex_removed = 0
    big_thr = math.radians(max(1.0, 5.0 * eps_deg))
    mi = np.flatnonzero(moving)
    if mi.size:
        e0 = mi[0]
        piv_idx, piv_vals, _pk = _zigzag_pivots(err[e0:mi[-1] + 1], eps)
        for a, b_, va, vb in zip(piv_idx[:-1], piv_idx[1:],
                                 piv_vals[:-1], piv_vals[1:]):
            if abs(vb - va) >= big_thr and excl[e0 + a:e0 + b_ + 1].any():
                apex_removed += 1

    # --- lag decomposition (§1.1)
    lag = decompose_lag(t, ref, resp, apex_excl_s=apex_excl_s, profile=prof)

    # --- eff sentinel (§1.6, non-gating; tolerate missing channel, K2)
    eff_std_nm = eff_hf_ratio = None
    if eff is not None:
        eff = np.asarray(eff, dtype=float)
        ke = eff[keep]
        ke = ke[np.isfinite(ke)]
        if ke.size >= 8:
            eff_std_nm = float(np.std(ke))
            x = ke - np.mean(ke)
            spec = np.abs(np.fft.rfft(x)) ** 2
            freqs = np.fft.rfftfreq(len(x), d=dt_med)
            tot = float(spec[1:].sum())
            eff_hf_ratio = (float(spec[freqs >= hf_cut_hz].sum()) / tot
                            if tot > 0 else 0.0)

    exc_t = (float(t[mi[0]]), float(t[mi[-1]])) if mi.size else (float("nan"),) * 2
    return dict(
        metrics_version=2,
        # calipers (§1.7 — auditable 口径 travels with the numbers)
        k_sigma=k_sigma, apex_excl_s=apex_excl_s, eps_min_deg=eps_min_deg,
        eps_adapt_deg=eps_deg, eps_source=eps_source,
        sigma_floor_deg=sigma_floor_deg, q_step_deg=q_step_deg,
        q_step_source=q_step_source,
        n_hold_samples=n_hold, n_kept=n_kept, n_total=len(t),
        excite_window_s=exc_t,
        # significant-tooth statistics (§1.3)
        jump_count_v2=n_teeth,
        jump_rate_hz_v2=(n_teeth / kept_dur) if kept_dur > 0 else 0.0,
        jump_mean_deg_v2=(float(np.mean(amps)) * DEG if amps.size else 0.0),
        jump_p95_deg=(float(np.percentile(amps, 95)) * DEG if amps.size else 0.0),
        jump_max_deg_v2=(float(np.max(amps)) * DEG if amps.size else 0.0),
        n_legs=int(amps.size),
        # apex diagnostics (§1.2 / G0-4)
        n_apex=n_apex, apex_removed_big_teeth=apex_removed,
        # lag decomposition (§1.1)
        k_v_s=lag["k_v_s"], c_deg=lag["c_deg"], b_deg=lag["b_deg"],
        resid_std_deg=lag["resid_std_deg"],
        lag_at_rate_deg=lag["lag_at_rate_deg"], rate_deg_s=lag["rate_deg_s"],
        fit_ok=lag["fit_ok"], n_fit=lag["n_fit"],
        n_directions=lag["n_directions"],
        # eff sentinel (§1.6, not part of G0)
        eff_std_nm=eff_std_nm, eff_hf_ratio=eff_hf_ratio,
    )


def _step_events_v2(ref: np.ndarray, dt: float, min_delta_rad: float):
    """Like ``_step_events`` but keeps the moving-run end = ref plateau arrival.

    Returns (i_onset, i_arrival, i_next, delta) per event. Kept separate so the
    v1 function stays byte-identical (SOP-08 §1.7); detection logic mirrors
    ``_step_events`` exactly and the two lists align one-to-one.
    """
    d = np.diff(ref)
    move_eps = max(min_delta_rad * 0.05, 1e-4)
    moving = np.abs(d) > move_eps
    onsets = []
    i = 0
    n = len(d)
    while i < n:
        if moving[i]:
            j = i + 1
            while j < n and moving[j]:
                j += 1
            delta = ref[j] - ref[i]
            if abs(delta) >= min_delta_rad:
                onsets.append((i, j, delta))
            i = j
        else:
            i += 1
    events = []
    for k, (i0, run_end, delta) in enumerate(onsets):
        i_next = onsets[k + 1][0] if k + 1 < len(onsets) else len(ref) - 1
        events.append((i0, run_end, i_next, delta))
    return events


def step_metrics_v2(
    t: np.ndarray,
    ref: np.ndarray,
    resp: np.ndarray,
    *,
    band: float = 0.05,
    ess_win_s: float = 0.3,
    min_delta_deg: float = 2.0,
) -> List[dict]:
    """v1 per-step fields + ``ts_ms_v2`` counted from ref plateau arrival.

    v1 ``ts_ms`` starts at the first moving sample, so a slew-limited edge
    (amp 15° default ≈125 ms of commanded ramp) puts a hard floor under it —
    it cannot resolve the sub-125 ms settling A2 targets (SOP-08 §1.4). v2
    re-references the SAME settle sample to the instant ref reaches its
    plateau: ``ts_ms_v2 = ts_ms − ref_arrival_ms`` (clamped at 0 if the
    response is already in-band before the command stops ramping).
    ``ref_arrival_ms`` (the commanded slew duration) is included per step.
    v1 fields are produced by the untouched ``step_metrics``.
    """
    t = np.asarray(t, dtype=float)
    ref = np.asarray(ref, dtype=float)
    resp = np.asarray(resp, dtype=float)
    steps = step_metrics(t, ref, resp, band=band, ess_win_s=ess_win_s,
                         min_delta_deg=min_delta_deg)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    events = [ev for ev in _step_events_v2(ref, dt, math.radians(min_delta_deg))
              if ev[2] - ev[0] >= 2]           # mirror v1's short-event skip
    assert len(events) == len(steps), "v1/v2 step event detection diverged"
    out = []
    for (i0, i_arr, _i_next, _delta), s in zip(events, steps):
        arrival_ms = float((t[i_arr] - t[i0]) * 1000.0)
        s2 = dict(s)
        s2["ref_arrival_ms"] = arrival_ms
        s2["ts_ms_v2"] = (max(0.0, s["ts_ms"] - arrival_ms)
                          if s["ts_ms"] is not None else None)
        out.append(s2)
    return out


def summarize_steps_v2(steps: List[dict],
                       noise_floor_deg: Optional[float] = None) -> dict:
    """v1 summary fields + worst-case ``ts_v2`` and ``ess_ratio`` (§1.4/§1.5)."""
    base = summarize_steps(steps)
    ts2 = [s["ts_ms_v2"] for s in steps if s.get("ts_ms_v2") is not None]
    base.update(
        metrics_version=2,
        ts_v2_max_ms=(max(ts2) if ts2 else None),
        noise_floor_deg=noise_floor_deg,
        ess_ratio=ess_ratio(base["ess_max_deg"], noise_floor_deg),
    )
    return base


# ---------------------------------------------------------------------------
# End-effector RMSE
# ---------------------------------------------------------------------------
def geodesic_angle(R_ref: np.ndarray, R_resp: np.ndarray) -> float:
    """Geodesic angle (rad) between two rotation matrices."""
    R = R_ref.T @ R_resp
    c = (np.trace(R) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, c))))


def ee_errors(T_ref: np.ndarray, T_resp: np.ndarray):
    """Per-sample position vector error, |pos| error, and angular error.

    Args:
        T_ref/T_resp: (N,4,4) homogeneous transforms.

    Returns:
        (dp (N,3) m, e_pos (N,) m, e_ang (N,) rad).
    """
    T_ref = np.asarray(T_ref, dtype=float)
    T_resp = np.asarray(T_resp, dtype=float)
    dp = T_resp[:, :3, 3] - T_ref[:, :3, 3]
    e_pos = np.linalg.norm(dp, axis=1)
    e_ang = np.array([
        geodesic_angle(T_ref[k, :3, :3], T_resp[k, :3, :3])
        for k in range(len(T_ref))
    ])
    return dp, e_pos, e_ang


def ee_rmse(T_ref: np.ndarray, T_resp: np.ndarray) -> dict:
    """Position/orientation RMSE + max + per-axis position RMSE (mm/deg)."""
    dp, e_pos, e_ang = ee_errors(T_ref, T_resp)
    return dict(
        pos_rmse_mm=rmse(e_pos) * 1000.0,
        pos_max_mm=float(np.max(e_pos)) * 1000.0 if e_pos.size else float("nan"),
        ang_rmse_deg=rmse(e_ang) * DEG,
        ang_max_deg=float(np.max(e_ang)) * DEG if e_ang.size else float("nan"),
        per_axis_rmse_mm=dict(
            x=rmse(dp[:, 0]) * 1000.0,
            y=rmse(dp[:, 1]) * 1000.0,
            z=rmse(dp[:, 2]) * 1000.0,
        ),
    )


# ---------------------------------------------------------------------------
# End-effector refine metrics (SOP-11 §2.2 — the three J_ee terms)
# ---------------------------------------------------------------------------
def _xcorr_lag_samples(ref: np.ndarray, resp: np.ndarray,
                       max_lag: int) -> Tuple[float, float]:
    """Integer shift k maximising the normalised correlation between the
    reference shifted *right* by k and the response.

    Positive k means ``resp(t) ≈ ref(t − k)`` — the response *lags* the
    reference.  Correlation is re-normalised on every overlap window so edge
    effects do not bias toward large |k|.  Returns ``(k, peak_corr)`` with
    parabolic sub-sample refinement on k.
    """
    r = np.asarray(ref, dtype=float)
    s = np.asarray(resp, dtype=float)
    n = len(r)
    r = r - r.mean()
    s = s - s.mean()
    if n < 4 or float(np.sum(r * r)) <= 0.0 or float(np.sum(s * s)) <= 0.0:
        return 0.0, 0.0
    max_lag = int(min(max_lag, n // 4))  # keep ≥ 3/4 of the trace overlapping
    corrs: List[float] = []
    ks = list(range(-max_lag, max_lag + 1))
    for k in ks:
        if k >= 0:
            a, b = r[: n - k] if k else r, s[k:]
        else:
            a, b = r[-k:], s[: n + k]
        d = math.sqrt(float(np.sum(a * a)) * float(np.sum(b * b)))
        corrs.append(float(np.sum(a * b) / d) if d > 0.0 else 0.0)
    i_best = int(np.argmax(corrs))
    k_best = ks[i_best]
    k_sub = float(k_best)
    if 0 < i_best < len(corrs) - 1:
        y0, y1, y2 = corrs[i_best - 1], corrs[i_best], corrs[i_best + 1]
        curvature = y0 - 2.0 * y1 + y2
        if curvature < 0.0:  # genuine peak → parabolic vertex in [-0.5, 0.5]
            k_sub = k_best + (y0 - y2) / (2.0 * curvature)
    return k_sub, corrs[i_best]


def ee_phase_lag(T_ref: np.ndarray, T_resp: np.ndarray, sample_hz: float,
                 max_lag_s: float = 1.0) -> float:
    """Phase lag (ms) of the EE response behind the reference.

    相位滞后 (SOP-11 §2.2): per-axis normalised cross-correlation between the
    ref/resp position traces, combined with weights ∝ per-axis reference
    variance — static axes (e.g. the normal of a planar trajectory) drop out
    automatically.  Positive = response lags reference.
    """
    p_ref = np.asarray(T_ref, dtype=float)[:, :3, 3]
    p_resp = np.asarray(T_resp, dtype=float)[:, :3, 3]
    max_lag = int(round(max_lag_s * sample_hz))
    w_sum = 0.0
    acc = 0.0
    for a in range(3):
        v = float(np.var(p_ref[:, a]))
        if v <= 1e-12:
            continue
        k, _ = _xcorr_lag_samples(p_ref[:, a], p_resp[:, a], max_lag)
        w_sum += v
        acc += v * k
    if w_sum <= 0.0:
        return 0.0
    return acc / w_sum / sample_hz * 1000.0


def ee_normal_jitter_std(T_ref: np.ndarray, T_resp: np.ndarray,
                         normal: np.ndarray, sample_hz: float,
                         lag_ms: Optional[float] = None) -> float:
    """Std (mm) of the normal-direction EE position error, lag-aligned.

    法向抖动 std (SOP-11 §2.2): the phase lag is estimated (or passed in) and
    removed by shifting the reference before projecting the residual onto
    ``normal`` — a pure tracking delay must not count as jitter, the same
    lag/jitter decoupling as the joint-level resid_std (SOP-08 §1.1).
    ``normal`` need not be unit length (e.g. ``[0, 1, 0]`` for an xz-plane
    circle).
    """
    n = np.asarray(normal, dtype=float)
    norm = float(np.linalg.norm(n))
    if norm <= 0.0:
        raise ValueError("normal must be non-zero")
    n = n / norm
    if lag_ms is None:
        lag_ms = ee_phase_lag(T_ref, T_resp, sample_hz)
    k = int(round(lag_ms / 1000.0 * sample_hz))
    p_ref = np.asarray(T_ref, dtype=float)[:, :3, 3]
    p_resp = np.asarray(T_resp, dtype=float)[:, :3, 3]
    n_samples = len(p_ref)
    if abs(k) >= n_samples - 4:
        return 0.0
    if k >= 0:
        a, b = p_resp[k:], p_ref[: n_samples - k]  # resp(t) vs ref(t−k)
    else:
        a, b = p_resp[: n_samples + k], p_ref[-k:]
    e_n = (a - b) @ n
    return float(np.std(e_n) * 1000.0)


def ee_terminal_error(T_ref: np.ndarray, T_resp: np.ndarray,
                      tail_frac: float = 0.1) -> dict:
    """Terminal-window pose error: mean |dp| (mm) + mean geodesic angle (deg).

    到位误差 (SOP-11 §2.2): with a dedicated terminal hold segment, pass a
    ``tail_frac`` covering just the hold; without one, the last 10 % of the
    trajectory is used.  Mean (not RMSE) keeps the metric linear in the
    residual, matching the ess family on the joint side.
    """
    if not 0.0 < tail_frac <= 1.0:
        raise ValueError(f"tail_frac must be in (0, 1], got {tail_frac}")
    _, e_pos, e_ang = ee_errors(T_ref, T_resp)
    n_tail = max(1, int(round(len(e_pos) * tail_frac)))
    return dict(
        pos_mm=float(np.mean(e_pos[-n_tail:]) * 1000.0),
        ang_deg=float(np.mean(e_ang[-n_tail:]) * DEG),
    )


def ee_refine_metrics(T_ref: np.ndarray, T_resp: np.ndarray, sample_hz: float,
                      normal: np.ndarray, max_lag_s: float = 1.0,
                      tail_frac: float = 0.1) -> dict:
    """All three J_ee terms of SOP-11 §2.2 in one call (lag computed once)."""
    lag_ms = ee_phase_lag(T_ref, T_resp, sample_hz, max_lag_s=max_lag_s)
    term = ee_terminal_error(T_ref, T_resp, tail_frac=tail_frac)
    return dict(
        terminal_pos_mm=term["pos_mm"],
        terminal_ang_deg=term["ang_deg"],
        normal_jitter_std_mm=ee_normal_jitter_std(
            T_ref, T_resp, normal, sample_hz, lag_ms=lag_ms),
        phase_lag_ms=lag_ms,
    )


if __name__ == "__main__":
    # synthetic square-wave sanity check
    fs = 100.0
    t = np.arange(0, 8, 1 / fs)
    ref = np.where((t % 4) < 2, math.radians(20), math.radians(0))
    # first-order response with a little overshoot
    resp = np.zeros_like(ref)
    tau = 0.08
    for i in range(1, len(t)):
        resp[i] = resp[i - 1] + (ref[i - 1] - resp[i - 1]) * (1 / fs) / tau
    steps = step_metrics(t, ref, resp)
    print("step summary:", summarize_steps(steps)["n_steps"], "events")
    for s in steps:
        print(" ", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in s.items()})
    tri = triangle_metrics(t, ref, resp)
    print("triangle:", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in tri.items()})

    # v2 synthetic self-check (SOP-08 P1/P2): known-parameter round trip.
    # Full assertions live in tests/test_metrics_v2.py; this is the smoke view.
    from a1z.analysis import synth
    t2, ref2, _qd = synth.make_triangle_ref()
    resp2 = synth.lag_teeth_response(
        t2, ref2, _qd, k_v_s=-0.16, c_deg=-0.30, b_deg=0.05,
        noise_std_deg=0.02, quant_step_deg=0.13,
        teeth=((2.0, 1.2), (6.2, 0.9), (10.0, 1.5)), seed=0)
    m2 = triangle_metrics_v2(t2, ref2, resp2)
    print("triangle_v2 (inject k_v=-0.16 c=-0.30 b=0.05, 3 teeth):")
    for k in ("k_v_s", "c_deg", "b_deg", "resid_std_deg", "lag_at_rate_deg",
              "eps_adapt_deg", "sigma_floor_deg", "q_step_deg",
              "jump_count_v2", "jump_p95_deg", "n_apex", "apex_removed_big_teeth"):
        v = m2[k]
        print(f"  {k} = {v:.4f}" if isinstance(v, float) else f"  {k} = {v}")
    t3, ref3, _ = synth.make_square_ref()
    resp3 = synth.first_order_response(t3, ref3, tau=0.05)
    s2 = step_metrics_v2(t3, ref3, resp3)
    print("step_v2:", [(round(s["ref_arrival_ms"], 1), s["ts_ms"] and round(s["ts_ms"], 1),
                        s["ts_ms_v2"] and round(s["ts_ms_v2"], 1)) for s in s2])
