"""Metric computation (pure numpy, no hardware).

Two joint-unit-test metric families plus Cartesian RMSE, all matching the
口径 in SOP-01 §5 / SOP-03 §5:

* :func:`step_metrics` — per-event overshoot / settling-time / steady-state
  error from a square-wave excitation. Event onsets and amplitudes come from
  the *reference* signal (script-generated, so no edge-detection error).
* :func:`triangle_metrics` — error-trajectory range / std / jump statistics
  from a triangle sweep (the stick-slip signature, ODE B0).
* :func:`rmse` / :func:`ee_rmse` — joint and end-effector RMSE.

Everything is computed in rad/m internally; callers convert to deg/mm for
reporting.
"""
from __future__ import annotations

import math
from typing import List, Optional

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
