#!/usr/bin/env python3
"""Offline re-label of an old BO session with the CURRENT cost spec (SOP-11 §9.4).

No hardware. For each trial under ``<session>/trials/t*/`` it reads
``meta.json`` + ``unit-J*-square.csv`` + ``unit-J*-triangle.csv``,
recomputes v2 metrics, and applies the current ``compute_joint_cost``
(i.e. whatever ``COST_SPEC_VERSION`` the code carries — v4 added the
overshoot soft term, v5 added normalisation floors + term clip) to produce
the new cost.  The output CSV is intended for warm-starting a new
Optuna study via ``study.add_trial()`` (devlog 2026-07-30 Q4-9-2;
consumed by ``OptStudy._inject_warm_start``).

The original ``watchdog_verdict`` is preserved; we do **not** tighten the
overshoot hard constraint here -- the ``overshoot`` soft-cost term
carries the penalty instead, keeping the search space large.

Usage:
    python -m a1z.analysis.script.recompute_cost_v4 \\
        --session /path/to/2026-07-30-run-opt-phaseA-J6 \\
        --output /path/to/2026-07-30-run-opt-phaseA-J6/relabel-v5.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from a1z.analysis import metrics as M
from a1z.analysis.optimize.cost_spec import compute_joint_cost

DEG = 180.0 / math.pi


def _load_csv(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t, ref, resp) from a unit CSV."""
    d = np.genfromtxt(path, delimiter=",", names=True)
    if d.dtype.names is None or d.size < 3:
        raise ValueError(f"empty or malformed CSV ({d.size} rows)")
    return (
        np.asarray(d["t"], float),
        np.asarray(d["ref"], float),
        np.asarray(d["resp"], float),
    )


def _joint_from_csv_name(path: Path) -> int:
    """unit-J6-square.csv -> 0-based joint index 5."""
    name = path.stem  # unit-J6-square
    parts = name.split("-")
    if len(parts) < 2 or not parts[1].startswith("J"):
        raise ValueError(f"cannot parse joint from {path.name}")
    return int(parts[1][1:]) - 1


def recompute_trial(trial_dir: Path) -> Optional[Dict]:
    """Return a dict with V4 cost for one trial, or None if unusable."""
    meta_path = trial_dir / "meta.json"
    if not meta_path.exists():
        return None
    meta = json.loads(meta_path.read_text())

    theta = meta.get("theta", {})
    kp = theta.get("kp")
    zeta_hat = theta.get("zeta_hat")
    if kp is None or zeta_hat is None:
        return None

    # Find the square/triangle CSVs for the active joint.
    sq_files = sorted(trial_dir.glob("unit-J*-square.csv"))
    tri_files = sorted(trial_dir.glob("unit-J*-triangle.csv"))
    if not sq_files or not tri_files:
        return None

    sq_path = sq_files[0]
    tri_path = tri_files[0]
    j = _joint_from_csv_name(sq_path)

    # Default fallback values.
    lag_deg = 0.0
    resid_std_deg = 0.0
    ts_ms = 0.0
    ess_deg = 0.0
    overshoot_pct = 0.0

    # Triangle leg: lag + resid_std.
    try:
        t, ref, resp = _load_csv(tri_path)
        if len(t) > 10:
            tri_m = M.triangle_metrics_v2(t, ref, resp)
            lag_deg = tri_m.get("lag_at_rate_deg", 0.0) or 0.0
            resid_std_deg = tri_m.get("resid_std_deg", 0.0) or 0.0
    except Exception as e:  # noqa: BLE001
        return {
            "trial_id": meta.get("trial_id"),
            "theta_kp": kp,
            "theta_zeta": zeta_hat,
            "cost_new": None,
            "cost_breakdown": None,
            "lag_deg": None,
            "ts_ms": None,
            "resid_std_deg": None,
            "ess_deg": None,
            "overshoot_pct": None,
            "watchdog_ok": False,
            "skip_reason": f"triangle_error:{type(e).__name__}:{e}",
        }

    # Step leg: ts, ess, overshoot.
    try:
        t, ref, resp = _load_csv(sq_path)
        if len(t) > 10:
            noise_floor = M.hold_noise_floor_from_trace(t, ref, resp)
            steps = M.step_metrics_v2(t, ref, resp)
            if steps:
                step_summary = M.summarize_steps_v2(steps, noise_floor_deg=noise_floor)
                ts_ms = step_summary.get("ts_max_ms")
                if ts_ms is None:
                    ts_ms = 2000.0
                ts_ms = ts_ms or 0.0
                ess_deg = step_summary.get("ess_max_deg", 0.0) or 0.0
                overshoot_pct = step_summary.get("overshoot_max_pct", 0.0) or 0.0
    except Exception as e:  # noqa: BLE001
        return {
            "trial_id": meta.get("trial_id"),
            "theta_kp": kp,
            "theta_zeta": zeta_hat,
            "cost_new": None,
            "cost_breakdown": None,
            "lag_deg": lag_deg,
            "ts_ms": None,
            "resid_std_deg": resid_std_deg,
            "ess_deg": None,
            "overshoot_pct": None,
            "watchdog_ok": False,
            "skip_reason": f"square_error:{type(e).__name__}:{e}",
        }

    cost_new, breakdown = compute_joint_cost(
        j, lag_deg, ts_ms, resid_std_deg, ess_deg, overshoot_pct
    )

    # Preserve original watchdog verdict as the feasibility flag.
    wd = meta.get("watchdog_verdict", "")
    watchdog_ok = (wd == "ok" or wd == "")

    return {
        "trial_id": meta.get("trial_id"),
        "theta_kp": kp,
        "theta_zeta": zeta_hat,
        "cost_new": cost_new,
        "cost_breakdown": breakdown,
        "lag_deg": lag_deg,
        "ts_ms": ts_ms,
        "resid_std_deg": resid_std_deg,
        "ess_deg": ess_deg,
        "overshoot_pct": overshoot_pct,
        "watchdog_ok": watchdog_ok,
        "skip_reason": "",
    }


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Offline V4 cost re-label for BO trial archive (devlog Q4-9-2)")
    ap.add_argument("--session", required=True, type=str,
                    help="BO session dir containing trials/t*/")
    ap.add_argument("--output", required=True, type=str,
                    help="Output CSV path")
    args = ap.parse_args()

    session_dir = Path(args.session)
    trials_dir = session_dir / "trials"
    if not trials_dir.exists():
        raise SystemExit(f"no trials/ dir under {session_dir}")

    trial_dirs = sorted(p for p in trials_dir.glob("t*") if p.is_dir())
    rows: List[Dict] = []
    n_ok = 0
    n_skip = 0

    for td in trial_dirs:
        rec = recompute_trial(td)
        if rec is None:
            n_skip += 1
            continue
        if rec["cost_new"] is None:
            n_skip += 1
        else:
            n_ok += 1
        rows.append(rec)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "trial_id", "theta_kp", "theta_zeta", "cost_new",
        "cost_lag", "cost_ts", "cost_resid", "cost_ess", "cost_overshoot",
        "lag_deg", "ts_ms", "resid_std_deg", "ess_deg", "overshoot_pct",
        "watchdog_ok", "skip_reason",
    ]

    with open(out_path, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            bd = r.get("cost_breakdown") or {}
            w.writerow({
                "trial_id": r["trial_id"],
                "theta_kp": f"{r['theta_kp']:.6f}",
                "theta_zeta": f"{r['theta_zeta']:.6f}",
                "cost_new": (f"{r['cost_new']:.6f}" if r["cost_new"] is not None else ""),
                "cost_lag": f"{bd.get('lag', ''):.6f}" if bd else "",
                "cost_ts": f"{bd.get('ts', ''):.6f}" if bd else "",
                "cost_resid": f"{bd.get('resid', ''):.6f}" if bd else "",
                "cost_ess": f"{bd.get('ess', ''):.6f}" if bd else "",
                "cost_overshoot": f"{bd.get('overshoot', ''):.6f}" if bd else "",
                "lag_deg": f"{r['lag_deg']:.6f}" if r["lag_deg"] is not None else "",
                "ts_ms": f"{r['ts_ms']:.2f}" if r["ts_ms"] is not None else "",
                "resid_std_deg": f"{r['resid_std_deg']:.6f}" if r["resid_std_deg"] is not None else "",
                "ess_deg": f"{r['ess_deg']:.6f}" if r["ess_deg"] is not None else "",
                "overshoot_pct": f"{r['overshoot_pct']:.2f}" if r["overshoot_pct"] is not None else "",
                "watchdog_ok": "1" if r["watchdog_ok"] else "0",
                "skip_reason": r.get("skip_reason", ""),
            })

    print(f"[recompute_cost_v4] trials processed: {len(rows)}")
    print(f"[recompute_cost_v4] usable: {n_ok}  skipped: {n_skip}")
    print(f"[recompute_cost_v4] output: {out_path}")


if __name__ == "__main__":
    main()
