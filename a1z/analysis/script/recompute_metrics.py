#!/usr/bin/env python3
"""Offline v2-metrics recompute over archived + supplementary runs (SOP-08 P3).

No hardware. For each run dir under ``--root`` it reads the per-joint
``unit-J*-{square,triangle}.csv`` + ``meta.json``, recomputes the v2 metrics
(``metrics.triangle_metrics_v2`` / ``step_metrics_v2``), and writes an additive
``metrics-v2.json`` inside that run dir (never touching existing files —
SOP-08 §1.7). A flat ``metrics-v2-summary.csv`` (one row per run×joint) is
written to the gate product dir for the P4 evaluator.

Hold segments and the excitation window are recovered from the scripted
(noise-free) ``ref`` itself (``metrics.ref_speed_profile``), NOT from the
runtime ``excite_window`` — so a run recomputes identically regardless of what
slicing happened at capture time. Tolerant of:
  * K2 — 07-20-era CSVs without an ``eff`` column (eff sentinels → null);
  * K3 — era comes from the manifest, not the run;
  * missing/failed captures — a run with no triangle/square CSV, or an empty
    one, is recorded with a ``skipped_reason`` rather than crashing the batch
    ("宁可显式跳过,不可静默补零", SOP-08 R5).

Gate calipers (SOP-08 §9.2, finalized on the P1 synthetic rig):
  * ``--k-sigma`` 4.0    — ε = max(k·σ_floor, 2·q_step, ε_min);
  * ``--apex-excl-s`` 0.55 — kd=4-era apex/hold transient ≈ 3.4·τ (τ=kd/kp);
    the library default 0.2 s leaves the S3 J6 flip shoulders in (jump_p95
    2.1° > the G0-4 1° line). Recorded per run so P4 is auditable.

Usage:
    python -m a1z.analysis.script.recompute_metrics \\
        --root /path/to/02-test-log/02-a1z/01-output \\
        --out  /path/to/02-test-log/02-a1z/03-metrics-v2-gate/metrics-v2-summary.csv \\
        [--manifest .../runs-manifest.csv] [--glob '2026-07-2*'] \\
        [--apex-excl-s 0.55] [--k-sigma 4.0]
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import subprocess
from pathlib import Path

import numpy as np

from a1z.analysis import metrics as M

DEG = 180.0 / math.pi
GATE_K_SIGMA = 4.0
GATE_APEX_EXCL_S = 0.55        # SOP-08 §9.2-4 (kd=4-era, ≈3.4·τ)

# Fields lifted into the flat summary (order = column order).
_TRI_KEYS = [
    "lag_at_rate_deg", "lag_signed_deg", "k_v_s", "c_deg", "b_deg",
    "resid_std_deg", "rate_deg_s", "kv_c_separable", "fit_ok", "n_fit",
    "n_directions",
    "jump_count_v2", "jump_rate_hz_v2", "jump_mean_deg_v2", "jump_p95_deg",
    "jump_max_deg_v2", "n_legs",
    "eps_adapt_deg", "eps_source", "sigma_floor_deg", "q_step_deg",
    "q_step_source", "n_apex", "apex_removed_big_teeth",
    "eff_std_nm", "eff_hf_ratio",
]
_SQ_KEYS = [
    "overshoot_max_pct", "ts_max_ms", "ts_v2_max_ms",
    "ess_max_deg", "noise_floor_deg", "ess_ratio", "n_steps",
]


def _code_commit() -> dict:
    # GALAXEA-A1Z is subtree-integrated into the root repo (no own .git), so
    # HEAD is the root commit; scope the dirty check to the analysis dir via a
    # cwd-relative pathspec so the report records whether the recompute ran
    # against uncommitted metric code.
    analysis_dir = Path(M.__file__).resolve().parent  # .../a1z/analysis
    try:
        sha = subprocess.check_output(
            ["git", "-C", str(analysis_dir), "rev-parse", "HEAD"],
            text=True, stderr=subprocess.DEVNULL).strip()
        dirty = bool(subprocess.check_output(
            ["git", "-C", str(analysis_dir), "status", "--porcelain", "."],
            text=True, stderr=subprocess.DEVNULL).strip())
        return {"code_commit": sha, "analysis_dirty": dirty}
    except Exception:
        return {"code_commit": None, "analysis_dirty": None}


def _load_unit_csv(path: Path):
    """(t, ref, resp, eff|None) from a unit CSV. Raises on empty/malformed."""
    d = np.genfromtxt(path, delimiter=",", names=True)
    if d.dtype.names is None or d.size < 3:
        raise ValueError(f"empty or malformed CSV ({d.size} rows)")
    eff = np.asarray(d["eff"], float) if "eff" in d.dtype.names else None
    return (np.asarray(d["t"], float), np.asarray(d["ref"], float),
            np.asarray(d["resp"], float), eff)


def _hold_noise_floor_deg(t, ref, resp):
    """Noise-floor σ (deg) from the leading+trailing holds (SOP-08 §1.5).

    Thin wrapper over ``metrics.hold_noise_floor_from_trace`` (single source of
    truth, also used by run_test's v2 wiring).
    """
    return M.hold_noise_floor_from_trace(t, ref, resp)


def recompute_run(rundir: Path, *, k_sigma: float, apex_excl_s: float) -> dict:
    """Recompute v2 metrics for every joint/wave present in one run dir."""
    meta = json.loads((rundir / "meta.json").read_text())
    joints = sorted(
        {p.name.split("-")[1] for p in rundir.glob("unit-J*-*.csv")},
        key=lambda s: int(s[1:]))
    per_joint = {}
    for j in joints:
        jr = {}
        tri = rundir / f"unit-{j}-triangle.csv"
        sq = rundir / f"unit-{j}-square.csv"
        if tri.exists():
            try:
                t, ref, resp, eff = _load_unit_csv(tri)
                jr["triangle"] = M.triangle_metrics_v2(
                    t, ref, resp, eff=eff, k_sigma=k_sigma,
                    apex_excl_s=apex_excl_s)
            except Exception as e:  # noqa: BLE001
                jr["triangle"] = {"skipped_reason": f"{type(e).__name__}: {e}"}
        if sq.exists():
            try:
                t, ref, resp, _eff = _load_unit_csv(sq)
                nf = _hold_noise_floor_deg(t, ref, resp)
                steps = M.step_metrics_v2(t, ref, resp)
                jr["square"] = M.summarize_steps_v2(steps, noise_floor_deg=nf)
            except Exception as e:  # noqa: BLE001
                jr["square"] = {"skipped_reason": f"{type(e).__name__}: {e}"}
        if jr:
            per_joint[j] = jr
    return {
        "metrics_version": 2,
        "run_dir": rundir.name,
        "recompute": {
            "k_sigma": k_sigma, "apex_excl_s": apex_excl_s,
            **_code_commit(),
            "meta_gap_us": (meta.get("hardware") or {}).get("inter_cmd_gap_us"),
            "meta_gap_us_live": (meta.get("hardware") or {}).get(
                "inter_cmd_gap_us_live"),
        },
        "joints": per_joint,
    }


def _load_manifest(path: Path | None) -> dict:
    if not path or not path.exists():
        return {}
    return {r["run_dir"]: r for r in csv.DictReader(path.open())}


def _flat_rows(v2: dict, man_row: dict) -> list:
    """One flat summary row per joint (triangle + square fields side by side)."""
    rows = []
    for j, jr in v2["joints"].items():
        row = {
            "run_dir": v2["run_dir"],
            "joint": j,
            "era": man_row.get("era", ""),
            "gap_us": man_row.get("gap_us", ""),
            "gap_us_live": v2["recompute"].get("meta_gap_us_live", ""),
            "kp": man_row.get("kp_applied", ""),
            "kd": man_row.get("kd_applied", ""),
            "period_triangle_s": man_row.get("period_triangle_s", ""),
            "date": man_row.get("date", ""),
            "commit": man_row.get("commit", ""),
        }
        tri = jr.get("triangle", {})
        row["tri_skipped"] = tri.get("skipped_reason", "")
        for k in _TRI_KEYS:
            row[f"tri_{k}"] = tri.get(k, "")
        sq = jr.get("square", {})
        row["sq_skipped"] = sq.get("skipped_reason", "")
        for k in _SQ_KEYS:
            row[f"sq_{k}"] = sq.get(k, "")
        rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(description="Offline v2-metrics recompute (SOP-08 P3)")
    ap.add_argument("--root", required=True, help="dir holding <date>-run-<id>/ run dirs")
    ap.add_argument("--out", required=True, help="summary CSV path")
    ap.add_argument("--manifest", default=None, help="runs-manifest.csv for era/gap/kp join")
    ap.add_argument("--glob", default="*", help="run-dir name glob (default all)")
    ap.add_argument("--k-sigma", type=float, default=GATE_K_SIGMA)
    ap.add_argument("--apex-excl-s", type=float, default=GATE_APEX_EXCL_S)
    args = ap.parse_args()

    root = Path(args.root)
    man = _load_manifest(Path(args.manifest) if args.manifest else None)
    run_dirs = sorted(p for p in root.glob(args.glob) if p.is_dir())

    all_rows, n_ok, n_empty, n_err = [], 0, 0, 0
    for rd in run_dirs:
        if not (rd / "meta.json").exists():
            continue
        try:
            v2 = recompute_run(rd, k_sigma=args.k_sigma, apex_excl_s=args.apex_excl_s)
        except Exception as e:  # noqa: BLE001 — never let one run sink the batch
            print(f"[ERR ] {rd.name}: {type(e).__name__}: {e}")
            n_err += 1
            continue
        (rd / "metrics-v2.json").write_text(
            json.dumps(v2, indent=2, ensure_ascii=False, default=_json_default))
        if not v2["joints"]:
            n_empty += 1
            print(f"[skip] {rd.name}: no unit CSVs")
            continue
        n_ok += 1
        all_rows.extend(_flat_rows(v2, man.get(rd.name, {})))

    cols = (["run_dir", "joint", "era", "gap_us", "gap_us_live", "kp", "kd",
             "period_triangle_s", "date", "commit", "tri_skipped"]
            + [f"tri_{k}" for k in _TRI_KEYS]
            + ["sq_skipped"] + [f"sq_{k}" for k in _SQ_KEYS])
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=cols)
        w.writeheader()
        for r in all_rows:
            w.writerow({c: r.get(c, "") for c in cols})

    print(f"\n[recompute] runs with data: {n_ok}  empty: {n_empty}  errors: {n_err}")
    print(f"[recompute] summary rows (run×joint): {len(all_rows)} → {args.out}")
    print(f"[recompute] caliper: k_sigma={args.k_sigma} apex_excl_s={args.apex_excl_s}s")


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    raise TypeError(f"not JSON-serializable: {type(o)}")


if __name__ == "__main__":
    main()
