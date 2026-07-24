#!/usr/bin/env python3
"""Offline per-joint Coulomb-friction (τ̂_c) regression from triangle eff traces.

SOP-09 P0-6 (零机时离线作业): anchor the error-integral clamp τ_I,max = 1.2·τ̂_c
(SOP-09 §3) from the effort channel of existing triangle unit runs.

Method — direction-antisymmetric effort across sweep rates. During a constant-
speed triangle sweep the moving-phase torque is

    eff(q̇) ≈ g(q) + β_v·q̇ + τ_c·sign(q̇)

The gravity/bias term g(q) is position- (hence direction-) symmetric, so the
half-difference between the two sweep directions cancels it:

    D(rate) = ½·( mean eff[q̇>0] − mean eff[q̇<0] ) = τ_c + β_v·|q̇|_rate

A single triangle sweeps at ONE |q̇|, which makes q̇ and sign(q̇) collinear —
τ_c and β_v are then unidentifiable from one run (a naive per-run fit splits the
lag arbitrarily and does NOT reproduce the trusted J6≈0.13 Nm). With ≥2 sweep
rates for a joint, D vs |q̇| is a line whose **zero-speed intercept is τ̂_c**
(gravity-independent). This tool therefore groups inputs by joint and only
reports a τ̂_c for joints with ≥2 distinct rates; single-rate joints are flagged
UNRELIABLE and left NaN (SOP-09: 缺标定的关节不启用积分).

To calibrate a joint, run its triangle at ≥2 periods (cf. G0D-p2 / p8,
`--period-triangle 2` and `8`), then pass all the unit-J<k>-triangle.csv here.

Usage:
    python -m a1z.analysis.script.regress_tau_c FILE [FILE ...]
    python -m a1z.analysis.script.regress_tau_c --json out.json FILE ...
"""
from __future__ import annotations

import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

import numpy as np

_JOINT_RE = re.compile(r"unit-J(\d)-", re.IGNORECASE)
_SWEEP_FRAC = 0.5  # keep |q̇| > frac·peak → exclude apex/hold


def _load(path: Path):
    t, ref, eff = [], [], []
    with open(path) as fp:
        rdr = csv.DictReader(fp)
        if "eff" not in (rdr.fieldnames or []):
            raise ValueError(f"{path}: no 'eff' column (schema {rdr.fieldnames})")
        for row in rdr:
            t.append(float(row["t"])); ref.append(float(row["ref"]))
            eff.append(float(row["eff"]))
    return np.array(t), np.array(ref), np.array(eff)


def sweep_point(t, ref, eff, sweep_frac: float = _SWEEP_FRAC):
    """One (|q̇|_med, D) point for a single triangle run.

    D = ½·(mean eff over q̇>0 − mean eff over q̇<0) over the sweep phase — the
    direction-antisymmetric effort (gravity-cancelled) = τ_c + β_v·|q̇|.
    """
    qd = np.gradient(ref, t)
    peak = np.max(np.abs(qd))
    if peak <= 0:
        raise ValueError("reference has no motion (q̇≡0)")
    m = np.abs(qd) > sweep_frac * peak
    up, dn = m & (qd > 0), m & (qd < 0)
    if up.sum() < 5 or dn.sum() < 5:
        raise ValueError(f"too few directional sweep samples (+{int(up.sum())}/-{int(dn.sum())})")
    D = 0.5 * (eff[up].mean() - eff[dn].mean())
    return float(np.median(np.abs(qd[m]))), float(D), int(m.sum())


def fit_joint(points):
    """points = [(|q̇|, D), ...]. ≥2 distinct rates → (τ_c, β_v, r2); else None."""
    qd = np.array([p[0] for p in points]); D = np.array([p[1] for p in points])
    if len(np.unique(np.round(qd, 4))) < 2:
        return None
    A = np.column_stack([np.ones(len(qd)), qd])
    coef, *_ = np.linalg.lstsq(A, D, rcond=None)
    resid = D - A @ coef
    ss_tot = float(np.sum((D - D.mean()) ** 2))
    r2 = 1.0 - float(np.sum(resid ** 2)) / ss_tot if ss_tot > 0 else float("nan")
    return {"tau_c_nm": abs(float(coef[0])), "beta_v": float(coef[1]),
            "r2": r2, "n_rates": len(qd)}


def main():
    ap = argparse.ArgumentParser(description="offline multi-rate τ_c regression (SOP-09 P0-6)")
    ap.add_argument("files", nargs="+", help="unit-J<k>-triangle.csv files (≥2 rates/joint)")
    ap.add_argument("--json", default=None, help="write τ_c vector (6,) + detail as JSON")
    args = ap.parse_args()

    by_joint = defaultdict(list)
    for f in args.files:
        p = Path(f)
        mo = _JOINT_RE.search(p.name)
        if not mo:
            print(f"[skip] cannot parse joint from {p.name}"); continue
        try:
            pt = sweep_point(*_load(p))
        except Exception as e:  # noqa: BLE001
            print(f"[{p.name}] FAILED: {e}"); continue
        by_joint[int(mo.group(1))].append((*pt, p.name))

    tau_c = np.full(6, np.nan)
    print(f"\n{'joint':<6}{'rates':>6}{'tau_c(Nm)':>11}{'beta_v':>9}{'R2':>7}   status")
    print("-" * 60)
    detail = {}
    for j1 in sorted(by_joint):
        pts = by_joint[j1]
        fit = fit_joint([(q, d) for q, d, _n, _name in pts])
        if fit is None:
            q, d, _n, _name = pts[0]
            print(f"J{j1:<5}{len(pts):>6}{'—':>11}{'—':>9}{'—':>7}   "
                  f"UNRELIABLE (1 rate, |q̇|={q:.3f}, D={d:.3f}) → NaN/disabled")
            detail[f"J{j1}"] = {"single_rate_D_nm": d, "qd": q, "reliable": False}
        else:
            tau_c[j1 - 1] = fit["tau_c_nm"]
            print(f"J{j1:<5}{fit['n_rates']:>6}{fit['tau_c_nm']:>11.4f}"
                  f"{fit['beta_v']:>9.3f}{fit['r2']:>7.3f}   OK")
            detail[f"J{j1}"] = {**fit, "reliable": True}

    print(f"\n_TAU_C_HAT = {np.array2string(tau_c, precision=4, separator=', ')}")
    print("NaN joints stay disabled (SOP-09: 缺标定的关节不启用积分).")
    if args.json:
        Path(args.json).write_text(json.dumps(
            {"tau_c_nm": tau_c.tolist(), "detail": detail}, indent=2))
        print(f"[wrote] {args.json}")


if __name__ == "__main__":
    main()
