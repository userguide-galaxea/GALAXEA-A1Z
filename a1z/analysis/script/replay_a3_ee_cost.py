#!/usr/bin/env python3
"""E5 offline self-test (devlog 2026-08-01 计划 E5-①, zero hardware).

Replays the archived A3 Pdef/PphA EE trajectories through the v10 EE cost:
``J_ee(PphA) < J_ee(Pdef)`` MUST hold — otherwise the EE metric definitions
are wrong and the E-segment goes back to E1 before any machine time is spent.

The A3 archives are EE-only runs (no joint triangle/step legs), so the joint
leg of J_total is not recomputed here; the ordering conclusion for J_total
follows from J_ee ordering plus the Phase A A3 result (PphA improved 5/6
joint RMSE, devlog 2026-08-01「Phase A 收官」).

Usage:
    python -m a1z.analysis.script.replay_a3_ee_cost \
        --pdef $TEST_LOG_ROOT/02-a1z/01-output/2026-08-01-run-A3-Pdef-ee-r1-ee-01 \
        --ppha $TEST_LOG_ROOT/02-a1z/01-output/2026-08-01-run-A3-PphA-ee-r1-ee-01

Exit code 0 = ordering holds, 1 = violated (metric definitions suspect).
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

from a1z.analysis.metrics import ee_refine_metrics
from a1z.analysis.optimize.cost_spec import (
    COST_SPEC_VERSION,
    W_EE_TOTAL,
    W_JT_TOTAL,
    compute_ee_cost,
)
from a1z.analysis.report import read_ee_pose_csv

# The A3 archives are xz-plane circles (r=40mm, T=8s x 2cyc, no terminal
# hold) — metrics use the devlog 2026-08-01 E1 smoke口径: normal = y,
# tail_frac = 0.1 (trailing window, not a true hold).
NORMAL = np.array([0.0, 1.0, 0.0])
TAIL_FRAC = 0.1


def _default_dir(name: str) -> str:
    root = os.environ.get("TEST_LOG_ROOT")
    if root is None:
        return ""
    return str(Path(root) / "02-a1z" / "01-output" / name)


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pdef", default=_default_dir("2026-08-01-run-A3-Pdef-ee-r1-ee-01"),
                    help="A3 run dir with the pre-Phase-A SDK default gains")
    ap.add_argument("--ppha", default=_default_dir("2026-08-01-run-A3-PphA-ee-r1-ee-01"),
                    help="A3 run dir with the Phase A BO best (frozen) gains")
    return ap.parse_args()


def _metrics_for(run_dir: Path) -> dict:
    t_ref, T_ref = read_ee_pose_csv(run_dir / "ee-traj-ref.csv")
    t_resp, T_resp = read_ee_pose_csv(run_dir / "ee-traj-response.csv")
    if len(t_ref) != len(t_resp):
        n = min(len(t_ref), len(t_resp))
        T_ref, T_resp = T_ref[:n], T_resp[:n]
        t_ref = t_ref[:n]
    dt = float(np.median(np.diff(t_ref)))
    fs = 1.0 / dt
    return ee_refine_metrics(T_ref, T_resp, fs, NORMAL, tail_frac=TAIL_FRAC)


def main() -> int:
    args = _parse_args()
    pdef_dir = Path(args.pdef)
    ppha_dir = Path(args.ppha)
    for d in (pdef_dir, ppha_dir):
        if not (d / "ee-traj-ref.csv").exists():
            print(f"ERROR: ee-traj-ref.csv not found in {d}", file=sys.stderr)
            return 1

    rows = {}
    for tag, d in (("Pdef", pdef_dir), ("PphA", ppha_dir)):
        m = _metrics_for(d)
        cost, bd = compute_ee_cost(m)
        rows[tag] = (m, cost, bd)

    print(f"[E5] cost_spec {COST_SPEC_VERSION} — A3 archive replay "
          f"(xz circle, normal=y, tail_frac={TAIL_FRAC})")
    hdr = f"{'metric':<24}{'Pdef':>12}{'PphA':>12}{'dir':>6}"
    print(hdr)
    print("-" * len(hdr))
    for key in ("terminal_pos_mm", "terminal_ang_deg",
                "normal_jitter_std_mm", "phase_lag_ms"):
        a = rows["Pdef"][0][key]
        b = rows["PphA"][0][key]
        print(f"{key:<24}{a:>12.3f}{b:>12.3f}{'OK' if b < a else 'BAD':>6}")
    print("-" * len(hdr))
    c_def = rows["Pdef"][1]
    c_pha = rows["PphA"][1]
    print(f"{'J_ee (v10)':<24}{c_def:>12.4f}{c_pha:>12.4f}"
          f"{'OK' if c_pha < c_def else 'BAD':>6}")
    for tag in ("Pdef", "PphA"):
        bd = rows[tag][2]
        print(f"  {tag} breakdown: " + "  ".join(
            f"{k}={v:.3f}" for k, v in bd.items()))
    print(f"[E5] note: J_total = {W_EE_TOTAL}*J_ee + {W_JT_TOTAL}*J_joint; "
          f"the A3 archives carry no joint legs — J_joint ordering rests on "
          f"the Phase A A3 result (PphA improved 5/6 joint RMSE).")

    ok = c_pha < c_def
    print(f"[E5] {'PASS' if ok else 'FAIL'}: J_ee(PphA) < J_ee(Pdef) "
          f"{'holds' if ok else 'VIOLATED — metric definitions suspect, back to E1'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
