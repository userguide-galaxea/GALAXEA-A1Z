#!/usr/bin/env python3
"""CLI entry point for the BO parameter optimisation pipeline (SOP-11).

Usage:
    python -m a1z.analysis.script.run_optimize --joint 6 --name phaseA-J6 --n-trials 40
    python -m a1z.analysis.script.run_optimize --joints 4,5,6 --name eeRefine-J456 --n-trials 20
    python -m a1z.analysis.script.run_optimize --resume /path/to/session-dir
"""
from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="BO-based MIT controller parameter optimisation (SOP-11)")
    ap.add_argument("--joint", type=int, choices=range(1, 7), metavar="1-6",
                    help="Joint to optimise (1-based, Phase A)")
    ap.add_argument("--joints", type=str, default=None, metavar="4,5,6",
                    help="Comma-separated 1-based joint subset for the "
                         "E-segment multi-joint refine (SOP-11 §6.3). "
                         "Mutually exclusive with --joint.")
    ap.add_argument("--name", type=str, default=None,
                    help="Session name (used in output dir)")
    ap.add_argument("--n-trials", type=int, default=40,
                    help="Number of BO trials (default 40)")
    ap.add_argument("--phase", choices=["A", "B"], default="A",
                    help="Optimisation phase (default A)")
    ap.add_argument("--vel-ff", action="store_true", default=True,
                    help="Enable velocity feedforward (default on)")
    ap.add_argument("--no-vel-ff", action="store_false", dest="vel_ff")
    ap.add_argument("--can", default="can0",
                    help="CAN channel (default can0)")
    ap.add_argument("--resume", type=str, default=None, metavar="SESSION_DIR",
                    help="Resume from an existing session directory")
    ap.add_argument("--warm-start", type=str, default=None, metavar="RELABEL_CSV",
                    help="Inject re-labelled historic trials (output CSV of "
                         "recompute_cost_v4.py) into a freshly created study "
                         "(devlog 2026-07-30 Q4-9-2). Ignored on --resume.")
    ap.add_argument("--watchdog-calib", type=str, default=None,
                    help="Path to watchdog thresholds JSON (default: "
                         "$TEST_LOG_ROOT/02-a1z/05-b1-watchdog-calib/v4/watchdog_thresholds-active.json)")
    # Integral parameters (forwarded to OptimizeRunner -> get_a1z_robot)
    ap.add_argument("--ki-level", default="K0",
                    help="Integral level (K0=off, K1-K3=enabled)")
    ap.add_argument("--integral-joints", default=None,
                    help="Comma-separated 1-based joint indices for integral")
    return ap.parse_args()


def main() -> None:
    args = _parse_args()

    # Hard check TEST_LOG_ROOT (SOP-07 compliance)
    test_log_root = os.environ.get("TEST_LOG_ROOT")
    if test_log_root is None:
        print("ERROR: TEST_LOG_ROOT not set. Export it first:\n"
              "  export TEST_LOG_ROOT=/path/to/02-test-log",
              file=sys.stderr)
        sys.exit(1)

    opt_root = Path(test_log_root) / "02-a1z" / "02-para-opt"
    opt_root.mkdir(parents=True, exist_ok=True)

    # Joint subset (E-segment refine) parsing
    joints1 = None
    if args.joints:
        try:
            joints1 = sorted({int(x) for x in args.joints.split(",") if x.strip()})
        except ValueError:
            print(f"ERROR: --joints must be comma-separated ints, got "
                  f"{args.joints!r}", file=sys.stderr)
            sys.exit(1)
        if len(joints1) < 2 or any(j < 1 or j > 6 for j in joints1):
            print("ERROR: --joints needs >=2 joints in 1..6 "
                  "(use --joint for a single joint)", file=sys.stderr)
            sys.exit(1)
    if args.joint is not None and joints1 is not None:
        print("ERROR: --joint and --joints are mutually exclusive",
              file=sys.stderr)
        sys.exit(1)

    if args.resume:
        session_dir = Path(args.resume)
        if not session_dir.exists():
            print(f"ERROR: session dir not found: {session_dir}", file=sys.stderr)
            sys.exit(1)
        # Read study.json for joint info
        import json
        study_json = session_dir / "study.json"
        if study_json.exists():
            with open(study_json) as f:
                info = json.load(f)
            joint1 = info.get("joint", args.joint)
            joints1 = joints1 or info.get("joints")
            phase = info.get("phase", args.phase)
            vel_ff = info.get("vel_ff", args.vel_ff)
        else:
            if args.joint is None and joints1 is None:
                print("ERROR: --joint/--joints required when resuming without "
                      "study.json", file=sys.stderr)
                sys.exit(1)
            joint1 = args.joint
            phase = args.phase
            vel_ff = args.vel_ff
    else:
        if args.joint is None and joints1 is None:
            print("ERROR: --joint or --joints is required for new sessions",
                  file=sys.stderr)
            sys.exit(1)
        joint1 = args.joint
        phase = args.phase
        vel_ff = args.vel_ff
        if joints1:
            name = args.name or f"phaseE-J{''.join(str(j) for j in joints1)}"
        else:
            name = args.name or f"phase{phase}-J{joint1}"
        date = datetime.now().strftime("%Y-%m-%d")
        session_dir = opt_root / f"{date}-run-opt-{name}"

    # Integral kwargs
    integral_joints = None
    if args.integral_joints:
        integral_joints = [int(x) for x in args.integral_joints.split(",") if x.strip()]

    # Watchdog calibration: default to the v3 active thresholds
    watchdog_calib_path = None
    if args.watchdog_calib:
        watchdog_calib_path = Path(args.watchdog_calib)
    elif test_log_root:
        default_wd = (
            Path(test_log_root)
            / "02-a1z"
            / "05-b1-watchdog-calib"
            / "v4"
            / "watchdog_thresholds-active.json"
        )
        if default_wd.exists():
            watchdog_calib_path = default_wd

    print(f"[run_optimize] Session: {session_dir}")
    print(f"[run_optimize] Joint={joint1}  Joints={joints1}  Phase={phase}  "
          f"n_trials={args.n_trials}  vel_ff={vel_ff}")
    if watchdog_calib_path:
        print(f"[run_optimize] Watchdog calib: {watchdog_calib_path}")

    from a1z.analysis.optimize.study import OptStudy

    study = OptStudy(
        session_dir=session_dir,
        joint1=joint1,
        joints1=joints1,
        n_trials=args.n_trials,
        phase=phase,
        vel_ff=vel_ff,
        can_channel=args.can,
        warm_start_path=Path(args.warm_start) if args.warm_start else None,
        integral_level=args.ki_level,
        integral_joints=integral_joints,
        watchdog_calib_path=watchdog_calib_path,
    )
    summary = study.run()
    print(f"\n[run_optimize] Complete. Best gains at: {session_dir / 'best_gains.json'}")


if __name__ == "__main__":
    main()
