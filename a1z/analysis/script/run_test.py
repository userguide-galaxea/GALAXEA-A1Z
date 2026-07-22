#!/usr/bin/env python3
"""CLI entry for the a1z.analysis test module (SOP-03 §6).

Modes:
    joint   — one joint's dynamic-response unit test (square + triangle)
    joints  — all joints in sequence (J1→J6), each a full transit cycle
    ee      — end-effector trajectory tracking RMSE only
    all     — all joint unit tests + EE tracking, one run dir

--dry-run: no hardware. For ee, runs the offline IK pipeline + gate and writes
joints-traj-ref.csv + IK preview PNG. For joint(s), prints the safe windows.

Output root is overridable via the TEST_LOG_ROOT env var (see
01-docs/04-sops/07-测试数据管理-SOP.md): if set, run dirs go to
$TEST_LOG_ROOT/02-a1z/01-output/ instead of the local a1z/analysis/output/.
"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import subprocess
from pathlib import Path

import numpy as np

from a1z.analysis import ee_pipeline as ee
from a1z.analysis import metrics as M
from a1z.analysis import report as R
from a1z.analysis import safety

DEG = 180.0 / math.pi
_OUTPUT_ROOT = (Path(os.environ["TEST_LOG_ROOT"]) / "02-a1z" / "01-output"
                if os.environ.get("TEST_LOG_ROOT")
                else Path(__file__).resolve().parent.parent / "output")


def _parse_gains(s: str | None):
    if s is None:
        return None
    vals = [float(x) for x in s.split(",")]
    if len(vals) == 1:
        return ("scalar", vals[0])
    if len(vals) == 6:
        return ("vector", np.array(vals))
    raise argparse.ArgumentTypeError("gain must be 1 or 6 comma-separated values")


def _resolve_gains(spec, joint1, default):
    """Return a full (6,) gain vector from a --kp/--kd spec.

    scalar → override only the excited joint (joints modes use per-joint calls,
    so 'scalar' targets whichever joint is under test); vector → replace all.
    """
    if spec is None:
        return None
    kind, val = spec
    if kind == "vector":
        return val
    g = np.asarray(default, dtype=float).copy()
    if joint1 is not None:
        g[joint1 - 1] = val
    return g


def _git_info() -> dict:
    root = Path(__file__).resolve().parents[3]  # GALAXEA-A1Z repo root
    try:
        sha = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"],
                                      text=True).strip()
        dirty = bool(subprocess.check_output(
            ["git", "-C", str(root), "status", "--porcelain"], text=True).strip())
        return {"galaxea_a1z_commit": sha, "dirty": dirty}
    except Exception:
        return {"galaxea_a1z_commit": None, "dirty": None}


def build_args():
    ap = argparse.ArgumentParser(description="a1z.analysis test runner")
    ap.add_argument("--mode", required=True, choices=["joint", "joints", "ee", "all"])
    ap.add_argument("--run", default=None, help="run id (e.g. 01); required unless "
                    "joint(s) --dry-run")
    ap.add_argument("--date", default=datetime.date.today().isoformat())
    ap.add_argument("--can", default="can0")
    ap.add_argument("--joint", type=int, default=None, help="1-based (mode joint)")
    ap.add_argument("--wave", default="both", choices=["square", "triangle", "both"])
    ap.add_argument("--amp-deg", type=float, default=15.0)
    ap.add_argument("--period", type=float, default=4.0)
    ap.add_argument("--period-triangle", type=float, default=None,
                    help="triangle-only sweep period (s); default = --period "
                         "(scan-rate diagnostic, SOP-03 §10.16)")
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--margin-deg", type=float, default=5.0)
    ap.add_argument("--edge-rate", type=float, default=240.0)  # reserved (square slew)
    ap.add_argument("--kp", type=_parse_gains, default=None)
    ap.add_argument("--kd", type=_parse_gains, default=None)
    ap.add_argument("--ee-kind", default="circle", choices=["circle", "line"])
    ap.add_argument("--plane", default="xz", choices=["xy", "xz", "yz"])
    ap.add_argument("--radius", type=float, default=0.04)
    ap.add_argument("--period-ee", type=float, default=8.0)
    ap.add_argument("--cycles-ee", type=int, default=2)
    ap.add_argument("--q-nom-deg", default="-20,35,-25,-25,0,0")
    ap.add_argument("--transit-speed", type=float, default=15.0)
    ap.add_argument("--jump-eps", type=float, default=0.1)
    ap.add_argument("--gap-us", type=float, default=None,
                    help="inter-command CAN pacing gap (µs), forwarded to "
                         "get_a1z_robot(inter_cmd_gap_us=…); default None = SDK "
                         "default 250 (SOP-06), 0 = unpaced/B0 fault regime. "
                         "The resolved value is machine-recorded in meta.json "
                         "(SOP-08 K1) — the SDK reads no env var since 0584cf4.")
    ap.add_argument("--sample-hz", type=int, default=100)
    ap.add_argument("--dry-run", action="store_true")
    return ap.parse_args()


# ---------------------------------------------------------------------------
def _joints_for_mode(args):
    if args.mode == "joint":
        if args.joint is None:
            raise SystemExit("--joint required for --mode joint")
        return [args.joint]
    if args.mode in ("joints", "all"):
        return [1, 2, 3, 4, 5, 6]
    return []


def _waves(args):
    return ("square", "triangle") if args.wave == "both" else (args.wave,)


def _stamp_pd_defaults(meta, robot):
    """Record the robot's live default PD gains into meta once it's started.

    Also reads back the live inter-command CAN gap from the motor chain
    (`inter_cmd_gap_us_live`) so meta carries the *applied* pacing truth, not
    just the harness-side resolved value — divergence between the two flags a
    harness/SDK mismatch (SOP-08 K1/R8).
    """
    info = robot.get_robot_info()
    meta["pd"]["default_kp"] = info["default_kp"].tolist()
    meta["pd"]["default_kd"] = info["default_kd"].tolist()
    chain = getattr(robot, "_motor_chain", None)
    if chain is not None and hasattr(chain, "inter_cmd_gap_s"):
        meta["hardware"]["inter_cmd_gap_us_live"] = chain.inter_cmd_gap_s * 1e6


def _record_applied(meta, key, kp, kd):
    """Record the exact gains commanded for one joint / phase (as applied)."""
    meta["pd"]["applied"][key] = {
        "kp": np.asarray(kp, dtype=float).tolist(),
        "kd": np.asarray(kd, dtype=float).tolist(),
    }


def run_joint_units(args, rundir, meta, result):
    """Run joint unit tests, write per-joint CSV/PNG, fill result['unit_tests']."""
    from a1z.analysis.runner import JointUnitTestRunner
    waves = _waves(args)
    runner = JointUnitTestRunner(
        args.can, amp_deg=args.amp_deg, margin_deg=args.margin_deg,
        period=args.period, period_triangle=args.period_triangle,
        cycles=args.cycles, waves=waves,
        edge_rate_deg_s=args.edge_rate,
        sample_hz=args.sample_hz, transit_speed_deg_s=args.transit_speed,
        inter_cmd_gap_us=args.gap_us)
    unit = {}
    failed = []
    try:
        runner.start()
        _stamp_pd_defaults(meta, runner.robot)
        for j1 in _joints_for_mode(args):
            kp = _resolve_gains(args.kp, j1, runner.robot.get_robot_info()["default_kp"])
            kd = _resolve_gains(args.kd, j1, runner.robot.get_robot_info()["default_kd"])
            runner.kp, runner.kd = kp, kd
            # kp/kd are None when no override → the joint runs on defaults; record
            # the actual 6-vector applied (effective_gains resolves None→default).
            eff_kp, eff_kd = runner.effective_gains()
            _record_applied(meta, f"J{j1}", eff_kp, eff_kd)
            # Per-joint fault isolation: a transit/arrival fail-safe (e.g. J6
            # self-collision) aborts THIS joint but must not sink the whole run.
            # run_joint's own finally has already slow-returned the arm to zero,
            # so the next joint starts from a safe pose. The failure is recorded
            # and surfaced; remaining joints + EE (mode=all) still produce data.
            try:
                out = runner.run_joint(j1)
                unit[f"J{j1}"] = _process_joint(args, rundir, j1, out)
            except Exception as e:  # noqa: BLE001
                if runner.robot.is_estopped:
                    raise  # estop is global — do not soldier on into more motion
                msg = f"{type(e).__name__}: {e}"
                print(f"[J{j1}] SKIPPED — {msg}")
                unit[f"J{j1}"] = {"error": msg}
                failed.append(j1)
    finally:
        runner.shutdown()
    result["unit_tests"] = unit
    if failed:
        result["joint_failures"] = failed
        print(f"[run] joint unit tests skipped: {['J%d' % j for j in failed]}")


def _process_joint(args, rundir, j1, out) -> dict:
    """Metrics + CSV + PNG for one joint's waveform captures."""
    jr = {}
    plot_payload = {}
    for name, cap in out.items():
        t, ref, resp, tr = cap["t"], cap["ref"], cap["resp"], cap["traj"]
        j = j1 - 1
        ref_j, resp_j = ref[:, j], resp[:, j]
        # Same-tick effort of the excited joint (Nm) — the stick-slip vs
        # command-path discriminator (SOP-03 §5.4); recorded, not metricized.
        eff_j = cap["eff"][:, j] if cap.get("eff") is not None else None
        R.write_unit_csv(rundir.path(f"unit-J{j1}-{name}.csv"), t, ref_j, resp_j, eff_j)
        if name == "square":
            # Full trace: the leading hold is flat (no false edge) but keeping it
            # lets the rising edge at hold_pre be detected, and the trailing hold
            # gives the last step room to settle. Slicing at hold_pre would clip
            # the first edge.
            steps = M.step_metrics(t, ref_j, resp_j)
            m = M.summarize_steps(steps)
        else:
            t0, t1 = tr.excite_window
            seg = (t >= t0) & (t <= t1)
            m = M.triangle_metrics(t[seg], ref_j[seg], resp_j[seg],
                                   jump_eps_deg=args.jump_eps)
        jr[name] = m
        plot_payload[name] = dict(t=t, ref=ref_j, resp=resp_j, eff=eff_j, metrics=m)
    R.plot_unit(rundir.path(f"unit-J{j1}.png"), j1,
                plot_payload.get("square"), plot_payload.get("triangle"))
    return jr


def run_ee(args, rundir, meta, result, *, dry_run: bool):
    """EE tracking (or dry-run gate only). Fills result['ee'] + joints_rmse_deg."""
    from a1z.analysis.runner import EETrackingRunner
    q_nom_deg = [float(x) for x in args.q_nom_deg.split(",")]
    # EE tracking honors only a full 6-vector override (a scalar --kp targets a
    # single tested joint, which is meaningless for a whole-arm trajectory);
    # otherwise it runs on SDK defaults.
    ee_kp = args.kp[1] if args.kp and args.kp[0] == "vector" else None
    ee_kd = args.kd[1] if args.kd and args.kd[0] == "vector" else None
    runner = EETrackingRunner(
        args.can, q_nom_deg=q_nom_deg, ee_kind=args.ee_kind, plane=args.plane,
        radius=args.radius, period=args.period_ee, cycles=args.cycles_ee,
        sample_hz=args.sample_hz, transit_speed_deg_s=args.transit_speed,
        kp=ee_kp, kd=ee_kd, inter_cmd_gap_us=args.gap_us)
    off = runner.solve_offline()
    g = off["gate"]
    lim = safety.effective_limits(safety.LIMIT_BUFFER_RAD, ee.urdf_limits(runner.kin))
    meta["ee_traj"]["ee_frame"] = ee.ee_frame_name(runner.kin)
    print("[ee] dry-run gate:", g.summary())

    # Always emit the IK'd joint reference + preview for inspection.
    R.write_joint_csv(rundir.path("joints-traj-ref.csv"), off["t"], off["q_ref"])
    R.plot_ik_preview(rundir.path("ik-preview.png"), off["t"], off["q_ref"], lim)
    result.setdefault("ee", {})["ik_convergence_rate"] = g.ik_convergence_rate

    if dry_run:
        result["ee"]["gate_passed"] = g.passed
        if not g.passed:
            print("[ee] GATE FAILED — fix params before live run (SOP-03 §4.4)")
        return
    if not g.passed:
        raise SystemExit("[ee] gate failed; aborting live run (see summary above)")

    try:
        runner.start()
        _stamp_pd_defaults(meta, runner.robot)
        eff_kp, eff_kd = runner.effective_gains()
        _record_applied(meta, "ee", eff_kp, eff_kd)
        live = runner.run(off)
    finally:
        runner.shutdown()

    t = live["t"]
    T_ref, T_resp = live["T_ref"], live["T_resp"]
    dp, e_pos, e_ang = M.ee_errors(T_ref, T_resp)
    ee_m = M.ee_rmse(T_ref, T_resp)
    ee_m["ik_convergence_rate"] = g.ik_convergence_rate
    result["ee"].update(ee_m)
    # joint RMSE over the tracking window (response vs IK'd reference).
    jrmse = [M.rmse(live["q_resp"][:, i] - live["q_ref"][:, i]) * DEG for i in range(6)]
    result["joints_rmse_deg"] = jrmse

    # CSVs
    R.write_ee_pose_csv(rundir.path("ee-traj-ref.csv"), t, T_ref)
    R.write_ee_pose_csv(rundir.path("ee-traj-response.csv"), t, T_resp)
    R.write_ee_error_csv(rundir.path("ee-traj-error.csv"), t, dp, e_pos, e_ang)
    R.write_joint_csv(rundir.path("joints-traj-response.csv"), t, live["q_resp"])
    R.write_joint_csv(rundir.path("joints-traj-error.csv"), t,
                      live["q_resp"] - live["q_ref"])
    # PNGs
    rmse_txt = (f"pos RMSE={ee_m['pos_rmse_mm']:.3f}mm (max {ee_m['pos_max_mm']:.3f})\n"
                f"ang RMSE={ee_m['ang_rmse_deg']:.3f}° (max {ee_m['ang_max_deg']:.3f})")
    R.plot_ee_traj(rundir.path("ee-traj.png"), args.plane, T_ref, T_resp,
                   t, e_pos, e_ang, rmse_txt)
    R.plot_joints_traj(rundir.path("joints-traj.png"), t, live["q_ref"], live["q_resp"])
    notes = [f"RMSE={jrmse[i]:.3f}°" for i in range(6)]
    R.plot_joints_error(rundir.path("joints-traj-error.png"), t,
                        live["q_ref"], live["q_resp"], notes)


def build_meta(args) -> dict:
    # Machine-record the CAN pacing gap per run (SOP-08 K1): resolve --gap-us
    # through the same helper the robot factory uses, so meta carries the value
    # the run actually constructs with (None → SDK default 250 µs). A live
    # readback (`inter_cmd_gap_us_live`) is added at robot start by
    # _stamp_pd_defaults; dry runs carry only the resolved value.
    from a1z.robots.get_robot import _resolve_inter_cmd_gap_us
    lim = safety.effective_limits()
    eff = {}
    for j1 in safety.SAFE_RANGES_DEG:
        lo, hi = safety.clamp_wave_window(j1, amp_deg=args.amp_deg, margin_deg=args.margin_deg)
        eff[f"J{j1}"] = [round(lo * DEG, 2), round(hi * DEG, 2)]
    return {
        "date": args.date, "run_id": args.run, "mode": args.mode,
        "dry_run": args.dry_run,
        "git": _git_info(),
        "hardware": {"can_channel": args.can, "sample_hz": args.sample_hz,
                     "sdk_loop_hz": 250,
                     "inter_cmd_gap_us": _resolve_inter_cmd_gap_us(args.gap_us)},
        "pd": {
            # default_kp/kd read back from the robot at start() (§2.2);
            # applied[...] = the exact per-joint / per-phase gains commanded.
            "default_kp": None, "default_kd": None,
            "override": {"kp": args.kp[1].tolist() if args.kp and args.kp[0] == "vector"
                         else (args.kp[1] if args.kp else None),
                         "kd": args.kd[1].tolist() if args.kd and args.kd[0] == "vector"
                         else (args.kd[1] if args.kd else None)},
            "applied": {},
        },
        "safety": {
            "safe_ranges_deg": {f"J{j}": list(v) for j, v in safety.SAFE_RANGES_DEG.items()},
            "preconditions_deg": {f"J{j}": {f"J{k}": val for k, val in p.items()}
                                  for j, p in safety.PRECONDITIONS_DEG.items()},
            "margin_deg": args.margin_deg,
            "effective_wave_deg": eff,
            "transit_speed_deg_s": args.transit_speed,
        },
        "excitation": {"amp_deg": args.amp_deg, "period_s": args.period,
                       "period_triangle_s": (args.period_triangle
                                              if args.period_triangle is not None
                                              else args.period),
                       "cycles": args.cycles, "edge_rate_deg_s": args.edge_rate},
        "ee_traj": {"kind": args.ee_kind, "plane": args.plane, "radius_m": args.radius,
                    "period_s": args.period_ee, "cycles": args.cycles_ee,
                    "q_nom_deg": [float(x) for x in args.q_nom_deg.split(",")],
                    "ee_frame": None,
                    "ik": {"pos_threshold": 1e-4, "ori_threshold": 1e-3, "max_iters": 500}},
    }


def main():
    args = build_args()

    # dry-run for joint modes: just print windows, no run dir, no hardware.
    if args.dry_run and args.mode in ("joint", "joints"):
        print("[dry-run] safe windows (effective wave, deg):")
        for j1 in _joints_for_mode(args):
            lo, hi = safety.clamp_wave_window(j1, amp_deg=args.amp_deg,
                                              margin_deg=args.margin_deg)
            print(f"  J{j1}: [{lo * DEG:+.1f}, {hi * DEG:+.1f}]  "
                  f"pre={safety.PRECONDITIONS_DEG.get(j1, {})}")
        safety.assert_wave_windows_within_limits()
        print("[dry-run] all windows within effective limits OK")
        return

    if args.run is None:
        raise SystemExit("--run is required (only joint(s) --dry-run may omit it)")
    rundir = R.RunDir(_OUTPUT_ROOT, args.date, args.run)
    meta = build_meta(args)
    result: dict = {}
    print(f"[run] {rundir.root}  mode={args.mode} dry_run={args.dry_run}")

    try:
        if args.mode in ("joint", "joints", "all"):
            if not args.dry_run:
                run_joint_units(args, rundir, meta, result)
        if args.mode in ("ee", "all"):
            run_ee(args, rundir, meta, result, dry_run=args.dry_run)
    finally:
        R.write_json(rundir.path("meta.json"), meta)
        R.write_json(rundir.path("result.json"), result)
        print(f"[run] wrote meta.json + result.json → {rundir.root}")


if __name__ == "__main__":
    main()
