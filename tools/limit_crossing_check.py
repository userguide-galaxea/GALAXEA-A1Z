#!/usr/bin/env python3
"""Per-joint soft-limit crossing check (P5 hardware gate).

For every selected joint and boundary, the procedure is:

  0. posture: move the whole arm to the PRE-CALIBRATED safe posture for
     that joint (loaded from ``--poses-file``); the non-tested joints are
     then held FIXED at that posture for the whole joint run. If the poses
     file is missing/incomplete (or ``--calibrate`` is given), the script
     switches to calibration mode first: zero-gravity hand-guiding, one
     posture capture per joint, written back to the poses file;
  1. approach: ``move_joints`` at low speed to a position ``--margin`` rad
     INSIDE the boundary (controlled, minimum-jerk);
  2. crossing: ramp the streamed target at ``--rate-hz`` from the inside
     position to ``--overshoot`` rad BEYOND the boundary over ``--frames``
     frames (linear interpolation, no step excitation) — the always-clip
     policy must park the command exactly on the boundary without
     estop/disable;
  3. recovery: ramp the target back inside — tracking must resume
     smoothly.

Safety properties of this procedure (vs. injecting a huge out-of-range
target from the zero pose):

  * the arm is pre-positioned near the boundary, so each crossing produces
    only ~(--margin + small) rad of actual motion;
  * the injected overshoot is small (default 0.10 rad, still beyond the
    0.05 rad tolerance so the large-violation/clip path is exercised), which
    bounds the excursion if the clip policy ever regresses;
  * PD gains are scaled by ``--kp-scale`` (default 0.3) for the WHOLE run —
    re-asserted on every streamed frame, because ``command_joint_pos``
    silently resets gains to the full defaults;
  * all posture transitions (approach, joint-to-joint, post-calibration)
    are minimum-jerk ``move_joints`` at ``--speed`` (default 0.2 rad/s);
    the zero-gravity → position-hold transition after calibration uses the
    SDK's ``sync_to_measured`` gain ramp (C6), so there is no gain jolt
    toward a stale command position;
  * crossing/recovery targets are ramped, not stepped (a step onto the
    clipped boundary produced a visible jolt + overshoot cycle on J1);
  * **preloaded safe postures + per-boundary operator confirmation**:
    joint-level sweeps ignore the workspace — e.g. J2's upper limit
    (3.142 rad) folds the arm toward the table / itself depending on
    posture. Non-tested joints are held at the calibrated posture loaded
    from the poses file (not wherever the arm happened to be), and before
    every boundary the script prints the planned motion and waits for
    [Enter] (skip with ``s``, abort with ``q``). ``--yes`` bypasses the
    prompt only for unattended re-runs;
  * every frame is monitored: if the measured position ever crosses the
    physical boundary outward by more than 0.02 rad, or the robot estops /
    faults, the run is aborted immediately (soft estop) and logged ABORT.

Every step is logged to ``--log-dir``/limit_crossing.log (default
``test-logs/<YYYYMMDD>-<tag>-limit-cross/``); the final verdict table uses
PASS / FAIL / ABORT per joint-boundary, matching the SOP record fields.

The poses file (default ``tools/limit_crossing_poses.json``) maps each
joint name to a full 6-joint posture that is collision-free for that
joint's whole limit sweep, e.g.::

    {"joint1": [0.0, 1.2, -1.1, 0.0, 0.4, 0.0], "joint2": [...], ...}

Usage:
    python tools/limit_crossing_check.py --config a1z_g1z.yaml --tag M3
    python tools/limit_crossing_check.py --config a1z_g1z.yaml --joints 2 --calibrate
"""

import argparse
import json
import logging
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

from a1z.config import add_config_argument, config_to_robot_kwargs, load_config
from a1z.robots.get_robot import get_a1z_robot

# Hard safety bound: measured position may never cross the physical boundary
# outward by more than this during any phase (rad).
_HARD_EXCURSION_RAD = 0.02
# Acceptance slack when comparing the clipped command to the boundary (rad).
_CLIP_ASSERT_TOL = 1e-6


def _parse_joints(spec: str) -> list[int]:
    """Parse '1,3,5' or '1-6' (1-based) into 0-based indices."""
    joints = set()
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = part.split("-", 1)
            joints.update(range(int(lo), int(hi) + 1))
        elif part:
            joints.add(int(part))
    return sorted(j - 1 for j in joints)


def _setup_logger(log_dir: Path) -> logging.Logger:
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("limit_crossing_check")
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    file_handler = logging.FileHandler(log_dir / "limit_crossing.log")
    file_handler.setFormatter(fmt)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(fmt)
    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)
    return logger


def _fault_state(robot) -> str:
    return robot.get_fault_status()["state"]


class _AbortRun(Exception):
    """Raised on any safety violation; the run is logged ABORT and stopped."""


def _confirm(logger: logging.Logger, label: str, inside: float, beyond: float,
             assume_yes: bool) -> str:
    """Ask the operator before moving to a boundary. Returns run/skip/abort.

    Joint-level limit tests move one joint while the rest hold their pose;
    whether that sweep is collision-free (table, self, cables) depends on the
    current arm posture, which the script cannot know — the operator must
    confirm. Use --yes only for unattended re-runs of already-vetted joints.
    """
    if assume_yes:
        return "run"
    if not sys.stdin.isatty():
        logger.error("%s: non-interactive stdin without --yes; refusing to move",
                     label)
        return "abort"
    print(f"\n>>> 即将测试 {label}: 先逼近 {inside:+.4f} rad，再斜坡跨越边界"
          f"（注入目标 {beyond:+.4f} rad）")
    print("    请确认：当前姿态下该关节此行程无桌面/自体/线缆碰撞风险，"
          "工作半径清空，安全员值守断电通路。")
    print("    [Enter]=执行  s=跳过本边界  q=中止全部: ", end="", flush=True)
    choice = sys.stdin.readline().strip().lower()
    if choice == "s":
        return "skip"
    if choice == "q":
        return "abort"
    return "run"


def _load_or_calibrate_poses(robot, joints, limits, kp, kd, args,
                             logger: logging.Logger) -> dict:
    """Load per-joint safe postures from the poses file; calibrate if missing.

    Calibration mode switches the arm to zero gravity so the operator can
    hand-guide it to a collision-free posture for each joint under test;
    the captured 6-joint postures are written back to the poses file.
    Returns {name: np.ndarray(6)}. Raises _AbortRun when calibration is
    needed but impossible (non-interactive stdin).
    """
    path = Path(args.poses_file)
    poses: dict[str, np.ndarray] = {}
    if path.exists() and not args.calibrate:
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            logger.error("poses file %s unreadable (%s); recalibrating", path, exc)
            raw = {}
        for name, value in raw.items():
            arr = np.asarray(value, dtype=float)
            ok = (
                arr.shape == (6,)
                and np.all(np.isfinite(arr))
                and all(limits[i][0] <= arr[i] <= limits[i][1] for i in range(6))
            )
            if ok:
                poses[name] = arr
            else:
                logger.error("poses file entry %s invalid or out of limits; "
                             "will recalibrate it", name)

    missing = [j for j in joints if f"joint{j + 1}" not in poses]
    if not missing:
        logger.info("poses loaded from %s: %s", path, sorted(poses))
        return poses

    if not sys.stdin.isatty():
        raise _AbortRun(
            f"poses file {path} missing entries for "
            f"{[f'joint{j + 1}' for j in missing]} and stdin is "
            "non-interactive; cannot calibrate"
        )
    logger.info("calibration mode for %s", [f"joint{j + 1}" for j in missing])
    print("\n=== 标定模式（零重力手扶） ===")
    print("对每个待标定关节：把机械臂拖到“该关节全程跨越上下限位都无碰撞")
    print("（桌面/本体/线缆）”的安全位形，按 Enter 采集。采集的是全臂 6 维位形；")
    print("正式测试时被测关节由脚本驱动跨越，其余关节固定在你采集的位形。")
    input("确认安全员就位、断电通路可达后，按 Enter 进入零重力 ...")
    robot.set_gravity_mode(True)
    for j in missing:
        name = f"joint{j + 1}"
        lo, hi = limits[j]
        print(f"\n[{name}] 限位 [{lo:+.3f}, {hi:+.3f}] rad")
        while True:
            answer = input("拖臂到安全位形后按 Enter 采集（q=中止） ...").strip().lower()
            if answer == "q":
                raise _AbortRun("operator aborted during calibration")
            pose = robot.get_joint_pos()[:6]
            # The captured posture becomes a move_joints target later, so
            # every joint must sit inside its own soft limits.
            bad = [i for i in range(6)
                   if not (limits[i][0] <= pose[i] <= limits[i][1])]
            if bad:
                detail = "; ".join(
                    f"joint{i + 1}={pose[i]:+.4f} 超出 "
                    f"[{limits[i][0]:+.3f}, {limits[i][1]:+.3f}]"
                    for i in bad
                )
                logger.warning("calibration capture rejected for %s: %s",
                               name, detail)
                print(f"[{name}] 位形无效：{detail}")
                print("请把相关关节收回到限位以内，再按 Enter 重新采集。")
                continue
            poses[name] = pose.copy()
            logger.info("calibrated %s pose=%s rad", name,
                        np.round(pose, 4).tolist())
            print(f"[{name}] 已采集 {np.round(pose, 4).tolist()}")
            break
    # Exit zero gravity via the SDK's safe transition: follow the measured
    # pose while ramping gains from 0 to the scaled targets (a plain
    # set_gravity_mode(False) would apply full default gains toward the
    # stale pre-calibration command position for one tick).
    measured = robot.get_joint_pos()[:6]
    robot.move_joints(measured, speed=args.speed, kp=kp, kd=kd,
                      sync_to_measured=True, gain_ramp_s=1.0)
    robot.zero_gravity_mode = False
    logger.info("exited zero gravity via sync_to_measured gain ramp")

    merged = {}
    if path.exists():
        try:
            merged = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            merged = {}
    merged.update({name: pose.tolist() for name, pose in poses.items()})
    path.write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")
    logger.info("poses written to %s", path)
    print(f"\n标定完成，位形已写入 {path}；后续运行将直接加载。")
    return poses


def _stream(robot, joint: int, q_base: np.ndarray,
            target_from: float, target_to: float,
            kp: np.ndarray, kd: np.ndarray,
            frames: int, rate_hz: float,
            lower: float, upper: float, logger: logging.Logger, phase: str):
    """Ramp `joint` from `target_from` to `target_to` over `frames` at `rate_hz`.

    Non-tested joints are commanded to the fixed preloaded posture `q_base`
    on every frame, and the (scaled) `kp`/`kd` are re-asserted every frame
    because ``command_joint_pos`` would silently reset gains to the full
    defaults. The target is interpolated linearly frame by frame (no step
    excitation), and every frame is monitored. Returns (measured_min,
    measured_max) for the joint during the phase. Raises _AbortRun on
    estop/fault or hard excursion beyond the boundary.
    """
    period = 1.0 / rate_hz
    measured_lo, measured_hi = np.inf, -np.inf
    for frame in range(frames):
        t0 = time.monotonic()
        q = q_base.copy()
        q[joint] = target_from + (target_to - target_from) * (frame + 1) / frames
        accepted = robot.command_joint_state({
            "pos": q,
            "vel": np.zeros(6),
            "kp": kp,
            "kd": kd,
        })
        if not accepted:
            raise _AbortRun(
                f"{phase}: frame {frame} command rejected (joint{joint + 1})"
            )
        if robot.is_estopped or _fault_state(robot) != "RUNNING":
            raise _AbortRun(
                f"{phase}: robot left RUNNING at frame {frame} "
                f"(state={_fault_state(robot)})"
            )
        measured = robot.get_joint_pos()[joint]
        measured_lo = min(measured_lo, measured)
        measured_hi = max(measured_hi, measured)
        if measured < lower - _HARD_EXCURSION_RAD or measured > upper + _HARD_EXCURSION_RAD:
            raise _AbortRun(
                f"{phase}: joint{joint + 1} measured {measured:+.4f} rad crossed "
                f"physical boundary [{lower:+.4f}, {upper:+.4f}] at frame {frame}"
            )
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    logger.info(
        "%s: joint%d measured range [%+.4f, %+.4f] over %d frames",
        phase, joint + 1, measured_lo, measured_hi, frames,
    )
    return measured_lo, measured_hi


def _check_boundary(robot, joint: int, which: str, limits, pose: np.ndarray,
                    kp: np.ndarray, kd: np.ndarray, args,
                    logger: logging.Logger) -> str:
    """Run approach/crossing/recovery for one joint boundary. Returns verdict.

    `pose` is the preloaded safe posture; non-tested joints are driven to
    and held at it for the whole boundary run.
    """
    lower, upper = limits[joint]
    boundary = upper if which == "upper" else lower
    margin = args.margin
    inside = boundary - margin if which == "upper" else boundary + margin
    beyond = boundary + args.overshoot if which == "upper" else boundary - args.overshoot
    label = f"joint{joint + 1}-{which}"

    try:
        # 1. Move to the preloaded posture, with the tested joint driven to
        #    the near-boundary position; verify the posture is reached.
        q = pose.copy()
        q[joint] = inside
        logger.info("%s: approach to posture %s rad with joint%d=%+.4f "
                    "(speed %.2f rad/s)",
                    label, np.round(pose, 3).tolist(), joint + 1, inside,
                    args.speed)
        robot.move_joints(q, speed=args.speed, kp=kp, kd=kd)
        if _fault_state(robot) != "RUNNING":
            raise _AbortRun(f"{label}: not RUNNING after approach "
                            f"(state={_fault_state(robot)})")
        err = np.abs(robot.get_joint_pos()[:6] - q)
        logger.info("%s: approach done, measured %s rad",
                    label, np.round(robot.get_joint_pos()[:6], 4).tolist())
        # Scaled-gain steady-state error on gravity-loaded joints (J2) was
        # measured at ~0.09 rad on 2026-08-14; 0.10 rad tolerance accepts
        # that while still catching a genuinely wrong posture (non-tested
        # joints track to ~0.005 rad).
        if float(err.max()) > 0.10:
            raise _AbortRun(
                f"{label}: posture not reached, max error {float(err.max()):.4f} rad"
            )

        # 2. Crossing: ramp the target from inside to beyond the boundary;
        #    the command must clip exactly onto the boundary without estop.
        _stream(robot, joint, q, inside, beyond, kp, kd, args.frames,
                args.rate_hz, lower, upper, logger, f"{label}-crossing")
        clipped = robot.get_command_state()["pos"][joint]
        logger.info("%s: clipped command %+.6f rad (boundary %+.6f)",
                    label, clipped, boundary)
        if abs(clipped - boundary) > _CLIP_ASSERT_TOL:
            logger.error("%s: FAIL clipped command %+.6f != boundary %+.6f",
                         label, clipped, boundary)
            return "FAIL"

        # 3. Recovery: ramp back inside; tracking must resume.
        _stream(robot, joint, q, beyond, inside, kp, kd, args.frames,
                args.rate_hz, lower, upper, logger, f"{label}-recovery")
        recovered = robot.get_command_state()["pos"][joint]
        if abs(recovered - inside) > _CLIP_ASSERT_TOL:
            logger.error("%s: FAIL recovery command %+.6f != inside %+.6f",
                         label, recovered, inside)
            return "FAIL"
        logger.info("%s: recovery command back to %+.6f rad", label, recovered)
        logger.info("%s: PASS", label)
        return "PASS"
    except _AbortRun as exc:
        logger.error("%s: ABORT %s", label, exc)
        robot.estop(reason=f"limit crossing safety abort: {exc}",
                    fault_code="LIMIT_CROSS_ABORT")
        return "ABORT"
    except Exception:
        # e.g. move_joints ValueError on an invalid target — log cleanly as
        # ABORT instead of dying with a bare traceback.
        logger.error("%s: ABORT unexpected exception:\n%s",
                     label, traceback.format_exc())
        robot.estop(reason=f"limit crossing unexpected error in {label}",
                    fault_code="LIMIT_CROSS_ABORT")
        return "ABORT"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_config_argument(parser)
    parser.add_argument("--can", default=None, help="CAN channel override.")
    parser.add_argument("--bustype", default=None, help="CAN backend override.")
    parser.add_argument("--joints", default="1-6",
                        help="Joints to test, 1-based: '1-6' or '1,3,5' (default: all).")
    parser.add_argument("--boundaries", choices=["upper", "lower", "both"],
                        default="both", help="Which boundary to cross (default: both).")
    parser.add_argument("--margin", type=float, default=0.15,
                        help="Standoff inside the boundary for approach/recovery (rad).")
    parser.add_argument("--overshoot", type=float, default=0.10,
                        help="Target overshoot beyond the boundary (rad); must exceed "
                             "the 0.05 rad limit tolerance to exercise the clip path.")
    parser.add_argument("--frames", type=int, default=50,
                        help="Streamed frames per phase.")
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="Streaming rate (Hz).")
    parser.add_argument("--speed", type=float, default=0.2,
                        help="Approach move_joints speed (rad/s).")
    parser.add_argument("--kp-scale", type=float, default=0.3,
                        help="Scale factor on default PD gains for this run.")
    parser.add_argument("--tag", default="M1",
                        help="Environment-matrix id used in the default log dir name.")
    parser.add_argument("--log-dir", default=None,
                        help="Log directory (default test-logs/<YYYYMMDD>-<tag>-limit-cross).")
    parser.add_argument("--yes", action="store_true",
                        help="Skip per-boundary operator confirmations (only for "
                             "unattended re-runs of already-vetted joints).")
    parser.add_argument("--poses-file",
                        default="tools/limit_crossing_poses.json",
                        help="Per-joint safe posture file (default "
                             "tools/limit_crossing_poses.json).")
    parser.add_argument("--calibrate", action="store_true",
                        help="Force posture calibration (zero-gravity hand-guiding) "
                             "even if the poses file exists.")
    args = parser.parse_args()

    if args.overshoot <= 0.05:
        parser.error("--overshoot must exceed the 0.05 rad limit tolerance")
    if not (0.0 < args.kp_scale <= 1.0):
        parser.error("--kp-scale must be in (0, 1]")

    log_dir = Path(args.log_dir) if args.log_dir else Path(
        f"test-logs/{datetime.now():%Y%m%d}-{args.tag}-limit-cross"
    )
    logger = _setup_logger(log_dir)

    config = load_config(args.config) if args.config else {}
    kwargs = config_to_robot_kwargs(config)
    if args.can is not None:
        kwargs["can_channel"] = args.can
    if args.bustype is not None:
        kwargs["bustype"] = args.bustype
    # The test requires non-tested joints to stay fixed — force
    # position-hold mode regardless of the config's zero_gravity_mode.
    kwargs["zero_gravity_mode"] = False

    joints = _parse_joints(args.joints)
    logger.info("log dir: %s", log_dir)
    logger.info("config=%s can=%s bustype=%s joints=%s boundaries=%s",
                args.config, kwargs.get("can_channel", "can0"),
                kwargs.get("bustype", "auto"),
                [j + 1 for j in joints], args.boundaries)
    logger.info("margin=%.3f overshoot=%.3f frames=%d rate=%.1fHz speed=%.2f kp_scale=%.2f",
                args.margin, args.overshoot, args.frames, args.rate_hz,
                args.speed, args.kp_scale)

    robot = get_a1z_robot(**kwargs)
    info = robot.get_robot_info()
    limits = info["joint_limits"]
    if limits is None:
        logger.error("joint limits are None for this config; refuse to run")
        return 2
    logger.info("joint limits (rad): %s", [(round(lo, 4), round(hi, 4)) for lo, hi in limits])

    results = {}
    try:
        kp = info["default_kp"] * args.kp_scale
        kd = info["default_kd"] * args.kp_scale
        logger.info("starting with scaled gains kp=%s kd=%s",
                    np.round(kp, 2).tolist(), np.round(kd, 3).tolist())
        robot.start(initial_kp=kp, initial_kd=kd)

        poses = _load_or_calibrate_poses(robot, joints, limits, kp, kd,
                                         args, logger)

        which_list = ["upper", "lower"] if args.boundaries == "both" else [args.boundaries]
        for joint in joints:
            for which in which_list:
                name = f"joint{joint + 1}-{which}"
                lower, upper = limits[joint]
                boundary = upper if which == "upper" else lower
                inside = boundary - args.margin if which == "upper" else boundary + args.margin
                beyond = (boundary + args.overshoot if which == "upper"
                          else boundary - args.overshoot)
                action = _confirm(logger, name, inside, beyond, args.yes)
                if action == "abort":
                    raise _AbortRun("operator aborted before " + name)
                if action == "skip":
                    results[name] = "SKIP"
                    logger.info("%s: SKIP (operator choice)", name)
                    continue
                results[name] = _check_boundary(
                    robot, joint, which, limits, poses[name.split("-")[0]],
                    kp, kd, args, logger,
                )
                if results[name] == "ABORT":
                    raise _AbortRun("safety abort; remaining checks skipped")
    except _AbortRun as exc:
        logger.error("run aborted: %s", exc)
    except KeyboardInterrupt:
        logger.error("run aborted: KeyboardInterrupt (Ctrl+C)")
    finally:
        robot.stop()

    logger.info("---- verdict ----")
    for name, verdict in results.items():
        logger.info("%s: %s", name, verdict)
    untested = [f"joint{j + 1}" for j in joints
                if not any(k.startswith(f"joint{j + 1}-") for k in results)]
    for name in untested:
        logger.info("%s: SKIP (aborted before reaching it)", name)

    if any(v in ("FAIL", "ABORT") for v in results.values()) or untested:
        logger.error("OVERALL: FAIL (see %s)", log_dir / "limit_crossing.log")
        return 1
    skipped = [k for k, v in results.items() if v == "SKIP"]
    if skipped:
        logger.info("OVERALL: PASS with operator-skipped boundaries %s "
                    "(record the reason in the test log)", skipped)
        return 0
    logger.info("OVERALL: PASS (log: %s)", log_dir / "limit_crossing.log")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
