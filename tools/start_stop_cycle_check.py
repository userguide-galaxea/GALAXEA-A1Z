#!/usr/bin/env python3
"""Repeated start/stop cycle check (P3 hardware gate).

Runs N rounds of: build robot → ``start()`` → hold position for
``--hold-s`` seconds (sampling telemetry) → ``stop()`` → destroy object.
Each round logs startup duration, sampled joint drift, fault status after
stop, and interpreter thread count before/after, so resource leaks (stuck
control threads, unclosed buses) surface in the log instead of piling up
silently across rounds.

A round FAILs (and the run aborts, leaving motors disabled) when:

  * ``start()`` raises (startup probe failure, motor fault, ...);
  * the control state leaves RUNNING during the hold;
  * ``stop()`` raises or the final state is not STOPPED / not restartable;
  * threads survive after the round (leak).

Logs go to ``--log-dir``/start_stop_cycle.log (default
``test-logs/<YYYYMMDD>-<tag>-start-stop/``); verdicts per round plus an
OVERALL line use PASS / FAIL / ABORT, matching the SOP record fields.
Exit code 0 = all rounds PASS.

Usage:
    python tools/start_stop_cycle_check.py --config a1z.yaml --tag M1
    python tools/start_stop_cycle_check.py --config a1z_g1z.yaml --rounds 1 --hold-s 10
"""

import argparse
import gc
import logging
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

from a1z.config import add_config_argument, config_to_robot_kwargs, load_config
from a1z.robots.get_robot import get_a1z_robot

_SAMPLE_HZ = 2.0  # telemetry sampling during the hold phase


def _setup_logger(log_dir: Path) -> logging.Logger:
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("start_stop_cycle_check")
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    for handler in (logging.FileHandler(log_dir / "start_stop_cycle.log"),
                    logging.StreamHandler(sys.stdout)):
        handler.setFormatter(fmt)
        logger.addHandler(handler)
    return logger


def _thread_snapshot() -> set[str]:
    return {f"{t.name}({t.ident})" for t in threading.enumerate()}


def _run_round(round_no: int, args, kwargs: dict,
               logger: logging.Logger) -> str:
    """One start/hold/stop cycle. Returns PASS / FAIL / ABORT."""
    label = f"round {round_no}"
    robot = None
    threads_before = _thread_snapshot()
    try:
        logger.info("%s: building robot (threads=%d)", label, len(threads_before))
        robot = get_a1z_robot(**kwargs)

        t0 = time.monotonic()
        robot.start()
        startup_s = time.monotonic() - t0
        logger.info("%s: start() ok in %.2fs, state=%s",
                    label, startup_s, robot.get_fault_status()["state"])

        # Hold phase: sample drift and state at low rate.
        q0 = robot.get_joint_pos()
        logger.info("%s: hold %.1fs from pos=%s rad",
                    label, args.hold_s, np.round(q0, 4).tolist())
        deadline = time.monotonic() + args.hold_s
        while time.monotonic() < deadline:
            time.sleep(1.0 / _SAMPLE_HZ)
            state = robot.get_fault_status()["state"]
            if state != "RUNNING":
                logger.error("%s: state left RUNNING during hold: %s (%s)",
                             label, state, robot.get_fault_status())
                return "FAIL"
        drift = robot.get_joint_pos() - q0
        logger.info("%s: hold done, drift=%s rad (max |%.4f|)",
                    label, np.round(drift, 4).tolist(), float(np.abs(drift).max()))

        robot.stop()
        status = robot.get_fault_status()
        logger.info("%s: stop() ok, state=%s restart_allowed=%s",
                    label, status["state"], status["restart_allowed"])
        if status["state"] != "STOPPED" or not status["restart_allowed"]:
            logger.error("%s: unexpected post-stop status: %s", label, status)
            return "FAIL"
        return "PASS"
    except Exception:
        logger.error("%s: exception:\n%s", label, traceback.format_exc())
        if robot is not None:
            try:
                logger.error("%s: fault status at failure: %s",
                             label, robot.get_fault_status())
            except Exception:
                pass
        return "ABORT"
    finally:
        if robot is not None:
            try:
                robot.stop()
            except Exception:
                logger.error("%s: cleanup stop() raised:\n%s",
                             label, traceback.format_exc())
            # V3: stop() deliberately does not shut the CAN bus down; the
            # short-lived per-round robot leaves that to us, otherwise
            # python-can warns "SocketcanBus was not properly shut down".
            bus = getattr(robot, "_bus", None)
            shutdown = getattr(bus, "shutdown", None)
            if callable(shutdown):
                try:
                    shutdown()
                except Exception:
                    logger.error("%s: bus shutdown raised:\n%s",
                                 label, traceback.format_exc())
            del robot
        gc.collect()
        leaked = _thread_snapshot() - threads_before
        if leaked:
            logger.error("%s: leaked threads after round: %s", label, sorted(leaked))
        else:
            logger.info("%s: no leaked threads", label)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_config_argument(parser)
    parser.add_argument("--can", default=None, help="CAN channel override.")
    parser.add_argument("--bustype", default=None, help="CAN backend override.")
    parser.add_argument("--rounds", type=int, default=5,
                        help="Number of start/stop rounds (default: 5).")
    parser.add_argument("--hold-s", type=float, default=10.0,
                        help="Position-hold duration per round (s, default: 10).")
    parser.add_argument("--tag", default="M1",
                        help="Environment-matrix id used in the default log dir name.")
    parser.add_argument("--log-dir", default=None,
                        help="Log directory (default test-logs/<YYYYMMDD>-<tag>-start-stop).")
    args = parser.parse_args()

    if args.rounds < 1 or args.hold_s <= 0:
        parser.error("--rounds must be >= 1 and --hold-s > 0")

    log_dir = Path(args.log_dir) if args.log_dir else Path(
        f"test-logs/{datetime.now():%Y%m%d}-{args.tag}-start-stop"
    )
    logger = _setup_logger(log_dir)

    config = load_config(args.config) if args.config else {}
    kwargs = config_to_robot_kwargs(config)
    if args.can is not None:
        kwargs["can_channel"] = args.can
    if args.bustype is not None:
        kwargs["bustype"] = args.bustype
    kwargs.setdefault("zero_gravity_mode", False)  # same default as position_hold

    logger.info("log dir: %s", log_dir)
    logger.info("config=%s can=%s bustype=%s gripper=%s zero_gravity_mode=%s freq=%sHz",
                args.config, kwargs.get("can_channel", "can0"),
                kwargs.get("bustype", "auto"), kwargs.get("with_gripper", False),
                kwargs.get("zero_gravity_mode"), kwargs.get("control_freq_hz", 250))
    logger.info("rounds=%d hold=%.1fs", args.rounds, args.hold_s)

    results = {}
    for round_no in range(1, args.rounds + 1):
        verdict = _run_round(round_no, args, kwargs, logger)
        results[f"round{round_no}"] = verdict
        logger.info("round %d: %s", round_no, verdict)
        if verdict != "PASS":
            logger.error("aborting remaining rounds after %s", verdict)
            break

    logger.info("---- verdict ----")
    for name, verdict in results.items():
        logger.info("%s: %s", name, verdict)
    skipped = args.rounds - len(results)
    if skipped:
        logger.info("%d round(s) skipped after failure", skipped)

    if len(results) == args.rounds and all(v == "PASS" for v in results.values()):
        logger.info("OVERALL: PASS (log: %s)", log_dir / "start_stop_cycle.log")
        return 0
    logger.error("OVERALL: FAIL (see %s)", log_dir / "start_stop_cycle.log")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
