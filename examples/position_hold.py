#!/usr/bin/env python3
"""Position hold example for the A1Z arm.

Starts in position-hold mode (PD + gravity comp) at the current position,
then optionally moves to a target joint configuration.

Usage:
    # Hold current position:
    python examples/position_hold.py

    # Move to target (radians):
    python examples/position_hold.py --q_target 0,0.6,0.4,-0.5,0,0

    # Move to target (degrees):
    python examples/position_hold.py --q_target_deg 0,30,0,-45,0,0

    # With an attached G1Z gripper (7-DOF target):
    python examples/position_hold.py --with-gripper --q_target 0,0.6,0.4,-0.5,0,0,0.5
"""

import argparse
import signal
import sys
import time

import numpy as np

from a1z.robots.get_robot import get_a1z_robot
from a1z.config import add_config_argument, load_config, config_to_robot_kwargs


def parse_target_q(q_target: str, q_target_deg: str, with_gripper: bool) -> np.ndarray:
    if q_target and q_target_deg:
        raise ValueError("--q_target and --q_target_deg are mutually exclusive")
    s = q_target_deg if q_target_deg else q_target
    if not s:
        return np.array([])
    q = np.fromstring(s, sep=",", dtype=np.float64)
    expected = 7 if with_gripper else 6
    if q.shape[0] != expected:
        raise ValueError(f"Expected {expected} values, got {q.shape[0]}: {s}")
    if q_target_deg:
        q = np.deg2rad(q)
    return q


def main():
    parser = argparse.ArgumentParser(description="A1Z position hold")
    parser.add_argument("--gravity_factor", type=float, default=None,
                        help="Gravity compensation scale.")
    parser.add_argument("--freq", type=int, default=None, help="Control loop frequency (Hz).")
    parser.add_argument("--can", default=None, help="CAN channel.")
    parser.add_argument("--q_target", type=str, default="",
                        help="Target joint angles (rad), comma-separated. Length=6 (or 7 with --with-gripper).")
    parser.add_argument("--q_target_deg", type=str, default="",
                        help="Target joint angles (degrees), comma-separated. Length=6 (or 7 with --with-gripper).")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Movement speed (rad/s) for moving to target.")
    parser.add_argument("--with-gripper", action="store_true",
                        help="Attach the G1Z gripper (adds 7th DOF).")
    add_config_argument(parser)
    args = parser.parse_args()

    config = load_config(args.config) if args.config else {}
    kwargs = config_to_robot_kwargs(config)
    if args.can is not None:
        kwargs["can_channel"] = args.can
    if args.gravity_factor is not None:
        kwargs["gravity_comp_factor"] = args.gravity_factor
    if args.freq is not None:
        kwargs["control_freq_hz"] = args.freq
    if args.with_gripper:
        kwargs["with_gripper"] = True
    with_gripper = kwargs.get("with_gripper", False)

    q_target = parse_target_q(args.q_target, args.q_target_deg, with_gripper)

    print("=" * 60)
    print(f"  A1Z Position Hold")
    print(f"  Gravity factor:  {kwargs.get('gravity_comp_factor', 1.0)}")
    print(f"  Control freq:    {kwargs.get('control_freq_hz', 250)} Hz")
    print(f"  CAN channel:     {kwargs.get('can_channel', 'can0')}")
    print(f"  With gripper:    {with_gripper}")
    if q_target.size > 0:
        print(f"  Target (rad):    {np.round(q_target, 3)}")
        print(f"  Target (deg):    {np.round(np.degrees(q_target), 1)}")
    print("=" * 60)

    kwargs.setdefault("zero_gravity_mode", False)
    robot = get_a1z_robot(**kwargs)

    signal.signal(signal.SIGINT, signal.default_int_handler)

    try:
        robot.start()

        if q_target.size > 0:
            print(f"\nMoving to target at {args.speed} rad/s...")
            robot.move_joints(q_target, speed=args.speed)
            print("Target reached.")

        print("\nHolding position. Press Ctrl+C to stop.\n")

        while robot.is_running:
            state = robot.get_joint_state()
            pos_deg = np.degrees(state["pos"])
            eff = state["eff"]
            extra = ""
            if args.with_gripper:
                grip = robot.get_gripper_pos()
                extra = f"  grip: {grip:.2f}"
            print(
                f"  pos(deg): [{', '.join(f'{p:7.2f}' for p in pos_deg)}]{extra}  "
                f"eff(Nm): [{', '.join(f'{e:6.2f}' for e in eff)}]",
                end="\r",
            )
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        if robot.is_running:
            print("\nReturning to zero...")
            robot.move_joints(np.zeros(6), speed=args.speed * 0.5)
            time.sleep(0.3)
        robot.stop()
        print("\nDone.")


if __name__ == "__main__":
    main()
