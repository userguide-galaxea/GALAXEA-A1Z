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
"""

import argparse
import signal
import sys
import time

import numpy as np

from a1z.robots.get_robot import get_a1z_robot


def parse_target_q(q_target: str, q_target_deg: str) -> np.ndarray:
    if q_target and q_target_deg:
        raise ValueError("--q_target and --q_target_deg are mutually exclusive")
    s = q_target_deg if q_target_deg else q_target
    if not s:
        return np.array([])
    q = np.fromstring(s, sep=",", dtype=np.float64)
    if q.shape[0] != 6:
        raise ValueError(f"Expected 6 values, got {q.shape[0]}: {s}")
    if q_target_deg:
        q = np.deg2rad(q)
    return q


def main():
    parser = argparse.ArgumentParser(description="A1Z position hold")
    parser.add_argument("--gravity_factor", type=float, default=1.0,
                        help="Gravity compensation scale.")
    parser.add_argument("--freq", type=int, default=250, help="Control loop frequency (Hz).")
    parser.add_argument("--can", default="can0", help="CAN channel.")
    parser.add_argument("--q_target", type=str, default="",
                        help="Target joint angles (rad), comma-separated, length=6.")
    parser.add_argument("--q_target_deg", type=str, default="",
                        help="Target joint angles (degrees), comma-separated, length=6.")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Movement speed (rad/s) for moving to target.")
    args = parser.parse_args()

    q_target = parse_target_q(args.q_target, args.q_target_deg)

    print("=" * 60)
    print(f"  A1Z Position Hold")
    print(f"  Gravity factor:  {args.gravity_factor}")
    print(f"  Control freq:    {args.freq} Hz")
    print(f"  CAN channel:     {args.can}")
    if q_target.size == 6:
        print(f"  Target (rad):    {np.round(q_target, 3)}")
        print(f"  Target (deg):    {np.round(np.degrees(q_target), 1)}")
    print("=" * 60)

    robot = get_a1z_robot(
        can_channel=args.can,
        gravity_comp_factor=args.gravity_factor,
        zero_gravity_mode=False,
        control_freq_hz=args.freq,
    )

    def signal_handler(sig, frame):
        print("\nCtrl+C received, stopping...")
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        robot.start()

        if q_target.size == 6:
            print(f"\nMoving to target at {args.speed} rad/s...")
            robot.move_joints(q_target, speed=args.speed)
            print("Target reached.")

        print("\nHolding position. Press Ctrl+C to stop.\n")

        while robot.is_running:
            state = robot.get_joint_state()
            pos_deg = np.degrees(state["pos"])
            eff = state["eff"]
            print(
                f"  pos(deg): [{', '.join(f'{p:7.2f}' for p in pos_deg)}]  "
                f"eff(Nm): [{', '.join(f'{e:6.2f}' for e in eff)}]",
                end="\r",
            )
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()
        print("\nDone.")


if __name__ == "__main__":
    main()
