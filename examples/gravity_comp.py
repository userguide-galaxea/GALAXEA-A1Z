#!/usr/bin/env python3
"""Gravity compensation example for the A1Z arm.

Usage:
    # Zero-gravity (floating) mode, default URDF (A1Z_2kg.urdf):
    python examples/gravity_comp.py

    # Use nogripper URDF, start with small gravity factor (recommended):
    python examples/gravity_comp.py --urdf a1z/robot_models/a1z/A1Z_nogripper.urdf --gravity_factor 0.3

    # Full gravity compensation with nogripper URDF:
    python examples/gravity_comp.py --urdf a1z/robot_models/a1z/A1Z_nogripper.urdf --gravity_factor 1.0

    # Position hold mode:
    python examples/gravity_comp.py --mode hold --urdf a1z/robot_models/a1z/A1Z_nogripper.urdf

    # Custom CAN channel:
    python examples/gravity_comp.py --can can1 --urdf a1z/robot_models/a1z/A1Z_nogripper.urdf

Available URDF models (a1z/robot_models/a1z/):
    A1Z_2kg.urdf          -- default, with 2kg end-effector payload
    A1Z_nogripper.urdf    -- no gripper / bare flange
    A1XGEN2_Noumenon.urdf -- A1X Gen2 variant
"""

import argparse
import signal
import sys
import time

import numpy as np

from a1z.robots.get_robot import get_a1z_robot


def main():
    parser = argparse.ArgumentParser(description="A1Z gravity compensation")
    parser.add_argument("--mode", choices=["gravity", "hold"], default="gravity",
                        help="gravity: zero-gravity (floating). hold: position hold + gravity comp.")
    parser.add_argument("--gravity_factor", type=float, default=1.0,
                        help="Gravity compensation scale (0=off, 1=full). Start small (e.g. 0.3).")
    parser.add_argument("--freq", type=int, default=250, help="Control loop frequency (Hz).")
    parser.add_argument("--can", default="can0", help="CAN channel.")
    parser.add_argument("--urdf", default=None, help="Override URDF path.")
    parser.add_argument("--kd", type=str, default=None,
                        help="Override kd gains, comma-separated (6 values). "
                             "E.g. --kd 0.2,0.2,0.2,0.1,0.1,0.1")
    args = parser.parse_args()

    kd_override = None
    if args.kd is not None:
        kd_override = np.fromstring(args.kd, sep=",", dtype=np.float64)
        if kd_override.shape[0] != 6:
            raise ValueError(f"--kd expects 6 values, got {kd_override.shape[0]}")

    zero_gravity = (args.mode == "gravity")

    print("=" * 60)
    print(f"  A1Z Gravity Compensation")
    print(f"  Mode:            {'Zero-gravity (floating)' if zero_gravity else 'Position hold + gravity comp'}")
    print(f"  Gravity factor:  {args.gravity_factor}")
    print(f"  Control freq:    {args.freq} Hz")
    print(f"  CAN channel:     {args.can}")
    if kd_override is not None:
        print(f"  kd override:     {kd_override}")
    print("=" * 60)

    robot = get_a1z_robot(
        can_channel=args.can,
        gravity_comp_factor=args.gravity_factor,
        zero_gravity_mode=zero_gravity,
        control_freq_hz=args.freq,
        urdf_path=args.urdf,
    )

    def signal_handler(sig, frame):
        print("\nCtrl+C received, stopping...")
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        robot.start(initial_kd=kd_override)
        print("\nRobot running. Press Ctrl+C to stop.\n")

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
