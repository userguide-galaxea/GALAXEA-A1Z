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
from a1z.config import add_config_argument, load_config, config_to_robot_kwargs


def main():
    parser = argparse.ArgumentParser(description="A1Z gravity compensation")
    parser.add_argument("--mode", choices=["gravity", "hold"], default="gravity",
                        help="gravity: zero-gravity (floating). hold: position hold + gravity comp.")
    parser.add_argument("--gravity_factor", type=float, default=None,
                        help="Gravity compensation scale (0=off, 1=full). Start small (e.g. 0.3).")
    parser.add_argument("--freq", type=int, default=None, help="Control loop frequency (Hz).")
    parser.add_argument("--can", default=None, help="CAN channel.")
    parser.add_argument("--urdf", default=None, help="Override URDF path.")
    parser.add_argument("--with-gripper", action="store_true",
                        help="Enable G1Z gripper (overrides config if set).")
    parser.add_argument("--kd", type=str, default=None,
                        help="Override kd gains, comma-separated (6 values). "
                             "E.g. --kd 0.2,0.2,0.2,0.1,0.1,0.1")
    add_config_argument(parser)
    args = parser.parse_args()

    config = load_config(args.config) if args.config else {}
    kwargs = config_to_robot_kwargs(config)

    # CLI flags override config file values.
    if args.can is not None:
        kwargs["can_channel"] = args.can
    if args.gravity_factor is not None:
        kwargs["gravity_comp_factor"] = args.gravity_factor
    if args.freq is not None:
        kwargs["control_freq_hz"] = args.freq
    if args.urdf is not None:
        kwargs["urdf_path"] = args.urdf
    if args.with_gripper:
        kwargs["with_gripper"] = True

    kd_override = None
    if args.kd is not None:
        kd_override = np.fromstring(args.kd, sep=",", dtype=np.float64)
        if kd_override.shape[0] != 6:
            raise ValueError(f"--kd expects 6 values, got {kd_override.shape[0]}")

    zero_gravity = (args.mode == "gravity")
    kwargs.setdefault("zero_gravity_mode", zero_gravity)
    # --mode hold implies position hold regardless of config zero_gravity_mode.
    kwargs["zero_gravity_mode"] = zero_gravity

    print("=" * 60)
    print(f"  A1Z Gravity Compensation")
    print(f"  Mode:            {'Zero-gravity (floating)' if zero_gravity else 'Position hold + gravity comp'}")
    print(f"  Gravity factor:  {kwargs.get('gravity_comp_factor', 1.0)}")
    print(f"  Control freq:    {kwargs.get('control_freq_hz', 250)} Hz")
    print(f"  CAN channel:     {kwargs.get('can_channel', 'can0')}")
    print(f"  With gripper:    {kwargs.get('with_gripper', False)}")
    if kd_override is not None:
        print(f"  kd override:     {kd_override}")
    print("=" * 60)

    robot = get_a1z_robot(**kwargs)

    signal.signal(signal.SIGINT, signal.default_int_handler)

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
        if robot.is_running:
            print("\nReturning to zero...")
            robot.move_joints(np.zeros(6), speed=0.3)
            time.sleep(0.3)
        robot.stop()
        print("\nDone.")


if __name__ == "__main__":
    main()
