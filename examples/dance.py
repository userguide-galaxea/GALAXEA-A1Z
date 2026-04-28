#!/usr/bin/env python3
"""Dance sequence for the A1Z arm.

Choreographs the arm through a series of named poses: salute, wave,
twist, reach, and bow.  With --gripper the gripper participates as well.

Usage:
    # Full dance:
    python examples/dance.py

    # Custom speed and gripper:
    python examples/dance.py --speed 0.7 --gripper

    # Run only selected moves:
    python examples/dance.py --moves salute,wave,bow
"""

import argparse
import signal
import sys
import time

import numpy as np

from a1z.robots.get_robot import get_a1z_robot


def _deg(*angles: float) -> np.ndarray:
    return np.deg2rad(np.array(angles, dtype=np.float64))


# ---------------------------------------------------------------------------
# Choreography poses  (J1, J2, J3, J4, J5, J6) in degrees → converted above
#   J1 base yaw    : ±120°
#   J2 shoulder    :  0° – 180°
#   J3 elbow       : -180° – 0°
#   J4 wrist rot   : ±85°
#   J5 wrist pitch : ±85°
#   J6 wrist roll  : ±115°
# ---------------------------------------------------------------------------
POSES = {
    "home":      _deg(  0,  60,  -60,   0,   0,   0),
    "ready":     _deg(  0,  30,  -30,   0,  45,   0),
    "salute":    _deg( 30,  35,  -80,   0,  80,  90),
    "wave_l":    _deg(-80,  60,  -60,   0,  60,  90),
    "wave_r":    _deg( 80,  60,  -60,   0,  -60, -90),
    "nod_a":     _deg(  0,  70,  -60,  50,   0,   0),
    "nod_b":     _deg(  0,  70,  -60,   0,   0,   0),
    "shake_a":   _deg(  0,  70,  -60,   0,  40,   0),
    "shake_b":   _deg(  0,  70,  -60,   0, -40,   0),
    "reach":     _deg(  0,  20,  -30,   0,  60,   0),
    "bow":       _deg(  0, 110, -130,   0,   0,   0),
}

# Each move: (label, list of (pose_key, speed_multiplier, pause_s))
MOVES = {
    "salute": [
        ("salute",  1.0, 0.8),
        ("home",    0.8, 0.0),
    ],
    "wave": [
        ("ready",   1.0, 0.0),
        ("wave_l",  1.5, 0.1),
        ("wave_r",  1.5, 0.1),
        ("wave_l",  1.5, 0.1),
        ("wave_r",  1.5, 0.1),
        ("home",    1.0, 0.0),
    ],
    "nod": [
        ("nod_a",   1.2, 0.0),
        ("nod_b",   1.2, 0.0),
        ("home",    1.0, 0.0),
    ],
    "shake": [
        ("shake_a", 1.2, 0.0),
        ("shake_b", 1.2, 0.0),
        ("shake_a", 1.2, 0.0),
        ("shake_b", 1.2, 0.0),
        ("home",    1.0, 0.0),
    ],
    "reach": [
        ("reach",   0.9, 0.5),
        ("home",    0.9, 0.0),
    ],
    "bow": [
        ("home",    0.7, 0.0),
        ("bow",     0.5, 0.8),
        ("home",    0.5, 0.0),
    ],
}

DEFAULT_ORDER = ["salute", "wave", "nod", "reach", "bow"]


class Dance:
    def __init__(self, robot, base_speed: float, with_gripper: bool):
        self.robot = robot
        self.base_speed = base_speed
        self.with_gripper = with_gripper

    def _move(self, pose_key: str, speed_mul: float, pause: float, label: str = "") -> None:
        tag = label or pose_key
        print(f"    {tag}")
        self.robot.move_joints(POSES[pose_key], speed=self.base_speed * speed_mul)
        if pause > 0:
            time.sleep(pause)

    def _grip(self, value: float, pause: float = 0.35) -> None:
        if self.with_gripper:
            self.robot.command_gripper(value)
            time.sleep(pause)

    def run_move(self, move_name: str) -> None:
        print(f"  [{move_name}]")

        if self.with_gripper and move_name == "salute":
            self.robot.move_joints(POSES["salute"], speed=self.base_speed)
            self._grip(0.0)
            self._grip(1.0, pause=0.6)
            self.robot.move_joints(POSES["home"], speed=self.base_speed * 0.8)
        elif self.with_gripper and move_name == "reach":
            self.robot.move_joints(POSES["reach"], speed=self.base_speed * 0.9)
            time.sleep(0.5)
            self._grip(0.0, pause=0.4)
            self._grip(1.0, pause=0.3)
            self.robot.move_joints(POSES["home"], speed=self.base_speed * 0.9)
        else:
            for pose_key, spd_mul, pause in MOVES[move_name]:
                self._move(pose_key, spd_mul, pause)

    def run(self, order: list[str]) -> None:
        print("\n  Starting dance sequence")
        print("  " + "-" * 40)

        # Go to home first
        print("  [home]")
        self.robot.move_joints(POSES["home"], speed=self.base_speed * 0.7)
        self._grip(1.0)
        time.sleep(0.4)

        for move_name in order:
            if move_name not in MOVES:
                print(f"  [skip] unknown move '{move_name}'")
                continue
            self.run_move(move_name)
            time.sleep(0.2)

        # Return home and close gripper
        print("  [home]")
        self.robot.move_joints(POSES["home"], speed=self.base_speed * 0.6)
        self._grip(0.0)
        print("  " + "-" * 40)
        print("  Dance complete")


def main() -> None:
    parser = argparse.ArgumentParser(description="A1Z dance sequence")
    parser.add_argument("--can", default="can0", help="CAN channel.")
    parser.add_argument("--speed", type=float, default=0.6,
                        help="Base movement speed in rad/s (default 0.6).")
    parser.add_argument("--gripper", action="store_true",
                        help="Enable gripper.")
    parser.add_argument(
        "--moves",
        default=",".join(DEFAULT_ORDER),
        help=f"Comma-separated list of moves to perform (default: {','.join(DEFAULT_ORDER)}).",
    )
    args = parser.parse_args()

    order = [m.strip() for m in args.moves.split(",") if m.strip()]
    known = set(MOVES.keys())
    unknown = [m for m in order if m not in known]
    if unknown:
        print(f"Unknown moves: {unknown}")
        print(f"Available: {', '.join(sorted(known))}")
        sys.exit(1)

    print("=" * 50)
    print("  A1Z Dance")
    print(f"  CAN     : {args.can}")
    print(f"  Speed   : {args.speed} rad/s")
    print(f"  Gripper : {'yes' if args.gripper else 'no'}")
    print(f"  Moves   : {', '.join(order)}")
    print("=" * 50)

    robot = get_a1z_robot(
        can_channel=args.can,
        gravity_comp_factor=1.0,
        zero_gravity_mode=False,
        with_gripper=args.gripper,
    )

    def _stop(sig, frame):
        print("\n  Ctrl+C — stopping safely...")
        if robot.is_running:
            print("  [zero]")
            robot.move_joints(np.zeros(robot.num_dofs()), speed=args.speed * 0.5)
        robot.stop()
        robot._bus.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _stop)

    try:
        robot.start()
        dance = Dance(robot, base_speed=args.speed, with_gripper=args.gripper)
        dance.run(order)
    except Exception as exc:
        print(f"\n  Error: {exc}")
    finally:
        if robot.is_running:
            print("  [zero]")
            robot.move_joints(np.zeros(robot.num_dofs()), speed=args.speed * 0.5)
        robot.stop()
        robot._bus.shutdown()
        print("  Motors off.")


if __name__ == "__main__":
    main()
