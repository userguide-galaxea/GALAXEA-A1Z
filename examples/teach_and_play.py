#!/usr/bin/env python3
"""Teach-and-play example for the A1Z arm.

Usage
-----
    # Record and save:
    python examples/teach_and_play.py record teach.json
    python examples/teach_and_play.py record teach.json --can can1 --sample-hz 100

    # Load and play:
    python examples/teach_and_play.py play teach.json
    python examples/teach_and_play.py play teach.json --speed 0.5 --loop
"""

import argparse
import signal
import threading
import time

import numpy as np

from a1z.robots.arm_robot import ArmRobot
from a1z.robots.get_robot import get_a1z_robot


def _wait_enter(prompt: str) -> None:
    print(prompt, end="", flush=True)
    input()


def cmd_record(args: argparse.Namespace) -> None:
    robot = get_a1z_robot(
        can_channel=args.can,
        zero_gravity_mode=True,
        gravity_comp_factor=1.0,
    )
    signal.signal(signal.SIGINT, signal.default_int_handler)

    print("=" * 60)
    print("  A1Z Teach — Record")
    print(f"  CAN:        {args.can}")
    print(f"  Sample Hz:  {args.sample_hz}")
    print(f"  Save to:    {args.file}")
    print("=" * 60)

    robot.start()
    print("[record] Arm running in zero-gravity mode.\n")

    try:
        _wait_enter("[record] Press ENTER to START recording...")
        robot.start_recording(sample_hz=args.sample_hz)
        print("[record] Recording — move the arm freely.  Press ENTER to STOP.")

        _stop_display = threading.Event()

        def _display():
            while not _stop_display.is_set():
                state = robot.get_joint_state()
                pos_deg = np.degrees(state["pos"])
                print(
                    f"  pos(deg): [{', '.join(f'{p:7.2f}' for p in pos_deg)}]",
                    end="\r",
                )
                time.sleep(0.1)

        disp = threading.Thread(target=_display, daemon=True)
        disp.start()

        input()
        _stop_display.set()
        disp.join(timeout=0.5)
        print()

        trajectory = robot.stop_recording()
        if not trajectory:
            print("[record] No frames recorded.  Exiting.")
            return

        duration = trajectory[-1][0]
        print(f"[record] Recorded {len(trajectory)} frames ({duration:.2f}s).")

        ArmRobot.save_recording(trajectory, args.file)
        print(f"[record] Saved to {args.file}\n")

    except KeyboardInterrupt:
        print("\n[record] Interrupted.")
    finally:
        if robot.is_running:
            print("[record] Returning to zero...")
            robot.move_joints(np.zeros(6), speed=0.3)
            time.sleep(0.3)
        robot.stop()
        print("[record] Stopped.")


def cmd_play(args: argparse.Namespace) -> None:
    robot = get_a1z_robot(
        can_channel=args.can,
        zero_gravity_mode=False,
        gravity_comp_factor=1.0,
    )
    signal.signal(signal.SIGINT, signal.default_int_handler)

    print("=" * 60)
    print("  A1Z Teach — Play")
    print(f"  CAN:        {args.can}")
    print(f"  File:       {args.file}")
    print(f"  Speed:      {args.speed}x")
    print(f"  Loop:       {'yes' if args.loop else 'no'}")
    print("=" * 60)

    print(f"[play] Loading trajectory from {args.file}...")
    trajectory = ArmRobot.load_recording(args.file)
    duration = trajectory[-1][0] if trajectory else 0.0
    print(f"[play] Loaded {len(trajectory)} frames ({duration:.2f}s).\n")

    robot.start()

    try:
        start_pos = trajectory[0][1]
        print("[play] Returning to start position...")
        robot.move_joints(start_pos, speed=0.4)
        print("[play] Ready.\n")

        play_duration = duration / args.speed
        loop_count = 0
        while True:
            _wait_enter(f"[play] Press ENTER to PLAY ({play_duration:.1f}s at {args.speed}x)...")
            loop_count += 1
            print(f"[play] Playing (loop {loop_count})...")
            robot.play_trajectory(trajectory, speed_factor=args.speed)
            print("[play] Playback complete.")

            if not args.loop:
                break

            robot.move_joints(start_pos, speed=0.6)

    except KeyboardInterrupt:
        print("\n[play] Interrupted.")
    finally:
        if robot.is_running:
            print("[play] Returning to zero...")
            robot.move_joints(np.zeros(6), speed=0.3)
            time.sleep(0.3)
        robot.stop()
        print("[play] Stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(description="A1Z teach-and-play")
    parser.add_argument("--can", default="can0", help="CAN channel (default: can0)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_record = sub.add_parser("record", help="Record a trajectory and save to file")
    p_record.add_argument("file", help="Output JSON file, e.g. teach.json")
    p_record.add_argument("--sample-hz", type=int, default=50, dest="sample_hz",
                          help="Recording sample rate in Hz (default: 50)")

    p_play = sub.add_parser("play", help="Load and play back a saved trajectory")
    p_play.add_argument("file", help="Input JSON file, e.g. teach.json")
    p_play.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed factor (default: 1.0)")
    p_play.add_argument("--loop", action="store_true",
                        help="Loop playback until Ctrl+C")

    args = parser.parse_args()

    if args.cmd == "record":
        cmd_record(args)
    else:
        cmd_play(args)


if __name__ == "__main__":
    main()
