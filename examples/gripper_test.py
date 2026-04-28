#!/usr/bin/env python3
"""Standalone gripper test — init to open then interactive open/close.

Usage:
    python examples/gripper_test.py
    python examples/gripper_test.py --can can0

Enter a normalized opening [0.0 = closed, 1.0 = fully open].
Type q to quit.

A background thread runs the 100 Hz control loop and prints real-time
pos / torque on one line. The main thread waits for the next input.
"""

import argparse
import sys
import threading
import time

import can

from a1z.motor_drivers.motor_b_driver import MotorB
from a1z.robots.gripper import (
    GRIPPER_CAN_ID,
    GRIPPER_CLOSE_RAD,
    GRIPPER_MOTOR_RANGES,
    GRIPPER_OPEN_RAD,
    Gripper,
)


def _control_loop(
    gripper: Gripper,
    motor: MotorB,
    stop_evt: threading.Event,
    monitor_evt: threading.Event,
) -> None:
    """100 Hz loop: drain CAN feedback, send gripper command, print status."""
    while not stop_evt.is_set():
        msg = motor.bus.recv(timeout=0)
        if msg is not None and int(msg.arbitration_id) == motor.motor_id:
            fb = motor.parse_feedback(msg)
            if fb is not None:
                motor.last_feedback = fb

        gripper.step()

        if monitor_evt.is_set():
            fb = motor.last_feedback
            if fb is not None:
                sys.stdout.write(
                    f"\r  pos={fb.position:+6.3f}rad  torque={fb.torque:+6.3f}Nm"
                    f"  cmd={gripper.get_pos():.2f}   "
                )
                sys.stdout.flush()

        time.sleep(0.01)


def main():
    parser = argparse.ArgumentParser(description="A1Z standalone gripper test")
    parser.add_argument("--can", default="can0", help="CAN channel.")
    args = parser.parse_args()

    print(f"[gripper] SocketCAN {args.can}")
    bus = can.interface.Bus(args.can, interface="socketcan")
    motor = MotorB(motor_id=GRIPPER_CAN_ID, bus=bus, ranges=GRIPPER_MOTOR_RANGES)
    gripper = Gripper(motor)
    gripper.enable()

    stop_evt = threading.Event()
    monitor_evt = threading.Event()

    ctrl_thread = threading.Thread(
        target=_control_loop,
        args=(gripper, motor, stop_evt, monitor_evt),
        daemon=True,
    )

    try:
        gripper.home()

        ctrl_thread.start()
        monitor_evt.set()  # 开启实时监控

        print()
        print(f"[gripper] open={GRIPPER_OPEN_RAD:.2f} rad  close={GRIPPER_CLOSE_RAD:.2f} rad")
        print("[gripper] 输入开度 [0.0=关闭, 1.0=全开]，q 退出")
        print()

        while True:
            monitor_evt.clear()                      # 暂停监控行，避免和 prompt 混叠
            sys.stdout.write("\r" + " " * 60 + "\r")
            sys.stdout.flush()
            time.sleep(0.02)                         # 等后台线程停止写入

            try:
                raw = input("开度> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if raw.lower() == "q":
                break
            try:
                norm = float(raw)
            except ValueError:
                print("  请输入 0.0~1.0 之间的数值")
                monitor_evt.set()
                continue

            gripper.command(max(0.0, min(1.0, norm)))
            monitor_evt.set()                        # 恢复实时监控

    finally:
        stop_evt.set()
        gripper.disable()
        bus.shutdown()
        print("\n[gripper] Done.")


if __name__ == "__main__":
    main()
