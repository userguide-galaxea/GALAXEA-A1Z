#!/usr/bin/env python3
"""Gripper motor center-point calibration (automated).

Automatically homes to the close mechanical stop via velocity, then drives to
the midpoint (half_travel rad toward open), and writes the midpoint as flash zero.

After calibration:
  close_rad = +half_travel  (e.g. +2.87)
  open_rad  = -half_travel  (e.g. -2.87)
  Both within (-π, π) — no encoder wrap after power cycle.

Protocol (motor manual V1.0, p.10-13):
  - 保存位置零点 (0xFE): sets zero in RAM only.
  - 存储参数 (0xAA → 0x7FF): writes all parameters to flash.
    Requires 失能 (disabled) mode.

Usage:
    python3 tools/gripper_set_zero.py
    python3 tools/gripper_set_zero.py --can can0
    python3 tools/gripper_set_zero.py --half-travel 2.87 -y
"""

import argparse
import select
import sys
import termios
import time
import tty

import can

from a1z.config import add_config_argument, load_config
from a1z.motor_drivers.can_backend import open_can_bus
from a1z.motor_drivers.motor_b_driver import MotorB
from a1z.robots.gripper import GRIPPER_CAN_ID, GRIPPER_MOTOR_RANGES

# MIT gains used only during calibration (not related to normal operation)
GRIPPER_KP = 10.0
GRIPPER_KD = 0.5

CLOSE_VEL   = 2.0    # homing velocity toward close stop (rad/s)
KD_HOME     = 1.8    # damping gain during homing (kp=0, velocity-only)
WARMUP_S    = 0.4    # seconds before stall check begins
STALL_WIN   = 8      # consecutive samples required to confirm stall
STALL_DELTA = 0.05   # position change threshold for stall (rad)
HOME_TIMEOUT = 5.0


def _recv_feedback(motor: MotorB) -> None:
    msg = motor.bus.recv(timeout=0)
    if msg is not None and len(msg.data) >= 6:
        fb = motor.parse_feedback(msg)
        if fb is not None:
            motor.last_feedback = fb


def _ping_recv(motor: MotorB) -> None:
    """Trigger feedback while motor is disabled."""
    motor.bus.send(can.Message(
        arbitration_id=motor.motor_id,
        data=bytes([0xFF] * 7 + [0xFD]),
        is_extended_id=False,
    ))
    deadline = time.time() + 0.05
    while time.time() < deadline:
        msg = motor.bus.recv(timeout=0.01)
        if msg is None:
            break
        if len(msg.data) >= 6:
            fb = motor.parse_feedback(msg)
            if fb is not None:
                motor.last_feedback = fb


def _home_to_close(motor: MotorB) -> None:
    """Velocity home to close mechanical stop via stall detection."""
    print(f"[calib] Homing to close → +{CLOSE_VEL} rad/s ...")
    t0 = time.time()
    pos_window = []

    while time.time() - t0 < HOME_TIMEOUT:
        motor.send_mit_command(pos=0.0, vel=CLOSE_VEL, kp=0.0, kd=KD_HOME, torque=0.0)
        _recv_feedback(motor)

        if time.time() - t0 > WARMUP_S:
            fb = motor.last_feedback
            if fb is not None:
                pos_window.append(fb.position)
                if len(pos_window) > STALL_WIN:
                    pos_window.pop(0)
                if len(pos_window) == STALL_WIN:
                    if abs(pos_window[-1] - pos_window[0]) < STALL_DELTA:
                        break

        time.sleep(0.01)

    fb = motor.last_feedback
    pos_str = f"{fb.position:+.3f}" if fb else "?"
    print(f"[calib] Close stop reached, pos={pos_str} rad ({time.time()-t0:.1f}s)")


def _set_ram_zero(motor: MotorB) -> None:
    motor.set_zero_ram()


def _write_flash_zero(motor: MotorB) -> None:
    """Set RAM zero then write all parameters to flash (motor must be disabled)."""
    _set_ram_zero(motor)
    canid_l = motor.motor_id & 0xFF
    canid_h = (motor.motor_id >> 8) & 0xFF
    motor.bus.send(can.Message(
        arbitration_id=0x7FF,
        data=bytes([canid_l, canid_h, 0xAA, 0x01]),
        is_extended_id=False,
    ))
    time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(
        description="Gripper motor center-point calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 tools/gripper_set_zero.py
  python3 tools/gripper_set_zero.py --can can0 --half-travel 2.87 -y
        """,
    )
    parser.add_argument("--can", default=None, help="CAN channel (default: can0)")
    parser.add_argument("--bustype", default=None,
                        help="python-can backend: socketcan, gs_usb, pcan, slcan. "
                             "Default: socketcan on Linux, gs_usb on macOS/Windows.")
    parser.add_argument("--half-travel", type=float, default=2.87,
                        help="Half of total gripper travel in rad (default: 2.87). "
                             "After calibration: open_rad=-X, close_rad=+X")
    parser.add_argument("-y", "--yes", action="store_true", help="Skip confirmation")
    add_config_argument(parser)
    args = parser.parse_args()

    config = load_config(args.config) if args.config else {}
    channel = args.can or config.get("can_channel") or "can0"
    bustype = args.bustype or config.get("bustype")

    half = args.half_travel

    print("=" * 60)
    print("  Gripper Center-Point Calibration")
    print("=" * 60)
    if args.config:
        print(f"  Config   : {args.config}")
    print(f"  CAN      : {channel}")
    print(f"  Motor ID : 0x{GRIPPER_CAN_ID:02X}")
    print(f"  Half travel: {half:.2f} rad")
    print()
    print("  Procedure:")
    print("    1. Velocity home to CLOSE mechanical stop (auto stall detect)")
    print("    2. RAM zero at close stop")
    print(f"    3. Drive {-half:+.2f} rad toward OPEN (midpoint)")
    print("    4. Write midpoint as flash zero (0xFE + 0xAA)")
    print()
    print("  Result:")
    print(f"    open_rad:  {-half:.2f}")
    print(f"    close_rad: {+half:.2f}")
    print("=" * 60)

    if not args.yes:
        try:
            ans = input("\nReady? Gripper must be able to move freely. (y/N): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nCancelled.")
            sys.exit(0)
        if ans != "y":
            print("Cancelled.")
            sys.exit(0)

    bus = open_can_bus(channel=channel, bustype=bustype)
    motor = MotorB(motor_id=GRIPPER_CAN_ID, bus=bus, ranges=GRIPPER_MOTOR_RANGES)

    try:
        # Step 1: home to close mechanical stop
        motor.clear_error()
        motor.enable()
        motor.set_ctrl_mode(1)  # calibration uses MIT mode regardless of flash setting
        _home_to_close(motor)

        # Step 2: disable then set RAM zero (motor must be disabled to accept 0xFE)
        motor.disable()
        time.sleep(0.1)
        _set_ram_zero(motor)
        time.sleep(0.1)
        print("[calib] RAM zero set at close stop.")

        # Step 3: re-enable and drive to midpoint (-half_travel rad)
        motor.clear_error()
        motor.enable()
        motor.set_ctrl_mode(1)
        midpoint = -half
        print(f"[calib] Driving to midpoint ({midpoint:+.2f} rad) ...")
        t0 = time.time()
        while time.time() - t0 < 6.0:
            motor.send_mit_command(
                pos=midpoint, vel=0.0, kp=GRIPPER_KP, kd=GRIPPER_KD, torque=0.0
            )
            _recv_feedback(motor)
            fb = motor.last_feedback
            if fb is not None:
                err = fb.position - midpoint
                sys.stdout.write(
                    f"\r  pos={fb.position:+.3f}  target={midpoint:+.3f}  err={err:+.3f}   "
                )
                sys.stdout.flush()
                if abs(err) < 0.05:
                    break
            time.sleep(0.01)
        print()

        # Step 4: disable then write flash zero
        motor.disable()
        time.sleep(0.1)

        _ping_recv(motor)
        fb = motor.last_feedback
        pos_str = f"{fb.position:+.3f}" if fb else "?"
        print(f"[calib] At midpoint, pos={pos_str} rad. Writing flash zero ...")

        _write_flash_zero(motor)
        print("[calib] Flash zero written.")

        # Verify
        time.sleep(0.1)
        _ping_recv(motor)
        fb = motor.last_feedback
        verified = f"{fb.position:+.4f}" if fb else "?"
        print(f"[calib] Verification: position after flash write = {verified} rad (expect ≈ 0.0)")

        # Live monitor
        print()
        print("[calib] Live monitor (motor disabled — move gripper by hand to verify).")
        print("[calib] Press q or Ctrl+C to exit.")
        print()

        fd = sys.stdin.fileno()
        old_tty = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                _ping_recv(motor)
                fb = motor.last_feedback
                if fb is not None:
                    sys.stdout.write(
                        f"\r  pos={fb.position:+.4f} rad  vel={fb.velocity:+.3f} rad/s  "
                        f"torque={fb.torque:+.3f} Nm   "
                    )
                    sys.stdout.flush()
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch in ("q", "Q", "\x03"):
                        break
                time.sleep(0.05)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)
            print()

    except KeyboardInterrupt:
        print("\n\nAborted.")
        motor.disable()
    finally:
        bus.shutdown()

    print()
    print("=" * 60)
    print("  Calibration complete. Power-cycle the motor to verify persistence.")
    print()
    print("  Confirm these values match a1z/robots/gripper.py:")
    print()
    print(f"    GRIPPER_OPEN_RAD  = {-half:.2f}")
    print(f"    GRIPPER_CLOSE_RAD = {+half:.2f}")
    print("=" * 60)


if __name__ == "__main__":
    main()
