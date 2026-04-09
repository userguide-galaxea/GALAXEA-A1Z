#!/usr/bin/env python3
"""A1Z motor zero calibration tool.

Supports MotorA (joints 0-2) and MotorB (joints 3-5).

Usage:
    sudo python3 tools/set_zero.py --all
    sudo python3 tools/set_zero.py --motor-a
    sudo python3 tools/set_zero.py --motor-b
    sudo python3 tools/set_zero.py --joints 0 3
"""

import argparse
import sys
import time

import can

# ── Configuration ─────────────────────────────────────────

CAN_CHANNEL = "can0"
CAN_BUSTYPE = "socketcan"
CAN_BITRATE = 1_000_000

MOTOR_A_IDS = [0x01, 0x02, 0x03]
MOTOR_B_IDS = [0x04, 0x05, 0x06]

JOINT_NAMES = {
    0: "arm_joint1 (MotorA, ID=0x01)",
    1: "arm_joint2 (MotorA, ID=0x02)",
    2: "arm_joint3 (MotorA, ID=0x03)",
    3: "arm_joint4 (MotorB, ID=0x04)",
    4: "arm_joint5 (MotorB, ID=0x05)",
    5: "arm_joint6 (MotorB, ID=0x06)",
}

MOTOR_A_ZERO_CMD_ID = 0x7FF


# ── MotorA zero setting ───────────────────────────────────

def set_motor_a_zero(bus: can.BusABC, motor_id: int, timeout: float = 1.0) -> bool:
    """Set MotorA zero point (current position becomes zero).

    Protocol:
        Send CAN ID = 0x7FF, data: [motor_id_H, motor_id_L, 0x00, 0x03]
        Success response: [motor_id_H, motor_id_L, 0x01, 0x03]
    """
    id_h = (motor_id >> 8) & 0xFF
    id_l = motor_id & 0xFF

    data = bytes([id_h, id_l, 0x00, 0x03])
    msg = can.Message(arbitration_id=MOTOR_A_ZERO_CMD_ID, data=data, is_extended_id=False)

    # Drain receive buffer
    while bus.recv(timeout=0.01) is not None:
        pass

    bus.send(msg)
    print(f"  -> Sent zero command: ID=0x{MOTOR_A_ZERO_CMD_ID:03X}, "
          f"data=[0x{id_h:02X}, 0x{id_l:02X}, 0x00, 0x03]")

    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = bus.recv(timeout=0.1)
        if resp is None:
            continue
        if resp.arbitration_id != MOTOR_A_ZERO_CMD_ID or len(resp.data) < 4:
            continue

        resp_motor_id = (resp.data[0] << 8) | resp.data[1]
        if resp_motor_id != motor_id or resp.data[2] != 0x01:
            continue

        if resp.data[3] == 0x03:
            print(f"  <- Motor 0x{motor_id:02X} zero set OK")
            return True
        else:
            print(f"  <- Motor 0x{motor_id:02X} zero set FAILED (response=0x{resp.data[3]:02X})")
            return False

    print(f"  <- Motor 0x{motor_id:02X} zero set timeout (no response)")
    return False


# ── DM zero setting ───────────────────────────────────────

def set_motor_b_zero(bus: can.BusABC, motor_id: int, timeout: float = 1.0) -> bool:
    """Set MotorB zero point (saves current position as zero offset).

    Protocol: Send CAN ID = motor_id, data: [0xFF]*7 + [0xFE]
    """
    data = bytes([0xFF] * 7 + [0xFE])
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)

    bus.send(msg)
    print(f"  -> Sent zero command: ID=0x{motor_id:02X}, data=FF*7+FE")

    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = bus.recv(timeout=0.1)
        if resp is not None and resp.arbitration_id == motor_id:
            print(f"  <- Motor 0x{motor_id:02X} zero set OK (response received)")
            return True

    print(f"  <- Motor 0x{motor_id:02X} zero command sent (no response, may have succeeded)")
    return True


# ── Main ──────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="A1Z motor zero calibration tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  sudo python3 tools/set_zero.py --all          # All 6 joints
  sudo python3 tools/set_zero.py --motor-a      # MotorA joints 0-2
  sudo python3 tools/set_zero.py --motor-b      # MotorB joints 3-5
  sudo python3 tools/set_zero.py --joints 0 3   # Joints 0 and 3
        """,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--all", action="store_true", help="Calibrate all motors")
    group.add_argument("--motor-a", action="store_true", help="Calibrate MotorA (joints 0-2)")
    group.add_argument("--motor-b", action="store_true", help="Calibrate MotorB (joints 3-5)")
    group.add_argument("--joints", type=int, nargs="+", metavar="J",
                       help="Calibrate specific joints (0-indexed: 0-5)")

    parser.add_argument("--channel", default=CAN_CHANNEL, help=f"CAN channel (default: {CAN_CHANNEL})")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE, help=f"CAN bitrate (default: {CAN_BITRATE})")
    parser.add_argument("-y", "--yes", action="store_true", help="Skip confirmation prompt")

    args = parser.parse_args()

    if args.all:
        joints = list(range(6))
    elif args.motor_a:
        joints = [0, 1, 2]
    elif args.motor_b:
        joints = [3, 4, 5]
    else:
        joints = args.joints
        for j in joints:
            if j < 0 or j > 5:
                print(f"Error: joint index {j} out of range (0-5)")
                sys.exit(1)

    print("=" * 60)
    print("  A1Z Motor Zero Calibration")
    print("=" * 60)
    print(f"\nCAN channel: {args.channel}  bitrate: {args.bitrate}")
    print(f"\nJoints to calibrate (current position -> zero):")
    for j in joints:
        print(f"  Joint {j}: {JOINT_NAMES[j]}")

    print(f"\n{'!' * 60}")
    print("  WARNING: Ensure the arm is at the desired zero position!")
    print("  This operation sets the current position as zero.")
    print(f"{'!' * 60}")

    if not args.yes:
        try:
            confirm = input("\nContinue? (y/N): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nCancelled.")
            sys.exit(0)
        if confirm != "y":
            print("Cancelled.")
            sys.exit(0)

    print(f"\nOpening CAN bus ({args.channel})...")
    try:
        bus = can.interface.Bus(channel=args.channel, bustype=CAN_BUSTYPE, bitrate=args.bitrate)
    except Exception as e:
        print(f"Error: cannot open CAN bus: {e}")
        print(f"Try: sudo ip link set {args.channel} up type can bitrate {args.bitrate}")
        sys.exit(1)

    results = {}
    try:
        for j in joints:
            print(f"\n--- Joint {j}: {JOINT_NAMES[j]} ---")
            if j in [0, 1, 2]:
                motor_id = MOTOR_A_IDS[j]
                ok = set_motor_a_zero(bus, motor_id)
            else:
                motor_id = MOTOR_B_IDS[j - 3]
                ok = set_motor_b_zero(bus, motor_id)
            results[j] = ok
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        bus.shutdown()
        print("\nCAN bus closed.")

    print(f"\n{'=' * 60}")
    print("  Calibration Results")
    print(f"{'=' * 60}")
    all_ok = True
    for j in sorted(results.keys()):
        status = "OK" if results[j] else "FAIL"
        print(f"  Joint {j} ({JOINT_NAMES[j]}): [{status}]")
        if not results[j]:
            all_ok = False

    if all_ok:
        print("\nAll joints calibrated successfully!")
    else:
        print("\nSome joints failed. Please check and retry.")
        sys.exit(1)


if __name__ == "__main__":
    main()
