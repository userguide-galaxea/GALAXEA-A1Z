#!/usr/bin/env python3
"""A1Z limit-based zero calibration tool.

Flow (per joint):
  1. User manually moves joint to mechanical limit
  2. Set temporary zero at limit position
  3. Position-control motor to the true zero point
  4. Set final zero at the true zero position

Usage:
    sudo python3 tools/set_zero_limit.py --all
    sudo python3 tools/set_zero_limit.py --joints 0 3
    sudo python3 tools/set_zero_limit.py --all --upper
"""

import argparse
import math
import sys
import time

import can

from a1z.motor_drivers.motor_a_driver import MotorA, MotorARanges
from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges

# ── Configuration ─────────────────────────────────────────

CAN_BUSTYPE = "socketcan"
CAN_BITRATE = 1_000_000
MOTOR_A_ZERO_CMD_ID = 0x7FF

MOVE_FREQ_HZ = 200
MOVE_RAMP_SPEED = 0.5       # rad/s
MOVE_SETTLE_TIME = 0.5      # s
MOVE_POS_THRESHOLD = 0.02   # rad

CALIB_KP = [15.0, 15.0, 15.0, 10.0, 10.0, 10.0]
CALIB_KD = [0.8,  0.8,  0.8,  0.5,  0.5,  0.5]

JOINT_LIMITS = [
    (-2.094, 2.094),
    (0.0,    3.142),
    (0.0,    3.142),
    (-1.309, 1.309),
    (-1.484, 1.484),
    (-2.007, 2.007),
]

# MotorA ranges (EC-A4315-P2-36)
_MOTOR_A_RANGES = MotorARanges(
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
    pos_min=-12.5, pos_max=12.5,
    vel_min=-18.0, vel_max=18.0,
    torque_min=-70.0, torque_max=70.0,
    current_fb_min=-30.0, current_fb_max=30.0,
)

# MotorB default ranges
_MOTOR_B_RANGES_DEFAULT = MotorBRanges(
    pos_min=-12.5, pos_max=12.5,
    vel_min=-30.0, vel_max=30.0,
    torque_min=-12.0, torque_max=12.0,
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
)

# Joint 3 higher torque range
_MOTOR_B_RANGES_JOINT3 = MotorBRanges(
    pos_min=-12.5, pos_max=12.5,
    vel_min=-30.0, vel_max=30.0,
    torque_min=-28.0, torque_max=28.0,
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
)

JOINT_INFO = [
    {"name": "arm_joint1", "type": "motor_a", "motor_id": 0x01},
    {"name": "arm_joint2", "type": "motor_a", "motor_id": 0x02},
    {"name": "arm_joint3", "type": "motor_a", "motor_id": 0x03},
    {"name": "arm_joint4", "type": "motor_b", "motor_id": 0x04},
    {"name": "arm_joint5", "type": "motor_b", "motor_id": 0x05},
    {"name": "arm_joint6", "type": "motor_b", "motor_id": 0x06},
]


# ── CAN helpers ───────────────────────────────────────────

def drain_bus(bus: can.BusABC, timeout: float = 0.02):
    while True:
        msg = bus.recv(timeout=timeout)
        if msg is None:
            break


def recv_feedback(bus: can.BusABC, motor_id: int, motor, timeout: float = 0.05):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=0.01)
        if msg is None:
            continue
        if msg.arbitration_id == motor_id:
            fb = motor.parse_feedback(msg)
            if fb is not None:
                motor.last_feedback = fb
                return fb
    return None


def send_mit_and_recv(bus, motor, motor_type, pos, vel, kp, kd, torque):
    if motor_type == "motor_a":
        motor.send_mit_command(pos=pos, vel=vel, kp=kp, kd=kd, torque=torque, mode=0)
    else:
        motor.send_mit_command(pos=pos, vel=vel, kp=kp, kd=kd, torque=torque)
    return recv_feedback(bus, motor.motor_id, motor, timeout=0.02)


# ── Zero setting ──────────────────────────────────────────

def set_motor_a_zero(bus: can.BusABC, motor_id: int, timeout: float = 1.0) -> bool:
    id_h = (motor_id >> 8) & 0xFF
    id_l = motor_id & 0xFF
    data = bytes([id_h, id_l, 0x00, 0x03])
    msg = can.Message(arbitration_id=MOTOR_A_ZERO_CMD_ID, data=data, is_extended_id=False)

    drain_bus(bus)
    bus.send(msg)
    print(f"    Sent MotorA zero cmd: ID=0x{MOTOR_A_ZERO_CMD_ID:03X} "
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
            print(f"    MotorA 0x{motor_id:02X} zero set OK")
            return True
        else:
            print(f"    MotorA 0x{motor_id:02X} zero set FAILED (0x{resp.data[3]:02X})")
            return False

    print(f"    MotorA 0x{motor_id:02X} zero set timeout")
    return False


def set_motor_b_zero(bus: can.BusABC, motor_id: int, timeout: float = 1.0) -> bool:
    data = bytes([0xFF] * 7 + [0xFE])
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"    Sent MotorB zero cmd: ID=0x{motor_id:02X} data=FF*7+FE")

    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = bus.recv(timeout=0.1)
        if resp is not None and resp.arbitration_id == motor_id:
            print(f"    MotorB 0x{motor_id:02X} zero set OK")
            return True

    print(f"    MotorB 0x{motor_id:02X} zero cmd sent (no response, may have succeeded)")
    return True


def set_zero(bus: can.BusABC, joint_idx: int) -> bool:
    info = JOINT_INFO[joint_idx]
    if info["type"] == "motor_a":
        return set_motor_a_zero(bus, info["motor_id"])
    else:
        return set_motor_b_zero(bus, info["motor_id"])


# ── Motor creation ────────────────────────────────────────

def create_motor(bus: can.BusABC, joint_idx: int):
    info = JOINT_INFO[joint_idx]
    if info["type"] == "motor_a":
        return MotorA(motor_id=info["motor_id"], bus=bus, ranges=_MOTOR_A_RANGES)
    else:
        ranges = _MOTOR_B_RANGES_JOINT3 if joint_idx == 3 else _MOTOR_B_RANGES_DEFAULT
        return MotorB(motor_id=info["motor_id"], bus=bus, ranges=ranges)


# ── Position-controlled move ──────────────────────────────

def move_to_position(bus, motor, motor_type, joint_idx, target_pos, start_pos=0.0):
    distance = target_pos - start_pos
    if abs(distance) < 0.001:
        print(f"    Offset is ~0, no movement needed")
        return True

    direction = 1.0 if distance > 0 else -1.0
    total_distance = abs(distance)
    kp = CALIB_KP[joint_idx]
    kd = CALIB_KD[joint_idx]
    dt = 1.0 / MOVE_FREQ_HZ

    print(f"    Moving: {start_pos:.4f} -> {target_pos:.4f} rad "
          f"(dist={total_distance:.4f}, speed<={MOVE_RAMP_SPEED} rad/s)")

    motor.enable()
    time.sleep(0.05)

    for _ in range(10):
        send_mit_and_recv(bus, motor, motor_type, pos=start_pos, vel=0, kp=kp, kd=kd, torque=0)
        time.sleep(dt)

    current_target = start_pos
    move_start = time.time()
    timeout = total_distance / MOVE_RAMP_SPEED + 5.0

    while True:
        loop_start = time.time()
        step = direction * MOVE_RAMP_SPEED * dt
        current_target += step

        if direction > 0 and current_target >= target_pos:
            current_target = target_pos
        elif direction < 0 and current_target <= target_pos:
            current_target = target_pos

        fb = send_mit_and_recv(bus, motor, motor_type, pos=current_target, vel=0, kp=kp, kd=kd, torque=0)

        elapsed = time.time() - move_start
        if fb is not None and int(elapsed * MOVE_FREQ_HZ) % MOVE_FREQ_HZ == 0:
            actual_pos = fb.position
            progress = min(100, abs(actual_pos - start_pos) / total_distance * 100) if total_distance > 0 else 100
            print(f"    Progress: {progress:5.1f}%  target={current_target:+.4f}  "
                  f"actual={actual_pos:+.4f}  error={current_target - actual_pos:+.4f}")

        if abs(current_target - target_pos) < 0.001:
            break

        if elapsed > timeout:
            print(f"    Movement timeout!")
            return False

        sleep_time = dt - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    print(f"    Reached target, settling...")
    settle_start = time.time()
    converged = False

    while time.time() - settle_start < MOVE_SETTLE_TIME + 2.0:
        loop_start = time.time()
        fb = send_mit_and_recv(bus, motor, motor_type, pos=target_pos, vel=0, kp=kp, kd=kd, torque=0)
        if fb is not None:
            error = abs(fb.position - target_pos)
            if error < MOVE_POS_THRESHOLD and time.time() - settle_start > MOVE_SETTLE_TIME:
                converged = True
                print(f"    Settled: pos={fb.position:+.4f}, error={error:.4f} rad")
                break
        sleep_time = dt - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    if not converged:
        if fb is not None:
            print(f"    Warning: convergence poor (error={abs(fb.position - target_pos):.4f} rad)")
        else:
            print(f"    Warning: no feedback received")

    for _ in range(20):
        send_mit_and_recv(bus, motor, motor_type, pos=target_pos, vel=0, kp=0, kd=kd, torque=0)
        time.sleep(dt)

    return True


# ── Single joint calibration ─────────────────────────────

def calibrate_joint(bus, joint_idx, limit_dir):
    info = JOINT_INFO[joint_idx]
    lo, hi = JOINT_LIMITS[joint_idx]
    limit_angle = lo if limit_dir == "lower" else hi
    offset = -limit_angle

    print(f"\n{'='*60}")
    print(f"  Joint {joint_idx}: {info['name']} ({info['type'].upper()}, ID=0x{info['motor_id']:02X})")
    print(f"  Limit direction: {'lower' if limit_dir == 'lower' else 'upper'}")
    print(f"  Limit angle: {limit_angle:.3f} rad ({math.degrees(limit_angle):.1f} deg)")
    print(f"  Offset to zero: {offset:.3f} rad ({math.degrees(offset):.1f} deg)")
    print(f"{'='*60}")

    print(f"\n  [Step 1/4] Manually move joint {joint_idx} ({info['name']}) to the "
          f"{'lower' if limit_dir == 'lower' else 'upper'} limit")
    try:
        input(f"  Press Enter when ready...")
    except (EOFError, KeyboardInterrupt):
        print("\n  Cancelled")
        return False

    print(f"\n  [Step 2/4] Setting temporary zero at limit...")
    ok = set_zero(bus, joint_idx)
    if not ok:
        print(f"  Temporary zero set failed!")
        return False

    if abs(offset) < 0.001:
        print(f"\n  [Step 3/4] Zero coincides with limit, no movement needed")
        print(f"\n  [Step 4/4] Zero calibration complete")
        return True

    print(f"\n  [Step 3/4] Moving to true zero position (offset {offset:+.3f} rad)...")
    motor = create_motor(bus, joint_idx)

    try:
        ok = move_to_position(bus, motor, info["type"], joint_idx, target_pos=offset, start_pos=0.0)
    except KeyboardInterrupt:
        print(f"\n  Interrupted! Disabling motor...")
        motor.disable()
        return False
    except Exception as e:
        print(f"\n  Error: {e}")
        motor.disable()
        return False

    motor.disable()
    time.sleep(0.1)

    if not ok:
        print(f"  Movement failed!")
        return False

    print(f"\n  [Step 4/4] Setting final zero at true zero position...")
    time.sleep(0.3)
    ok = set_zero(bus, joint_idx)
    if not ok:
        print(f"  Final zero set failed!")
        return False

    print(f"\n  Joint {joint_idx} ({info['name']}) calibration complete!")
    return True


# ── Main ──────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="A1Z limit-based zero calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  sudo python3 tools/set_zero_limit.py --all
  sudo python3 tools/set_zero_limit.py --joints 0 3
  sudo python3 tools/set_zero_limit.py --all --upper

Joint limits (URDF):
  Joint 0 (arm_joint1): [-2.094, +2.094] rad  zero in middle
  Joint 1 (arm_joint2): [ 0.000, +3.142] rad  zero = lower limit
  Joint 2 (arm_joint3): [ 0.000, +3.142] rad  zero = lower limit
  Joint 3 (arm_joint4): [-1.309, +1.309] rad  zero in middle
  Joint 4 (arm_joint5): [-1.484, +1.484] rad  zero in middle
  Joint 5 (arm_joint6): [-2.007, +2.007] rad  zero in middle
        """,
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--all", action="store_true", help="Calibrate all joints")
    group.add_argument("--joints", type=int, nargs="+", metavar="J", help="Calibrate specific joints (0-5)")

    limit_group = parser.add_mutually_exclusive_group()
    limit_group.add_argument("--lower", action="store_true", default=True, help="Use lower limit (default)")
    limit_group.add_argument("--upper", action="store_true", help="Use upper limit")
    limit_group.add_argument("--limit-dir", choices=["lower", "upper"], help="Specify limit direction")

    parser.add_argument("--channel", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("--kp-scale", type=float, default=1.0, help="KP gain scale factor (default: 1.0)")
    parser.add_argument("--speed", type=float, default=MOVE_RAMP_SPEED, help=f"Move speed rad/s (default: {MOVE_RAMP_SPEED})")

    args = parser.parse_args()

    joints = list(range(6)) if args.all else args.joints
    for j in joints:
        if j < 0 or j > 5:
            print(f"Error: joint index {j} out of range (0-5)")
            sys.exit(1)

    if args.limit_dir:
        limit_dir = args.limit_dir
    elif args.upper:
        limit_dir = "upper"
    else:
        limit_dir = "lower"

    global MOVE_RAMP_SPEED
    MOVE_RAMP_SPEED = args.speed
    for i in range(6):
        CALIB_KP[i] *= args.kp_scale

    print("=" * 60)
    print("  A1Z Limit-Based Zero Calibration")
    print("=" * 60)
    print(f"\nCAN channel: {args.channel}")
    print(f"Limit direction: {'lower' if limit_dir == 'lower' else 'upper'}")
    print(f"Move speed: {MOVE_RAMP_SPEED} rad/s")
    print(f"\nCalibration plan:")
    for j in joints:
        info = JOINT_INFO[j]
        lo, hi = JOINT_LIMITS[j]
        limit_angle = lo if limit_dir == "lower" else hi
        offset = -limit_angle
        note = " (zero=limit, no move)" if abs(offset) < 0.001 else ""
        print(f"  Joint {j} [{info['name']}]: "
              f"{'lower' if limit_dir == 'lower' else 'upper'}={limit_angle:+.3f} rad, "
              f"offset={offset:+.3f} rad{note}")

    try:
        input("\nPress Enter to start...")
    except (EOFError, KeyboardInterrupt):
        print("\nCancelled")
        sys.exit(0)

    print(f"\nOpening CAN bus ({args.channel})...")
    try:
        bus = can.interface.Bus(channel=args.channel, bustype=CAN_BUSTYPE, bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"Error: cannot open CAN bus: {e}")
        print(f"Try: sudo ip link set {args.channel} up type can bitrate {CAN_BITRATE}")
        sys.exit(1)

    results = {}
    try:
        for j in joints:
            ok = calibrate_joint(bus, j, limit_dir)
            results[j] = ok
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
    finally:
        bus.shutdown()
        print("\nCAN bus closed.")

    print(f"\n{'='*60}")
    print(f"  Calibration Results")
    print(f"{'='*60}")
    all_ok = True
    for j in sorted(results.keys()):
        info = JOINT_INFO[j]
        status = "OK" if results[j] else "FAIL"
        print(f"  Joint {j} [{info['name']}]: [{status}]")
        if not results[j]:
            all_ok = False

    if all_ok:
        print(f"\nAll joints calibrated successfully!")
    else:
        print(f"\nSome joints failed. Please check and retry.")
        sys.exit(1)


if __name__ == "__main__":
    main()
