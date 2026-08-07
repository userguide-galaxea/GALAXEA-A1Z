#!/usr/bin/env python3
"""力位混控模式夹爪测试脚本

两项测试：
  1. 自由行程  — 空载开合，验证能到达目标位置
  2. 力矩饱和  — 手动阻挡，验证力矩被钳位、不会随位置误差增大

用法:
    python examples/gripper_hybrid_test.py
    python examples/gripper_hybrid_test.py --torque 2.0
    python examples/gripper_hybrid_test.py --transport socketcan --can can0
    python examples/gripper_hybrid_test.py --tests free
    python examples/gripper_hybrid_test.py --tests clamp
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
    MOTOR_PEAK_TORQUE_NM,
    Gripper,
)


def _status_line(motor: MotorB, gripper: Gripper) -> str:
    fb = motor.last_feedback
    if fb is None:
        return "  (等待反馈...)"
    norm = gripper.get_feedback_norm()
    status = fb.error_message if fb.error else "ok"
    return (
        f"  pos={fb.position:+6.3f}rad  norm={norm:.2f}"
        f"  torque={fb.torque:+5.2f}Nm  vel={fb.velocity:+5.2f}rad/s"
        f"  [{status}]"
    )


def _drain(motor: MotorB, duration: float = 0.02) -> None:
    t_end = time.time() + duration
    while time.time() < t_end:
        msg = motor.bus.recv(timeout=0.0)
        if msg is None:
            break
        if int(msg.arbitration_id) == motor.motor_id:
            fb = motor.parse_feedback(msg)
            if fb is not None:
                motor.last_feedback = fb


def _control_loop(gripper: Gripper, motor: MotorB, stop_evt: threading.Event) -> None:
    """100 Hz 控制循环：接收反馈、发送混控指令。"""
    while not stop_evt.is_set():
        _drain(motor, duration=0.002)
        gripper.step()
        time.sleep(0.01)


def test_free_travel(gripper: Gripper, motor: MotorB) -> None:
    """测试 1：空载开合，验证能到达目标位置。"""
    print("\n── 测试 1：自由行程 ──")
    print("  请确保夹爪无遮挡，按 Enter 开始...")
    input()

    for label, cmd in [("关闭", 0.0), ("半开", 0.5), ("全开", 1.0)]:
        gripper.command(cmd)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            _drain(motor)
            fb = motor.last_feedback
            if fb is not None:
                norm = gripper.get_feedback_norm()
                sys.stdout.write(
                    f"\r  目标={label}({cmd:.1f})  反馈={norm:.3f}  "
                    f"torque={fb.torque:+5.2f}Nm   "
                )
                sys.stdout.flush()
            time.sleep(0.05)
        norm = gripper.get_feedback_norm()
        if abs(norm - cmd) > 0.1:
            print(f"\n  ✗ 未到达目标 {label}（反馈={norm:.3f}，误差={abs(norm-cmd):.3f}）")
        else:
            print(f"\n  ✓ 到达 {label}（反馈={norm:.3f}）")

    gripper.command(1.0)
    time.sleep(1.0)
    print("  ✓ 自由行程测试完成")


def test_torque_clamp(gripper: Gripper, motor: MotorB) -> None:
    """测试 2：手动阻挡夹爪，验证力矩被钳位而不持续增大。

    力位混控的关键特性：
    - MIT 模式：τ = kp×err，位置误差存在时力矩持续增大
    - 力位混控：电流被硬件截断在 i_des，力矩不随位置误差增大
    """
    print("\n── 测试 2：力矩饱和（请用手阻挡夹爪）──")
    print(f"  当前 max_torque = {gripper._i_des * MOTOR_PEAK_TORQUE_NM:.1f} Nm"
          f"  (i_des={gripper._i_des:.3f})")
    print("  发出关闭指令，请在夹爪运动时用手轻轻阻挡...")
    print("  观察：位置停住，力矩应钳位在约 max_torque，按 Enter 继续...")
    input()

    gripper.command(0.0)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        _drain(motor)
        fb = motor.last_feedback
        if fb is not None:
            norm = gripper.get_feedback_norm()
            sys.stdout.write(
                f"\r  norm={norm:.3f}  torque={fb.torque:+5.2f}Nm  "
                f"vel={fb.velocity:+5.2f}rad/s   "
            )
            sys.stdout.flush()
        time.sleep(0.05)
    print()

    gripper.command(1.0)
    time.sleep(1.0)
    print("  ✓ 力矩饱和测试完成")


def main() -> None:
    parser = argparse.ArgumentParser(description="力位混控夹爪测试")
    parser.add_argument("--transport", choices=["g4ros", "socketcan"], default="g4ros",
                        help="g4ros: 经 lemo 主板 g4spi_node 的 ROS2 话题（默认）;"
                             "socketcan: 旧 USB-CAN 直连。")
    parser.add_argument("--arm-side", choices=["left", "right"], default="left",
                        help="g4ros 传输的臂侧（默认 left）。")
    parser.add_argument("--can", default="can0", help="CAN 通道（仅 socketcan），默认 can0")
    parser.add_argument(
        "--torque", type=float, default=0.5,
        help=f"最大夹持力矩 (Nm)，默认 0.5，峰值 {MOTOR_PEAK_TORQUE_NM} Nm",
    )
    parser.add_argument(
        "--tests", nargs="*",
        choices=["free", "clamp"],
        default=["free", "clamp"],
        help="要运行的测试项，默认 free clamp",
    )
    args = parser.parse_args()

    run_free  = "free"  in args.tests
    run_clamp = "clamp" in args.tests

    if args.transport == "g4ros":
        print(f"[hybrid] g4ros (arm side: {args.arm_side})  max_torque={args.torque:.1f} Nm"
              f"  (i_des={args.torque / MOTOR_PEAK_TORQUE_NM:.4f})")
        from a1z.motor_drivers.ros_topic_bus import RosTopicBus
        bus = RosTopicBus(arm_side=args.arm_side)
    else:
        print(f"[hybrid] SocketCAN={args.can}  max_torque={args.torque:.1f} Nm"
              f"  (i_des={args.torque / MOTOR_PEAK_TORQUE_NM:.4f})")
        bus = can.interface.Bus(args.can, interface="socketcan")
    motor = MotorB(motor_id=GRIPPER_CAN_ID, bus=bus, ranges=GRIPPER_MOTOR_RANGES)
    gripper = Gripper(motor, max_torque=args.torque)

    stop_evt = threading.Event()
    ctrl_thread = threading.Thread(
        target=_control_loop,
        args=(gripper, motor, stop_evt),
        daemon=True,
    )

    try:
        print("[hybrid] 使能电机 → 切换力位混控模式 (mode 4)...")
        gripper.enable()
        time.sleep(0.1)

        _drain(motor, duration=0.1)
        fb = motor.last_feedback
        if fb is not None:
            print(f"[hybrid] 初始位置={fb.position:+.3f} rad  状态={fb.error_message}")

        print("[hybrid] 归位到全开位置...")
        gripper.home()

        ctrl_thread.start()

        print(f"[hybrid] 就绪  open={GRIPPER_OPEN_RAD:.2f} rad  "
              f"close={GRIPPER_CLOSE_RAD:.2f} rad\n")

        if run_free:
            test_free_travel(gripper, motor)
        if run_clamp:
            test_torque_clamp(gripper, motor)

    except KeyboardInterrupt:
        print("\n[hybrid] 中断")
    finally:
        # 先打开夹爪（控制线程此时仍在运行），再停线程
        gripper.command(1.0)
        time.sleep(0.3)
        stop_evt.set()
        time.sleep(0.05)
        gripper.disable()
        bus.shutdown()
        print("[hybrid] 完成")


if __name__ == "__main__":
    main()
