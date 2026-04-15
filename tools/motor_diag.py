#!/usr/bin/env python3
"""A1Z 电机通信诊断与故障排查工具。

功能：
  --scan        扫描所有电机，检查 CAN 通信是否正常
  --monitor     持续监控电机状态（位置/速度/温度/错误码）
  --listen      被动监听 CAN 总线报文（不发送任何指令）
  --probe J     探测指定关节：使能 → 发送零指令 → 读反馈 → 失能

用法：
  # 检查 CAN 接口是否正常
  python tools/motor_diag.py --check-can

  # 扫描所有 6 个电机
  python tools/motor_diag.py --scan

  # 只扫描 MotorA 或 MotorB
  python tools/motor_diag.py --scan --type motor_a
  python tools/motor_diag.py --scan --type motor_b

  # 持续监控所有电机（Ctrl+C 退出）
  python tools/motor_diag.py --monitor

  # 被动监听 CAN 总线 5 秒
  python tools/motor_diag.py --listen --duration 5

  # 探测关节 3（4340）
  python tools/motor_diag.py --probe 3
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import can
import numpy as np

# ── 默认配置（与 get_robot.py 一致） ──────────────────

CAN_CHANNEL = "can0"
CAN_BUSTYPE = "socketcan"
CAN_BITRATE = 1_000_000

JOINT_CONFIG = {
    # joint_idx: (name, motor_type, can_id)
    0: ("arm_joint1", "MOTOR_A", 0x01),
    1: ("arm_joint2", "MOTOR_A", 0x02),
    2: ("arm_joint3", "MOTOR_A", 0x03),
    3: ("arm_joint4", "DM4340", 0x04),
    4: ("arm_joint5", "DM4310", 0x05),
    5: ("arm_joint6", "DM4310", 0x06),
}

# MotorA 反馈解析范围
MOTOR_A_POS_RANGE = (-12.5, 12.5)
MOTOR_A_VEL_RANGE = (-18.0, 18.0)
MOTOR_A_CUR_RANGE = (-30.0, 30.0)

# MotorB 反馈解析范围
MOTOR_B_POS_RANGE = (-12.5, 12.5)
MOTOR_B_VEL_RANGE = (-30.0, 30.0)
MOTOR_B_TOR_RANGE_4340 = (-28.0, 28.0)
MOTOR_B_TOR_RANGE_4310 = (-12.0, 12.0)

MOTOR_B_ERROR_CODES = {
    0x0: "正常",
    0x1: "过压",
    0x2: "欠压",
    0x3: "过流",
    0x4: "MOS 过温",
    0x5: "线圈过温",
    0x6: "通信丢失",
    0x7: "过载",
    0x8: "位置越限",
}

MOTOR_A_KT = 2.8  # 电流→扭矩转换系数


# ── 工具函数 ──────────────────────────────────────────

def uint_to_float(u: int, x_min: float, x_max: float, bits: int) -> float:
    u = max(0, min(int(u), (1 << bits) - 1))
    span = x_max - x_min
    return float(u) * span / ((1 << bits) - 1) + x_min


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = min(max(x, x_min), x_max)
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


@dataclass
class MotorStatus:
    joint_idx: int
    name: str
    motor_type: str
    can_id: int
    online: bool = False
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0  # Nm (MotorA: current*KT, MotorB: torque)
    error_code: int = 0
    error_msg: str = ""
    temp_motor: int = 0
    temp_mos: int = 0
    response_time_ms: float = 0.0
    raw_data: bytes = b""


def parse_motor_a_feedback(data: bytes) -> dict:
    """解析 MotorA 反馈帧。"""
    if len(data) < 8:
        return {}
    frame = int.from_bytes(data, byteorder="big", signed=False)
    error_code = (frame >> 56) & 0x1F
    pos_raw = (frame >> 40) & 0xFFFF
    vel_raw = (frame >> 28) & 0xFFF
    cur_raw = (frame >> 16) & 0xFFF
    temp_motor = (frame >> 8) & 0xFF
    temp_mos = frame & 0xFF

    return {
        "error_code": error_code,
        "position": uint_to_float(pos_raw, *MOTOR_A_POS_RANGE, 16),
        "velocity": uint_to_float(vel_raw, *MOTOR_A_VEL_RANGE, 12),
        "current": uint_to_float(cur_raw, *MOTOR_A_CUR_RANGE, 12),
        "effort": uint_to_float(cur_raw, *MOTOR_A_CUR_RANGE, 12) * MOTOR_A_KT,
        "temp_motor": temp_motor,
        "temp_mos": temp_mos,
    }


def parse_motor_b_feedback(data: bytes, tor_range: Tuple[float, float]) -> dict:
    """解析 MotorB 反馈帧。"""
    if len(data) < 8:
        return {}
    error_int = (data[0] & 0xF0) >> 4
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5]

    return {
        "error_code": error_int,
        "error_msg": MOTOR_B_ERROR_CODES.get(error_int, f"未知({error_int:#x})"),
        "position": uint_to_float(p_int, *MOTOR_B_POS_RANGE, 16),
        "velocity": uint_to_float(v_int, *MOTOR_B_VEL_RANGE, 12),
        "effort": uint_to_float(t_int, *tor_range, 12),
        "temp_mos": int(data[6]),
        "temp_rotor": int(data[7]),
    }


def get_motor_b_torque_range(motor_type: str) -> Tuple[float, float]:
    if "4340" in motor_type:
        return MOTOR_B_TOR_RANGE_4340
    return MOTOR_B_TOR_RANGE_4310


# ── CAN 接口检查 ─────────────────────────────────────

def check_can_interface(channel: str) -> Tuple[bool, str]:
    """检查 CAN 接口状态，返回 (是否正常, 详情)。"""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", channel],
            capture_output=True, text=True, timeout=5,
        )
        output = result.stdout.strip()
        if result.returncode != 0 or not output:
            return False, f"接口 {channel} 不存在。检查 USB-CAN 适配器是否连接。"

        if "state UP" not in output and "LOWER_UP" not in output:
            return False, (
                f"接口 {channel} 存在但未启动。请执行:\n"
                f"  sudo ip link set {channel} down\n"
                f"  sudo ip link set {channel} type can bitrate 1000000 restart-ms 100 txqueuelen 1000\n"
                f"  sudo ip link set {channel} up"
            )

        if "bitrate" in output:
            for part in output.split():
                if part.isdigit() and output[output.index(part) - 8:output.index(part)].strip().endswith("bitrate"):
                    break

        return True, f"接口 {channel} 正常 (UP)"

    except FileNotFoundError:
        return False, "未找到 ip 命令。请确认运行在 Linux 环境。"
    except subprocess.TimeoutExpired:
        return False, "ip 命令超时。"


def check_can_errors(channel: str) -> Optional[str]:
    """检查 CAN 总线错误计数。"""
    try:
        result = subprocess.run(
            ["ip", "-s", "-details", "link", "show", channel],
            capture_output=True, text=True, timeout=5,
        )
        output = result.stdout
        warnings = []

        if "bus-off" in output:
            warnings.append("CAN 总线 BUS-OFF！可能线路短路或终端电阻问题。")
        if "error-passive" in output:
            warnings.append("CAN 总线 ERROR-PASSIVE，错误计数较高。检查接线和终端电阻。")
        if "restarts" in output:
            for line in output.split("\n"):
                if "restarts" in line:
                    parts = line.split()
                    for i, p in enumerate(parts):
                        if p == "restarts" and i > 0:
                            count = parts[i - 1]
                            if count.isdigit() and int(count) > 0:
                                warnings.append(f"CAN 总线已重启 {count} 次，通信不稳定。")

        return "\n".join(warnings) if warnings else None

    except Exception:
        return None


# ── 扫描 ─────────────────────────────────────────────

def scan_motor(bus: can.BusABC, joint_idx: int, timeout: float = 0.3) -> MotorStatus:
    """探测单个电机：发使能帧 → 等回包 → 解析 → 失能。"""
    name, motor_type, can_id = JOINT_CONFIG[joint_idx]
    status = MotorStatus(
        joint_idx=joint_idx, name=name, motor_type=motor_type, can_id=can_id,
    )

    # 清空接收缓冲
    while bus.recv(timeout=0.0) is not None:
        pass

    # 发使能帧
    enable_data = bytes([0xFF] * 7 + [0xFC])
    t_send = time.time()
    try:
        bus.send(can.Message(arbitration_id=can_id, data=enable_data, is_extended_id=False))
    except can.CanOperationError as e:
        status.error_msg = f"CAN 发送失败: {e}"
        return status

    # 等待回包
    t_deadline = time.time() + timeout
    while time.time() < t_deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.arbitration_id != can_id:
            continue

        status.online = True
        status.response_time_ms = (time.time() - t_send) * 1000
        status.raw_data = bytes(msg.data)

        # 解析反馈
        if motor_type == "MOTOR_A":
            fb = parse_motor_a_feedback(msg.data)
            if fb:
                status.position = fb["position"]
                status.velocity = fb["velocity"]
                status.effort = fb["effort"]
                status.error_code = fb["error_code"]
                status.temp_motor = fb["temp_motor"]
                status.temp_mos = fb["temp_mos"]
        else:
            tor_range = get_motor_b_torque_range(motor_type)
            fb = parse_motor_b_feedback(msg.data, tor_range)
            if fb:
                status.position = fb["position"]
                status.velocity = fb["velocity"]
                status.effort = fb["effort"]
                status.error_code = fb["error_code"]
                status.error_msg = fb["error_msg"]
                status.temp_mos = fb["temp_mos"]
                status.temp_motor = fb.get("temp_rotor", 0)
        break

    # 失能
    disable_data = bytes([0xFF] * 7 + [0xFD])
    try:
        bus.send(can.Message(arbitration_id=can_id, data=disable_data, is_extended_id=False))
    except Exception:
        pass

    if not status.online:
        status.error_msg = "无响应"

    return status


def run_scan(bus: can.BusABC, joints: List[int]) -> List[MotorStatus]:
    """扫描指定关节列表。"""
    results = []
    for j in joints:
        status = scan_motor(bus, j)
        results.append(status)
        time.sleep(0.02)
    return results


def print_scan_results(results: List[MotorStatus]):
    """打印扫描结果表格。"""
    print()
    print("┌───────┬────────────┬──────────┬────────┬────────┬──────────────┬─────────┬─────────────────┐")
    print("│ Joint │ Name       │ Type     │ CAN ID │ Status │ Position(deg)│ Resp(ms)│ Error           │")
    print("├───────┼────────────┼──────────┼────────┼────────┼──────────────┼─────────┼─────────────────┤")

    for s in results:
        status_str = " OK " if s.online else "FAIL"
        pos_str = f"{np.degrees(s.position):8.2f}" if s.online else "    N/A "
        resp_str = f"{s.response_time_ms:5.1f}  " if s.online else "   N/A "
        error_str = s.error_msg if s.error_msg else "无"

        if s.online and s.motor_type != "MOTOR_A" and s.error_code != 0:
            error_str = MOTOR_B_ERROR_CODES.get(s.error_code, f"code={s.error_code:#x}")

        print(
            f"│   {s.joint_idx}   │ {s.name:10s} │ {s.motor_type:8s} │ 0x{s.can_id:02X}   │ {status_str} │ {pos_str}│ {resp_str}│ {error_str:15s} │"
        )

    print("└───────┴────────────┴──────────┴────────┴────────┴──────────────┴─────────┴─────────────────┘")

    online = sum(1 for s in results if s.online)
    total = len(results)
    print(f"\n在线: {online}/{total}")

    # 诊断建议
    offline = [s for s in results if not s.online]
    if offline:
        print("\n--- 故障诊断 ---")
        for s in offline:
            print(f"\n  [Joint {s.joint_idx}] {s.name} ({s.motor_type}, ID=0x{s.can_id:02X}): 无响应")
            print("  可能原因:")
            print("    1. 电机未上电 / 电源线松脱")
            print("    2. CAN 线接反（CANH/CANL 互换）或虚接")
            print(f"    3. 电机 CAN ID 不是 0x{s.can_id:02X}（需用厂家工具确认）")
            if s.motor_type == "MOTOR_A":
                print("    4. MotorA 固件未配置为 MIT 模式")
            else:
                print("    5. MotorB 安全超时已触发，需重新上电")
                print("    6. MotorB CAN 波特率不是 1 Mbps")

    # 温度检查
    hot = [s for s in results if s.online and (s.temp_motor > 70 or s.temp_mos > 70)]
    if hot:
        print("\n--- 温度警告 ---")
        for s in hot:
            print(f"  [Joint {s.joint_idx}] {s.name}: 电机温度={s.temp_motor}°C, MOS温度={s.temp_mos}°C")

    # 错误码检查
    errored = [s for s in results if s.online and s.error_code != 0 and s.motor_type != "MOTOR_A"]
    if errored:
        print("\n--- 电机错误 ---")
        for s in errored:
            msg = MOTOR_B_ERROR_CODES.get(s.error_code, f"未知({s.error_code:#x})")
            print(f"  [Joint {s.joint_idx}] {s.name}: 错误码=0x{s.error_code:X} ({msg})")
            if s.error_code == 0x6:
                print("    → 通信丢失：检查控制回路是否在发送指令，或 MotorB 超时保护触发")
            elif s.error_code in (0x4, 0x5):
                print("    → 过温：等待冷却后重试，检查散热和负载")
            elif s.error_code in (0x1, 0x2):
                print("    → 电压异常：检查电源电压是否在额定范围内")
            elif s.error_code == 0x3:
                print("    → 过流：检查是否堵转或负载过大")
            elif s.error_code == 0x7:
                print("    → 过载：降低负载或减小增益")
            print(f"    → 清除错误：发送 0xFF*7+0xFB 到 CAN ID 0x{s.can_id:02X}")


# ── 持续监控 ─────────────────────────────────────────

def run_monitor(bus: can.BusABC, joints: List[int], interval: float = 1.0):
    """持续监控电机状态。"""
    print("持续监控中（Ctrl+C 退出）...\n")

    # 先使能所有电机
    for j in joints:
        _, _, can_id = JOINT_CONFIG[j]
        enable_data = bytes([0xFF] * 7 + [0xFC])
        bus.send(can.Message(arbitration_id=can_id, data=enable_data, is_extended_id=False))
        time.sleep(0.01)

    # 为每个电机准备 MotorA 零指令（用于触发持续反馈）
    from a1z.motor_drivers.motor_a_driver import pack_motor_a_mit

    last_fb: Dict[int, dict] = {}
    comm_loss_count: Dict[int, int] = {j: 0 for j in joints}
    iteration = 0

    try:
        while True:
            iteration += 1

            # 发送零增益/零扭矩指令以触发反馈
            for j in joints:
                name, motor_type, can_id = JOINT_CONFIG[j]
                if motor_type == "MOTOR_A":
                    kp_u12 = float_to_uint(0.0, 0.0, 500.0, 12)
                    kd_u9 = float_to_uint(0.5, 0.0, 5.0, 9)
                    pos_u16 = float_to_uint(0.0, -12.5, 12.5, 16)
                    vel_u12 = float_to_uint(0.0, -18.0, 18.0, 12)
                    tor_u12 = float_to_uint(0.0, -70.0, 70.0, 12)
                    data = pack_motor_a_mit(0, kp_u12, kd_u9, pos_u16, vel_u12, tor_u12)
                else:
                    # MotorB 零指令：pos=0, vel=0, kp=0, kd=0.5, torque=0
                    tor_range = get_motor_b_torque_range(motor_type)
                    pos_u16 = float_to_uint(0.0, -12.5, 12.5, 16)
                    vel_u12 = float_to_uint(0.0, -30.0, 30.0, 12)
                    kp_u12 = float_to_uint(0.0, 0.0, 500.0, 12)
                    kd_u12 = float_to_uint(0.5, 0.0, 5.0, 12)
                    tor_u12 = float_to_uint(0.0, *tor_range, 12)
                    data = bytearray(8)
                    data[0] = (pos_u16 >> 8) & 0xFF
                    data[1] = pos_u16 & 0xFF
                    data[2] = (vel_u12 >> 4) & 0xFF
                    data[3] = ((vel_u12 & 0xF) << 4) | ((kp_u12 >> 8) & 0xF)
                    data[4] = kp_u12 & 0xFF
                    data[5] = (kd_u12 >> 4) & 0xFF
                    data[6] = ((kd_u12 & 0xF) << 4) | ((tor_u12 >> 8) & 0xF)
                    data[7] = tor_u12 & 0xFF
                    data = bytes(data)

                try:
                    bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
                except can.CanOperationError:
                    pass

            # 收集反馈
            t_end = time.time() + 0.05
            received_this_round = set()
            while time.time() < t_end:
                msg = bus.recv(timeout=0.01)
                if msg is None:
                    continue
                mid = msg.arbitration_id
                for j in joints:
                    _, motor_type, can_id = JOINT_CONFIG[j]
                    if can_id != mid:
                        continue
                    received_this_round.add(j)
                    if motor_type == "MOTOR_A":
                        fb = parse_motor_a_feedback(msg.data)
                    else:
                        fb = parse_motor_b_feedback(msg.data, get_motor_b_torque_range(motor_type))
                    if fb:
                        last_fb[j] = fb
                        comm_loss_count[j] = 0

            # 更新通信丢失计数
            for j in joints:
                if j not in received_this_round:
                    comm_loss_count[j] += 1

            # 打印状态
            if iteration % max(1, int(interval / 0.1)) == 0:
                # 清屏 + 表头
                print("\033[2J\033[H", end="")
                print(f"A1Z 电机监控  (iter={iteration}, Ctrl+C 退出)")
                print(f"{'─' * 95}")
                print(
                    f"{'Joint':>5} {'Name':>10} {'Type':>8} {'Pos(deg)':>10} "
                    f"{'Vel(r/s)':>10} {'Eff(Nm)':>9} {'TempM':>5} {'TempMOS':>7} "
                    f"{'Error':>6} {'Comm':>6}"
                )
                print(f"{'─' * 95}")

                for j in joints:
                    name, motor_type, can_id = JOINT_CONFIG[j]
                    fb = last_fb.get(j)
                    lost = comm_loss_count[j]

                    if fb is None:
                        print(
                            f"{j:>5} {name:>10} {motor_type:>8} {'N/A':>10} "
                            f"{'N/A':>10} {'N/A':>9} {'N/A':>5} {'N/A':>7} "
                            f"{'N/A':>6} {'LOST':>6}"
                        )
                        continue

                    pos_deg = np.degrees(fb["position"])
                    vel = fb["velocity"]
                    eff = fb["effort"]
                    err = fb["error_code"]
                    temp_m = fb.get("temp_motor", fb.get("temp_rotor", 0))
                    temp_mos = fb.get("temp_mos", 0)
                    comm_str = "OK" if lost == 0 else f"x{lost}"

                    err_str = f"0x{err:X}" if err != 0 else "OK"

                    print(
                        f"{j:>5} {name:>10} {motor_type:>8} {pos_deg:>10.2f} "
                        f"{vel:>10.3f} {eff:>9.2f} {temp_m:>5} {temp_mos:>7} "
                        f"{err_str:>6} {comm_str:>6}"
                    )

                # 警告
                warnings = []
                for j in joints:
                    fb = last_fb.get(j)
                    if comm_loss_count[j] > 5:
                        warnings.append(f"  [!] Joint {j}: 连续 {comm_loss_count[j]} 次无反馈")
                    if fb:
                        temp_m = fb.get("temp_motor", fb.get("temp_rotor", 0))
                        temp_mos = fb.get("temp_mos", 0)
                        if temp_m > 60 or temp_mos > 60:
                            warnings.append(f"  [!] Joint {j}: 温度偏高 (电机={temp_m}°C, MOS={temp_mos}°C)")
                        if fb["error_code"] != 0:
                            msg = MOTOR_B_ERROR_CODES.get(fb["error_code"], f"code=0x{fb['error_code']:X}")
                            warnings.append(f"  [!] Joint {j}: 错误 {msg}")

                if warnings:
                    print(f"\n{'─' * 95}")
                    print("警告:")
                    for w in warnings:
                        print(w)

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        # 失能所有电机
        for j in joints:
            _, _, can_id = JOINT_CONFIG[j]
            disable_data = bytes([0xFF] * 7 + [0xFD])
            try:
                bus.send(can.Message(arbitration_id=can_id, data=disable_data, is_extended_id=False))
            except Exception:
                pass
        print("\n所有电机已失能。")


# ── 被动监听 ─────────────────────────────────────────

def run_listen(bus: can.BusABC, duration: float):
    """被动监听 CAN 总线，不发送任何帧。"""
    print(f"被动监听 CAN 总线 {duration:.0f} 秒（不发送任何指令）...\n")

    known_ids = {cfg[2]: (idx, cfg[0], cfg[1]) for idx, cfg in JOINT_CONFIG.items()}
    msg_count: Dict[int, int] = {}

    t_end = time.time() + duration
    try:
        while time.time() < t_end:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            mid = msg.arbitration_id
            msg_count[mid] = msg_count.get(mid, 0) + 1
            data_hex = " ".join(f"{b:02X}" for b in msg.data)

            if mid in known_ids:
                j, name, mtype = known_ids[mid]
                tag = f"[J{j} {name} {mtype}]"
            else:
                tag = f"[未知设备]"

            remaining = t_end - time.time()
            print(
                f"[{remaining:5.1f}s] ID=0x{mid:03X} DLC={msg.dlc} "
                f"DATA={data_hex}  {tag}"
            )

    except KeyboardInterrupt:
        pass

    print(f"\n{'─' * 50}")
    print("统计:")
    if not msg_count:
        print("  未收到任何 CAN 报文。")
        print("  可能原因:")
        print("    1. 没有电机在线（未上电）")
        print("    2. CAN 总线未连接或接线错误")
        print("    3. 波特率不匹配")
    else:
        for mid in sorted(msg_count):
            count = msg_count[mid]
            if mid in known_ids:
                j, name, mtype = known_ids[mid]
                print(f"  ID=0x{mid:03X} ({name}): {count} 帧")
            else:
                print(f"  ID=0x{mid:03X} (未知): {count} 帧")


# ── 探测单关节 ───────────────────────────────────────

def run_probe(bus: can.BusABC, joint_idx: int):
    """详细探测单个关节。"""
    name, motor_type, can_id = JOINT_CONFIG[joint_idx]

    print(f"探测 Joint {joint_idx}: {name} ({motor_type}, CAN ID=0x{can_id:02X})")
    print(f"{'─' * 60}")

    # 步骤 1: 清空缓冲
    drained = 0
    while bus.recv(timeout=0.01) is not None:
        drained += 1
    if drained:
        print(f"[1/5] 清空接收缓冲: 丢弃 {drained} 帧")
    else:
        print("[1/5] 接收缓冲为空")

    # 步骤 2: 发使能帧
    print(f"[2/5] 发送使能帧 (0xFC) → ID=0x{can_id:02X} ...")
    enable_data = bytes([0xFF] * 7 + [0xFC])
    t_send = time.time()
    bus.send(can.Message(arbitration_id=can_id, data=enable_data, is_extended_id=False))

    # 步骤 3: 等待回包
    print("[3/5] 等待回包 (最多 1 秒) ...")
    response = None
    t_deadline = time.time() + 1.0
    while time.time() < t_deadline:
        msg = bus.recv(timeout=0.05)
        if msg is not None and msg.arbitration_id == can_id:
            response = msg
            break

    if response is None:
        print("  → 未收到回包！")
        print()
        print("诊断:")
        print(f"  1. 确认电机已上电")
        print(f"  2. 确认 CAN ID 是 0x{can_id:02X}")
        if motor_type == "MOTOR_A":
            print(f"  3. 确认 MotorA 固件已配置为 MIT 模式")
            print(f"  4. 尝试发送 MIT 零指令触发回包: --probe {joint_idx}")
        else:
            print(f"  3. 确认 MotorB CAN 波特率 = 1 Mbps")
            print(f"  4. 尝试清错: 发送 0xFF*7+0xFB 到 0x{can_id:02X}")
            print(f"  5. 检查 MotorB 超时保护是否触发（需重新上电）")
        # 失能
        bus.send(can.Message(arbitration_id=can_id, data=bytes([0xFF]*7+[0xFD]), is_extended_id=False))
        return

    resp_ms = (time.time() - t_send) * 1000
    data_hex = " ".join(f"{b:02X}" for b in response.data)
    print(f"  → 收到回包! 响应时间={resp_ms:.1f}ms")
    print(f"  → 原始数据: [{data_hex}]")

    # 步骤 4: 解析反馈
    print(f"[4/5] 解析反馈帧 ...")
    if motor_type == "MOTOR_A":
        fb = parse_motor_a_feedback(response.data)
        if fb:
            print(f"  位置:     {fb['position']:.4f} rad ({np.degrees(fb['position']):.2f} deg)")
            print(f"  速度:     {fb['velocity']:.4f} rad/s")
            print(f"  电流:     {fb['current']:.3f} A")
            print(f"  等效扭矩: {fb['effort']:.3f} Nm (KT={MOTOR_A_KT})")
            print(f"  错误码:   0x{fb['error_code']:02X}")
            print(f"  电机温度: {fb['temp_motor']}°C")
            print(f"  MOS温度:  {fb['temp_mos']}°C")
    else:
        tor_range = get_motor_b_torque_range(motor_type)
        fb = parse_motor_b_feedback(response.data, tor_range)
        if fb:
            print(f"  位置:     {fb['position']:.4f} rad ({np.degrees(fb['position']):.2f} deg)")
            print(f"  速度:     {fb['velocity']:.4f} rad/s")
            print(f"  扭矩:     {fb['effort']:.3f} Nm")
            print(f"  错误码:   0x{fb['error_code']:X} ({fb['error_msg']})")
            print(f"  MOS温度:  {fb['temp_mos']}°C")
            print(f"  转子温度: {fb['temp_rotor']}°C")

            if fb["error_code"] != 0:
                print()
                print(f"  *** 电机有错误！***")
                err = fb["error_code"]
                if err == 0x6:
                    print("  → 通信丢失: 电机长时间未收到指令。需要清错后持续发送控制帧。")
                elif err in (0x4, 0x5):
                    print("  → 过温: 等待冷却。检查散热条件和持续负载。")
                elif err in (0x1, 0x2):
                    print("  → 电压异常: 检查电源电压（额定 24V）。")
                elif err == 0x3:
                    print("  → 过流: 检查是否堵转。降低增益或限制扭矩指令。")
                print(f"  → 清除错误: 发送 0xFF*7+0xFB 到 0x{can_id:02X}")

    # 步骤 5: 失能
    print(f"[5/5] 发送失能帧 (0xFD)")
    disable_data = bytes([0xFF] * 7 + [0xFD])
    bus.send(can.Message(arbitration_id=can_id, data=disable_data, is_extended_id=False))
    print("  → 完成")


# ── 清除 DM 电机错误 ─────────────────────────────────

def run_clear_error(bus: can.BusABC, joints: List[int]):
    """清除 MotorB 错误码。"""
    for j in joints:
        name, motor_type, can_id = JOINT_CONFIG[j]
        if motor_type == "MOTOR_A":
            print(f"  Joint {j} ({name}): MotorA 不支持清错指令，跳过")
            continue
        clear_data = bytes([0xFF] * 7 + [0xFB])
        bus.send(can.Message(arbitration_id=can_id, data=clear_data, is_extended_id=False))
        print(f"  Joint {j} ({name}): 已发送清错指令 → 0x{can_id:02X}")
        time.sleep(0.01)


# ── Main ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="A1Z 电机通信诊断与故障排查",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python tools/motor_diag.py --check-can          检查 CAN 接口状态
  python tools/motor_diag.py --scan               扫描所有电机
  python tools/motor_diag.py --scan --type motor_a  只扫描 MotorA
  python tools/motor_diag.py --monitor             持续监控
  python tools/motor_diag.py --listen              被动监听 CAN 总线
  python tools/motor_diag.py --probe 3             探测关节 3
  python tools/motor_diag.py --clear-error         清除所有 MotorB 错误
  python tools/motor_diag.py --clear-error --joints 3 4  清除指定关节错误
        """,
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--check-can", action="store_true", help="检查 CAN 接口状态")
    group.add_argument("--scan", action="store_true", help="扫描电机通信")
    group.add_argument("--monitor", action="store_true", help="持续监控电机状态")
    group.add_argument("--listen", action="store_true", help="被动监听 CAN 总线")
    group.add_argument("--probe", type=int, metavar="JOINT", help="探测指定关节 (0-5)")
    group.add_argument("--clear-error", action="store_true", help="清除 DM 电机错误码")

    parser.add_argument("--channel", default=CAN_CHANNEL, help=f"CAN 通道 (默认: {CAN_CHANNEL})")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE, help=f"CAN 波特率 (默认: {CAN_BITRATE})")
    parser.add_argument("--type", choices=["all", "motor_a", "motor_b"], default="all", help="电机类型筛选")
    parser.add_argument("--joints", type=int, nargs="+", metavar="J", help="指定关节 (0-5)")
    parser.add_argument("--duration", type=float, default=5.0, help="监听时长/秒 (默认: 5)")

    args = parser.parse_args()

    # 确定要操作的关节
    if args.joints:
        joints = args.joints
    elif args.type == "motor_a":
        joints = [0, 1, 2]
    elif args.type == "motor_b":
        joints = [3, 4, 5]
    else:
        joints = list(range(6))

    for j in joints:
        if j not in JOINT_CONFIG:
            print(f"错误: 关节 {j} 不存在 (有效范围: 0-5)")
            sys.exit(1)

    print("=" * 60)
    print("  A1Z 电机诊断工具")
    print("=" * 60)

    # check-can 不需要打开 CAN 总线
    if args.check_can:
        print(f"\n检查 CAN 接口: {args.channel}")
        ok, detail = check_can_interface(args.channel)
        if ok:
            print(f"  [OK] {detail}")
        else:
            print(f"  [FAIL] {detail}")
            sys.exit(1)

        err_info = check_can_errors(args.channel)
        if err_info:
            print(f"\n  [WARN] {err_info}")
        else:
            print("  [OK] 无总线错误")
        return

    # 其余操作需要打开 CAN 总线
    print(f"\nCAN 通道: {args.channel}  波特率: {args.bitrate}")

    # 先检查接口
    ok, detail = check_can_interface(args.channel)
    if not ok:
        print(f"\n[FAIL] {detail}")
        sys.exit(1)

    try:
        bus = can.interface.Bus(channel=args.channel, bustype=CAN_BUSTYPE, bitrate=args.bitrate)
    except Exception as e:
        print(f"\n无法打开 CAN 总线: {e}")
        print(f"请检查: sudo ip link set {args.channel} up type can bitrate {args.bitrate}")
        sys.exit(1)

    try:
        if args.scan:
            print(f"\n扫描电机 (joints={joints}) ...")
            results = run_scan(bus, joints)
            print_scan_results(results)

        elif args.monitor:
            run_monitor(bus, joints, interval=1.0)

        elif args.listen:
            run_listen(bus, args.duration)

        elif args.probe is not None:
            if args.probe not in JOINT_CONFIG:
                print(f"错误: 关节 {args.probe} 不存在")
                sys.exit(1)
            run_probe(bus, args.probe)

        elif args.clear_error:
            run_clear_error(bus, joints)

    except KeyboardInterrupt:
        print("\n中断。")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass
        print("CAN 总线已关闭。")


if __name__ == "__main__":
    main()
