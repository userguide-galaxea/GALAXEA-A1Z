<a id="chinese"></a>

[中文](#chinese) | [English](#english)

# A1Z — 6-DOF 机械臂 Python SDK

<p align="center">
  <img src="docs/images/A1Z.png" alt="A1Z 机械臂" width="500"/>
</p>

A1Z 六轴机械臂的 Python 控制 SDK，提供 CAN 总线电机驱动、基于 Pinocchio 的重力补偿、正/逆运动学，以及零力示教和位置保持等功能。

## 硬件概览

| 关节 | 名称 | 电机类型 | CAN ID | 扭矩范围 |
|------|------|----------|--------|----------|
| 0 | arm_joint1 | MotorA | 0x01 | ±50 Nm |
| 1 | arm_joint2 | MotorA | 0x02 | ±50 Nm |
| 2 | arm_joint3 | MotorA | 0x03 | ±50 Nm |
| 3 | arm_joint4 | MotorB | 0x04 | ±25 Nm |
| 4 | arm_joint5 | MotorB | 0x05 | ±7 Nm |
| 5 | arm_joint6 | MotorB | 0x06 | ±7 Nm |

所有电机共用一条 CAN 总线（`can0`），波特率 1 Mbps，使用 MIT 力位混控协议。

## 项目结构

```
a1z/
├── pyproject.toml                 # 构建配置 (flit)
├── setup.py                       # setuptools 后备
├── README.md
├── a1z/                       # SDK 主包
│   ├── dynamics/
│   │   └── gravity_model.py       # Pinocchio RNEA 重力补偿
│   ├── motor_drivers/
│   │   ├── can_interface.py       # CAN 总线封装
│   │   ├── motor_a_driver.py      # MotorA 驱动 (MIT 混控)
│   │   ├── motor_b_driver.py      # MotorB 驱动 + MixedMotorChain
│   │   └── utils.py               # 数据结构, float↔uint 转换
│   ├── robots/
│   │   ├── robot.py               # Robot Protocol (抽象接口)
│   │   ├── arm_robot.py           # ArmRobot 实现 (控制回路+重力补偿)
│   │   ├── get_robot.py           # 工厂函数 get_a1z_robot()
│   │   └── kinematics.py          # FK/IK (Pinocchio)
│   ├── robot_models/
│   │   └── a1z/               # URDF 模型文件
│   └── utils/
│       └── utils.py               # RateRecorder, 日志工具
├── examples/
│   ├── gravity_comp.py            # 重力补偿示例
│   └── position_hold.py           # 位置保持示例
└── tools/
    ├── motor_diag.py              # 电机通信诊断与故障排查
    └── set_zero.py                # 电机零点标定
```

## 安装

### 依赖

- Python >= 3.10
- Linux + SocketCAN（需硬件 CAN 接口）
- URDF 模型文件（包内自带，见 `a1z/robot_models/a1z/`，默认使用 `A1Z_Flange.urdf`）

### 安装 SDK

```bash
# 不带夹爪的A1Z机械臂sdk
git clone https://github.com/userguide-galaxea/GALAXEA-A1Z.git

# 注：如购买G1Z夹爪，请直接使用本项目的gripper分支
git clone -b gripper https://github.com/userguide-galaxea/GALAXEA-A1Z.git


cd /path/to/GALAXEA-A1Z

# 开发模式安装（推荐）
pip install -e .

# 或直接安装
pip install .
```

依赖会自动安装：`numpy`、`python-can>=4.0`、`pin`（Pinocchio）。

### 配置 CAN 总线（SocketCAN 模式）

注意：检查can盒电阻是否正确安装！

使用 HHS USB-CANFD 适配器（VID/PID `a8fa:8598`）：

```bash
# 1. 加载驱动
sudo modprobe gs_usb

# 2. 将 HHS 适配器绑定到 gs_usb（已绑定时忽略报错）
sudo sh -c 'echo "a8fa 8598" > /sys/bus/usb/drivers/gs_usb/new_id' 2>/dev/null || true

# 3. 确认接口出现（单适配器通常为 can0）
ip link show type can

# 4. 配置并启动（1 Mbps）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 快速开始

### 使用 example 脚本

```bash
# 零力漂浮（默认 URDF A1Z_Flange.urdf，末端无负载）

# 从小补偿因子开始（推荐首次调试方式）
python examples/gravity_comp.py --gravity_factor 0.3

# 确认补偿方向正确后提升到全补偿
python examples/gravity_comp.py --gravity_factor 1.0

# 位置保持模式
python examples/gravity_comp.py --mode hold

# 位置保持 + 移动到目标
python examples/position_hold.py --q_target_deg 0,30,-20,-15,0,0 --speed 0.5
```

## API 参考

### `get_a1z_robot()`

工厂函数，创建配置好的 ArmRobot 实例：

```python
get_a1z_robot(
    can_channel="can0",           # CAN 通道名
    gravity_comp_factor=1.0,      # 重力补偿比例 (0=关闭, 1=全补偿)
    zero_gravity_mode=True,       # True=零力漂浮, False=位置保持
    control_freq_hz=250,          # 控制回路频率 (Hz)
    urdf_path=None,               # 覆盖 URDF 路径
    default_kp=None,              # 覆盖默认位置增益
    default_kd=None,              # 覆盖默认速度增益
) -> ArmRobot
```

### `ArmRobot` 主要方法

| 方法 | 说明 |
|------|------|
| `start(initial_kp, initial_kd)` | 使能电机，启动控制回路 |
| `stop()` | 平滑停机（0.3s 衰减），失能电机 |
| `get_joint_pos() -> np.ndarray` | 获取当前关节角 (rad) |
| `get_joint_state() -> dict` | 获取 `{pos, vel, eff}` |
| `command_joint_pos(pos)` | 设置目标关节角（使用默认 PD 增益） |
| `command_joint_state(joint_state)` | 设置目标关节角 + 自定义增益 |
| `move_joints(target, speed, kp, kd)` | 线性插值移动到目标位置（阻塞） |
| `is_running` | 控制回路是否在运行 |

### `Kinematics` 运动学

```python
from a1z.robots.kinematics import Kinematics

kin = Kinematics("/path/to/urdf")

# 正运动学 → 4x4 齐次变换矩阵
T = kin.fk(q)

# 逆运动学 (阻尼最小二乘)
converged, q_sol = kin.ik(target_pose, init_q=q0)
```

## 工具

### 电机通信诊断与故障排查

```bash
# 检查 CAN 接口是否正常
python tools/motor_diag.py --check-can

# 扫描所有 6 个电机（检查通信、读取状态、自动诊断）
python tools/motor_diag.py --scan

# 详细探测某个关节（完整收发流程 + 反馈解析）
python tools/motor_diag.py --probe 3

# 持续监控所有电机状态（位置/速度/温度/错误码）
python tools/motor_diag.py --monitor

# 被动监听 CAN 总线（不发任何指令，用于排查总线冲突）
python tools/motor_diag.py --listen --duration 10

# 清除 MotorB 错误码
python tools/motor_diag.py --clear-error
python tools/motor_diag.py --clear-error --joints 3 4
```

诊断脚本会自动检测并给出常见问题的排查建议：
- 电机无响应（未上电 / CAN 线反接 / ID 错误 / 固件模式）
- MotorB 错误码（过压/欠压/过流/过温/通信丢失/过载）
- CAN 总线异常（bus-off / error-passive / 重启次数）
- 温度预警

### 电机零点标定

```bash
# 标定所有电机（当前位置设为零点）
sudo python tools/set_zero.py --all

# 标定指定关节
sudo python tools/set_zero.py --joints 0 3
```


## 控制原理

### MIT 力位混控

电机固件执行：

```
τ_motor = kp × (pos_target − pos_actual) + kd × (vel_target − vel_actual) + τ_ff
```

SDK 在每个控制周期（默认 250 Hz）执行：

1. 从 CAN 总线读取所有电机反馈
2. 通过 Pinocchio RNEA 计算当前姿态下的重力补偿扭矩 `τ_g(q)`
3. 安全检查：`|τ_g|` 超过阈值则紧急停止
4. 合成最终扭矩：`τ_motor = (user_torque + τ_g × scale × factor) × joint_sign`
5. 裁剪到安全范围后下发

### 零力漂浮模式

`kp=0, kd=较小值`，仅靠重力补偿扭矩抵消重力，机械臂可自由拖拽。

### 位置保持模式

`kp=默认增益, kd=默认增益`，PD 控制 + 重力补偿。

## 安全注意事项

- 首次使用请将 `gravity_comp_factor` 设为较小值（如 0.3），确认补偿方向正确后再逐步增大
- 重力扭矩超过每关节安全阈值时会自动紧急停止
- 停机时会在 0.3s 内平滑衰减重力补偿并增加阻尼，避免突然失能导致机械臂下落
- 所有目标关节角会被裁剪到 URDF 限位范围内

## 关节限位

| 关节 | 名称 | 机械限位 (°) | 机械限位 (rad) | 软限位 (°) | 软限位 (rad) |
|------|------|-------------|--------------|-----------|-------------|
| 0 | arm_joint1 | [-130°, 130°] | [-2.269, 2.269] | [-120°, 120°] | [-2.094, 2.094] |
| 1 | arm_joint2 | [-1.94°, 192.78°] | [-0.034, 3.365] | [0°, 180°] | [0.000, 3.142] |
| 2 | arm_joint3 | [-200.38°, 0°] | [-3.497, 0.000] | [-180°, 0°] | [-3.142, 0] |
| 3 | arm_joint4 | [-91.88°, 110.38°] | [-1.604, 1.926] | [-85°, 85°] | [-1.484, 1.484] |
| 4 | arm_joint5 | [-90°, 90°] | [-1.571, 1.571] | [-85°, 85°] | [-1.484, 1.484] |
| 5 | arm_joint6 | [-120°, 120°] | [-2.094, 2.094] | [-115°, 115°] | [-2.007, 2.007] |

## 默认控制参数

| 参数 | 值 |
|------|------|
| 默认 KP | `[30, 30, 30, 20, 5, 5]` |
| 默认 KD | `[1, 1, 1, 0.5, 0.5, 0.5]` |
| 关节坐标系符号 | `[1, 1, -1, 1, -1, 1]` (关节3,5与URDF方向相反) |
| 重力扭矩缩放 | `[1, 1, 1, 1, 1, 1]` |
| 最大重力扭矩 | `[50, 50, 50, 24, 10, 10]` Nm |
| 扭矩限幅 | `[70, 70, 70, 27, 10, 10]` Nm |
| MotorA KT | 2.8 (电流→扭矩转换系数) |
| 控制频率 | 250 Hz |

## 开源许可

本项目基于 [MIT License](LICENSE) 开源，版权归 **星海图** 所有。

### 第三方依赖许可

| 依赖 | 许可证 | 说明 |
|------|--------|------|
| [numpy](https://numpy.org) | BSD-3-Clause | 数值计算 |
| [python-can](https://github.com/hardbyte/python-can) | LGPL-3.0 | CAN 总线通信 |
| [pinocchio (pin)](https://github.com/stack-of-tasks/pinocchio) | BSD-2-Clause | 机器人动力学计算 |

以上依赖均与 MIT 协议兼容，可自由用于商业和非商业项目。

---

<a id="english"></a>

[中文](#chinese) | [English](#english)

# A1Z — 6-DOF Robotic Arm Python SDK

<p align="center">
  <img src="docs/images/A1Z.png" alt="A1Z robotic arm" width="500"/>
</p>

A Python control SDK for the A1Z six-axis robotic arm, providing CAN-bus motor drivers, Pinocchio-based gravity compensation, forward/inverse kinematics, zero-force teaching, and position hold.

## Hardware Overview

| Joint | Name | Motor Type | CAN ID | Torque Range |
|-------|------|------------|--------|--------------|
| 0 | arm_joint1 | MotorA | 0x01 | ±50 Nm |
| 1 | arm_joint2 | MotorA | 0x02 | ±50 Nm |
| 2 | arm_joint3 | MotorA | 0x03 | ±50 Nm |
| 3 | arm_joint4 | MotorB | 0x04 | ±25 Nm |
| 4 | arm_joint5 | MotorB | 0x05 | ±7 Nm |
| 5 | arm_joint6 | MotorB | 0x06 | ±7 Nm |

All motors share a single CAN bus (`can0`) at 1 Mbps using the MIT position-velocity-torque mixed control protocol.

## Project Structure

```
a1z/
├── pyproject.toml                 # Build config (flit)
├── setup.py                       # setuptools fallback
├── README.md
├── a1z/                       # SDK main package
│   ├── dynamics/
│   │   └── gravity_model.py       # Pinocchio RNEA gravity compensation
│   ├── motor_drivers/
│   │   ├── can_interface.py       # CAN bus wrapper
│   │   ├── motor_a_driver.py      # MotorA driver (MIT mixed control)
│   │   ├── motor_b_driver.py      # MotorB driver + MixedMotorChain
│   │   └── utils.py               # Data structures, float↔uint conversion
│   ├── robots/
│   │   ├── robot.py               # Robot Protocol (abstract interface)
│   │   ├── arm_robot.py           # ArmRobot implementation (control loop + gravity comp)
│   │   ├── get_robot.py           # Factory function get_a1z_robot()
│   │   └── kinematics.py          # FK/IK (Pinocchio)
│   ├── robot_models/
│   │   └── a1z/               # URDF model files
│   └── utils/
│       └── utils.py               # RateRecorder, logging utilities
├── examples/
│   ├── gravity_comp.py            # Gravity compensation example
│   └── position_hold.py           # Position hold example
└── tools/
    ├── motor_diag.py              # Motor communication diagnostics
    └── set_zero.py                # Motor zero calibration
```

## Installation

### Prerequisites

- Python >= 3.10
- Linux + SocketCAN (hardware CAN interface required)
- URDF model files (bundled, see `a1z/robot_models/a1z/`, defaults to `A1Z_Flange.urdf`)

### Install the SDK

```bash
# A1Z arm SDK (without gripper)
git clone https://github.com/userguide-galaxea/GALAXEA-A1Z.git

# Note: if you have the G1Z gripper, use the gripper branch instead
git clone -b gripper https://github.com/userguide-galaxea/GALAXEA-A1Z.git

cd /path/to/GALAXEA-A1Z

# Development mode (recommended)
pip install -e .

# Or standard install
pip install .
```

Dependencies are installed automatically: `numpy`, `python-can>=4.0`, `pin` (Pinocchio).

### Configure the CAN Bus (SocketCAN)

> Note: verify that the CAN termination resistor is installed correctly!

Using the HHS USB-CANFD adapter (VID/PID `a8fa:8598`):

```bash
# 1. Load the driver
sudo modprobe gs_usb

# 2. Bind the HHS adapter to gs_usb (ignore errors if already bound)
sudo sh -c 'echo "a8fa 8598" > /sys/bus/usb/drivers/gs_usb/new_id' 2>/dev/null || true

# 3. Confirm the interface appears (usually can0 for a single adapter)
ip link show type can

# 4. Configure and bring up (1 Mbps)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## Quick Start

### Example Scripts

```bash
# Zero-force float (default URDF A1Z_Flange.urdf, no end-effector load)

# Start with a small compensation factor (recommended for first-time setup)
python examples/gravity_comp.py --gravity_factor 0.3

# Increase to full compensation once direction is confirmed correct
python examples/gravity_comp.py --gravity_factor 1.0

# Position hold mode
python examples/gravity_comp.py --mode hold

# Position hold + move to target
python examples/position_hold.py --q_target_deg 0,30,-20,-15,0,0 --speed 0.5
```

## API Reference

### `get_a1z_robot()`

Factory function that creates a configured `ArmRobot` instance:

```python
get_a1z_robot(
    can_channel="can0",           # CAN channel name
    gravity_comp_factor=1.0,      # Gravity compensation scale (0=off, 1=full)
    zero_gravity_mode=True,       # True=zero-force float, False=position hold
    control_freq_hz=250,          # Control loop frequency (Hz)
    urdf_path=None,               # Override URDF path
    default_kp=None,              # Override default position gains
    default_kd=None,              # Override default velocity gains
) -> ArmRobot
```

### `ArmRobot` Key Methods

| Method | Description |
|--------|-------------|
| `start(initial_kp, initial_kd)` | Enable motors and start the control loop |
| `stop()` | Smooth shutdown (0.3 s decay), disable motors |
| `get_joint_pos() -> np.ndarray` | Get current joint angles (rad) |
| `get_joint_state() -> dict` | Get `{pos, vel, eff}` |
| `command_joint_pos(pos)` | Set target joint angles (uses default PD gains) |
| `command_joint_state(joint_state)` | Set target joint angles + custom gains |
| `move_joints(target, speed, kp, kd)` | Interpolate to target position (blocking) |
| `is_running` | Whether the control loop is active |

### `Kinematics`

```python
from a1z.robots.kinematics import Kinematics

kin = Kinematics("/path/to/urdf")

# Forward kinematics → 4×4 homogeneous transform
T = kin.fk(q)

# Inverse kinematics (damped least squares)
converged, q_sol = kin.ik(target_pose, init_q=q0)
```

## Tools

### Motor Communication Diagnostics

```bash
# Check CAN interface
python tools/motor_diag.py --check-can

# Scan all 6 motors (check communication, read state, auto-diagnose)
python tools/motor_diag.py --scan

# Detailed probe of a specific joint (full TX/RX flow + feedback parsing)
python tools/motor_diag.py --probe 3

# Continuous monitoring of all motor states (position/velocity/temperature/error codes)
python tools/motor_diag.py --monitor

# Passive CAN bus listener (no commands sent, for diagnosing bus conflicts)
python tools/motor_diag.py --listen --duration 10

# Clear MotorB error codes
python tools/motor_diag.py --clear-error
python tools/motor_diag.py --clear-error --joints 3 4
```

The diagnostic script automatically detects and suggests remedies for common issues:
- Motor not responding (unpowered / reversed CAN wiring / wrong ID / firmware mode)
- MotorB error codes (overvoltage / undervoltage / overcurrent / overtemperature / comm loss / overload)
- CAN bus faults (bus-off / error-passive / restart count)
- Temperature warnings

### Motor Zero Calibration

```bash
# Calibrate all motors (set current position as zero)
sudo python tools/set_zero.py --all

# Calibrate specific joints
sudo python tools/set_zero.py --joints 0 3
```

## Control Architecture

### MIT Position-Velocity-Torque Mixed Control

The motor firmware executes:

```
τ_motor = kp × (pos_target − pos_actual) + kd × (vel_target − vel_actual) + τ_ff
```

The SDK runs the following at each control cycle (default 250 Hz):

1. Read all motor feedback from the CAN bus
2. Compute gravity compensation torque `τ_g(q)` via Pinocchio RNEA
3. Safety check: emergency stop if any `|τ_g|` exceeds the per-joint threshold
4. Compose final torque: `τ_motor = (user_torque + τ_g × scale × factor) × joint_sign`
5. Clip to safe range and send to motors

### Zero-Force Float Mode

`kp=0, kd=small value` — only gravity compensation torque counters gravity; the arm can be freely backdriven.

### Position Hold Mode

`kp=default gains, kd=default gains` — PD control plus gravity compensation.

## Safety

- For first use, set `gravity_comp_factor` to a small value (e.g. 0.3) and confirm compensation direction before increasing
- An emergency stop triggers automatically if gravity torques exceed per-joint safety thresholds
- On shutdown, gravity compensation decays smoothly over 0.3 s with increased damping to prevent sudden drops
- All target joint angles are clipped to the URDF joint limits

## Joint Limits

| Joint | Name | Mechanical (°) | Mechanical (rad) | Software (°) | Software (rad) |
|-------|------|----------------|-----------------|--------------|----------------|
| 0 | arm_joint1 | [-130°, 130°] | [-2.269, 2.269] | [-120°, 120°] | [-2.094, 2.094] |
| 1 | arm_joint2 | [-1.94°, 192.78°] | [-0.034, 3.365] | [0°, 180°] | [0.000, 3.142] |
| 2 | arm_joint3 | [-200.38°, 0°] | [-3.497, 0.000] | [-180°, 0°] | [-3.142, 0] |
| 3 | arm_joint4 | [-91.88°, 110.38°] | [-1.604, 1.926] | [-85°, 85°] | [-1.484, 1.484] |
| 4 | arm_joint5 | [-90°, 90°] | [-1.571, 1.571] | [-85°, 85°] | [-1.484, 1.484] |
| 5 | arm_joint6 | [-120°, 120°] | [-2.094, 2.094] | [-115°, 115°] | [-2.007, 2.007] |

## Default Control Parameters

| Parameter | Value |
|-----------|-------|
| Default KP | `[30, 30, 30, 20, 5, 5]` |
| Default KD | `[1, 1, 1, 0.5, 0.5, 0.5]` |
| Joint sign | `[1, 1, -1, 1, -1, 1]` (joints 3 and 5 are inverted relative to URDF) |
| Gravity torque scale | `[1, 1, 1, 1, 1, 1]` |
| Max gravity torque | `[50, 50, 50, 24, 10, 10]` Nm |
| Torque clip | `[70, 70, 70, 27, 10, 10]` Nm |
| MotorA KT | 2.8 (current-to-torque conversion factor) |
| Control frequency | 250 Hz |

## License

This project is open-sourced under the [MIT License](LICENSE), copyright © **Galaxea**.

### Third-Party Dependency Licenses

| Dependency | License | Description |
|------------|---------|-------------|
| [numpy](https://numpy.org) | BSD-3-Clause | Numerical computation |
| [python-can](https://github.com/hardbyte/python-can) | LGPL-3.0 | CAN bus communication |
| [pinocchio (pin)](https://github.com/stack-of-tasks/pinocchio) | BSD-2-Clause | Robot dynamics computation |

All dependencies are compatible with the MIT license and may be used freely in commercial and non-commercial projects.
