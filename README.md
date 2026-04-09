# A1Z — 6-DOF 机械臂 Python SDK

A1Z 六轴机械臂的 Python 控制 SDK，提供 CAN 总线电机驱动、基于 Pinocchio 的重力补偿、正/逆运动学，以及零力示教和位置保持等功能。

## 硬件概览

| 关节 | 名称 | 电机类型 | CAN ID | 扭矩范围 |
|------|------|----------|--------|----------|
| 0 | arm_joint1 | MotorA | 0x01 | ±80 Nm |
| 1 | arm_joint2 | MotorA | 0x02 | ±80 Nm |
| 2 | arm_joint3 | MotorA | 0x03 | ±80 Nm |
| 3 | arm_joint4 | MotorB | 0x04 | ±28 Nm |
| 4 | arm_joint5 | MotorB | 0x05 | ±12 Nm |
| 5 | arm_joint6 | MotorB | 0x06 | ±12 Nm |

所有电机共用一条 CAN 总线（`can0`），波特率 1 Mbps，使用 MIT 力位混控协议。

## 系统架构

```
                    Robot Protocol (robot.py)
                           │
                      ArmRobot (arm_robot.py)
                     ╱          │           ╲
          GravityModel    MixedMotorChain    Kinematics
       (gravity_model.py)  (motor_b_driver.py)   (kinematics.py)
              │            ╱          ╲           │
          Pinocchio     MotorA      MotorB    Pinocchio
           (RNEA)    (motor_a_driver) (motor_b_driver)  (FK/IK)
                          ╲          ╱
                        python-can (SocketCAN)
```

## 安装

### 依赖

- Python >= 3.10
- Linux + SocketCAN（需硬件 CAN 接口）
- URDF 模型文件（默认使用包内自带的 `a1z/robot_models/a1z/A1Z_2kg.urdf`）

### 安装 SDK

```bash
cd /path/to/a1z

# 开发模式安装（推荐）
pip install -e .

# 或直接安装
pip install .
```

依赖会自动安装：`numpy`、`python-can>=4.0`、`pin`（Pinocchio）。

### 配置 CAN 总线

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

## 快速开始

### 零力漂浮模式（零重力示教）

```python
from a1z.robots.get_robot import get_a1z_robot

robot = get_a1z_robot(
    can_channel="can0",
    gravity_comp_factor=1.0,
    zero_gravity_mode=True,
)
robot.start()
# 机械臂进入零力漂浮状态，可自由拖拽
# Ctrl+C 停止
robot.stop()
```

### 位置保持模式

```python
import numpy as np
from a1z.robots.get_robot import get_a1z_robot

robot = get_a1z_robot(
    can_channel="can0",
    zero_gravity_mode=False,
)
robot.start()

# 读取当前关节角
pos = robot.get_joint_pos()
print(f"当前关节角 (rad): {pos}")

# 移动到目标位置
target = np.array([0.0, 0.6, 0.4, -0.5, 0.0, 0.0])
robot.move_joints(target, speed=0.5)

robot.stop()
```

### 使用 example 脚本

```bash
# 零力漂浮
python examples/gravity_comp.py

# 从小补偿因子开始（安全调试）
python examples/gravity_comp.py --gravity_factor 0.3

# 位置保持
python examples/gravity_comp.py --mode hold

# 位置保持 + 移动到目标
python examples/position_hold.py --q_target_deg 0,30,0,-45,0,0 --speed 0.3
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

# 只扫描 MotorA 或 MotorB
python tools/motor_diag.py --scan --type motor_a
python tools/motor_diag.py --scan --type motor_b

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

# 仅标定 MotorA
sudo python tools/set_zero.py --motor-a

# 标定指定关节
sudo python tools/set_zero.py --joints 0 3
```

### 基于限位的零点标定

```bash
# 用机械限位标定关节 1（下限位方向）
sudo python tools/set_zero_limit.py --joints 1 --lower

# 用上限位方向
sudo python tools/set_zero_limit.py --joints 3 --upper
```

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
    ├── set_zero.py                # 电机零点标定
    └── set_zero_limit.py          # 基于限位的零点标定
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
4. 合成最终扭矩：`τ_ff = user_torque + τ_g × sign × scale × factor`
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

## 默认控制参数

| 参数 | 值 |
|------|------|
| 默认 KP | `[30, 30, 30, 20, 5, 5]` |
| 默认 KD | `[1, 1, 1, 0.5, 0.5, 0.5]` |
| 重力方向修正 | `[1, 1, 1, -1, 1, 1]` (关节4反向) |
| 重力扭矩缩放 | `[1, 1.1, 1.1, 1, 1, 1]` |
| 最大重力扭矩 | `[50, 50, 50, 24, 10, 10]` Nm |
| 扭矩限幅 | `[70, 70, 70, 27, 12, 12]` Nm |
| MotorA KT | 2.8 (电流→扭矩转换系数) |
| 控制频率 | 250 Hz |

## 许可

MIT License
