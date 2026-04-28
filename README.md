# A1Z — 6-DOF 机械臂 Python SDK（带G1Z夹爪）

A1Z 六轴机械臂的 Python 控制 SDK，提供 CAN 总线电机驱动、基于 Pinocchio 的重力补偿、正/逆运动学，以及零力示教和位置保持等功能。

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
│   │   ├── gripper.py             # Gripper 控制 (MotorB CAN ID 0x07)
│   │   ├── server.py              # Unix socket 控制服务端
│   │   └── kinematics.py          # FK/IK (Pinocchio)
│   ├── robot_models/
│   │   └── a1z/               # URDF 模型文件 (A1Z_G1Z.urdf 默认)
│   └── utils/
│       └── utils.py               # RateRecorder, 日志工具
├── examples/
│   ├── gravity_comp.py            # 重力补偿示例
│   ├── position_hold.py           # 位置保持示例
│   └── gripper_test.py            # 独立夹爪测试 (100 Hz + 交互输入)
└── tools/
    ├── a1zctl                     # 机械臂控制 CLI（serve/move/gripper/dance/stop）
    ├── gripper_set_zero.py        # 夹爪零点标定（出厂已完成，一般无需执行）
    ├── motor_diag.py              # 电机通信诊断与故障排查
    └── set_zero.py                # 电机零点标定
```


## 安装

### 依赖

- Python >= 3.10
- Linux + SocketCAN（需硬件 CAN 接口）
- URDF 模型文件（包内自带，见 `a1z/robot_models/a1z/`，默认使用 `A1Z_G1Z.urdf`，含夹爪末端）

### 安装 SDK

```bash
cd /path/to/a1z

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
# 零力漂浮（默认 URDF A1Z_G1Z.urdf，含夹爪）

# 从小补偿因子开始（推荐首次调试方式）
python examples/gravity_comp.py --gravity_factor 0.3

# 确认补偿方向正确后提升到全补偿
python examples/gravity_comp.py --gravity_factor 1.0

# 位置保持模式
python examples/gravity_comp.py --mode hold

# 位置保持 + 移动到目标
python examples/position_hold.py --q_target_deg 0,30,20,-15,0,0 --speed 0.5

# 独立夹爪测试（100 Hz 控制回路 + 交互式开度输入）
python examples/gripper_test.py --can can0
```

### 使用 a1zctl 服务端

```bash
# 终端 1：启动服务端（含夹爪）
python3 tools/a1zctl serve --with-gripper

# 终端 2：发送控制指令
python3 tools/a1zctl status              # 查看关节状态（含夹爪开度）
python3 tools/a1zctl move --preset home  # 移动到预置位
python3 tools/a1zctl move 0,60,-60,0,0,0 --speed 0.5
python3 tools/a1zctl gripper 0.5         # 夹爪到 50% 开度
python3 tools/a1zctl dance --moves salute,wave,nod
python3 tools/a1zctl info                # 查看所有预置位与限位
python3 tools/a1zctl stop               # 停止服务端
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
    with_gripper=False,           # True=启用夹爪 (CAN ID 0x07)
    gripper_max_torque=-1.0,      # 夹爪力限制 (Nm)，-1=不限制
    gripper_kp=None,              # 覆盖夹爪位置增益（默认 10.0）
    gripper_clog_threshold=None,  # 覆盖夹爪堵转扭矩阈值 (Nm)，默认 0.3
) -> ArmRobot
```

### `ArmRobot` 主要方法

| 方法 | 说明 |
|------|------|
| `start(initial_kp, initial_kd)` | 使能电机，启动控制回路（含夹爪归零） |
| `stop()` | 平滑停机（0.8s 衰减），失能电机 |
| `get_joint_pos() -> np.ndarray` | 获取当前关节角 (rad)；有夹爪时返回 7 元素数组（第 7 个为夹爪归一化开度） |
| `get_joint_state() -> dict` | 获取 `{pos, vel, eff}`（仅 6 轴） |
| `get_observations() -> dict` | 获取 `{joint_pos, gripper_pos, joint_vel, joint_eff}`；无夹爪时等同于 `get_joint_state` |
| `command_joint_pos(pos)` | 设置目标关节角；可传 7 元素数组，第 7 个为夹爪开度 [0, 1] |
| `command_joint_state(joint_state)` | 设置目标关节角 + 自定义增益 |
| `command_gripper(value)` | 设置夹爪目标开度 [0.0=关闭, 1.0=全开] |
| `get_gripper_pos() -> float\|None` | 获取当前夹爪指令开度；无夹爪返回 None |
| `move_joints(target, speed, kp, kd)` | 线性插值移动到目标位置（阻塞）；可传 7 元素数组 |
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

## 夹爪

夹爪出厂已完成零点标定，断电重启后无需归零，直接上电即可使用。

### 使用方法

```python
from a1z.robots.get_robot import get_a1z_robot

# 创建带夹爪的机械臂
robot = get_a1z_robot(
    with_gripper=True,
    gripper_max_torque=2.5,   # 可选：限制夹持力（Nm），防止夹坏物体
)
robot.start()   # 自动使能夹爪并归零到张开位

# 控制夹爪
robot.command_gripper(0.0)   # 关闭
robot.command_gripper(1.0)   # 张开
robot.command_gripper(0.5)   # 50% 开度

# 读取夹爪状态
norm = robot.get_gripper_pos()           # 当前指令开度
obs  = robot.get_observations()          # 包含 gripper_pos 的完整观测字典

# 同时控制关节和夹爪（7 元素数组，第 7 个为夹爪）
import numpy as np
robot.command_joint_pos(np.array([0, 0.5, -0.5, 0, 0, 0, 0.8]))
robot.move_joints(np.array([0, 0.3, -0.3, 0, 0, 0, 0.0]), speed=0.5)

robot.stop()
```

### 力限制（夹持保护）

当 `gripper_max_torque > 0` 时，`GripperForceLimiter` 自动介入：

- **堵转检测**：滑动窗口均值扭矩 > 阈值 且 速度 < 0.3 rad/s，判定为堵转
- **保护行为**：退回到刚好产生 `max_torque` 的位置，停止继续施力
- **自动解除**：指令方向转向张开，或扭矩降至 0.2 Nm 以下

```python
robot = get_a1z_robot(
    with_gripper=True,
    gripper_max_torque=2.5,        # 2.5 Nm 限制
    gripper_clog_threshold=0.4,    # 堵转判定阈值（默认 0.3 Nm）
)
```

## 控制原理

### MIT 力位混控

每个控制周期，SDK 通过 `send_mit_command` 向电机下发五元组（`pos`, `vel`, `kp`, `kd`, `torque`），电机固件在内部执行 PD + 前馈合力：

```python
# motor_a_driver.py / motor_b_driver.py — send_mit_command
def send_mit_command(self, pos: float, vel: float, kp: float, kd: float, torque: float) -> None:
    pos_u16    = float_to_uint(pos,    r.pos_min,    r.pos_max,    16)
    vel_u12    = float_to_uint(vel,    r.vel_min,    r.vel_max,    12)
    kp_u12     = float_to_uint(kp,     r.kp_min,     r.kp_max,     12)
    kd_u9      = float_to_uint(kd,     r.kd_min,     r.kd_max,      9)
    torque_u12 = float_to_uint(torque, r.torque_min, r.torque_max, 12)
    # 打包成 8 字节 CAN 帧下发
```

SDK 在每个控制周期（默认 250 Hz）的 `_update` 中执行：

```python
# arm_robot.py — _update()

# 1. 读取电机反馈
self._motor_chain.drain_and_update(self._bus)

# 2. Pinocchio RNEA 计算重力补偿扭矩
tau_g = self._gravity_model.compute_gravity_torque(q)

# 3. 安全检查
if np.any(np.abs(tau_g) > self._max_gravity_torque):
    raise RuntimeError(...)

# 4. 合成最终扭矩
tau_g_scaled   = tau_g * self._gravity_torque_scale
torques_urdf   = cmd.torque_ff + tau_g_scaled * self.gravity_comp_factor
motor_torques  = np.clip(torques_urdf * self._joint_sign, -self._torque_clip, self._torque_clip)

# 5. 下发给所有电机
self._motor_chain.send_commands(
    pos=cmd.pos * self._joint_sign,
    vel=cmd.vel * self._joint_sign,
    kp=cmd.kp,
    kd=cmd.kd,
    torque=motor_torques,
)
```

### 零力漂浮模式

```python
# get_a1z_robot(zero_gravity_mode=True) 启动时初始化：
self._command.kp = np.zeros(self._num_joints)        # 无位置刚度
self._command.kd = self._default_kd.copy() * 0.5    # 小阻尼
# 仅靠 tau_g 抵消重力，机械臂可自由拖拽
```

### 位置保持模式

```python
# get_a1z_robot(zero_gravity_mode=False) 启动时初始化：
self._command.kp = self._default_kp.copy()  # [30, 30, 30, 20, 5, 5]
self._command.kd = self._default_kd.copy()  # [1,  1,  1,  0.5, 0.5, 0.5]
# PD 控制 + 重力补偿，锁定到当前位置
```

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
| 关节坐标系符号 | `[1, 1, -1, 1, 1, 1]` (关节3与URDF方向相反) |
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
