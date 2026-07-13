<a id="chinese"></a>

[中文](#chinese) | [English](#english)

# A1Z — 6-DOF 机械臂 Python SDK（带G1Z夹爪）

<p align="center">
  <img src="docs/images/A1Z_G1Z.png" alt="A1Z 机械臂（带 G1Z 夹爪）" width="500"/>
</p>

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
│   ├── teach_and_play.py          # 零力示教录制与回放
│   └── gripper_hybrid_test.py     # 夹爪测试：自由行程 + 力矩饱和验证
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

# 注意拉取带夹爪的SDK需要指定分支
git clone -b gripper https://github.com/userguide-galaxea/GALAXEA-A1Z.git

cd /path/to/GALAXEA-a1z

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
python examples/position_hold.py --q_target_deg 0,30,-20,-15,0,0 --speed 0.5

# 夹爪测试（自由行程 + 力矩饱和验证，默认 0.5 Nm）
python examples/gripper_hybrid_test.py --can can0
```

> ⚠️ **如果上述命令报错或机械臂无反应**，请跳转到 [CAN 通信故障排查](#can-通信故障排查)。

### 零力示教与回放

`teach_and_play.py` 分为两个子命令：`record`（录制）和 `play`（回放）。

#### 录制轨迹

```bash
# 录制并保存到文件（默认 can0，50 Hz 采样）
python examples/teach_and_play.py record teach.json

# 指定 CAN 口和采样频率
python examples/teach_and_play.py --can can1 record teach.json --sample-hz 100
```

启动后机械臂进入零力漂浮模式，可自由拖拽：

```
[record] Arm running in zero-gravity mode.
[record] Press ENTER to START recording...   ← 按 Enter 开始录制
[record] Recording — move the arm freely.  Press ENTER to STOP.
                                             ← 拖动到目标轨迹后按 Enter 停止
[record] Recorded 243 frames (4.86s).
[record] Saved to teach.json
```

录制完成后机械臂自动回零位再失能。Ctrl+C 可随时中止（同样会回零再失能）。

#### 回放轨迹

```bash
# 以原速回放
python examples/teach_and_play.py play teach.json

# 0.5 倍速回放
python examples/teach_and_play.py play teach.json --speed 0.5

# 循环回放直到 Ctrl+C
python examples/teach_and_play.py play teach.json --loop

# 指定 CAN 口
python examples/teach_and_play.py --can can1 play teach.json
```

启动后机械臂先运动到轨迹起点，按 Enter 开始回放：

```
[play] Loaded 243 frames (4.86s).
[play] Returning to start position...
[play] Ready.
[play] Press ENTER to PLAY (4.86s at 1.0x)...   ← 按 Enter 播放
[play] Playing (loop 1)...
[play] Playback complete.
```

回放结束（或 Ctrl+C 中止）后机械臂自动回零位再失能。

### 使用 a1zctl 服务端（可用于openclaw交互）

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

## CAN 通信故障排查

如果运行 `python examples/gravity_comp.py --gravity_factor 0.3` 报错或机械臂无反应，最常见原因是部分较老或特定版本的 Linux 内核，其内置的 socketcan / `gs_usb` 驱动与本 CAN 盒存在兼容性问题。

### 1. 检查硬件接线

- CAN 盒（USB-CANFD 适配器）已插入电脑并牢固接触
- CAN 总线两端接线正确、无松动
- 机械臂电源已上电
- CAN 盒的终端电阻正确安装（参见上文"配置 CAN 总线"）

### 2. 检查内核版本并修复（推荐直接执行）

查看当前内核版本：

```bash
uname -r
```

- **方案 A**：升级 Linux 内核到 **6.8.0-124** 或更新版本（如果您有其他工作依赖当前内核，建议按照方案 B 操作）
- **方案 B**：按官方文档给内核 / 驱动打补丁 —— 详见 [Galaxea 内核补丁指引](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb?from=from_copylink)

### 3. 手动确认是否为内核兼容问题（可选）

如果不确定问题是否出在内核，或按上一步升级 / 打补丁后仍无法通信，可以手动向 J6 电机发送 CAN 指令来复现。

打开两个终端：**终端 A** 用 `candump can0` 监听总线收发；**终端 B** 依次向 J6 电机发送使能 / 运动 / 失能指令，观察终端 A 是否有反馈帧、J6 是否实际转动。

终端 A：

```bash
candump can0
```

终端 B：

```bash
# 1. 使能 J6
cansend can0 006#FFFFFFFFFFFFFFFC

# 2. 正向低速 0.5 rad/s
cansend can0 006#8000844000199800

# 3. 反向低速 -0.5 rad/s
cansend can0 006#80007BB000199800

# 4. 失能 J6
cansend can0 006#FFFFFFFFFFFFFFFD
```

预期表现：使能命令下发后，J6 应有反馈帧回到 `candump`；正 / 反向运动命令下发后，J6 应实际低速转动。

**判断依据**：如果使能命令完全没有反馈，关闭机械臂电源后重新上电 —— 若上电瞬间可以看到大约 **2 帧 CAN 数据**返回（电机上电反馈），但之后发送使能命令仍无回帧，基本可以确认是内核 / socketcan 兼容性问题，回到第 2 步执行方案 A 或 B。

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
    gripper_max_torque=0.5,       # 夹爪最大夹持力矩 (Nm)，默认 0.5 Nm
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

# 扫描所有 7 个电机（检查通信、读取状态、自动诊断）
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

## 夹爪

夹爪出厂已完成零点标定，断电重启后无需归零，直接上电即可使用。

### 使用方法

```python
from a1z.robots.get_robot import get_a1z_robot

# 创建带夹爪的机械臂
robot = get_a1z_robot(
    with_gripper=True,
    gripper_max_torque=0.5,   # 夹持力矩上限（Nm），默认 0.5 Nm
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

### 力矩限制（夹持保护）

`gripper_max_torque` 通过力位混控模式的硬件电流饱和环节直接限制夹持力矩：夹爪接触物体后，电流被钳位在 `i_des = max_torque / 11.0`，力矩不再随位置误差增大，无论物体尺寸如何均不会超力。

```python
robot = get_a1z_robot(
    with_gripper=True,
    gripper_max_torque=1.0,   # 1.0 Nm 限制，适合轻物体
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
| 关节坐标系符号 | `[1, 1, -1, 1, -1, 1]` (关节3，5与URDF方向相反) |
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

# A1Z — 6-DOF Robotic Arm Python SDK (with G1Z Gripper)

<p align="center">
  <img src="docs/images/A1Z_G1Z.png" alt="A1Z robotic arm with G1Z gripper" width="500"/>
</p>

A Python control SDK for the A1Z six-axis robotic arm, providing CAN-bus motor drivers, Pinocchio-based gravity compensation, forward/inverse kinematics, zero-force teaching, position hold, and G1Z gripper control.

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
│   │   ├── gripper.py             # Gripper control (MotorB CAN ID 0x07)
│   │   ├── server.py              # Unix socket control server
│   │   └── kinematics.py          # FK/IK (Pinocchio)
│   ├── robot_models/
│   │   └── a1z/               # URDF model files (defaults to A1Z_G1Z.urdf)
│   └── utils/
│       └── utils.py               # RateRecorder, logging utilities
├── examples/
│   ├── gravity_comp.py            # Gravity compensation example
│   ├── position_hold.py           # Position hold example
│   ├── teach_and_play.py          # Zero-force teach recording and playback
│   └── gripper_hybrid_test.py     # Gripper test: free travel + torque saturation
└── tools/
    ├── a1zctl                     # Arm control CLI (serve/move/gripper/dance/stop)
    ├── gripper_set_zero.py        # Gripper zero calibration (factory-done, rarely needed)
    ├── motor_diag.py              # Motor communication diagnostics
    └── set_zero.py                # Motor zero calibration
```

## Installation

### Prerequisites

- Python >= 3.10
- Linux + SocketCAN (hardware CAN interface required)
- URDF model files (bundled, see `a1z/robot_models/a1z/`, defaults to `A1Z_G1Z.urdf` which includes the gripper)

### Install the SDK

```bash
# Gripper branch is required for G1Z gripper support
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
# Zero-force float (default URDF A1Z_G1Z.urdf, includes gripper)

# Start with a small compensation factor (recommended for first-time setup)
python examples/gravity_comp.py --gravity_factor 0.3

# Increase to full compensation once direction is confirmed correct
python examples/gravity_comp.py --gravity_factor 1.0

# Position hold mode
python examples/gravity_comp.py --mode hold

# Position hold + move to target
python examples/position_hold.py --q_target_deg 0,30,-20,-15,0,0 --speed 0.5

# Gripper test (free travel + torque saturation, default 0.5 Nm)
python examples/gripper_hybrid_test.py --can can0
```

> ⚠️ **If the commands above error out or the arm does not respond**, jump to [CAN Communication Troubleshooting](#can-communication-troubleshooting).

### Zero-Force Teaching and Playback

`teach_and_play.py` has two sub-commands: `record` and `play`.

#### Record a Trajectory

```bash
# Record and save to file (default can0, 50 Hz sampling)
python examples/teach_and_play.py record teach.json

# Specify CAN channel and sample rate
python examples/teach_and_play.py --can can1 record teach.json --sample-hz 100
```

The arm enters zero-force float mode and can be freely backdriven:

```
[record] Arm running in zero-gravity mode.
[record] Press ENTER to START recording...   ← press Enter to begin
[record] Recording — move the arm freely.  Press ENTER to STOP.
                                             ← guide the arm, then press Enter
[record] Recorded 243 frames (4.86s).
[record] Saved to teach.json
```

The arm returns to zero position and disables after recording. Ctrl+C aborts safely (also returns to zero).

#### Play Back a Trajectory

```bash
# Play at original speed
python examples/teach_and_play.py play teach.json

# Play at 0.5× speed
python examples/teach_and_play.py play teach.json --speed 0.5

# Loop until Ctrl+C
python examples/teach_and_play.py play teach.json --loop

# Specify CAN channel
python examples/teach_and_play.py --can can1 play teach.json
```

The arm moves to the trajectory start position first, then waits for Enter:

```
[play] Loaded 243 frames (4.86s).
[play] Returning to start position...
[play] Ready.
[play] Press ENTER to PLAY (4.86s at 1.0x)...   ← press Enter to play
[play] Playing (loop 1)...
[play] Playback complete.
```

After playback (or Ctrl+C), the arm returns to zero and disables.

### Using the a1zctl Server

```bash
# Terminal 1: start the server (with gripper)
python3 tools/a1zctl serve --with-gripper

# Terminal 2: send control commands
python3 tools/a1zctl status              # show joint state (including gripper)
python3 tools/a1zctl move --preset home  # move to preset position
python3 tools/a1zctl move 0,60,-60,0,0,0 --speed 0.5
python3 tools/a1zctl gripper 0.5         # set gripper to 50% open
python3 tools/a1zctl dance --moves salute,wave,nod
python3 tools/a1zctl info                # list all presets and limits
python3 tools/a1zctl stop               # stop the server
```

## CAN Communication Troubleshooting

If `python examples/gravity_comp.py --gravity_factor 0.3` errors out or the arm does not respond, the most common cause is a compatibility issue between the built-in SocketCAN / `gs_usb` driver in some older or specific Linux kernel versions and this CAN adapter.

### 1. Check the wiring

- The USB-CANFD adapter is plugged in and seated firmly
- Both ends of the CAN bus are wired correctly and not loose
- The arm is powered on
- The CAN termination resistor is installed correctly (see "Configure the CAN Bus" above)

### 2. Check the kernel version and fix (recommended path)

Check your current kernel version:

```bash
uname -r
```

- **Option A**: upgrade the Linux kernel to **6.8.0-124** or newer (if you have other work that depends on the current kernel, prefer Option B)
- **Option B**: patch the kernel / driver as described in the [Galaxea kernel patch guide](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb?from=from_copylink)

### 3. Manually confirm the kernel compatibility issue (optional)

If you are not sure the kernel is the culprit, or CAN still does not work after step 2, manually reproduce the issue by sending CAN commands to the J6 motor.

Open two terminals. **Terminal A** monitors bus traffic with `candump can0`; **Terminal B** sends the enable / motion / disable commands to J6 in sequence. Watch Terminal A for feedback frames and check whether J6 actually rotates.

Terminal A:

```bash
candump can0
```

Terminal B:

```bash
# 1. Enable J6
cansend can0 006#FFFFFFFFFFFFFFFC

# 2. Forward at 0.5 rad/s
cansend can0 006#8000844000199800

# 3. Reverse at -0.5 rad/s
cansend can0 006#80007BB000199800

# 4. Disable J6
cansend can0 006#FFFFFFFFFFFFFFFD
```

Expected behavior: after the enable command, J6 should emit a feedback frame visible in `candump`; after forward / reverse, J6 should physically rotate at low speed.

**Diagnosis**: if the enable command produces no feedback at all, power-cycle the arm — if you see roughly **2 CAN frames** returned during power-on (the motor's boot-up feedback) but the enable command still produces nothing, this confirms a kernel / SocketCAN compatibility issue. Go back to step 2 and apply Option A or B.

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
    with_gripper=False,           # True=enable gripper (CAN ID 0x07)
    gripper_max_torque=0.5,       # Max gripping torque (Nm), default 0.5 Nm
) -> ArmRobot
```

### `ArmRobot` Key Methods

| Method | Description |
|--------|-------------|
| `start(initial_kp, initial_kd)` | Enable motors and start the control loop (gripper homing included) |
| `stop()` | Smooth shutdown (0.8 s decay), disable motors |
| `get_joint_pos() -> np.ndarray` | Get current joint angles (rad); returns 7-element array when gripper is attached (7th = normalized gripper position) |
| `get_joint_state() -> dict` | Get `{pos, vel, eff}` (arm joints only) |
| `get_observations() -> dict` | Get `{joint_pos, gripper_pos, joint_vel, joint_eff}`; equivalent to `get_joint_state` without gripper |
| `command_joint_pos(pos)` | Set target joint angles; accepts 7-element array (7th = gripper [0, 1]) |
| `command_joint_state(joint_state)` | Set target joint angles + custom gains |
| `command_gripper(value)` | Set gripper target position [0.0=closed, 1.0=fully open] |
| `get_gripper_pos() -> float\|None` | Get current gripper command position; returns None without gripper |
| `move_joints(target, speed, kp, kd)` | Interpolate to target position (blocking); accepts 7-element array |
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

# Scan all 7 motors (check communication, read state, auto-diagnose)
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

## Gripper

The gripper ships with factory zero calibration and requires no re-homing after power cycle.

### Usage

```python
from a1z.robots.get_robot import get_a1z_robot

robot = get_a1z_robot(
    with_gripper=True,
    gripper_max_torque=0.5,   # max gripping torque (Nm), default 0.5 Nm
)
robot.start()   # enables gripper and homes to open position

# Control the gripper
robot.command_gripper(0.0)   # close
robot.command_gripper(1.0)   # fully open
robot.command_gripper(0.5)   # 50% open

# Read gripper state
norm = robot.get_gripper_pos()           # current command position
obs  = robot.get_observations()          # full observation dict including gripper_pos

# Control arm and gripper together (7-element array, 7th = gripper)
import numpy as np
robot.command_joint_pos(np.array([0, 0.5, -0.5, 0, 0, 0, 0.8]))
robot.move_joints(np.array([0, 0.3, -0.3, 0, 0, 0, 0.0]), speed=0.5)

robot.stop()
```

### Torque Limiting (Grasp Protection)

`gripper_max_torque` limits gripping force directly via the hardware current saturation in MIT mixed control mode. Once the gripper contacts an object, current is clamped to `i_des = max_torque / 11.0` and torque no longer grows with position error, regardless of object size.

```python
robot = get_a1z_robot(
    with_gripper=True,
    gripper_max_torque=1.0,   # 1.0 Nm, suitable for lighter objects
)
```

## Control Architecture

### MIT Position-Velocity-Torque Mixed Control

Each control cycle, the SDK sends a five-tuple (`pos`, `vel`, `kp`, `kd`, `torque`) to each motor via `send_mit_command`. The motor firmware executes PD + feedforward internally:

```python
# motor_a_driver.py / motor_b_driver.py — send_mit_command
def send_mit_command(self, pos, vel, kp, kd, torque):
    pos_u16    = float_to_uint(pos,    r.pos_min,    r.pos_max,    16)
    vel_u12    = float_to_uint(vel,    r.vel_min,    r.vel_max,    12)
    kp_u12     = float_to_uint(kp,     r.kp_min,     r.kp_max,     12)
    kd_u9      = float_to_uint(kd,     r.kd_min,     r.kd_max,      9)
    torque_u12 = float_to_uint(torque, r.torque_min, r.torque_max, 12)
    # pack into 8-byte CAN frame and send
```

The SDK's `_update()` runs at each control cycle (default 250 Hz):

1. Read motor feedback from the CAN bus
2. Compute gravity compensation torque `τ_g(q)` via Pinocchio RNEA
3. Safety check: emergency stop if any `|τ_g|` exceeds the per-joint threshold
4. Compose final torque: `τ_motor = (user_torque + τ_g × scale × factor) × joint_sign`
5. Clip to safe range and send to all motors

### Zero-Force Float Mode

```python
# get_a1z_robot(zero_gravity_mode=True)
self._command.kp = np.zeros(self._num_joints)        # no position stiffness
self._command.kd = self._default_kd.copy() * 0.5    # low damping
# only τ_g counteracts gravity; the arm can be freely backdriven
```

### Position Hold Mode

```python
# get_a1z_robot(zero_gravity_mode=False)
self._command.kp = self._default_kp.copy()  # [30, 30, 30, 20, 5, 5]
self._command.kd = self._default_kd.copy()  # [1,  1,  1,  0.5, 0.5, 0.5]
# PD control + gravity compensation, locks to current position
```

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
