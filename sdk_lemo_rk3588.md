# GALAXEA-A1Z SDK 接入 LEMO 主板（RK3588）说明

本文档说明如何在 LEMO 主板（RK3588，运行 Linux + ROS2）上使用本 SDK 控制 A1Z 机械臂：
指令与反馈不再走 USB-CAN 适配器（SocketCAN），而是经板上的 G4 MCU 做 SPI↔CAN 透传桥。

相关代码：`lemo_main_board` 仓库 `jsc` 分支（Linux 侧 ROS2 节点 + G4 帧协议实现，
见 `src/g4spi.cpp` / `src/main.cpp`）；本仓库 `lemo` 分支。

## 两种 transport 对比

原方案（`transport="socketcan"`，默认）：

```
SDK (Python) → python-can SocketCAN (can0, 1 Mbps) → A1Z 电机 (CAN ID 1~7)
```

**lemo 方案：`transport="g4ros"`**——SDK 走嵌入式 ROS2 节点的 topic，
**逐帧收发**（MCU 固件一帧一帧透传，每条消息就是一帧 CAN）：

```
SDK (Python, 跑在 RK3588 上)
  → RosTopicBus (a1z/motor_drivers/ros_topic_bus.py, rclpy)
  → ROS2 topics: <side>_motor_send (下行, 一帧一条消息)
                 <side>_motor_data (上行, 一帧一条消息)
  → g4spi_node (lemo_main_board jsc 分支, 需保持运行)
  → G4 帧协议 over SPI → G4 固件透传到 CAN → A1Z 电机 (CAN ID 1~7)
```

节点独占 SPI 轮询（同时发布 leader 臂舵机/按键/连接状态 topic），SDK 只接触
ROS topic，不直接驱动 spidev。

两种 transport 都实现 python-can `BusABC` 兼容的 `send(msg)` / `recv(timeout)` 接口，
MotorA / MotorB / MixedMotorChain / Gripper / ArmRobot 的电机协议与控制代码
**零改动**，只有总线创建点（`get_a1z_robot()`）按 `transport` 参数分流。

## Topic 对应关系（g4ros）

`<side>` 为 `left` 或 `right`，消息包为 `lemo_main_board/msg`。
上下行共用同一个单帧消息格式（一帧 CAN 一条消息）：

```
std_msgs/Header header
uint16 id
uint8[8] data
```

| 方向 | Topic | 消息 | 内容 |
|---|---|---|---|
| SDK → 节点 | `<side>_motor_send` | `CanFrame` | 一条 CAN 命令帧（id = 电机 1~7 / 0x307 / 0x7FF） |
| 节点 → SDK | `<side>_motor_data` | `CanFrame` | 一条 CAN 反馈帧（id 同上） |

所有 CAN ID 走同一对 topic：手臂电机 1~6、夹爪 7（hybrid 指令为 `0x300+7`）、
以及 0x7FF 管理广播帧（MotorA enable/disable/设零、夹爪 mode-4 寄存器写）。
不足 8 字节的帧（如 4 字节的 MotorA enable）发送侧补零到固定 `uint8[8]`。

## 使用方法（g4ros）

前置条件（在 RK3588 上）：

```bash
# 1. colcon 编译 lemo_main_board 仓库 jsc 分支（含 msg 包），source 工作空间
# 2. 启动嵌入式节点（独占 SPI 轮询，同时发布 leader 臂数据）
ros2 run lemo_main_board g4spi_node   # 节点名以实际 CMake 目标为准
```

```python
from a1z.robots.get_robot import get_a1z_robot

robot = get_a1z_robot(
    transport="g4ros",    # 走 ROS2 topic 桥接
    arm_side="left",      # "left" -> left_* topics；"right" -> right_*
    with_gripper=True,
    zero_gravity_mode=False,
    control_freq_hz=250,
)
robot.start()
# ... 与 SocketCAN 方案完全相同的 API ...
robot.stop()
```

transport 参数一览：

| 值 | 说明 | 依赖 |
|---|---|---|
| `"socketcan"`（默认） | 直连 CAN | python-can + USB-CAN 适配器 |
| `"g4ros"` | 经 g4spi_node 的 ROS2 topic | rclpy + lemo_main_board msg 包 + 节点运行中 |

## SDK 侧改动清单（lemo 分支）

- 新增 `a1z/motor_drivers/ros_topic_bus.py`：`RosTopicBus`，**逐帧收发**——
  `send()` 把每条 CAN 帧立即发布为一条 `CanFrame` 消息（`<side>_motor_send`，
  不足 8 字节补零）；后台 executor 线程 spin rclpy 节点，订阅
  `<side>_motor_data` 逐帧解成 `can.Message`（`is_rx=True`，Linux 侧打时间戳）
  供 `recv()` 取出
- 修改 `a1z/robots/get_robot.py`：总线创建点按 `transport` 分流
- 新增 `tests/test_ros_topic_bus.py`：补零/解码助手与 import 守卫的单元测试，
  无硬件/ROS 依赖

## 注意事项与限制

1. **0x7FF 管理帧**：g4ros 逐帧按 ID 透传，MotorA 的 enable/disable/设零广播帧
   和夹爪 mode-4 寄存器写都会原样下发给 MCU——**需与嵌入式确认固件会把 0x7FF
   帧真实转发到 CAN**，否则 `start()` 会卡在启动反馈探测、夹爪 hybrid 指令被忽略。
2. **MotorB 特殊帧可以透传**：`0xFC/0xFD/0xFB/0xFE`（enable/disable/清错/设零）是
   电机自身 ID（4~6）上的普通 8 字节数据，会原样转发；`stop()` 的 disable
   因此仍然生效。
3. **CAN 帧节奏**：SocketCAN 方案中 SDK 每 tick 发 7 帧、帧间插 250 µs 间隔
   （SOP-05/06，防止 J6 应答槽被占）。g4ros 下逐帧经 topic 转发，帧间隔由
   SDK 发送节奏和节点转发延迟共同决定，需实测确认 J6 反馈无饿死。
4. **节点必须在线**：`transport="g4ros"` 要求 g4spi_node 正在运行，否则命令无人
   下发、反馈无人发布。
5. **反馈新鲜度**：反馈时间戳仍在 Linux 侧按到达时间打，与 SocketCAN 方案一致，
   `stale_feedback_estop_s=0.2` 的急停语义不变。
6. **主从（leader）数据**：leader 臂舵机数据、按键、连接状态走 G4 的其它 CMD
   （0x01/0x02/0x03），由 g4spi_node 发布为 ROS topic，不受影响。

## 联调步骤建议

1. 与嵌入式同事确认上文第 1 条的固件行为（0x7FF 转发）
2. 板上 colcon 编译 lemo_main_board（jsc 分支），source 后启动 `g4spi_node`，
   `ros2 topic echo /left_motor_data` 应能看到反馈帧持续刷新
3. 跑 `examples/position_hold.py`（加 `transport="g4ros"`）验证 6 关节反馈与位置保持
4. 再验证夹爪（`with_gripper=True`）与重力补偿模式

## 测试

```bash
python -m pytest tests/test_ros_topic_bus.py -q
```
