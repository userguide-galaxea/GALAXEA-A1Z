# GALAXEA-A1Z SDK 接入 LEMO 主板（RK3588）说明

本文档说明如何在 LEMO 主板（RK3588，运行 Linux + ROS2）上使用本 SDK 控制 A1Z 机械臂：
指令与反馈不再走 USB-CAN 适配器（SocketCAN），而是经板上的 G4 MCU 做 SPI↔CAN 透传桥。

相关代码：`lemo_main_board` 仓库 `jsc` 分支（Linux 侧 ROS2 节点 + G4 帧协议实现，
见 `src/g4spi.cpp` / `src/main.cpp`）；本仓库 `lemo` 分支。

## 三种 transport 对比

原方案（`transport="socketcan"`，默认）：

```
SDK (Python) → python-can SocketCAN (can0, 1 Mbps) → A1Z 电机 (CAN ID 1~7)
```

**方案 B：`transport="g4ros"`（当前采用）**——SDK 走嵌入式 ROS2 节点的 topic：

```
SDK (Python, 跑在 RK3588 上)
  → RosTopicBus (a1z/motor_drivers/ros_topic_bus.py, rclpy)
  → ROS2 topics: <side>_a1z_send / <side>_claw_send (下行)
                 <side>_a1z_data / <side>_claw     (上行)
  → g4spi_node (lemo_main_board jsc 分支, 需保持运行)
  → G4 帧协议 over SPI → G4 固件透传到 CAN → A1Z 电机 (CAN ID 1~7)
```

方案 A：`transport="g4spi"`（保留为备选）——SDK 自己驱动 spidev 直连 G4，不依赖 ROS。
**注意：与 g4spi_node 互斥**，两个进程不能同时轮询同一个 spidev（帧流会互相拆烂）。
需要 leader 臂数据（舵机/按键/连接状态 topic 由节点独占发布）时用方案 B。

三种 transport 都实现 python-can `BusABC` 兼容的 `send(msg)` / `recv(timeout)` 接口，
MotorA / MotorB / MixedMotorChain / Gripper / ArmRobot 的电机协议与控制代码
**零改动**，只有总线创建点（`get_a1z_robot()`）按 `transport` 参数分流。

## Topic 对应关系（方案 B）

`<side>` 为 `left`（G4 CMD 0x11）或 `right`（0x12），消息包为 `lemo_main_board/msg`：

| 方向 | Topic | 消息 | 内容 |
|---|---|---|---|
| SDK → 节点 | `<side>_a1z_send` | `A1zFrame`（6×`MotorCanFrame`） | 电机 1~6 的 8 字节 CAN 裸命令 |
| SDK → 节点 | `<side>_claw_send` | `Claw` | 夹爪 8 字节（发往 CAN ID `0x300+7`） |
| 节点 → SDK | `<side>_a1z_data` | `A1zFrame` | 电机 1~6 的 8 字节 CAN 裸反馈 |
| 节点 → SDK | `<side>_claw` | `Claw` | 夹爪反馈（来自 CAN ID 7） |

**重要**：节点的 `send_pending` 只有在 A1zFrame 和 Claw **都**有待发时才会组帧发给
MCU（`main.cpp`）。因此 `RosTopicBus` 每次 flush 都会同时发布 A1zFrame 和最近一次的
Claw 值；在收到第一条夹爪命令之前（建议 `with_gripper=True`），命令不会真正下发，
总线会打 warning 提示。

## 使用方法（方案 B）

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
    with_gripper=True,    # 见上文：没有夹爪命令节点不会下发
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
| `"g4spi"` | 直连 spidev（与节点互斥） | spidev（`pip install a1z[g4spi]`） |

`g4spi` 专用参数：`spi_device`（默认 `/dev/spidev0.0`）、`spi_speed_hz`（默认 10 MHz）。

## G4 帧协议背景（方案 A 直连时用，与 src/g4spi.cpp 一致）

```
Header[2]  Length[2]  CMD_ID[1]  Param[N]  Index[2]  CRC16[2]
FF FD      小端序     命令字     业务数据  小端序    小端序
```

- `Length = CMD_ID(1) + Param(N) + Index(2) + CRC(2) = N + 5`，整帧不超过 1024 字节
- CRC 为 CRC-16/CCITT-FALSE，范围是除最后 2 个 CRC 字节外的全部字节
- A1Z 数据通道 CMD `0x11` = 左臂 / `0x12` = 右臂（上下行同号），Param 固定
  56 字节 = 7 × 8：槽位 0~5 为电机 1~6 的 8 字节 CAN 裸数据，槽位 6（偏移 48）为夹爪
- 方案 B 下组帧/解析由 g4spi_node 完成，SDK 只接触 8 字节裸 CAN 数据

## SDK 侧改动清单（lemo 分支）

- 新增 `a1z/motor_drivers/command_image.py`：两个 lemo transport 共用的 56 字节
  命令镜像（CAN ID → 槽位映射、每 tick 凑齐 6 电机槽触发 flush、0x7FF 管理帧
  丢弃告警），防止两份槽位逻辑漂移
- 新增 `a1z/motor_drivers/ros_topic_bus.py`（**方案 B**）：`RosTopicBus`，
  `send()` 缓冲进命令镜像、凑齐即发布 `A1zFrame`+`Claw`；后台 executor 线程 spin
  rclpy 节点，订阅回调把 `<side>_a1z_data`/`<side>_claw` 解成 `can.Message`
  （ID 1~6 + 夹爪 7，`is_rx=True`，Linux 侧打时间戳）供 `recv()` 取出
- 新增 `a1z/motor_drivers/spi_bus.py`（方案 A，备选）：`G4SpiBus`，G4 帧协议
  纯 Python 移植 + 后台 2 kHz 轮询线程
- 修改 `a1z/robots/get_robot.py`：总线创建点按 `transport` 分流
- 新增 `tests/test_spi_bus.py` / `tests/test_ros_topic_bus.py`：帧协议、槽位映射、
  解码助手等单元测试，均无硬件/ROS 依赖
- `pyproject.toml`：新增 `g4spi` optional extra（`spidev`；rclpy 由 ROS2 提供，不入依赖）

## 注意事项与限制

1. **0x7FF 管理帧无法透传（关键）**：MotorA 的 enable/disable/设零广播帧（`0x7FF`）
   和夹爪 mode-4 寄存器写（`0x7FF` + `0x55`）不在透传通道内，两种 lemo transport
   都会丢弃并打 warning。因此 **G4 固件必须自行完成电机上电 enable 和夹爪 mode 4
   （力位混合）设置**，否则 `start()` 会卡在启动反馈探测、夹爪 hybrid 指令被电机忽略。
   同理，`tools/set_zero.py` 等设零工具经此通道不可用。
2. **MotorB 特殊帧可以透传**：`0xFC/0xFD/0xFB/0xFE`（enable/disable/清错/设零）是
   电机自身 ID（4~6）上的普通 8 字节数据，会随槽位原样转发；`stop()` 的 disable
   因此仍然生效。需向嵌入式同事确认固件是"原样转发 7 个槽"，避免意外行为。
3. **CAN 帧节拍下移**：SocketCAN 方案中 SDK 每 tick 发 7 帧、帧间插 250 µs 间隔
   （SOP-05/06，防止 J6 应答槽被占）。走 G4 桥后 CAN 侧的逐帧 pacing 由固件负责，
   需确认固件实现了等价间隔。
4. **g4spi 与 g4spi_node 互斥**：`transport="g4spi"` 运行时不能同时跑 ROS 节点。
   `transport="g4ros"` 则要求节点必须在线，否则命令无人下发、反馈无人发布。
5. **反馈新鲜度**：反馈时间戳仍在 Linux 侧按到达时间打，与 SocketCAN 方案一致，
   `stale_feedback_estop_s=0.2` 的急停语义不变。
6. **主从（leader）数据**：leader 臂舵机数据、按键、连接状态走 G4 的其它 CMD
   （0x01/0x02/0x03），由 g4spi_node 发布为 ROS topic；方案 B 下节点照常运行，
   这些数据不受影响。

## 联调步骤建议

1. 与嵌入式同事确认上文第 1、2、3 条的固件行为
2. 板上 colcon 编译 lemo_main_board（jsc 分支），source 后启动 `g4spi_node`，
   `ros2 topic echo /left_a1z_data` 应能看到反馈持续刷新
3. 跑 `examples/position_hold.py`（加 `transport="g4ros"`）验证 6 关节反馈与位置保持
4. 再验证夹爪（`with_gripper=True`）与重力补偿模式

## 测试

```bash
python -m pytest tests/test_spi_bus.py tests/test_ros_topic_bus.py -q
```
