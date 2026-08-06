# GALAXEA-A1Z SDK 接入 LEMO 主板（RK3588）说明

本文档说明如何在 LEMO 主板（RK3588，运行 Linux + spidev）上使用本 SDK 控制 A1Z 机械臂：
指令与反馈不再走 USB-CAN 适配器（SocketCAN），而是经板上的 G4 MCU 做 SPI↔CAN 透传桥。

相关代码：`lemo_main_board` 仓库 `jsc` 分支（Linux 侧 ROS2 节点 + G4 帧协议实现，
见 `src/g4spi.cpp` / `src/main.cpp`）；本仓库 `lemo` 分支。

## 架构对比

原方案（`transport="socketcan"`，默认）：

```
SDK (Python) → python-can SocketCAN (can0, 1 Mbps) → A1Z 电机 (CAN ID 1~7)
```

LEMO 主板方案（`transport="g4spi"`）：

```
SDK (Python, 跑在 RK3588 上)
  → G4SpiBus (a1z/motor_drivers/spi_bus.py, spidev)
  → G4 帧协议 over SPI (RK3588 主机, G4 从机)
  → G4 固件透传到 CAN 总线
  → A1Z 电机 (CAN ID 1~7)
```

`G4SpiBus` 实现了与 python-can `BusABC` 相同的 `send(msg)` / `recv(timeout)` 接口，
因此 MotorA / MotorB / MixedMotorChain / Gripper / ArmRobot 的电机协议与控制代码
**零改动**，只有总线创建点（`get_a1z_robot()`）按 `transport` 参数分流。

## G4 帧协议（与 lemo_main_board src/g4spi.cpp 保持一致）

```
Header[2]  Length[2]  CMD_ID[1]  Param[N]  Index[2]  CRC16[2]
FF FD      小端序     命令字     业务数据  小端序    小端序
```

- `Length = CMD_ID(1) + Param(N) + Index(2) + CRC(2) = N + 5`，整帧不超过 1024 字节
- CRC 为 CRC-16/CCITT-FALSE，范围是除最后 2 个 CRC 字节外的全部字节
- `Index` 为发送计数，每发一帧 +1，可用于丢帧统计
- SPI 全双工：发送的同时 MISO 收到的字节也进解析器；无数据要发时用 0x00 dummy
  字节为 G4 提供时钟（`G4SpiBus` 由后台 2 kHz 轮询线程负责，对应 ROS 节点的 receive loop）

A1Z 数据通道（CMD `0x11` = 左臂 / `0x12` = 右臂，上下行同号）：

- Param 固定 56 字节 = 7 × 8：槽位 0~5 为电机 1~6 的 8 字节 CAN 裸数据，
  槽位 6（偏移 48）为夹爪的 8 字节 CAN 裸数据（命令发往 CAN ID `0x300+7`，
  反馈来自 CAN ID 7）
- 透传的是 **CAN 裸数据**，与 SDK 的 MIT/混合模式帧格式一一对应，协议解析仍在 SDK 侧

## 使用方法

```python
from a1z.robots.get_robot import get_a1z_robot

robot = get_a1z_robot(
    transport="g4spi",            # 默认 "socketcan"
    arm_side="left",              # "left" -> CMD 0x11；"right" -> CMD 0x12
    spi_device="/dev/spidev0.0",  # 按实际接线改
    spi_speed_hz=10_000_000,      # 与嵌入式 ROS 节点默认一致（10 MHz）
    with_gripper=True,
    zero_gravity_mode=False,
    control_freq_hz=250,
)
robot.start()
# ... 与 SocketCAN 方案完全相同的 API ...
robot.stop()
```

依赖：板上需安装 `spidev`（`pip install spidev`，或 `pip install a1z[g4spi]`）。
`spidev` 为惰性导入，开发机上不装也能正常 import SDK。

新增参数（仅 `transport="g4spi"` 时生效）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `transport` | `"socketcan"` | `"g4spi"` 走 LEMO 主板 G4 SPI 桥 |
| `spi_device` | `/dev/spidev0.0` | spidev 节点 |
| `spi_speed_hz` | 10 MHz | SPI 时钟 |
| `arm_side` | `"left"` | 控制左臂（0x11）还是右臂（0x12） |

## SDK 侧改动清单（lemo 分支）

- 新增 `a1z/motor_drivers/spi_bus.py`：
  - `crc16_ccitt_false()` / `build_frame()` / `G4FrameParser`：G4 帧协议纯函数与
    字节流解析器（支持帧跨 SPI 传输拆分、噪声/CRC 错误重同步）
  - `G4SpiBus`：`send()` 把 CAN 数据写入 56 字节命令镜像对应槽位，6 个电机槽凑齐
    即作为一个 G4 帧发出（对齐 SDK 每 tick 发 6+1 帧的节奏，夹爪槽携带最近一次值）；
    `recv()` 把上行帧解成 7 个 `can.Message`（ID 1~6 + 夹爪 7，`is_rx=True`，
    Linux 侧打时间戳），兼容 `MixedMotorChain` 的回显过滤与反馈路由；
    后台 2 kHz 轮询线程持续为 G4 提供 SPI 时钟
- 修改 `a1z/robots/get_robot.py`：总线创建点按 `transport` 分流
- 新增 `tests/test_spi_bus.py`：11 个单元测试（CRC 标准向量、帧布局、解析重同步、
  槽位映射、左右臂 CMD、管理帧丢弃、recv 解码），无硬件依赖
- `pyproject.toml`：新增 `g4spi` optional extra（`spidev`）

## 注意事项与限制

1. **0x7FF 管理帧无法透传（关键）**：MotorA 的 enable/disable/设零广播帧（`0x7FF`）
   和夹爪 mode-4 寄存器写（`0x7FF` + `0x55`）不在 56 字节透传通道内，`G4SpiBus`
   会丢弃并打 warning。因此 **G4 固件必须自行完成电机上电 enable 和夹爪 mode 4
   （力位混合）设置**，否则 `start()` 会卡在启动反馈探测、夹爪 hybrid 指令被电机忽略。
   同理，`tools/set_zero.py` 等设零工具经此通道不可用。
2. **MotorB 特殊帧可以透传**：`0xFC/0xFD/0xFB/0xFE`（enable/disable/清错/设零）是
   电机自身 ID（4~6）上的普通 8 字节数据，会随槽位原样转发；`stop()` 的 disable
   因此仍然生效。需向嵌入式同事确认固件是"原样转发 7 个槽"，避免意外行为。
3. **CAN 帧节拍下移**：SocketCAN 方案中 SDK 每 tick 发 7 帧、帧间插 250 µs 间隔
   （SOP-05/06，防止 J6 应答槽被占）。走 G4 桥后每 tick 只有一个 56 字节 SPI 帧，
   CAN 侧的逐帧 pacing 由固件负责，需确认固件实现了等价间隔。
4. **反馈新鲜度**：反馈时间戳仍在 Linux 侧按到达时间打，与 SocketCAN 方案一致，
   `stale_feedback_estop_s=0.2` 的急停语义不变。
5. **主从（leader）数据**：leader 臂舵机数据、按键、连接状态走 G4 的其它 CMD
   （0x01/0x02/0x03），由嵌入式 ROS 节点处理，与本 SDK 传输层无关。

## 联调步骤建议

1. 与嵌入式同事确认上文第 1、2、3 条的固件行为
2. 板上确认 spidev 节点与速率：`ls /dev/spidev*`
3. 先跑 `examples/position_hold.py`（加 `transport="g4spi"`）验证 6 关节反馈与位置保持
4. 再验证夹爪（`with_gripper=True`）与重力补偿模式

## 测试

```bash
python -m pytest tests/test_spi_bus.py -q
```
