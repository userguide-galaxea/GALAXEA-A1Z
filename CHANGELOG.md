# Changelog

## [Unreleased] — dev + G1Z 跨平台验证续写（2026-09-02）

> 本节记录 `codex/cross-platform-g1z-fixes` 相对 `dev` 的新增修复与实机结果。
> 下方“历史基线”原样保留 `feature/can-backend-safety` 当时的结论；其中
> Windows 未验证、Zadig 必需等描述仅代表当时状态，当前使用方式以本节和
> `README.md` 为准。

### 当前结论

| 平台 | 末端配置 | CAN 状态 | 重力补偿控制频率 | 结论 |
|------|----------|----------|------------------|------|
| macOS | 无夹爪 | HHS `a8fa:8598` 实机通过 | 250 Hz | 已验证 |
| macOS | G1Z 夹爪 | HHS `a8fa:8598` 实机通过 | 250 Hz | 已验证 |
| Windows / Python 3.11 | G1Z 夹爪 | HHS 实机通信与控制通过 | 约 168–176 Hz | 可运行，未达到 250 Hz |
| Windows / Python 3.10 | G1Z 夹爪 | HHS 实机通信与控制通过 | 约 65–70 Hz | 不建议用于当前控制配置 |

- macOS 已确认有无 G1Z 夹爪都能保持 250 Hz，因此增加夹爪通信帧并不是
  Windows 约 172 Hz 的通用根因。
- Windows 已完成适配器打开、1 Mbps 配置、六关节反馈及重力补偿实测，但
  当前不得宣称 250 Hz 已通过验证。
- 测试机现有驱动可直接被 libusb 访问，因此未使用 Zadig；其他 Windows
  机器仅在设备无法被 libusb 打开时才需要按 `README.md` 排查驱动。

### 本轮主要修改

#### 控制循环与诊断

- 将 `dev` 中缺失的绝对期限调度重新整合到控制循环，使用
  `time.monotonic()` 计算下一周期，避免相对 `sleep()` 持续累积漂移。
- `examples/gravity_comp.py` 新增 `--profile-loop`，并保留
  `A1Z_PROFILE_LOOP=1` 环境变量入口。
- profiler 改为在 `_update()` 内独立统计和每 2 秒输出，使不同控制路径都能
  产生一致的阶段耗时日志；启动时会明确提示 profiling 已启用。
- profiling 模式下状态信息使用普通日志输出，避免 `\r` 刷新覆盖频率和耗时
  诊断结果。
- 支持 `A1Z_TRACE_CAN=1` 输出 CAN 收发诊断，便于继续定位 Windows USB
  传输瓶颈。

#### macOS libusb 运行时

- 优先使用 `libusb-package` 提供的动态库，并通过 `ctypes.CDLL()` 与
  `usb1.loadLibrary()` 显式加载，避免系统未安装独立 libusb 时检测不到设备。
- PyUSB 的设备枚举改用同一个 bundled libusb backend，保证设备识别、HHS
  选择和实际打开使用一致的运行时。
- `tools/verify_can_mac.sh` 同步使用 bundled backend；已在 HHS
  `a8fa:8598` 上通过枚举、打开、1 Mbps 设置和测试帧发送。

#### Windows、安装与开源文档

- Windows 不再通过 pip 安装 `pin`，改为在 Conda 环境安装
  `conda-forge::pinocchio`；项目依赖使用平台 marker 避免包冲突。
- Windows 推荐 Python 3.11；`.[gs_usb]` 已安装 `libusb-package`、`libusb1`
  等运行时，不再要求额外执行 Conda `libusb` 安装步骤。
- Zadig 改为条件排障步骤，不再描述为所有 Windows 用户的必需安装项。
- README 补充 macOS、Windows、G1Z、USB-CAN 验证、控制频率现状和安全说明，
  GitHub issue 链接改为 GitLab。
- README 与 `tools/a1zctl` 的命令统一使用当前环境的 `python` 和
  `python -m pip`，避免 Conda 已激活但 `python3`/裸 `pip` 指向其他解释器。
- 克隆说明指向当前公开开发分支：

```bash
git clone --branch codex/cross-platform-g1z-fixes --single-branch \
  https://gitlab.galaxea-ai.com/menglian.li/a1z.git GALAXEA-A1Z
```

#### 安全与工具

- `tools/motor_diag.py` 的主动电机命令增加人工确认，降低误操作风险。
- `tools/verify_can_win.py` 明确测试范围和结果含义。
- Pinocchio 导入失败提示按平台给出正确安装方法。

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `README.md` | 开源安装、GitLab 克隆、跨平台实测结果、G1Z 与故障排查 |
| `a1z/motor_drivers/can_backend.py` | macOS bundled libusb 加载及 HHS/PyUSB backend 一致化 |
| `a1z/robots/arm_robot.py` | 绝对期限调度、控制循环 profiler 与诊断输出 |
| `a1z/dynamics/gravity_model.py` | Windows Pinocchio 安装提示 |
| `a1z/robots/kinematics.py` | Windows Pinocchio 安装提示 |
| `examples/gravity_comp.py` | `--profile-loop` 入口和 profiling 状态输出 |
| `tools/verify_can_mac.sh` | bundled libusb 设备扫描与验证 |
| `tools/verify_can_win.py` | Windows 验证说明和检测逻辑 |
| `tools/motor_diag.py` | 主动诊断命令确认保护 |
| `tools/a1zctl` | 统一使用当前环境 Python 的帮助命令 |
| `pyproject.toml` / `setup.py` | Pinocchio 平台依赖约束 |

### 软件验证

- `python -m unittest discover -q tests`：19 项测试通过。
- Python 编译检查通过。
- `git diff --check` 通过。

### 已知限制与后续工作

- Windows + HHS + G1Z 当前稳定在约 168–176 Hz，尚未达到配置目标 250 Hz；
  后续应使用 `--profile-loop` 和 `A1Z_TRACE_CAN=1` 区分 USB 提交、反馈接收、
  动力学计算和 Windows 调度器开销。
- Windows Python 3.10 的约 65–70 Hz 结果明显较差，当前文档推荐 3.11，
  但尚未完成更多 Python/Windows 版本的系统性对照。
- Zadig 是否需要取决于设备当前绑定的 Windows 驱动。驱动已支持 libusb 时
  不应重复替换；只有验证工具无法枚举或打开设备时再进入驱动排障。

## [历史基线：feature/can-backend-safety] — macOS / Windows 跨平台适配

### 当前状态

- Linux 继续使用 SocketCAN，原有 `can0` 工作流保持兼容。
- macOS 使用 gs_usb 用户态后端；HHS USB-CANFD 适配器
  `a8fa:8598` 已完成实机适配。
- Windows 已接入 gs_usb 和 PCAN 后端，但尚未完成对应硬件的实机验证。
- macOS HHS 默认启用已验证的异步 USB 收发路径，无需再设置
  `A1Z_GS_USB_ASYNC_TX=1`。仅在诊断旧同步路径时设置
  `A1Z_GS_USB_ASYNC_TX=0`。
- macOS 的 `gravity_factor=1.0` 重力补偿手感已恢复到接近 Linux，并完成
  300 秒稳定性测试。此前的“自主向前”跨平台差异已通过异步收发和 Linux
  位时序对齐解决。

### 主要修改

#### 跨平台 CAN

- 新增统一的 `open_can_bus()` 工厂：
  - Linux 默认选择 `socketcan`。
  - macOS / Windows 默认选择 `gs_usb`。
  - 仍可通过 `bustype` 显式选择 PCAN、slcan 等 python-can 后端。
- `get_a1z_robot()`、示例程序和诊断工具均支持 OS 自动选择通道与后端。
- HHS `a8fa:8598` 适配包括：
  - 设备 VID/PID 识别。
  - USB OUT endpoint `0x01`。
  - 非 Linux 平台的 kernel-driver 兼容处理。
  - 普通 20 字节 classic-CAN host frame。
  - TX 硬件回显过滤。
- macOS HHS 默认使用专用异步 USB 后端：
  - 预提交多个 TX transfer，并通过硬件回显释放发送槽位。
  - 持续提交 RX transfer，将有效反馈放入有界队列。
  - 控制线程只读取内存中的最新反馈，不再被同步 USB 读取阻塞。
- 其他 gs_usb 设备仍使用 python-can 标准实现。
- CAN bus 在机器人停止后显式 `shutdown()`，避免资源未释放警告。

#### CAN 位时序

Linux 6.8 内核 gs_usb 在同一 HHS 80 MHz 控制器、1 Mbps 下使用：

```text
sample_point=75%
brp=2
prop_seg=14
phase_seg1=15
phase_seg2=10
sjw=5
```

macOS HHS 后端已改用相同参数。原 macOS 用户态配置为 87.5% 采样点、
`brp=5`、`prop_seg=1`、`phase_seg1=12`、`phase_seg2=2`、`sjw=2`。
名义波特率虽然同为 1 Mbps，但采样点及同步能力不同，是此前 macOS 与
Linux 重力补偿手感差异的主要原因。

#### 控制循环

- 调度由相对 `sleep()` 改为基于 `time.monotonic()` 的绝对期限调度。
- macOS 实机控制频率由约 223–228 Hz 恢复到约 248–250 Hz。
- 状态输出统一使用 logging，避免主线程状态行与后台日志互相覆盖。
- `position_hold.py` 删除重复的 `Done.` 输出。
- `gravity_comp.py` 和 `position_hold.py` 新增 `--duration <seconds>`；
  `0` 表示持续运行至 Ctrl+C。
- `gravity_comp.py` 默认在 Ctrl+C 或定时结束后缓慢回零。零重力模式退出时，
  先让命令位置持续跟随实测位置，并在 0.75 秒内用 minimum-jerk 曲线平滑
  提升位置控制增益；随后从最新实测位置以 `0.3 rad/s` 生成回零轨迹，避免
  `kp=0` 直接切换到默认增益时因旧命令位置产生猛突。`--no-return-zero`
  可改为立即失能；
  `--return-zero` 保留用于命令兼容。
- 控制故障或急停不会执行主动回零，而是直接进入安全失能路径。

### 安全机制

- 启动时先发送零增益探测命令，并在 0.5 秒内等待六个关节的有效反馈。
  反馈不完整时拒绝启动并立即失能，避免用零值或旧状态计算首帧命令。
- 只在 CAN ID 匹配且 payload 成功解析后更新对应关节的反馈时间。
  未知 ID、短帧及解析失败帧不能刷新总线存活状态。
- 使用 monotonic clock 逐关节监控反馈年龄：
  - J1–J5：50 ms 警告，200 ms 急停。
  - J6：500 ms 警告，2 s 急停。
- J6 使用较宽阈值是因为实机确认其固件反馈存在阶段性稀疏，曾在其余五轴及
  总线正常时记录到 504 ms 间隔；主承重关节阈值未因此放宽。
- 保留关节位置、速度、力矩、温度、通信错误及电机 fault 的急停检查。
- 可用 `A1Z_DIAG_VEL_LIMIT=<rad/s>` 在诊断时设置更严格的速度急停阈值。

### 诊断能力

以下开关默认关闭，不改变正常控制逻辑：

- `A1Z_PROFILE_LOOP=1`
  - 输出读取、命令准备、动力学、发送及完整周期耗时。
- `A1Z_TRACE_CAN=1`
  - 每秒汇总 TX、硬件回显、有效 RX、CAN ID/DLC 分布。
  - 记录发送调用耗时、相邻发送间隔、J1 到 J6 提交跨度。
  - 记录 TX-RX 延迟、反馈年龄及 RX 队列水位/丢弃数。
  - Linux SocketCAN 仅采集统计，不改变原有接收语义。
- `A1Z_TRACE_CONTROL=1`
  - 记录关节位置、反馈速度、模型力矩、URDF 坐标系力矩和电机命令力矩。

这些诊断以及自动化测试属于现场定位、安全和防回归能力，均予以保留。
开发阶段使用的一次性断线压力脚本 `tools/stress_async_hhs.py` 已删除。

### 已解决问题

| 问题 | 原因 | 修复与验证 |
|------|------|------------|
| macOS 无 SocketCAN，SDK 无法打开 `can0` | 原实现硬编码 Linux 后端 | 按 OS 自动选择后端；macOS HHS 枚举、开总线及测试帧发送通过 |
| gs_usb 找不到 HHS 适配器 | VID/PID 不在默认识别列表 | 增加 `a8fa:8598` 识别 |
| HHS 无法发送 CAN 帧 | 设备 OUT endpoint 为 `0x01` | HHS 专用端点适配，六轴命令与反馈通过 |
| TX 回显被解析为电机反馈 | 用户态 gs_usb 会返回 `is_rx=False` 回显 | 过滤硬件回显，消除饱和假位置及假 fault |
| `recv(timeout=0)` 意外阻塞 | 丢弃回显后错误使用额外超时 | 恢复非阻塞语义，并最终采用持续异步 RX |
| 250 Hz 控制只能达到约 223–228 Hz | 相对睡眠累积调度误差 | 绝对期限调度后稳定在约 248–250 Hz |
| macOS 与 Linux 重力补偿手感不同 | 用户态同步 USB 时序及 CAN 位时序不同 | 默认异步收发并对齐 Linux 位时序，1.0 手感和稳定性复测通过 |
| 任意帧可掩盖单轴反馈冻结 | 仅监控总线级活动 | 改为逐关节、仅成功解析反馈刷新时间戳 |
| 统一 200 ms 阈值导致 J6 误停 | J6 固件反馈阶段性稀疏 | 使用逐关节分级阈值；J1–J5 保持严格保护 |
| 程序停止后资源释放警告 | 未显式关闭 CAN bus | 停止路径调用 `shutdown()` |

### 实机验证

#### macOS HHS

- `tools/verify_can_mac.sh`：
  - 内置 libusb 加载成功。
  - 识别 `a8fa:8598`。
  - 1 Mbps 总线初始化和测试帧发送成功。
- 异步后端断线压力验证：
  - 250 个周期/秒，每周期六帧。
  - 10 秒发送 15,000 帧。
  - 未出现 transfer 错误、发送槽耗尽或队列丢弃。
- `gravity_factor=1.0` 自由拖动复测：
  - 用户确认手感与 Linux 接近。
  - 未再感觉此前的自主向前趋势。
  - 控制频率约 250 Hz，TX/硬件回显完整，RX 队列无丢弃。
- 300 秒 `gravity_factor=1.0` position-hold：
  - 完整到达定时时间并正常失能。
  - TX/echo 约 1500 帧/秒。
  - J1–J5 约 250 帧/秒，J6 约 149 帧/秒。
  - J1–J6 提交跨度平均约 0.70 ms、最大约 2.01 ms。
  - 未出现反馈急停或 CAN 错误。
- 30 秒 `gravity_factor=0` 位置保持：
  - 控制频率约 249–250 Hz。
  - J1–J5 约 248–251 帧/秒。
  - J6 一次达到 500 ms 警告后自行恢复，未达到 2 秒急停阈值。

#### Linux 对照

- 使用相同源码、URDF、关节符号、重力缩放、MotorA 力矩常数及
  `python-can 4.6.1` 对照。
- 控制侧确认 `tau_model` 与 `tau_urdf` 一致；J3 模型力矩约
  `-4.30 Nm` 正确转换为电机坐标约 `+4.30 Nm`。
- SocketCAN 稳态每轴 TX 约 250 Hz；J1–J5 RX 约 250 Hz。
- Linux J6 的反馈频率本身低于其他关节，证实 J6 稀疏反馈并非 macOS
  异步后端独有。

### 测试

- `python -m unittest discover -q tests`：12 项测试通过。
- Python 源码编译检查通过。
- `git diff --check` 通过。
- 当前回归覆盖：
  - gs_usb 通道字符串到设备索引转换。
  - HHS 默认异步后端及显式同步回退。
  - TX 回显过滤和非阻塞接收。
  - CAN trace 包装及统计。
  - 未知 ID、短帧不能刷新反馈时间。
  - 单关节反馈陈旧可独立触发保护。
  - 零重力退出时增益提升阶段持续跟随实测位置，不使用陈旧命令位置。

### 新增文件

| 文件 | 说明 |
|------|------|
| `a1z/motor_drivers/can_backend.py` | 跨平台 CAN 选择、HHS 适配、异步收发、回显过滤及诊断 |
| `tools/verify_can_mac.sh` | macOS HHS 枚举和通信验证 |
| `tools/verify_can_win.py` | Windows CAN 通信验证 |
| `tools/setup_can.sh` | Linux SocketCAN 配置与发送自检 |
| `tests/test_can_backend.py` | CAN 后端回归测试 |
| `tests/test_feedback_safety.py` | 逐关节反馈安全回归测试 |
| `tests/test_safe_return.py` | 零重力模式安全回零增益切换测试 |

### 主要修改文件

| 文件 | 改动 |
|------|------|
| `a1z/robots/get_robot.py` | 使用统一 CAN 工厂，增加 `bustype`，按 OS 选择默认通道 |
| `a1z/robots/arm_robot.py` | 绝对期限调度、总线释放、逐关节反馈保护及控制诊断 |
| `a1z/motor_drivers/can_interface.py` | 使用统一 CAN 工厂 |
| `a1z/motor_drivers/motor_b_driver.py` | 有效反馈计数及逐关节反馈时间 |
| `examples/gravity_comp.py` | 跨平台参数、定时运行、退出策略和日志输出 |
| `examples/position_hold.py` | 跨平台参数、定时运行及日志修复 |
| `examples/teach_and_play.py` | 跨平台 CAN 参数 |
| `tools/motor_diag.py` | 跨平台后端和接口检查 |
| `tools/set_zero.py` | 跨平台后端和通道 |
| `pyproject.toml` / `setup.py` | `[gs_usb]`、`[pcan]` 可选依赖 |
| `README.md` | 中英文安装、配置、API 和故障排查 |

### 安装与验证

```bash
# Linux
pip install -e .
sudo bash tools/setup_can.sh

# macOS
pip install -e ".[gs_usb]"
bash tools/verify_can_mac.sh

# Windows（另需使用 Zadig 为 gs_usb 设备安装 WinUSB）
pip install -e ".[gs_usb]"
python tools/verify_can_win.py

# 任意平台使用 PEAK PCAN
pip install -e ".[pcan]"
```

### 已知限制与后续工作

| 优先级 | 项目 | 说明 |
|--------|------|------|
| P1 | 动力学模型标定 | 通信差异已解决，但 URDF 质量、质心、实际末端负载及关节零位仍应通过多姿态实验继续标定 |
| P1 | 启动力矩渐变 | 零重力模式会从探测命令切换到补偿力矩，尚未实现可配置 torque ramp |
| P1 | 逐关节补偿参数 | 当前主要使用全局 `gravity_factor`，后续标定可增加逐关节比例和零位偏置 |
| P2 | J6 反馈稀疏 | 已通过独立阈值安全兼容，但应继续确认是电机固件策略还是协议特性 |
| P2 | HHS 运行时兼容实现 | 部分 gs_usb/PyUSB 兼容处理仍是运行时 patch，后续可收敛为专用类或上游修改 |
| P2 | python-can 私有接口 | 回显兼容路径使用 `_recv_internal()`，升级 python-can 时需要回归 |
| P2 | Windows / PCAN 实机 | 代码和文档已接入，仍需对应平台与硬件验证 |

### 历史调查摘要

macOS 初期可连接并驱动机械臂，但 `gravity_factor=1.0` 时出现与 Linux 不同的
自主向前趋势、松手续行和阻尼手感。调查按以下顺序排除了多个问题：

1. 修复 TX 回显误解析，消除假位置和假 fault。
2. 修复非阻塞读取和控制调度，使控制频率恢复到 250 Hz。
3. 对比相同源码、URDF、符号、力矩比例及原始 CAN payload，未发现模型计算
   或命令编解码的跨平台差异。
4. 引入持续 RX 和异步 TX，显著降低六轴提交跨度，但仅靠异步发送仍曾在
   `gravity_factor=0.7` 复现向前趋势。
5. 最终发现 Linux 内核与 macOS 用户态使用不同 CAN 位时序。对齐 Linux
   的 75% 采样点及完整 timing 参数后，`gravity_factor=1.0` 手感恢复，
   随后的重复拖动和 300 秒稳定性测试均通过。

因此最终结论是：此前问题不是单一的 URDF、关节符号或 `kd` 错误，而是
macOS 用户态 USB 收发语义和 CAN 位时序共同造成的跨平台差异，其中位时序
不一致是自主运动现象的主要原因。
