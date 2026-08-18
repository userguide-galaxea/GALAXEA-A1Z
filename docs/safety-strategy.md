# A1Z 安全策略报告与重构规划

> 版本：v1.0 · 2026-08-05
> 范围：`a1z/` SDK、`examples/`、`tools/`、`tests/`、配置文件、a1zctl server 与文档
> 性质：安全审计 + 目标架构设计 + 分阶段实施规划

---

## 0. 摘要

本次审计对 A1Z 机械臂 SDK 的全部安全机制做了逐文件核查，并对照 ISO 10218-1:2025、
ISO/TS 15066 以及 Universal Robots / Franka Emika / Doosan / Kinova 等商用协作臂的
安全架构做了差距分析。

总体结论：**这套 SDK 的安全底子比多数开源机械臂项目好**——逐关节反馈过期 estop、
温度/速度/电机错误码硬故障断使能、软急停保持重力补偿、失能确认 + flush、
CAN 回环帧过滤等机制都是正确的设计。但存在三类系统性问题：

1. **"出厂默认值不安全"**：仓库自带的 `a1z.yaml` 把 `gravity_comp_factor` 设为 1.0、
   把未经验证的高 kp（代码默认值的 3–24 倍）作为默认配置分发，与 README 自身的
   安全建议直接矛盾。这是当前**可能性最高**的伤害来源。
2. **"失效时不出声"（fail-silent）**：编码层 `float_to_uint` 对一切越限命令静默
   clamp、扭矩 clip 无日志、全链路无 NaN 防护（NaN 命令能穿透所有限值校验）。
   上层校验一旦有洞，错误动作会被无声地执行。
3. **"安全阈值硬编码、外部通道不设防"**：所有安全参数焊死在 SDK 里不可审计、
   不可收紧；a1zctl 的 Unix socket 无认证、无心跳、无命令超时，而
   `docs/openclaw-integration.md` 给这个"AI 代理直接驱动真臂"的通道写了 65 行
   文档却没有一句安全警告。

因此本报告的建议不是推倒整个安全体系，而是：**保留驱动层/控制层骨架，推翻重做
"命令验证管线、运行时安全监控、安全配置层、外部控制通道"四个子系统**（详见 §6）。

---

## 1. 审计范围与方法

- 代码：`a1z/` 全部 18 个 Python 文件（重点：`robots/arm_robot.py` 1287 行、
  `motor_drivers/can_backend.py` 1043 行）、`examples/` 5 个示例、`tools/` 7 个工具、
  `tests/` 3 个测试文件、`a1z.yaml` / `a1z_g1z.yaml`、README 与 docs。
- 方法：静态审查（限值定义与执行点、错误处理路径、并发模型）+ 测试覆盖分析 +
  行业对标。所有结论附 `文件:行号` 引用。
- 免责：本审计未做真机动态测试；对电机固件行为的判断基于代码中的协议注释与
  错误码表，固件内部保护逻辑不在审计范围内。

---

## 2. 现状评估

### 2.1 已有机制：值得保留的部分

当前安全体系分三层，骨架是健康的：

**驱动层**（`a1z/motor_drivers/`）
- 编码范围截断（`float_to_uint`，`motor_drivers/utils.py:15-20`）
- 电机错误码解析（过压/欠压/过流/过温/通信丢失/过载/位置超程，
  `motor_b_driver.py:30-41`）
- CAN 回环帧过滤（`EchoFilterBus`，`can_backend.py:130-261`，防止把自己的
  MIT 命令帧误解析为电机反馈）
- 逐关节反馈时间戳跟踪（`motor_b_driver.py:301,389,393-401`）

**机器人层**（`a1z/robots/arm_robot.py`）
- 250 Hz 控制循环，每周期跑 `_check_runtime_safety()`，硬故障 → 紧急断使能
- 命令侧三级关节限值校验（`_validate_joint_pos` L1248-1273 /
  `_clip_joint_pos` L1198-1246 / `_accept_or_reject_stream` L392-410）
- 反馈过期分级处理：J1–J5 warn 50ms / estop 200ms，J6 warn 500ms / estop 2000ms
  （`_check_feedback_stale` L1172-1196）
- 温度阈值：MOS 85°C / 线圈 90°C estop（`_check_motor_temps` L1133-1159）
- 实测速度超限 estop（`_check_velocity_limits` L1161-1170）
- 重力力矩超限"停止而非饱和"（L1014-1018）——方向正确
- 软急停 `estop()`：kp 清零、命令钉在实测位、重力补偿保持运行、手臂不塌落
  （L542-567），`release()` 显式恢复（L569-583）
- 启动反馈门槛：收齐全部关节反馈才允许控制，0.5s 超时断使能（L257-274）
- 失能双发 + `flush_tx` 确认（`motor_b_driver.py:312-330`，
  `arm_robot.py:309-328`）
- 频率看门狗：<80 Hz 连续 6s 断使能（L870-917）
- 平滑运动：minimum-jerk 插值、时长按加速度上限反推（L720-750）、
  `max_jump_rad` 防 IK 肘翻转（L701-711）、`sync_to_measured` + 增益爬坡
  防"抓回"猛跳（L653-689）

**示例/工具层**
- `gravity_comp.py` / `position_hold.py` / `teach_and_play.py` 的退出序列用
  `SIG_IGN` 保护，保证第二次 Ctrl+C 不打断失能——这是很好的范式
- `gripper_set_zero.py`：堵转检测归位、写 flash 前显式失能、写完 ping 验证
- `motor_diag.py`：失能确认失败时明确打印 ERROR

### 2.2 问题清单（按严重度分级）

#### P0 — 实质性风险，可能造成人身或设备伤害

| # | 问题 | 位置 | 说明 |
|---|------|------|------|
| P0-1 | 配置文件默认值危险 | `a1z.yaml:20,37` | `gravity_comp_factor: 1.0` 与 README（L560）"首次使用从 0.3 开始"的建议矛盾；`default_kp` 为代码默认值的 3–24 倍，注释自述"待真机 A/B 验证，如有震荡注释掉回退"——把未验证的高增益当默认配置分发 |
| P0-2 | 全链路无 NaN/inf 校验 | 全包 | 命令侧 NaN 使 `pos < lo` / `pos > hi` 恒为 False，**穿透全部限值校验**，最终在 `float_to_uint` 的 `int(nan)` 处"意外"抛错兜底；反馈侧 NaN 会污染重力补偿与位置保持计算。这不是设计，是侥幸 |
| P0-3 | 实测位置越软限值只告警不处置 | `arm_robot.py:1085-1112` | 刻意设计（teleop 友好），但后果是手臂可以无限期停留在软限位之外，软限位事实上只对"命令"有效，对"状态"无效 |
| P0-4 | 外部控制通道零防护 | `robots/server.py`、`tools/a1zctl`、`docs/openclaw-integration.md` | Unix socket 无认证（本机任意用户可发 move）、无命令心跳/超时（上游崩溃后手臂无限期保持最后命令）、文档零安全警告——而这个通道的卖点恰恰是"AI 代理自然语言驱动真臂" |
| P0-5 | 回放速度无上限 | `examples/teach_and_play.py:232-233` | `--speed 10` 可让录制轨迹 10 倍速回放，无封顶 |
| P0-6 | 测试帧 ID 注释错误 | `tools/setup_can.sh:36`、`verify_can_mac.sh:67`、`verify_can_win.py:65` | 注释称 0x7FF 是"不存在的 CAN ID"，实际 0x7FF 正是零点/存储参数广播命令 ID。当前全零 payload 恰好无害，但注释误导后人构造变体帧即可触发持久性零点写入 |
| P0-7 | 示例引导危险动作且参数无上限 | `examples/gripper_hybrid_test.py:106-111,140` | 测试 2 明确要求"用手阻挡运动中的夹爪"，且 `--torque` 无上限校验（可达电机峰值扭矩） |

#### P1 — 架构性弱点，侵蚀安全体系的完整性

| # | 问题 | 位置 | 说明 |
|---|------|------|------|
| P1-1 | 编码层静默 clamp | `motor_drivers/utils.py:15-20` | 一切越限命令被无声截断后继续执行。上层校验一旦有洞，错误动作无声发生。这是 fail-silent 哲学，与安全要求的 fail-loud 相反 |
| P1-2 | 扭矩 clip 无告警 | `arm_robot.py:1023` | `_TORQUE_CLIP` 静默截断，用户无从知道自己的命令被打了折 |
| P1-3 | 无命令心跳/超时 | `arm_robot.py` | `command_joint_pos` 之后上游停发，手臂无限期保持最后位置。流式控制（teleop、AI 代理）上游崩溃即"僵死在工作空间任意位姿" |
| P1-4 | 安全阈值全部硬编码 | `arm_robot.py`、`get_robot.py` | 限值、超时、温度阈值、扭矩上限全部焊死在代码里；配置层只有 `min_freq_hz` 一个安全字段（还被注释掉了）。不可审计、不可按场景收紧、无法在启动时打印核对 |
| P1-5 | CAN bus / 夹爪反馈无锁并发 | `gripper.py:111,115,141` | 主线程（gripper enable/home）与控制线程（`_dispatch_feedback`）并发写 `motor.last_feedback`；`bus.send` 双线程调用依赖底层总线内部锁 |
| P1-6 | 无 atexit / 信号处理 | `arm_robot.py:1285-1287` | 仅靠 `__del__` 兜底，解释器退出期不可靠；`ArmRobot` 层没有 SIGINT/SIGTERM 处理器 |
| P1-7 | 温度缺失返回 0.0 | `motor_b_driver.py:448-449` | 无反馈电机温度读数为 0.0，"传感器永远读不到"只能靠 stale-feedback 间接兜底，无直接诊断 |
| P1-8 | 无碰撞/外力监测 | — | 只有被动限值，没有基于模型偏差（实测力矩 vs 期望力矩）的碰撞检测。对比 Franka/UR/Kinova，这是协作场景的核心缺失能力 |
| P1-9 | 安全测试覆盖缺口 | `tests/` | 限位裁剪、扭矩 clip、重力 estop、平滑停机、min_freq 保护、estop 锁存/恢复、警告路径——均无测试。现有测试集中在反馈过期与启动/失能路径 |
| P1-10 | 无停止分级与恢复协议 | `arm_robot.py` | 目前只有"软 estop 悬停"和"断使能"两极，没有中间级（降速/降力运行），没有明确的故障→确认→恢复状态机（2026-08-05 起位置超程 0xF 已改为保护性悬停，见 `_HOLD_FAULT_CODES`，其余故障仍是二元跳变） |

#### P2 — 代码质量类隐患

| # | 问题 | 位置 |
|---|------|------|
| P2-1 | 安全断言用裸 `assert`，`python -O` 下失效 | `gravity_model.py:33,50,72-74` |
| P2-2 | 安全相关 numpy 数组（`_TORQUE_CLIP`、`_max_gravity_torque`）作为可变共享对象未 copy | `arm_robot.py:103-106` |
| P2-3 | `gravity_comp_factor` / `gravity_torque_scale` 无范围校验（可为负、可 >1） | `arm_robot.py:100-104` |
| P2-4 | `get_error_codes()` 对无反馈电机返回 0（"无错误"），靠调用方自觉区分 | `motor_b_driver.py:421-427` |
| P2-5 | `set_zero.py` 标定 MotorB 后电机留在使能状态，无显式失能 | `tools/set_zero.py:104-120` |
| P2-6 | `motor_diag.py --clear-error` 清错后电机可能立即恢复出力，无警告 | `tools/motor_diag.py:793-803` |
| P2-7 | README 内部矛盾：`stop()` 衰减时间 0.3s vs 0.8s 两处不一致 | `README.md` L383/L560 vs L522 |
| P2-8 | 控制循环 `time.sleep` 无漂移补偿，长期频率低于标称 | `arm_robot.py:922-925` |
| P2-9 | 启动限值检查仅 warning 不阻止 | `arm_robot.py:1275-1283` |
| P2-10 | 两份 yaml 除 `with_gripper` 外完全相同，重复维护易漂移 | `a1z.yaml` / `a1z_g1z.yaml` |

---

## 3. 行业参考

### 3.1 标准体系

**ISO 10218-1:2025 / ISO 10218-2:2025**（2025-04-01 生效，2011 版以来首次大修）
- ISO/TS 15066:2016 不再独立存在，其"功率与力限制"等协作应用要求已并入
  10218 系列；新增网络安全、功能安全与机器人分类要求。
- 四种认可的协作操作方式：**功率与力限制（PFL）、手动引导（hand guiding）、
  速度与间距监控（SSM）、安全级监控停止（safety-rated monitored stop）**。
  （来源：[EVS Robot, 2026-07](https://www.evsint.com/collaborative-robot-safety-standards-2026-iso-10218-2025-ts-15066/)）

**ISO/TS 15066（已并入 10218）**
- 定义 29 个人体区域的准静态/瞬态接触力与压强阈值（瞬态约为准静态的 2 倍）；
  协作场景 TCP 速度的事实上限约 250 mm/s。
  （来源：[InMotion, 2026-04](https://www.inmotion.global/resources/cobot-safety/collaborative-robot-safety-standards/)）

**IEC 60204-1 停止分级**
- Category 0：立即断电（不可控停止）
- Category 1：受控减速停止后断电
- Category 2：受控停止、动力保持
- a1z 的"软 estop 重力补偿悬停"≈ Category 2，"断使能"≈ Category 0，
  **缺 Category 1**（先受控停下再断电）——这正是 `stop()` 的 0.8s 衰减想做的事，
  但没有被形式化为停止分级。

**ISO 13849-1（功能安全）**
- 商用臂的安全监控达到 PLd / Category 3：双通道冗余、安全处理器独立于主控。
  纯软件 SDK 无法达到 PL 等级，但**"监控独立于控制路径"**的架构思想可以而且
  应该借鉴。

### 3.2 商用协作臂做法

**Universal Robots（e-Series）**
- 17 个可配置安全功能，ISO 13849-1 PLd Cat 3 认证，**冗余安全处理器独立于
  主控处理器**。
- 限值维度：关节位置、关节速度、TCP 平面（最多 8 个虚拟墙）、TCP 速度
  （协作默认 250 mm/s）、TCP 力（100–250 N 可配）、肘部速度/力、动量、功率、
  停止时间、停止距离。
- 触发力限停止后自动"back-off"退到力不超限的位置；所有安全功能通过带密码的
  Safety 配置页设定，**可审计**。
  （来源：[UR Safety Functions Table](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR7e/safetyFunctionsAndinterfaces/safety_functions_table1.htm)、
  [UR vs FANUC 对比, 2026-01](https://seraphim.vn/pages/robotics/universal-robots-vs-fanuc.html)）

**Doosan Robotics**
- **Normal Mode / Reduced Mode 双套限值**：检测到人或进入协作区域后切换到
  更严的 Reduced 限值集。
- 超限触发 Protective Stop；提供 Safety Recovery Mode 让人在保护停止状态下
  把臂移回安全位姿再复位。
  （来源：[Doosan Robot Limits 手册](https://manual.doosanrobotics.com/en/user-manual/3.6.0/1-m-h-series/robot-limits)）

**Franka Emika**
- 7 轴全部带关节力矩传感器，**基于模型的碰撞检测**：实测力矩与动力学模型
  期望力矩偏差超限即触发反射式停止（reflex stop），再加手动引导。
  核心思想：安全不是只有"别超过限值"，还有"世界和模型不符时立刻停"。

**Kinova**
- 关节力矩传感 + 执行器固件层安全限值，SDK 层限值只是最外层。

### 3.3 可借鉴的设计哲学

1. **监控独立于控制**：安全检查不应和控制计算共用同一条数据通路假设；
   安全监控要假设控制路径本身可能出错。
2. **fail-loud，不是 fail-silent**：任何被裁剪/拒绝的命令都必须留下痕迹
   （日志/计数器/状态位），静默截断在安全语境里等于隐瞒。
3. **分级响应，不是二元跳变**：正常 → 告警 → 降级运行（降速降力）→
   受控停止 → 断电，按严重度选择，而不是"要么忍要么断"。
4. **阈值可配置、可审计、启动时打印**：安全参数是部署决策，不是代码常量。
5. **恢复需要显式人工确认**：故障消除 ≠ 自动恢复运行（Doosan 的
   Protective Stop 复位、UR 的密码配置都是这个思想）。
6. **模型偏差即异常**：有动力学模型就能做碰撞检测，这是桌面臂低成本
   实现"类力控安全"的现实路径。

---

## 4. a1z 威胁模型

桌面级 6 轴臂 + 夹爪，CAN 总线，无安全认证硬件，典型使用场景：实验室开发、
遥操作、示教回放、AI 代理（openclaw）远程驱动。

| 编号 | 威胁 | 现有防护 | 缺口 |
|------|------|----------|------|
| T1 | 错误/恶意命令导致飞车、撞限位 | 命令侧三级限值校验、速度/增益上限 | NaN 穿透校验（P0-2）；编码层静默 clamp（P1-1） |
| T2 | CAN 断线/USB 拔出导致盲控 | 逐关节 stale estop、频率看门狗 | 无（较完善） |
| T3 | 伪造/回环帧污染状态估计 | EchoFilterBus | 无源地址白名单/帧认证（成本所限，可接受） |
| T4 | 外部通道被滥用（无认证 socket、AI 代理） | 无 | 全部缺失（P0-4） |
| T5 | 配置错误（高增益、全额重力补偿、错零点） | 部分启动检查 | 默认值本身危险（P0-1）；参数无范围校验（P2-3） |
| T6 | 电机过温/过流/故障 | 固件错误码 + 软件温度 estop | 温度缺失不可诊断（P1-7） |
| T7 | 人手进入工作空间被夹/撞 | 无（只有被动限值） | 无碰撞检测（P1-8）；无 Reduced 模式（P1-10） |
| T8 | 断电/崩溃时手臂下落 | 软 estop 保持重力补偿；disable flush | `__del__` 不可靠（P1-6）；无 atexit |
| T9 | 标定工具误写零点 | 交互确认 | 测试帧注释误导（P0-6）；标定后留使能（P2-5） |
| T10 | 并发竞争导致命令错乱 | 状态/命令锁分离 | bus/夹爪反馈无锁（P1-5） |

---

## 5. 目标安全架构

```
┌─────────────────────────────────────────────────────────┐
│ L5 配置与文档层   安全配置 schema · 启动审计打印 · 危险确认 │
├─────────────────────────────────────────────────────────┤
│ L4 系统层         atexit+信号处理 · 命令心跳/超时 · 通道认证 │
├─────────────────────────────────────────────────────────┤
│ L3 安全监控层     SafetyMonitor 状态机（独立于控制逻辑）     │
│                   NORMAL→WARN→REDUCED→STOP→ESTOP→恢复确认  │
├─────────────────────────────────────────────────────────┤
│ L2 命令验证管线   统一入口 · fail-closed · NaN 拒绝 · 限速  │
├─────────────────────────────────────────────────────────┤
│ L1 编码/驱动层    fail-loud（拒绝或告警，绝不静默截断）      │
├─────────────────────────────────────────────────────────┤
│ L0 固件层         电机错误码 · 夹爪电流限幅 · (可下发限值)   │
└─────────────────────────────────────────────────────────┘
```

### L0 固件层（保留 + 调研）
- 保留现有错误码监控。调研 MotorA/MotorB 固件是否支持下发位置/速度/电流限值；
  若支持，把 L5 配置中的硬限值下发到固件，实现"软件死了固件还守着"。

### L1 编码/驱动层（推翻静默 clamp）
- `float_to_uint` 越限时记 warning（每关节限速 1 Hz）并计入诊断计数器；
  或者由调用方选择 `strict=True` 直接抛错。
- 目标：**任何一层截断命令，都必须让上层知道**。

### L2 命令验证管线（推翻重做）
- 收敛为一个 `validate_command()`：NaN/inf 一律拒绝；位置/速度/增益按 L5 配置
  校验；拒绝即拒帧 + 计数 + 日志，语义全入口统一（阻塞入口抛异常、流式入口
  拒帧的现状保留，但内部走同一条管线）。
- 增加命令速率限制（每入口 token bucket），防上游失控刷爆总线。

### L3 安全监控层（推翻重做，核心）
把现在散落的 `_check_*` 函数集合重构为独立 `SafetyMonitor`：

- **状态机**：`NORMAL → WARNING → REDUCED → PROTECTIVE_STOP → ESTOP_DISABLE`，
  每级对应 IEC 60204-1 式响应：
  - WARNING：日志 + 事件回调，不改变行为（现状的 1 Hz 告警路径）
  - REDUCED：自动切到降速/降力参数集（借鉴 Doosan 双模式）
  - PROTECTIVE_STOP：受控停住、动力保持（≈ 现有软 estop，Category 2）
  - ESTOP_DISABLE：受控衰减后断使能（补 Category 1），或直接断（Category 0，
    保留给温度/电机故障等硬故障）
- **恢复协议**：脱离 REDUCED/STOP/ESTOP 必须显式调用 `acknowledge()` /
  `release()`，故障原因记入事件日志；不允许自动复位。
- **新增监测项**：碰撞检测（实测力矩 vs 模型期望力矩偏差）、实测位置越限
  升级处置（现为仅告警）、命令心跳超时。
- 监控器持有自己的状态副本，不依赖控制循环的中间变量（监控独立于控制）。

### L4 系统层
- `ArmRobot.start()` 注册 atexit 钩子 + 可选 SIGINT/SIGTERM 处理器
  （默认不劫持，提供 `install_signal_handlers=True` 参数），失能序列内
  SIG_IGN 保护（推广 examples 里已有的好范式）。
- 流式命令入口增加心跳：`command_joint_pos/state` 携带或隐式刷新心跳，
  超过 `command_timeout_s`（默认 0.5s）未收到新命令 → REDUCED/STOP。
- server：socket 文件权限 0600 + 可选 token 认证；每个会话独立命令超时；
  危险命令（高速度、大行程）需确认标志。

### L5 配置与文档层
- 新增 `[safety]` 配置段，把目前硬编码的全部阈值外置：关节限值、速度上限、
  扭矩上限、温度阈值、反馈超时、命令超时、心跳、碰撞检测阈值、Reduced 模式
  缩放系数。
- 启动时打印生效安全配置摘要（审计性）；配置值做范围校验（含
  `gravity_comp_factor ∈ [0, 1]`）。
- 配置默认值即"首次使用安全值"：`gravity_comp_factor: 0.3`，kp 回到
  已验证的代码默认值，激进值以注释形式附在文件里供进阶用户显式启用。
- 文档：README 安全章节重写（停止分级、恢复协议、openclaw 通道风险）；
  `openclaw-integration.md` 增加安全章节；修正 0.3s/0.8s 矛盾。

---

## 6. 推翻重做 vs 修补的判定

**推翻重做（4 项）**
1. **运行时安全检查** → `SafetyMonitor` 状态机（§5-L3）。现有 `_check_*`
   函数集合的"检查逻辑"本身大多正确，但组织方式（平铺、二元响应、无恢复
   协议）无法支撑分级安全，必须重构。检查阈值数据可迁移。
2. **命令校验** → 统一命令验证管线（§5-L2）。现状三处校验语义不一
   （抛错/拒帧/clip）+ NaN 穿透，补丁式修复不如收敛重写。
3. **配置层** → 安全配置外置（§5-L5）。"阈值焊死在代码里"是设计哲学问题，
   不是改几个常量能解决的。
4. **外部控制通道** → server/a1zctl 安全加固（§5-L4）。认证、心跳、
   命令超时、文档全套补齐。

**保留 + 修补（其余全部）**
- 驱动层反馈跟踪、回环过滤、失能双发 + flush、启动反馈门槛、软 estop、
  增益爬坡、minimum-jerk、`max_jump_rad`、温度/速度/错误码检查逻辑——
  这些设计正确，按 P1/P2 清单修补即可（NaN、静默 clamp、锁、copy、
  assert 改显式异常等）。

---

## 7. 详细实施规划

### Phase 0 — 紧急修复（1 天，无架构变化，纯降险）

| 任务 | 文件 | 对应问题 |
|------|------|----------|
| `gravity_comp_factor` 默认改 0.3；`default_kp` 回退代码默认值，激进值改为注释 | `a1z.yaml`、`a1z_g1z.yaml` | P0-1 |
| 修正 0x7FF 测试帧注释，payload 改用明确无效的扩展帧或文档说明 | `tools/setup_can.sh`、`verify_can_mac.sh`、`verify_can_win.py` | P0-6 |
| `teach_and_play.py --speed` 加上限校验（如 ≤2.0） | `examples/teach_and_play.py` | P0-5 |
| `gripper_hybrid_test.py --torque` 加上限校验 + 危险动作警告措辞强化 | `examples/gripper_hybrid_test.py` | P0-7 |
| `set_zero.py` 标定后显式失能 | `tools/set_zero.py` | P2-5 |
| README 修正 0.3s/0.8s 矛盾；`--clear-error` 加警告 | `README.md`、`tools/motor_diag.py` | P2-6、P2-7 |
| `gravity_comp_factor`/`gravity_torque_scale` 范围校验 [0,1] | `arm_robot.py` | P2-3 |

### Phase 1 — 结构性修复（约 1 周，局部重构）

1. **NaN/inf 全链路防护**：`JointCommand`/`JointState` 入口统一 `np.isfinite`
   校验；反馈侧 `_dispatch_feedback` 后校验，NaN 反馈按"该帧无效"处理
   （不刷新 staleness 时间戳）。（P0-2）
2. **编码层 fail-loud**：`float_to_uint` 增加越限计数 + 限速告警；
   `_TORQUE_CLIP` clip 时记 warning。（P1-1、P1-2）
3. **命令心跳/超时**：流式入口隐式心跳 + `command_timeout_s`，超时进入
   保护性停止（暂挂在现有 estop 路径上，Phase 2 迁入状态机）。（P1-3）
4. **atexit + 信号处理**：`start()` 注册 atexit；`install_signal_handlers`
   可选参数；失能序列 SIG_IGN 化。（P1-6）
5. **并发修复**：`bus.send` 加发送锁（或全部串入控制线程队列）；
   gripper 反馈写入纳入锁保护。（P1-5）
6. **杂项**：可变数组 copy、裸 assert 改显式异常、温度缺失诊断、
   两份 yaml 合并为模板 + 覆盖。（P2-1/2/4/10）
7. **补测试**：限位裁剪、扭矩 clip、重力 estop、min_freq 保护、estop
   锁存/恢复、NaN 拒绝、心跳超时。（P1-9）

### Phase 2 — 架构重做（约 2–3 周）

1. **SafetyMonitor 状态机**：按 §5-L3 实现，迁移全部 `_check_*` 逻辑；
   停止分级（Cat 0/1/2）形式化；`acknowledge()` 恢复协议；事件日志。
2. **统一命令验证管线**：收敛三处校验为一处，语义文档化。
3. **安全配置 schema**：`[safety]` 段 + 启动审计打印 + 范围校验；
   旧硬编码常量作为默认回退。
4. **server 加固**：socket 权限、可选 token、每会话命令超时、危险命令确认；
   `openclaw-integration.md` 安全章节。
5. **实测位置越限处置升级**：从"仅告警"改为 告警 →（持续 N 秒）→ REDUCED/
   PROTECTIVE_STOP，保留 teleop 场景的宽限但消除"无限期停限位外"。（P0-3）

### Phase 3 — 能力扩展（约 3–4 周，可与 Phase 2 部分并行）

1. **碰撞检测**：基于 URDF 动力学模型，比较实测电流/力矩与模型期望值，
   偏差超阈值 → PROTECTIVE_STOP（借鉴 Franka 思路，无力矩传感器的低成本版）。
2. **Reduced 模式**：双套限值配置 + 手动/事件触发切换（借鉴 Doosan）。
3. **工作空间边界**：TCP 虚拟平面/圆柱限制（借鉴 UR，限末端而非仅关节）。
4. **固件限值下发**：调研并实现（若固件支持）。
5. **安全审计日志**：所有安全事件（告警/降级/停止/恢复/拒绝）落盘，
   带时间戳与原因码。

### 里程碑与验证

- 每 Phase 结束跑全量 `pytest` + 新增安全测试；Phase 2/3 需真机冒烟
  （空载 → 负载 → 人为制造 stale/越限/碰撞，观察分级响应）。
- 验收标准：Phase 1 后无 P0 项；Phase 2 后无 P1 项；Phase 3 后具备
  碰撞检测与 Reduced 模式，安全事件可审计。

---

## 8. 参考来源

- [Collaborative Robot Safety Standards 2026: ISO 10218:2025 — EVS Robot](https://www.evsint.com/collaborative-robot-safety-standards-2026-iso-10218-2025-ts-15066/)
- [Collaborative Robot Safety Standards: ISO 10218, ISO/TS 15066, ANSI/A3 R15.06-2025 — InMotion](https://www.inmotion.global/resources/cobot-safety/collaborative-robot-safety-standards/)
- [Universal Robots Safety Functions Table](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR7e/safetyFunctionsAndinterfaces/safety_functions_table1.htm)
- [Universal Robots vs FANUC: Cobot Safety Features — Seraphim](https://seraphim.vn/pages/robotics/universal-robots-vs-fanuc.html)
- [Doosan Robotics Robot Limits 手册](https://manual.doosanrobotics.com/en/user-manual/3.6.0/1-m-h-series/robot-limits)
- ISO 10218-1:2025 / ISO 10218-2:2025 / IEC 60204-1 / ISO 13849-1（标准原文）
