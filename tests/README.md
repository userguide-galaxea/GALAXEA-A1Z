# tests/ — A1Z SDK 离线测试套件

全部测试离线运行：不打开 CAN 接口、不启动控制线程、不触碰硬件。真机验证不属于本目录，按 `01-docs/04-sop/test-development-sop.md` 的阶段门禁与真机回归章节执行。

## 运行

```bash
cd /home/acton/Projects/sdk-release/02-ws/03-preopen-version/GALAXEA-A1Z
python -m pip install -e ".[dev]"     # 一次性环境准备
python -m pytest tests/ -v            # 全量
python -m pytest tests/test_motion_safety.py -v            # 单文件
python -m pytest tests/test_motion_safety.py::TestClipJointPos::test_clip_at_boundary -v  # 单用例
```

## 约定

- 文件命名 `tests/test_<被测主题>.py`；模块 docstring 写明覆盖点与移植背景。
- 新测试用 pytest 风格（裸函数 + `assert` / `pytest.raises` / `parametrize`）；早期文件中的 `unittest.TestCase` 风格保持原样，不混改。
- 优先断言行为契约（状态、故障码、命令内容），不依赖真实时序；需要"时间"时用 `monkeypatch` 替换 `a1z.robots.arm_robot.time.*`。

## 共享脚手架（conftest.py）

`conftest.py` 提供以下可复用件，新测试直接 import 或以 fixture 注入，不要再各自复制：

| 名称 | 形式 | 用途 |
|------|------|------|
| `FakeBus` | 类 / `fake_bus` fixture | 静默 CAN 总线 double：吞掉 send，recv 恒返回 None |
| `make_mixed_chain()` / `mixed_chain` | 工厂 / fixture | 3 MotorA（J1–J3）+ 3 MotorB（J4–J6）的 `MixedMotorChain`，不接硬件 |
| `motor_a_frame()` / `motor_b_frame()` | 函数 | ENCOS / 达妙协议反馈帧 8 字节打包 |
| `can_msg()` | 函数 | 构造 `can.Message`，`is_rx=False` 表示命令回显帧 |
| `make_command_robot()` / `command_robot` | 工厂 / fixture | 无链/总线/重力模型的 `ArmRobot`，命令门已打开，用于命令入口与 clip 测试 |

脚手架自身的冒烟覆盖见 `test_scaffolding_smoke.py`；double 行为变动会先在那里暴露。

更完整的编写指南、分层说明与阶段门禁见仓库文档 `01-docs/04-sop/test-development-sop.md`。
