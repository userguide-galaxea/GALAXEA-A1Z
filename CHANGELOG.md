# Changelog

## [0.1.0] - 2026-08-03

### Features
- 合并 dev-premerge 的 MIT 控制更新到 dev_gripper（不含 analysis 优化/标定框架）：
  - `ArmRobot` 新增 Coulomb 摩擦前馈：tanh 平滑路径（`CoulombConfig`）与硬符号路径（裸 ndarray），加入力矩求和
  - `CoulombConfig` 落位 `a1z/robots/friction.py`（原在 `a1z/analysis/optimize/friction.py`，剔除 analysis 后移至控制侧）
  - `IntegralConfig.from_level` 新增 `t_wind_s` 连续覆盖与 `clamp_scale`（τ_I,max = clamp_scale·τ̂_c）
  - `get_robot.py` 默认 PD 参数更新为大样本重新优化结果（Kp=[146.90, 62.95, 89.24, 120, 40, 100]，Kd=[5, 5, 5, 2.08, 1.51, 1.26]）
  - J2 关节限位放开为 ±π（URDF 与 `_JOINT_LIMITS` 同步），修复下放起始姿态抬升时遥操作指令被拒

### Design Rationale
- dev-premerge 的 `a1z/analysis/`（optuna 优化、看门狗标定、验证脚本）不并入 dev_gripper，仅保留运行时 MIT 控制路径改动；`CoulombConfig` 是 `ArmRobot` 构造期依赖，故移至 `a1z/robots/` 而非留在 analysis
- 启动反馈 probe 重试逻辑两边为平行实现，保留 dev_gripper 的参数化版本（`_STARTUP_FEEDBACK_PROBE_ATTEMPTS/_SETTLE_S` 常量）

### Notes & Caveats
- 后续若再从本地 SDK 同步 `arm_robot.py`，其 `CoulombConfig` import 仍指向 `a1z.analysis.optimize.friction`，需重新改回 `a1z.robots.friction`
- `tests/test_command_clear_semantics.py` 有 8 个既有失败（合并前即存在），与本版本改动无关
