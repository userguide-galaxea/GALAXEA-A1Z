"""Per-joint error-integral feedforward (leaky integrator) for the A1Z arm.

策略层 S2「误差积分前馈」的独立实现（SOP-09 §3）。纯 numpy、无硬件依赖，可单测。
主机侧 250 Hz 外环，输出 τ_I 并入 `_update()` 的力矩合成（clip 前），电机侧零改动。

控制律（SOP-09 §3 / devlog 2026-07-22 Q8(2)）：

    τ_I[k] = clamp(λ·τ_I[k-1] + ki·e[k]·Δt, ±τ_I,max),   e[k] = q_des[k] − q_meas[k]

防 hunting 四件套（缺一不上机）：
    ① 钳位   τ_I,max = 1.2·τ̂_c（逐关节）
    ② 泄漏   λ = 1 − Δt/T_leak（静止后积分自动衰减）
    ③ 误差死区 |e| < e_db 本拍只泄漏不积分（防量化涟漪打摆）
    ④ 调度   |q̇_des| > q̇_freeze 冻结积分；指令方向反转 τ_I ← τ_I/2
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional

import numpy as np

# 档位定标的典型稳态误差（§3：e_typ = 0.5° ≈ 0.0087 rad）。
E_TYP_RAD = float(np.deg2rad(0.5))

# 档位 → 爬满钳位时间 t_wind (s)；K0 = 关断（ki=0，结构上等价于无积分器）。
LEVELS: Dict[str, Optional[float]] = {"K0": None, "K1": 2.0, "K2": 0.5, "K3": 0.2}


def _joints_mask(joints: Optional[Iterable[int]], n: int) -> np.ndarray:
    """1-based 关节号 → (n,) bool mask。None → 全 True。"""
    if joints is None:
        return np.ones(n, dtype=bool)
    mask = np.zeros(n, dtype=bool)
    for j in joints:
        idx = int(j) - 1
        if not (0 <= idx < n):
            raise ValueError(f"integral joint {j} out of range 1..{n}")
        mask[idx] = True
    return mask


@dataclass
class IntegralConfig:
    """逐关节积分器配置（生效全向量即 meta 回读的唯一来源，SOP-09 P0-4）。"""

    ki: np.ndarray                                       # (n,) Nm/(rad·s)；0 = 该关节关断
    tau_i_max: np.ndarray                                # (n,) Nm = 1.2·τ̂_c（关断关节置 0）
    e_db_rad: np.ndarray                                 # (n,) 误差死区（默认 0.3°）
    t_leak_s: float = 1.0                                # λ = 1 − Δt/T_leak
    qd_freeze: float = 0.15                              # rad/s：|q̇_des| 超过即冻结积分
    level: str = "K0"                                    # 档位名（记账用）

    def __post_init__(self) -> None:
        self.ki = np.asarray(self.ki, dtype=float)
        self.tau_i_max = np.asarray(self.tau_i_max, dtype=float)
        self.e_db_rad = np.asarray(self.e_db_rad, dtype=float)
        n = self.ki.shape[0]
        if self.tau_i_max.shape[0] != n or self.e_db_rad.shape[0] != n:
            raise ValueError("ki / tau_i_max / e_db_rad length mismatch")
        # 关断关节的钳位边界必须是有限值，否则 np.clip 会把 τ_I 污染成 NaN。
        if np.any(~np.isfinite(self.tau_i_max)):
            raise ValueError(
                "tau_i_max has non-finite entries; disabled joints must be 0 "
                "(use IntegralConfig.from_level which zeroes them)"
            )

    @classmethod
    def from_level(
        cls,
        level: str,
        tau_c_hat: np.ndarray,
        joints: Optional[Iterable[int]] = None,
        t_leak_s: float = 1.0,
        e_db_deg: float = 0.3,
        qd_freeze: float = 0.15,
    ) -> "IntegralConfig":
        """按档位构造配置。

        ki = τ_I,max / (E_TYP_RAD · t_wind)；τ̂_c 为 NaN 或不在 ``joints`` 内的
        关节 ki=0 且 τ_I,max=0（enable mask 的实现形式）。K0 → 全零（等价关断）。
        """
        if level not in LEVELS:
            raise ValueError(f"unknown integral level {level!r}; choose from {list(LEVELS)}")
        tau_c_hat = np.asarray(tau_c_hat, dtype=float)
        n = tau_c_hat.shape[0]

        enabled = np.isfinite(tau_c_hat) & _joints_mask(joints, n)
        tau_i_max = np.where(enabled, 1.2 * tau_c_hat, 0.0)

        t_wind = LEVELS[level]
        if t_wind is None:  # K0
            ki = np.zeros(n)
        else:
            ki = np.where(enabled, tau_i_max / (E_TYP_RAD * t_wind), 0.0)

        # Keep ki / tau_i_max / enable_mask consistent: a joint with ki==0 (K0,
        # or unstandardised / out-of-scope) carries no clamp either, so meta
        # read-back never shows a live τ_I,max next to a dead ki.
        tau_i_max = np.where(ki != 0, tau_i_max, 0.0)

        return cls(
            ki=ki,
            tau_i_max=tau_i_max,
            e_db_rad=np.full(n, float(np.deg2rad(e_db_deg))),
            t_leak_s=float(t_leak_s),
            qd_freeze=float(qd_freeze),
            level=level,
        )

    def as_info(self) -> Dict[str, Any]:
        """生效全向量（get_robot_info()["integral"] / meta 回读的唯一来源）。"""
        return {
            "level": self.level,
            "ki": self.ki.tolist(),
            "tau_i_max": self.tau_i_max.tolist(),
            "t_leak_s": self.t_leak_s,
            "e_db_deg": np.round(np.rad2deg(self.e_db_rad), 4).tolist(),
            "qd_freeze": self.qd_freeze,
            "enable_mask": (self.ki != 0).tolist(),
        }


class JointErrorIntegrator:
    """τ_I[k] = clamp(λ·τ_I[k-1] + ki·e[k]·Δt, ±τ_I,max)，四件套齐装（SOP-09 §3）。"""

    def __init__(self, cfg: IntegralConfig, dt: float) -> None:
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        self.cfg = cfg
        self.dt = float(dt)
        # ② 泄漏系数：λ = 1 − Δt/T_leak，钳到 [0, 1]（T_leak < Δt 的退化情形不放大）。
        self._lam = float(np.clip(1.0 - self.dt / cfg.t_leak_s, 0.0, 1.0))
        self._n = cfg.ki.shape[0]
        self._enabled = cfg.ki != 0
        self.reset()

    def reset(self) -> None:
        """清零积分状态（非流式入口 / estop / 换配置时调用）。"""
        self._tau_i = np.zeros(self._n)
        self._last_qd_sign = np.zeros(self._n)

    def step(self, e: np.ndarray, qd_des: np.ndarray) -> np.ndarray:
        """推进一拍，返回 τ_I 副本。e = q_des − q_meas（URDF 系），qd_des = 指令速度。"""
        e = np.asarray(e, dtype=float)
        qd_des = np.asarray(qd_des, dtype=float)

        # ④ 反转：指令速度方向翻转的关节 τ_I ← τ_I/2（换向拖尾抑制）。
        cur_sign = np.sign(qd_des)
        flipped = (cur_sign != 0) & (self._last_qd_sign != 0) & (cur_sign != self._last_qd_sign)
        self._tau_i[flipped] *= 0.5
        # 只在有确定符号时更新记忆，保留上一非零符号（静止段不误判反转）。
        self._last_qd_sign = np.where(cur_sign != 0, cur_sign, self._last_qd_sign)

        # ① 泄漏：静止后积分自动衰减（对所有关节恒生效）。
        self._tau_i *= self._lam

        # ③ 死区 + ④ 冻结：|e| < e_db 或 |q̇_des| > q̇_freeze 的关节本拍只泄漏不积分。
        freeze = np.abs(qd_des) > self.cfg.qd_freeze
        deadzone = np.abs(e) < self.cfg.e_db_rad
        integrate = self._enabled & (~freeze) & (~deadzone)
        self._tau_i[integrate] += self.cfg.ki[integrate] * e[integrate] * self.dt

        # ② 钳位：越界风险机制上封死。
        np.clip(self._tau_i, -self.cfg.tau_i_max, self.cfg.tau_i_max, out=self._tau_i)
        return self._tau_i.copy()

    @property
    def tau_i(self) -> np.ndarray:
        return self._tau_i.copy()
