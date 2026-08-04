"""Cost-function specification and search-space constants for BO (SOP-11 §1–§2).

All hyper-parameters that affect cost *values* or search *behaviour* live in
``COST_SPEC_V1``.  Every trial meta records ``cost_spec_version``; every
``study.json`` snapshots the full spec dict.  **Changing a weight = bumping the
version string** so old studies are never contaminated.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Hardware constants (mirrored from get_robot.py / safety.py — kept literal
# to avoid importing CAN-level modules from an offline optimiser context).
# 2026-08-03: synced to the Phase-A BO gains frozen as SDK defaults on
# 2026-08-01 (get_robot.py._DEFAULT_KP/_DEFAULT_KD, devlog 2026-08-01
# 「Phase A 收官」).  Old pre-Phase-A defaults: kp [100, 60, 40, 30, 10, 25],
# kd [4.9, 4.5, 5.0, 2.0, 0.5, 4.0].
# ---------------------------------------------------------------------------
DEFAULT_KP = np.array([146.8988, 62.9454, 89.2416, 120.0, 40.0, 100.0])
DEFAULT_KD = np.array([5.0, 5.0, 5.0, 2.0776, 1.5059, 1.2553])
TAU_C_HAT = np.array([0.3442, 0.3665, 0.6371, 0.66, 0.143, 0.13])
TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])

# Equivalent diagonal inertia at the J6-precondition posture
# q = [0, 20, -20, 0, 0, 0] deg — computed via pinocchio.crba on
# A1Z_Flange.urdf (see devlog 2026-07-28 §3-2).
I_HAT = np.array([0.060028, 0.124217, 0.133801, 0.006245, 0.000137, 0.000016])

# ---------------------------------------------------------------------------
# Phase A search space  (SOP-11 §1.3)
# ---------------------------------------------------------------------------
KD_CAP = 5.0

# Per-joint kd floor: do not let the (kp, zeta_hat) reparameterisation drive
# low-inertia joints into near-zero damping.  J6 in particular can map
# zeta=0.40, kp=87 -> kd≈0.03, which caused a velocity-limit hard fault.
KD_MIN_FLOOR_FACTOR = 0.25   # keep at least 25% of default kd
KD_MIN_ABS = 0.3             # Nm/(rad/s); absolute lower bound
KD_MIN = np.maximum(DEFAULT_KD * KD_MIN_FLOOR_FACTOR, KD_MIN_ABS)

# Hardware velocity limits mirrored from ArmRobot._vel_limit (rad/s).
# Watchdog uses a fraction of these as an instantaneous absolute-velocity guard.
VEL_LIMIT_RAD_S = np.array([12.0, 12.0, 12.0, 7.0, 20.0, 20.0])
WATCHDOG_VEL_ABS_FACTOR = 0.5

def _phase_a_kp_range(j: int) -> Tuple[float, float]:
    kp0 = DEFAULT_KP[j]
    return (kp0 / 2.0, min(4.0 * kp0, 200.0))

def _phase_a_zeta_range() -> Tuple[float, float]:
    return (0.4, 1.2)

# ---------------------------------------------------------------------------
# Phase B search space  (SOP-11 §1.4)
# ---------------------------------------------------------------------------
def _phase_b_coulomb_range(j: int) -> Tuple[float, float]:
    c_ss = 1.5 * TAU_C_HAT[j]
    return (0.0, 2.0 * c_ss)  # 上界 3.0*tau_c_hat（v14 放宽：J1/J6 上界饱和）

PHASE_B_T_WIND_RANGE = (0.3, 3.0)      # log scale
PHASE_B_CLAMP_SCALE_RANGE = (0.8, 2.0)  # linear
PHASE_B_T_LEAK_RANGE = (0.5, 3.0)       # log scale

# §1.4 逐关节裁剪：积分维度仅对 τ̂_c 大且 ess 未达标的关节开启。
# v14（2026-08-03 B1 G0-ext 实测）：6/6 积分对 FAIL——E6 后 ess 已压进
# 积分死区（e_db=0.3°，J1/J5/J6 ess≈0.066°、J2 0.23°、J4 0.32°），积分无
# 靶子；唯一超死区的 J3（ess 0.48°）ess 仅 −1.4% 但 overshoot +261.7%、
# ts +314.6%（瞬态爬到 clamp 与 PD 相位裕度冲突）。按 §7.4 全关节移出；
# 复评条件：出现 >0.3° 持续偏置的场景（带载/接触）时重跑机制对。
PHASE_B_INTEGRAL_JOINTS: tuple = ()

# ---------------------------------------------------------------------------
# Cost function  (SOP-11 §2.1)
# ---------------------------------------------------------------------------
WEIGHTS_JOINT = np.array([0.30, 0.20, 0.20, 0.10, 0.20])

# Baselines: default-tuning metric values used for non-dimensionalisation.
# PROVISIONAL — populated from A2 归档 data; replace with B1 calibration
# data once available.  Per-joint baselines stored as shape-(6,) arrays;
# the active joint is selected at evaluation time.
# NOTE: keys are matched to metrics_v2 units: lag in deg, ts in ms,
# resid_std in deg, ess in deg, overshoot in percent.
BASELINE_LAG_DEG = np.full(6, 0.5)         # ~0.5 deg equivalent lag
BASELINE_TS_MS = np.full(6, 500.0)         # ~0.5 s settle time
BASELINE_RESID_STD_DEG = np.full(6, 0.5)   # ~0.5 deg residual std
BASELINE_ESS_DEG = np.full(6, 0.3)         # ~0.3 deg steady-state error
BASELINE_OVERSHOOT_PCT = np.full(6, 10.0)  # ~10% overshoot (2 deg step)

# --- Normalisation floors (devlog 2026-07-31 Q1) ---------------------------
# Denominator = max(baseline, floor), floor = NORM_FLOOR_K x cross-session
# sigma.  When a baseline measures ~0 (e.g. overshoot of a critically-damped
# joint), the term silently switches from a relative to an absolute scale
# instead of dividing by a near-zero number and drowning the weighted sum.
NORM_FLOOR_K = 3.0

# lag / resid_std: vf-口径 sigma(run) measured in A2
# (02-test-log/02-a1z/04-a2-velff-gate/a2-report.md §A2-2:
#  J6 lag 0.0037 deg / resid 0.0005 deg, J2 lag 0.0120 deg / resid 0.0021 deg),
# scaled by NORM_FLOOR_K per the A2-6 cross-session rule (cross ≈ 3x same).
# Joints without a measurement take the conservative max of the measured pair.
NORM_FLOOR_LAG_DEG = np.array([0.036, 0.036, 0.036, 0.036, 0.036, 0.011])
NORM_FLOOR_RESID_STD_DEG = np.array([0.0063, 0.0063, 0.0063, 0.0063, 0.0063, 0.0015])

# ts / ess / overshoot: no vf-口径 sigma measured yet -- PROVISIONAL absolute
# floors; backfill from A2/G0 archived step data at stage P2 (SOP-11 §14).
NORM_FLOOR_TS_MS = np.full(6, 50.0)
NORM_FLOOR_ESS_DEG = np.full(6, 0.03)
NORM_FLOOR_OVERSHOOT_PCT = np.full(6, 3.0)

# Per-term saturation: feasible-region terms need not be unbounded -- truly
# dangerous regions already pay PENALTY_COST via the watchdog.
TERM_CLIP = 20.0

PENALTY_COST = 1e4


def violation_surrogate(feasible_costs) -> float:
    """Objective value told to the GP for a watchdog-violated trial (v7).

    Feasibility travels via the constraint channel (GPSampler
    ``constraints_func`` → ConstrainedLogEI); the objective value only
    needs to keep the objective GP's fit on scale.  Policy comparison in
    devlog 2026-07-31 Q11: worst-observed-feasible preserves the valley
    structure best (3x max over-suppresses exploration; absolute 1e4 or
    raw partial costs destroy the standardised landscape).
    """
    import numpy as _np
    feas = [c for c in feasible_costs if c is not None and _np.isfinite(c)
            and c < PENALTY_COST]
    if not feas:
        return PENALTY_COST
    return min(PENALTY_COST, max(feas))


# GP observation noise reference — cross-session sigma (SOP-11 §2.5).
# DOCUMENTARY ONLY: Optuna 4.9 GPSampler exposes no noise-prior parameter
# (its internal Gamma noise prior is not configurable), so this value is
# recorded for traceability but is NOT wired into the sampler
# (devlog 2026-07-31 Q9 finding ②).
GP_NOISE_PRIOR = 0.05

# ---------------------------------------------------------------------------
# L0 fast-evaluation preset  (SOP-11 §6.2)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class L0Preset:
    tri_period: float = 4.0
    tri_cycles: int = 1
    tri_vel_ff: bool = True
    tri_amp_deg: float = 15.0
    step_amp_deg: float = 2.0
    step_events: int = 2
    step_period: float = 4.0
    step_cycles: int = 1
    hold_pre: float = 0.5
    hold_post: float = 0.5

L0 = L0Preset()

# ---------------------------------------------------------------------------
# L0 EE leg preset  (SOP-11 §12.1「+ EE 微步进」, cost_spec v10 — devlog
# 2026-08-01 E3): one small xz-circle cycle + a terminal hold at the anchor,
# reusing EETrackingRunner + the same dry-run gate.  The hold makes
# ee_terminal_error a true terminal-accuracy measure instead of a
# trailing-window average (devlog 2026-08-01 关键设计约束 3).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class L0EEPreset:
    plane: str = "xz"
    radius: float = 0.04          # m — same circle as the A3 baseline runs
    period: float = 8.0           # s per lap
    cycles: int = 1
    hold_s: float = 0.5           # terminal hold at the anchor pose
    settle_s: float = 1.0
    q_nom_deg: Tuple[float, ...] = (-20.0, 35.0, -25.0, -25.0, 0.0, 0.0)
    normal: Tuple[float, ...] = (0.0, 1.0, 0.0)   # xz-plane normal = y

    @property
    def tail_frac(self) -> float:
        """Fraction of the recorded trace covered by the terminal hold."""
        return self.hold_s / (self.cycles * self.period + self.hold_s)

L0EE = L0EEPreset()

# ---------------------------------------------------------------------------
# Aggregate spec (versioned, snapshotted into study.json)
# ---------------------------------------------------------------------------
# v9 (2026-08-01): hf_osc 通道增加阶跃沿豁免（watchdog.py TickWatchdog
# ``hf_step_exempt_deg`` / ``hf_step_exempt_s``）——v8 的 eff 差分 RMS 无法
# 区分方波沿的大力矩瞬态与持续振荡，session 2026-08-01-run-opt-phaseA-J6 的
# 4 次 hf_osc 违例全部是沿瞬态，导致「高 kp + 低 kd」对角带被约束通道整片
# 误标为不可行，BO 未收敛到实测更优的手调点（devlog 2026-08-01 归因分析）。
# 可行性判定语义改变 → 升版本，v8 及更早 session 不可 --resume 混用。
# v10 (2026-08-03): E 段 EE 合成代价落地（devlog 2026-08-01 计划 E2）——
# 新增 compute_ee_cost（SOP-11 §2.2 三项，基线取自 A3 Pdef 归档实测）与
# compute_total_cost（§2.3，0.65:0.35，A3-2 前提待 E7 补验）；
# DEFAULT_KP/DEFAULT_KD 同步为 2026-08-01 冻结的 Phase A BO 增益
#（get_robot.py SDK 默认值），KD_MIN 与 Phase A 搜索空间随动，J5/J6 退化
# 集合不变（单测锁定）。代价语义改变 → v9 及更早 session 不可 --resume 混用。
# v11 (2026-08-03): J6 hf_osc 通道禁用（v4 active 标定 JSON：
# hf_baseline_rms/theta_hf → null，primary_channel → vel_abs，与 J1–J5 一致）
# ——eeRefine-J456 session（v10 整臂 trial 增益 + kp6≤200）4/8 trial 三角腿
# 3 连击跳闸，RMS 0.088–0.089 全部落在 B1 标定模糊带
# (theta_hf=0.087, hi_min=0.0929) 内、低于全部激进组标定 run，且超出
# 2026-07-31 归档包线 envelope_max=0.0812：refine 运行条件（J4/J5 硬保持
# + 高 kp）把 J6 扫掠力矩纹波分布整体上移 ~10%，通道在新条件下与 J1–J5
# 同样不可分（违例/通过组 resp-ref p95 同为 ~0.4°）。残余覆盖：pos 15°
# 远场、eff 0.9×clip、vel_abs 0.5×硬件限速、trial 级 overshoot/jump 检查
# 与代价函数 jitter/overshoot 项；恢复条件 = refine 条件下 B1 式重标定
# （devlog 2026-08-03 归因，处置建议 2 的确认重测方案被本决策取代）。
# 可行性判定语义改变 → v10 及更早 session 不可 --resume 混用。
# v12 (2026-08-03): refine 空间按关节 regime 修正参数化——非退化关节
# (J1–J4) 从 (kp,kd) 直搜改回 (kp, ζ̂)（devlog 2026-08-01 §6 统一视角的
# 设计原则：惯量主导关节 kd 的自然尺度随 kp 动，裸 (kp,kd) 空间把
# 「高 kp + 低 kd」振荡角落（隐含 ζ<0.4）重新放进搜索空间——
# eeRefine-allJoints session t000–t002 连续 overshoot 违约（隐含
# ζ=0.18–0.33）即此所致）；退化关节 (J5/J6) 保持 (kp,kd) 直搜。
# 搜索行为改变 → v11 及更早 session 不可 --resume 混用。
# v13 (2026-08-03): Phase B 接线（devlog 2026-08-03 计划 B2）——搜索空间
# 重构为「kp 冻结 + coulomb_ff（恒开）+ 积分维度（仅
# PHASE_B_INTEGRAL_JOINTS=(3,4)，§1.4 逐关节裁剪，ess 复核属 B1）」，
# 移除旧 Phase B 空间中的 kp/zeta 维度与 t_leak 维度（默认不开，保持
# ≤3 维）；ArmRobot 新增 set_coulomb_config 运行时切换（与
# set_integral_config 同锁纪律）；Phase B session 目标 = J_total（关节
# 两腿 + EE 腿），种子 = 空间内最小干预点在线注入。
# 搜索行为改变 → v12 及更早 session 不可 --resume 混用。
# v14 (2026-08-03): B1 G0-ext 判定落实——积分维度整体淘汰
# （PHASE_B_INTEGRAL_JOINTS=()）：6/6 积分机制对 FAIL，ess 全部已压进
# 积分死区 e_db=0.3°（J1/J5/J6≈0.066°、J2 0.23°、J4 0.32°，纹丝不动或
# 变差），唯一超死区的 J3（0.48°）ess −1.4% 但 overshoot +261.7% /
# ts +314.6%。「积分提高精度」的前提是存在 >死区的持续偏置，E6 后该
# 前提不成立；复评条件 = 出现 >0.3° 持续偏置场景后重跑机制对。
# coulomb 侧 J1/J5/J6 PASS 准进；J2/J3/J4 方向成立（lag −53%~−76%）但
# 副作用越闸（resid +37.6% / overshoot +125% / +33.3%）→ 上界收紧待
# hat:0.5 重测定值（devlog 2026-08-03「B1 判定归因」）。
# 搜索行为改变 → v13 及更早 session 不可 --resume 混用。
COST_SPEC_VERSION = "v14"


def cost_spec_snapshot() -> dict:
    """Full serialisable snapshot for study.json reproducibility."""
    return {
        "version": COST_SPEC_VERSION,
        "weights_joint": WEIGHTS_JOINT.tolist(),
        "baselines": {
            "lag_deg": BASELINE_LAG_DEG.tolist(),
            "ts_ms": BASELINE_TS_MS.tolist(),
            "resid_std_deg": BASELINE_RESID_STD_DEG.tolist(),
            "ess_deg": BASELINE_ESS_DEG.tolist(),
            "overshoot_pct": BASELINE_OVERSHOOT_PCT.tolist(),
        },
        "norm_floors": {
            "k": NORM_FLOOR_K,
            "lag_deg": NORM_FLOOR_LAG_DEG.tolist(),
            "ts_ms": NORM_FLOOR_TS_MS.tolist(),
            "resid_std_deg": NORM_FLOOR_RESID_STD_DEG.tolist(),
            "ess_deg": NORM_FLOOR_ESS_DEG.tolist(),
            "overshoot_pct": NORM_FLOOR_OVERSHOOT_PCT.tolist(),
        },
        "term_clip": TERM_CLIP,
        "penalty_cost": PENALTY_COST,
        "violation_surrogate": "max(feasible_costs), capped at PENALTY_COST",
        # v10: EE-layer cost (compute_ee_cost) and composite (compute_total_cost).
        "weights_ee": WEIGHTS_EE.tolist(),
        "ee_terminal_pos_share": EE_TERMINAL_POS_SHARE,
        "baselines_ee": {
            "terminal_pos_mm": BASELINE_EE_TERMINAL_POS_MM,
            "terminal_ang_deg": BASELINE_EE_TERMINAL_ANG_DEG,
            "jitter_std_mm": BASELINE_EE_JITTER_STD_MM,
            "phase_lag_ms": BASELINE_EE_PHASE_LAG_MS,
        },
        "norm_floors_ee": {
            "terminal_pos_mm": NORM_FLOOR_EE_TERMINAL_POS_MM,
            "terminal_ang_deg": NORM_FLOOR_EE_TERMINAL_ANG_DEG,
            "jitter_std_mm": NORM_FLOOR_EE_JITTER_STD_MM,
            "phase_lag_ms": NORM_FLOOR_EE_PHASE_LAG_MS,
        },
        "w_total": {"ee": W_EE_TOTAL, "jt": W_JT_TOTAL},
        "base_total": {"ee": BASE_TOTAL_EE, "jt": BASE_TOTAL_JT},
        # v9: hf_osc step-edge exemption (see watchdog.py TickWatchdog).
        "hf_step_exempt_deg": 1.0,
        "hf_step_exempt_s": 0.6,
        # v11: J6 hf_osc channel disabled in the v4 active calib JSON
        # (2026-08-03 refine session 归因: sweep-ripple distribution shifted
        # out of the B1 calibration envelope under whole-arm refine gains;
        # channel inseparable, same as J1–J5).  pos/eff/vel_abs remain.
        "j6_hf_channel": "disabled",
        # v12: refine space parameterisation is per-joint — (kp, zeta_hat)
        # for non-degenerate joints (J1–J4), (kp, kd) for degenerate (J5/J6).
        "refine_param": "(kp,zeta_hat) non-degenerate / (kp,kd) degenerate",
        # v13: Phase B space — kp frozen; coulomb_ff always; integral dims
        # only for PHASE_B_INTEGRAL_JOINTS (§1.4 cropping).
        "phase_b_integral_joints": list(PHASE_B_INTEGRAL_JOINTS),
        "phase_b_ranges": {
            "coulomb_ff": "per-joint [0, 3.0*tau_c_hat] linear",
            "t_wind_s": list(PHASE_B_T_WIND_RANGE),
            "clamp_scale": list(PHASE_B_CLAMP_SCALE_RANGE),
            "t_leak_s": list(PHASE_B_T_LEAK_RANGE) + ["(default off)"],
        },
        # Joints whose (kp, zeta_hat) -> kd mapping collapses onto KD_MIN
        # for the whole space (devlog 2026-07-31 Q16): these search
        # (kp, kd) directly instead of (kp, zeta_hat).
        "kd_direct_search_joints": [
            j + 1 for j in range(6)
            if 2.0 * _phase_a_zeta_range()[1]
            * math.sqrt(_phase_a_kp_range(j)[1] * I_HAT[j]) < KD_MIN[j]
        ],
        "gp_noise_prior": GP_NOISE_PRIOR,
        "kd_cap": KD_CAP,
        "kd_min": KD_MIN.tolist(),
        "kd_min_floor_factor": KD_MIN_FLOOR_FACTOR,
        "kd_min_abs": KD_MIN_ABS,
        "vel_limit_rad_s": VEL_LIMIT_RAD_S.tolist(),
        "watchdog_vel_abs_factor": WATCHDOG_VEL_ABS_FACTOR,
        "I_hat": I_HAT.tolist(),
        "default_kp": DEFAULT_KP.tolist(),
        "default_kd": DEFAULT_KD.tolist(),
        "tau_c_hat": TAU_C_HAT.tolist(),
        "torque_clip": TORQUE_CLIP.tolist(),
        "l0": {
            "tri_period": L0.tri_period,
            "tri_cycles": L0.tri_cycles,
            "tri_vel_ff": L0.tri_vel_ff,
            "tri_amp_deg": L0.tri_amp_deg,
            "step_amp_deg": L0.step_amp_deg,
            "step_events": L0.step_events,
            "step_period": L0.step_period,
            "step_cycles": L0.step_cycles,
        },
        "l0_ee": {
            "plane": L0EE.plane,
            "radius": L0EE.radius,
            "period": L0EE.period,
            "cycles": L0EE.cycles,
            "hold_s": L0EE.hold_s,
            "settle_s": L0EE.settle_s,
            "q_nom_deg": list(L0EE.q_nom_deg),
            "normal": list(L0EE.normal),
        },
    }


def compute_joint_cost(
    j: int,
    lag_deg: float,
    ts_ms: float,
    resid_std_deg: float,
    ess_deg: float,
    overshoot_pct: float = 0.0,
) -> Tuple[float, Dict[str, float]]:
    """Weighted sum J_joint (SOP-11 §2.1).  Returns (cost, breakdown).

    Inputs are in metrics_v2 native units (deg / ms / percent), matching the
    corresponding baselines.  Each term is normalised by
    ``max(baseline, norm_floor)`` and saturated at ``TERM_CLIP``
    (devlog 2026-07-31 Q1).
    """
    w = WEIGHTS_JOINT
    terms = np.array([
        lag_deg / max(BASELINE_LAG_DEG[j], NORM_FLOOR_LAG_DEG[j]),
        ts_ms / max(BASELINE_TS_MS[j], NORM_FLOOR_TS_MS[j]),
        resid_std_deg / max(BASELINE_RESID_STD_DEG[j], NORM_FLOOR_RESID_STD_DEG[j]),
        ess_deg / max(BASELINE_ESS_DEG[j], NORM_FLOOR_ESS_DEG[j]),
        overshoot_pct / max(BASELINE_OVERSHOOT_PCT[j], NORM_FLOOR_OVERSHOOT_PCT[j]),
    ])
    terms = np.clip(terms, 0.0, TERM_CLIP)
    cost = float(w @ terms)
    breakdown = {
        "lag": float(w[0] * terms[0]),
        "ts": float(w[1] * terms[1]),
        "resid": float(w[2] * terms[2]),
        "ess": float(w[3] * terms[3]),
        "overshoot": float(w[4] * terms[4]),
    }
    return cost, breakdown


# ---------------------------------------------------------------------------
# EE-layer cost  (SOP-11 §2.2, cost_spec v10 — devlog 2026-08-01 E2)
# ---------------------------------------------------------------------------
# Baselines: measured on the A3 Pdef run (pre-Phase-A SDK defaults), archived
# xz-circle r=40mm T=8s×2cyc EE trajectory
# (02-test-log/02-a1z/01-output/2026-08-01-run-A3-Pdef-ee-r1-ee-01,
# tail_frac=0.1 — devlog 2026-08-01 E1 冒烟).
BASELINE_EE_TERMINAL_POS_MM = 17.42
BASELINE_EE_TERMINAL_ANG_DEG = 2.00
BASELINE_EE_JITTER_STD_MM = 1.395
BASELINE_EE_PHASE_LAG_MS = 166.5

# PROVISIONAL floors = 10% of the measured Pdef→PphA gap (the smallest tuning
# difference each metric demonstrably resolves; PphA archive
# 2026-08-01-run-A3-PphA-ee-r1-ee-01: 10.47mm / 1.21deg / 1.260mm / 110.2ms).
# All far below the baselines, hence inactive in practice; backfill with the
# NORM_FLOOR_K × cross-session σ rule once refine-session anchor repeats
# provide real σ (E7).  Changing floors = new version.
NORM_FLOOR_EE_TERMINAL_POS_MM = 0.70
NORM_FLOOR_EE_TERMINAL_ANG_DEG = 0.08
NORM_FLOOR_EE_JITTER_STD_MM = 0.014
NORM_FLOOR_EE_PHASE_LAG_MS = 5.6

# v1 (到位误差) : v2 (法向抖动 std) : v3 (相位滞后) — terminal error dominant
# (devlog 2026-08-01 E2 初值).  Inside v1 the position/orientation parts are
# combined 50/50 after individual normalisation.
WEIGHTS_EE = np.array([0.40, 0.30, 0.30])
EE_TERMINAL_POS_SHARE = 0.5

# Composite cost (SOP-11 §2.3): J_total = w_ee·J_ee/base + w_jt·J_joint/base.
# PREMISE UNDER REVIEW: the 0.65 EE dominance presumes A3-2
# (ρ(J_ee, task) > ρ(J_joint, task)), which has NOT been rank-tested
# (devlog 2026-08-01「与 SOP-11 §5 的偏离」); E7 back-computes ρ and a
# fallback to joint-dominant weights is the documented escape hatch.
W_EE_TOTAL = 0.65
W_JT_TOTAL = 0.35
# J_ee and J_joint are each already normalised to ≈1 at their reference
# tunings, so the §2.3 aggregate bases are 1.0 unless E6 anchor data
# justifies a different reference (change = new version).
BASE_TOTAL_EE = 1.0
BASE_TOTAL_JT = 1.0


def compute_ee_cost(j_ee: Dict[str, float]) -> Tuple[float, Dict[str, float]]:
    """Weighted sum J_ee (SOP-11 §2.2).  Returns (cost, breakdown).

    Input is the :func:`a1z.analysis.metrics.ee_refine_metrics` dict
    (``terminal_pos_mm`` / ``terminal_ang_deg`` / ``normal_jitter_std_mm`` /
    ``phase_lag_ms``).  Same normalisation as the joint layer:
    ``max(baseline, floor)`` denominators and ``TERM_CLIP`` saturation.
    """
    terms = np.array([
        j_ee["terminal_pos_mm"]
        / max(BASELINE_EE_TERMINAL_POS_MM, NORM_FLOOR_EE_TERMINAL_POS_MM),
        j_ee["terminal_ang_deg"]
        / max(BASELINE_EE_TERMINAL_ANG_DEG, NORM_FLOOR_EE_TERMINAL_ANG_DEG),
        j_ee["normal_jitter_std_mm"]
        / max(BASELINE_EE_JITTER_STD_MM, NORM_FLOOR_EE_JITTER_STD_MM),
        j_ee["phase_lag_ms"]
        / max(BASELINE_EE_PHASE_LAG_MS, NORM_FLOOR_EE_PHASE_LAG_MS),
    ])
    terms = np.clip(terms, 0.0, TERM_CLIP)
    w_term = WEIGHTS_EE[0]
    w_jit = WEIGHTS_EE[1]
    w_lag = WEIGHTS_EE[2]
    breakdown = {
        "terminal_pos": float(w_term * EE_TERMINAL_POS_SHARE * terms[0]),
        "terminal_ang": float(w_term * (1.0 - EE_TERMINAL_POS_SHARE) * terms[1]),
        "jitter": float(w_jit * terms[2]),
        "lag": float(w_lag * terms[3]),
    }
    return float(sum(breakdown.values())), breakdown


def compute_total_cost(
    j_joint: float,
    j_ee: float,
) -> Tuple[float, Dict[str, float]]:
    """Composite cost J_total (SOP-11 §2.3).  Returns (cost, breakdown)."""
    ee_term = W_EE_TOTAL * j_ee / BASE_TOTAL_EE
    jt_term = W_JT_TOTAL * j_joint / BASE_TOTAL_JT
    return ee_term + jt_term, {"ee": float(ee_term), "joint": float(jt_term)}
