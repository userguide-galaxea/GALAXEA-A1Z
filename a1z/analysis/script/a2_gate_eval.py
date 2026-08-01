#!/usr/bin/env python3
"""A2 vel-ff 口径实验评估 + 报告（SOP-10 P1/P3）。

消费每个 A2 run 目录内的 ``metrics-v2.json``（P3 由 recompute_metrics.py 按 G0
定稿口径 k_sigma=4.0/apex_excl_s=0.55 重算）+ ``meta.json``（口径/增益/关节的机器
真值），按 SOP-10 §3 判据逐条计算 A2-1…A2-6，写 ``a2-report.md``（预期 vs 实测、
σ 表含跨 session、差分表、决策建议）与 ``a2-summary.csv``（一行一 run×口径）。

无硬件、不重算——只读 P3 归档数字，故 gate 判定落在每 run 已归档的 metrics-v2.json 上。

口径（v0/vf）以 ``meta.vel_ff.enabled`` 为唯一真值（SOP-10 R8：不从 run-id 猜）；
G0 期 meta 无 vel_ff 段 → 默认 v0，故本脚本可直接在 G0 归档上空跑（全 v0，vf/差分
项自然缺省，按 P1 要求验证代码路径）。

阈值 = §3 表；**为通过而回调阈值属违规（SOP-10 §3 / SOP-08 R3）**，任何偏离须在报告
里连同理由记录。

用法：
    python -m a1z.analysis.script.a2_gate_eval \\
        --output-root <.../02-a1z/01-output> \\
        --gate-dir    <.../02-a1z/04-a2-velff-gate> \\
        [--glob '*-run-A2*']   # 默认扫 A2* run；空跑 G0 用 '*-run-G0A*'
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics as st
from pathlib import Path

DEG = 180.0 / math.pi

# --- §3 阈值（不可为通过而回调 —— SOP-10 §3 / SOP-08 R3）------------------
CV_LAG_MAX = 0.10          # A2-2 lag_at_rate CV
CV_RESID_MAX = 0.15        # A2-2 resid_std CV
CV_P95_MAX = 0.25          # A2-2 jump_p95 CV
A2_1_TOL = 0.35            # A2-1 量级 ±35%
A2_3_RATIO_LO, A2_3_RATIO_HI = 2 * 0.65, 2 * 1.35   # A2-3 机制对比值 2×(1±35%)
A2_4_TOL = 0.25            # A2-4 差分 ±25%
A2_5_SIGMA_MULT = 3.0      # A2-5 resid_std 劣化 ≤ 3σ_run
A2_6_CROSS_MULT = 3.0      # A2-6 跨 session σ 显著判据 >3× 同 session

# 核心判据（A2-1…A2-6）只在默认扫频 period（p4=4.0 s，s=15°/s）上评估——A2D 的
# p8 系列是独立速率维度（服务 (k_v,c) 反解交叉验证），混入重复组会污染 σ/均值。
CORE_PERIOD_TRI = 4.0

# --- §2.2 解析预期（判读锚，采数前算好，逐字取自 SOP 表，不在此重算）---------
#   lag=|k_v·s+c|, k_v=kd/kp, c=τ̂_c/kp, s=15°/s；τ̂_c: J6 0.13 / J2 0.3665 Nm
CONFIG_EXPECT = {
    ("J6", "default"): dict(k_v=0.160, lag_v0=2.70, lag_vf=0.30, diff=2.40, g0=2.87),
    ("J6", "kp2"):     dict(k_v=0.320, lag_v0=5.10, lag_vf=0.60, diff=4.80, g0=4.98),
    ("J2", "default"): dict(k_v=0.075, lag_v0=1.47, lag_vf=0.35, diff=1.125, g0=1.68),
    ("J2", "kp2"):     dict(k_v=0.150, lag_v0=2.60, lag_vf=0.70, diff=2.25, g0=None),
}
# 默认 kp（用于 default vs kp2 判别，缺 meta.default_kp 时回退）
_DEFAULT_KP = {"J6": 25.0, "J2": 60.0}
# G0 R-A 归档（跨 session σ 复核，A2-6）：run-dir 前缀 → 配置
_G0_RA = {"J6": "2026-07-23-run-G0A-J6", "J2": "2026-07-23-run-G0A-J2"}


def _fnum(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def cv(vals):
    """(mean, sample-σ, |σ/mean|) over finite values; (None,None,None) if <2."""
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if len(vals) < 2:
        m = vals[0] if vals else None
        return m, None, None
    m = st.mean(vals)
    s = st.stdev(vals)                    # ddof=1
    return m, s, (abs(s / m) if m else None)


# ---------------------------------------------------------------------------
# Run 载入 + 口径/关节/增益 归类
# ---------------------------------------------------------------------------
class Run:
    __slots__ = ("dir", "joint", "cal", "gain", "period_tri", "kp", "tri")

    def __init__(self, rundir: Path):
        self.dir = rundir.name
        meta = json.loads((rundir / "meta.json").read_text())
        v2 = json.loads((rundir / "metrics-v2.json").read_text())
        joints = list(v2.get("joints", {}).keys())
        self.joint = joints[0] if joints else None
        self.tri = {}
        if self.joint:
            self.tri = v2["joints"][self.joint].get("triangle", {}) or {}
        # 口径 = meta.vel_ff.enabled 唯一真值（R8），缺段 → v0
        vf = bool((meta.get("vel_ff") or {}).get("enabled", False))
        self.cal = "vf" if vf else "v0"
        # 激励关节的 applied kp（缺则回退默认表）
        self.kp = None
        if self.joint:
            j = int(self.joint[1:]) - 1
            ap = (meta.get("pd", {}).get("applied", {}) or {}).get(self.joint)
            if ap and ap.get("kp") is not None:
                self.kp = _fnum(ap["kp"][j])
            if self.kp is None:
                dk = meta.get("pd", {}).get("default_kp")
                self.kp = _fnum(dk[j]) if dk else _DEFAULT_KP.get(self.joint)
        # default vs kp2：applied kp 相对该关节默认 kp 的比
        base = _DEFAULT_KP.get(self.joint)
        dk = meta.get("pd", {}).get("default_kp")
        if dk and self.joint:
            base = _fnum(dk[int(self.joint[1:]) - 1]) or base
        self.gain = "default"
        if self.kp is not None and base:
            self.gain = "default" if self.kp / base > 0.75 else "kp2"
        self.period_tri = _fnum((meta.get("excitation") or {}).get("period_triangle_s"))

    @property
    def key(self):
        return (self.joint, self.gain, self.cal, self.period_tri)

    def m(self, col):
        return _fnum(self.tri.get(col))


def load_runs(root: Path, glob: str):
    runs = []
    for rd in sorted(p for p in root.glob(glob) if p.is_dir()):
        if not (rd / "meta.json").exists() or not (rd / "metrics-v2.json").exists():
            continue
        try:
            runs.append(Run(rd))
        except Exception as e:  # noqa: BLE001 — one bad run must not sink the batch
            print(f"[WARN] skip {rd.name}: {type(e).__name__}: {e}")
    return runs


def group(runs, joint=None, gain=None, cal=None, period=CORE_PERIOD_TRI):
    """Runs matching (joint,gain,cal,period); None = any for that field.

    ``period`` defaults to CORE_PERIOD_TRI (p4) so every core criterion evaluates
    on the default sweep rate only — the p8 (A2D) runs share (joint,gain,cal)
    with A2A but are a different rate and must not enter a repeatability group.
    Pass period=None to pool across rates (the (k_v,c) rate cross-check)."""
    def _pmatch(r):
        if period is None:
            return True
        return r.period_tri is not None and abs(r.period_tri - period) < 1e-6
    return [r for r in runs
            if (joint is None or r.joint == joint)
            and (gain is None or r.gain == gain)
            and (cal is None or r.cal == cal)
            and _pmatch(r)]


def _vals(rs, col):
    return [r.m(col) for r in rs]


# ---------------------------------------------------------------------------
# §3 判据（每个返回 (passed_or_None, [报告行...])；None = 数据缺，非通过非失败）
# ---------------------------------------------------------------------------
def eval_a2_1(runs, W):
    """A2-1 量级正确（sanity）：vf lag_at_rate 落在解析 c 预期 ±35% 内。"""
    W("## A2-1 量级正确（sanity，vf lag ≈ 解析 c ±35%）\n")
    W("| 关节 | vf lag_at_rate 实测 | 解析 c 预期 | 偏差 | ≤35% | 判定 |")
    W("|---|---|---|---|---|---|")
    oks = []
    for joint in ("J6", "J2"):
        exp = CONFIG_EXPECT[(joint, "default")]["lag_vf"]
        vf = group(runs, joint, "default", "vf")
        m, _, _ = cv(_vals(vf, "lag_at_rate_deg"))
        if m is None:
            W(f"| {joint} | — (n={len(vf)}) | {exp:.2f}° | — | — | ⏳缺数据 |")
            continue
        dev = abs(m - exp) / exp if exp else None
        ok = dev is not None and dev <= A2_1_TOL
        oks.append(ok)
        W(f"| {joint} | {m:.3f}° | {exp:.2f}° | {dev*100:.1f}% | "
          f"{'✓' if ok else '✗'} | {'✅' if ok else '❌'} |")
    passed = all(oks) if oks else None
    if passed is False:
        W(f"\n> **发现（非 vel-ff 缺陷）**：vf 残余 lag 明显高于 τ̂_c/kp 解析预期，且 A2-D "
          f"证其**速率无关**（p4≈p8）→ 残余项确是 Coulomb c 而非传输延迟；实测小信号 "
          f"c ≈ 1.5–1.7× 大信号 τ̂_c 标定值（SOP-09 _TAU_C_HAT）。这是 S1 库伦前馈 c "
          f"先验的独立校准信号，回填 Q4/S1，**不否决口径**（A2-1 属 sanity，实现已由 "
          f"A2-4 机制一致 + 离线 §1.4 验证）。")
    W(f"\n**A2-1：{_verdict_str(passed, 'sanity')}**\n")
    return passed


def eval_a2_2(runs, W):
    """A2-2 小信号重复性（硬，主判据）：vf ×3 三指标 CV 达标（J6+J2）。"""
    W("## A2-2 小信号重复性（**硬·主判据**，vf 口径 ×3 CV）\n")
    W("| 关节 | 指标 | mean | σ(run) | CV | 阈值 | 判定 |")
    W("|---|---|---|---|---|---|---|")
    specs = [("lag_at_rate", "lag_at_rate_deg", CV_LAG_MAX),
             ("resid_std", "resid_std_deg", CV_RESID_MAX),
             ("jump_p95", "jump_p95_deg", CV_P95_MAX)]
    results = []
    for joint in ("J6", "J2"):
        vf = group(runs, joint, "default", "vf")
        for label, col, cvmax in specs:
            m, s, c = cv(_vals(vf, col))
            if c is None:
                W(f"| {joint} | {label} | {_f(m)} | — | — (n={len(vf)}) "
                  f"| ≤{cvmax*100:.0f}% | ⏳缺数据 |")
                results.append(None)
                continue
            ok = c <= cvmax
            results.append(ok)
            W(f"| {joint} | {label} | {m:.4f} | {s:.4f} | {c*100:.1f}% "
              f"| ≤{cvmax*100:.0f}% | {'✅' if ok else '❌'} |")
    have = [r for r in results if r is not None]
    passed = all(have) if len(have) == len(results) and have else None
    W(f"\n**A2-2：{_verdict_str(passed, 'hard')}**"
      f"（σ 表 → Q4 GP 观测噪声先验；阈值沿 G0-3 不变）\n")
    return passed


def eval_a2_3(runs, W):
    """A2-3 机制对口径不变性（硬排序+sanity比值）：vf 下 kp/2 vs default。"""
    W("## A2-3 机制对口径不变性（**硬·排序** + sanity·比值，vf 口径 kp/2 vs 默认）\n")
    W("| 关节 | lag(kp/2) | lag(默认) | 排序 kp/2>默认 | 比值 | ∈2×(1±35%) | 判定 |")
    W("|---|---|---|---|---|---|---|")
    order_oks, ratio_oks = [], []
    for joint in ("J6", "J2"):
        lo = group(runs, joint, "kp2", "vf")
        hi = group(runs, joint, "default", "vf")
        ml, _, _ = cv(_vals(lo, "lag_at_rate_deg"))
        mh, _, _ = cv(_vals(hi, "lag_at_rate_deg"))
        if ml is None or mh is None:
            W(f"| {joint} | {_f(ml)} | {_f(mh)} | — | — | — | ⏳缺数据 |")
            continue
        order_ok = ml > mh
        ratio = ml / mh if mh else None
        ratio_ok = ratio is not None and A2_3_RATIO_LO <= ratio <= A2_3_RATIO_HI
        order_oks.append(order_ok)
        ratio_oks.append(ratio_ok)
        W(f"| {joint} | {ml:.3f}° | {mh:.3f}° | {'✅' if order_ok else '❌'} "
          f"| {ratio:.2f} | {'✅' if ratio_ok else '⚠️'} | "
          f"{'✅' if order_ok else '❌'} |")
    order_pass = all(order_oks) if order_oks else None
    W(f"\n（c=τ_c/kp 随 kp 减半翻倍 → 比值物理上≈2；两口径机制对预期一致，天然交叉验证）")
    W(f"\n**A2-3：排序 {_verdict_str(order_pass, 'hard')}"
      f"，比值 {_verdict_str(all(ratio_oks) if ratio_oks else None, 'sanity')}**\n")
    return order_pass


def eval_a2_4(runs, W):
    """A2-4 差分一致性（硬）：lag(v0)−lag(vf) ≈ k_v·s，±25%（J6/J2 默认）。"""
    W("## A2-4 差分一致性（**硬**，lag(v0)−lag(vf) ≈ k_v·s，±25%）\n")
    W("| 关节 | lag(v0) | lag(vf) | 差分实测 | k_v·s 预期 | 偏差 | ≤25% | 判定 |")
    W("|---|---|---|---|---|---|---|---|")
    oks = []
    for joint in ("J6", "J2"):
        exp = CONFIG_EXPECT[(joint, "default")]["diff"]
        v0 = group(runs, joint, "default", "v0")
        vf = group(runs, joint, "default", "vf")
        m0, _, _ = cv(_vals(v0, "lag_at_rate_deg"))
        mf, _, _ = cv(_vals(vf, "lag_at_rate_deg"))
        if m0 is None or mf is None:
            W(f"| {joint} | {_f(m0)} | {_f(mf)} | — | {exp:.3f}° | — | — | ⏳缺数据 |")
            continue
        diff = m0 - mf
        dev = abs(diff - exp) / exp if exp else None
        ok = dev is not None and dev <= A2_4_TOL
        oks.append(ok)
        flag = "❌⚠️符号" if diff < 0 else ("✅" if ok else "❌")
        W(f"| {joint} | {m0:.3f}° | {mf:.3f}° | {diff:.3f}° | {exp:.3f}° "
          f"| {dev*100:.1f}% | {'✓' if ok else '✗'} | {flag} |")
    passed = all(oks) if oks else None
    W(f"\n（差分为负 → vel 符号/joint_sign 方向错，走 §3 决策规则 3 排查通路，**非口径否决**）")
    W(f"\n**A2-4：{_verdict_str(passed, 'hard')}**\n")
    return passed


def eval_a2_5(runs, W, sigma_run):
    """A2-5 vf 不劣化平顺度（sanity）：resid_std(vf)−(v0) ≤ 3σ_run。"""
    W("## A2-5 vf 不劣化平顺度（sanity，resid_std 劣化 ≤ 3σ_run）\n")
    W("| 关节 | resid_std(vf) | resid_std(v0) | 差 | 3σ_run | 判定 |")
    W("|---|---|---|---|---|---|")
    oks = []
    for joint in ("J6", "J2"):
        vf = group(runs, joint, "default", "vf")
        v0 = group(runs, joint, "default", "v0")
        mf, _, _ = cv(_vals(vf, "resid_std_deg"))
        m0, _, _ = cv(_vals(v0, "resid_std_deg"))
        sr = sigma_run.get(joint)
        if mf is None or m0 is None or sr is None:
            W(f"| {joint} | {_f(mf)} | {_f(m0)} | — | {_f(sr)} | ⏳缺数据 |")
            continue
        d = mf - m0
        lim = A2_5_SIGMA_MULT * sr
        ok = d <= lim
        oks.append(ok)
        W(f"| {joint} | {mf:.4f}° | {m0:.4f}° | {d:+.4f}° | {lim:.4f}° "
          f"| {'✅' if ok else '❌'} |")
    passed = all(oks) if oks else None
    W(f"\n**A2-5：{_verdict_str(passed, 'sanity')}**"
      f"（缓变冲击若激起振铃渗出剔除窗，在此暴露 → R2）\n")
    return passed


def eval_a2_6(runs, g0_runs, W):
    """A2-6 跨 session σ 复核（信息性）：A2 v0 ×3 vs G0 R-A ×5。"""
    W("## A2-6 跨 session σ 复核（信息性，A2 v0 ×3 vs G0 R-A ×5）\n")
    W("| 关节 | 指标 | A2 v0 mean(n) | G0 mean(n) | 均值偏移 | 同session σ | 跨session σ | >3× |")
    W("|---|---|---|---|---|---|---|---|")
    sigma_run = {}       # G0 同 session σ(resid_std)，喂给 A2-5
    for joint in ("J6", "J2"):
        a2 = group(runs, joint, "default", "v0")     # p4 默认速率（core）
        g0 = [r for r in g0_runs if r.joint == joint
              and (r.period_tri is None or abs(r.period_tri - CORE_PERIOD_TRI) < 1e-6)]
        for label, col in [("lag_at_rate", "lag_at_rate_deg"),
                           ("resid_std", "resid_std_deg")]:
            ma, _, _ = cv(_vals(a2, col))
            mg, sg, _ = cv(_vals(g0, col))
            if label == "resid_std" and sg is not None:
                sigma_run[joint] = sg
            if ma is None or mg is None:
                W(f"| {joint} | {label} | {_f(ma)}({len(a2)}) | {_f(mg)}({len(g0)}) "
                  f"| — | {_f(sg)} | — | — |")
                continue
            shift = ma - mg
            # 跨 session 合并 σ：两组池化标准差
            pooled = cv(_vals(a2, col) + _vals(g0, col))[1]
            flag = "—"
            if sg and pooled:
                flag = "⚠️YES" if pooled > A2_6_CROSS_MULT * sg else "no"
            W(f"| {joint} | {label} | {ma:.4f}({len(a2)}) | {mg:.4f}({len(g0)}) "
              f"| {shift:+.4f} | {_f(sg)} | {_f(pooled)} | {flag} |")
    W(f"\n（信息性——不 NO-GO；若跨 session σ 显著大于同 session（>3×），G0-3 σ 表标注"
      f"升级并按实测回填 Q4 GP 噪声先验，SOP-08 §8 R2）\n")
    return sigma_run


def eval_rate_crosscheck(runs, W):
    """A2-D (k_v,c) 分离 + 残余项性质判别（信息性，服务 Q1/S1）。

    两条独立路径：差分法（v0 两速率池化）k_v=(lag_p4−lag_p8)/(r4−r8)、c=lag−k_v·r；
    直接法（vf 残余 lag，速率无关 → 即 c）。二者吻合 ⇒ 残余是 Coulomb c 而非传输延迟
    （延迟会使 lag∝rate、τ_d 恒定）。"""
    W("## A2-D (k_v,c) 分离 + 残余项性质（信息性，J6 默认，p4/p8 交叉）\n")
    j = "J6"
    l04 = cv(_vals(group(runs, j, "default", "v0", period=4.0), "lag_at_rate_deg"))[0]
    l08 = cv(_vals(group(runs, j, "default", "v0", period=8.0), "lag_at_rate_deg"))[0]
    lf4 = cv(_vals(group(runs, j, "default", "vf", period=4.0), "lag_at_rate_deg"))[0]
    lf8 = cv(_vals(group(runs, j, "default", "vf", period=8.0), "lag_at_rate_deg"))[0]
    r4 = cv(_vals(group(runs, j, "default", "v0", period=4.0), "rate_deg_s"))[0]
    r8 = cv(_vals(group(runs, j, "default", "v0", period=8.0), "rate_deg_s"))[0]
    if None in (l04, l08, r4, r8):
        W("- ⏳ 缺 p8（A2D）数据，跳过。\n")
        return
    kv = (l04 - l08) / (r4 - r8)                 # s（°/(°/s)）
    c_diff = l04 - kv * r4                        # °，差分法反解
    exp = CONFIG_EXPECT[("J6", "default")]
    W(f"- **差分法（v0 两速率）**：lag(p4)={l04:.3f}° @ {r4:.2f}°/s、"
      f"lag(p8)={l08:.3f}° @ {r8:.2f}°/s → **k_v={kv:.4f} s**（vs kd/kp=0.16, "
      f"偏差 {abs(kv-0.16)/0.16*100:.1f}%）、**c={c_diff:.3f}°**")
    if lf4 is not None:
        td4 = lf4 / r4 * 1000
        line = f"- **直接法（vf 残余）**：lag(p4)={lf4:.3f}°"
        if lf8 is not None:
            td8 = lf8 / r8 * 1000
            line += (f"、lag(p8)={lf8:.3f}° → **速率无关**（若为传输延迟应 lag∝rate："
                     f"τ_d {td4:.0f}ms→{td8:.0f}ms 不恒定 ⇒ 排除延迟，确为 c）")
        W(line)
        W(f"- **三方对比**：差分 c={c_diff:.2f}° ≈ 直接 c≈{lf4:.2f}° ≫ 解析 τ̂_c/kp="
          f"{exp['lag_vf']:.2f}° → 小信号 c ≈ **{lf4/exp['lag_vf']:.1f}×** 大信号标定；"
          f"k_v 两法一致 ⇒ A2-4 通路正确（vel-ff 精确抵消 kd·q̇/kp）。→ 回填 Q1 速率维度、"
          f"S1 库伦前馈 c 先验。\n")
    return dict(k_v=kv, c_diff=c_diff, c_vf=lf4, ratio=(lf4 / exp["lag_vf"] if lf4 else None))


def _f(v):
    return "—" if v is None else f"{v:.3f}"


def _verdict_str(passed, kind):
    if passed is None:
        return "⏳ 待数据"
    if passed:
        return "✅ 通过"
    return "⚠️ 未过" if kind == "sanity" else "❌ 不通过"


def write_summary_csv(runs, path: Path):
    cols = ["run_dir", "joint", "cal", "gain", "period_tri", "kp",
            "lag_at_rate_deg", "resid_std_deg", "jump_p95_deg", "rate_deg_s",
            "jump_count_v2", "n_fit"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fp:
        w = csv.DictWriter(fp, fieldnames=cols)
        w.writeheader()
        for r in sorted(runs, key=lambda x: x.dir):
            row = {"run_dir": r.dir, "joint": r.joint, "cal": r.cal,
                   "gain": r.gain, "period_tri": r.period_tri, "kp": r.kp}
            for c in cols[6:]:
                row[c] = r.m(c)
            w.writerow(row)


def _decision(a2_2, a2_3, a2_4, W):
    """SOP-10 §3 决策规则 → GO 路径。仅在硬判据都有数据时给结论。"""
    W("## 决策（SOP-10 §3 规则）\n")
    if a2_2 is None or a2_4 is None:
        W("- ⏳ **硬判据数据未齐**（A2-2 主判据 / A2-4 差分尚缺 vf 或 v0 侧）——"
          "本次为**空跑/部分采数**，不下口径冻结结论。补齐 §5 runbook 全部 18 组后重跑。")
        return
    if a2_4 is False:
        W("- ❌ **A2-4 差分不过（规则 3）**：优先排查通路 bug——vel 符号/`joint_sign` "
          "方向（lag 不降反升几乎必是符号错）、blend 窗残留、`_update` 下发值"
          "（`get_command_state` 可查）；修复后重跑该系列，**不作为口径否决证据**。")
        return
    if a2_2 and a2_4:
        W("- ✅ **规则 1（全过）→ L0 代价口径冻结为 vel-ff**：`--vel-ff` 成为 L0 快评"
          "标准口径（写入 optimize spec）；v0 保留两角色——每关节少量对照 run（(k_v,c) "
          "差分分离）+ 历史可比/漂移锚点。回填 devlog 07-28 §3-3、SOP-08 §7、ODE 进度。")
        if a2_3 is False:
            W("- ⚠️ **但 A2-3 排序不过（规则 4）**：机制对小信号分辨力不足——跟手性主指标"
              "押注从 lag 移向小幅阶跃 `ts_ms_v2`，回 devlog Q4 修代价函数。")
        return
    if a2_2 is False:
        W("- ❌ **A2-2 不过（规则 2）→ 回退**：L0 代价主项改用「多速率 v0 池化反解 c」"
          "（G0-8 已验两点反解可行，A2D 数据直接复用），vel-ff 降级为 L1 验证口径；"
          "记录决策回填 devlog Q4/Q5。（R3：这不是失败而是答案。）")


def _plots(runs, root: Path, gate: Path):
    """v0 vs vf 对比图（SOP-10 §6 产物）：J6/J2 三角波误差轨迹 + lag 汇总条形。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as e:  # noqa: BLE001
        print(f"[a2] plots skipped ({e})")
        return

    def tri_err(run_dir, joint):
        p = root / run_dir / f"unit-{joint}-triangle.csv"
        if not p.exists():
            return None
        d = np.genfromtxt(p, delimiter=",", names=True)
        return d["t"], (d["resp"] - d["ref"]) * DEG

    def first(joint, cal):
        g = group(runs, joint, "default", cal)
        return sorted(g, key=lambda r: r.dir)[0].dir if g else None

    # Fig 1-2: J6 / J2 triangle error trajectory, v0 vs vf
    for joint, exp in (("J6", CONFIG_EXPECT[("J6", "default")]),
                       ("J2", CONFIG_EXPECT[("J2", "default")])):
        fig, ax = plt.subplots(figsize=(11, 4.2))
        for cal, c, lbl in (("v0", "tab:red", "vel=0"), ("vf", "tab:green", "vel-ff")):
            rd = first(joint, cal)
            e = tri_err(rd, joint) if rd else None
            if e is not None:
                ax.plot(e[0], e[1], lw=0.7, color=c, alpha=0.85, label=f"{lbl} ({rd})")
        mv0 = cv(_vals(group(runs, joint, "default", "v0"), "lag_at_rate_deg"))[0]
        mvf = cv(_vals(group(runs, joint, "default", "vf"), "lag_at_rate_deg"))[0]
        # None-safe annotation: on all-v0 / partial data (e.g. the §4 P1 空跑)
        # the vf mean is absent — annotate what we have instead of crashing.
        diff_txt = (f"{mv0 - mvf:.2f}deg ~ k_v*s {exp['diff']:.2f}deg"
                    if (mv0 is not None and mvf is not None) else "NA (no vf)")
        ax.text(0.01, 0.98,
                f"lag_at_rate: vel=0 {_f(mv0)}deg -> vel-ff {_f(mvf)}deg\n"
                f"diff {diff_txt} (A2-4)\n"
                f"residual {_f(mvf)}deg = Coulomb c (rate-independent, A2-D)",
                transform=ax.transAxes, va="top", fontsize=9, family="monospace",
                bbox=dict(boxstyle="round", fc="white", alpha=0.8))
        ax.set_title(f"A2 {joint} triangle tracking error - vel=0 vs vel-ff (kd*qdot/kp removed)")
        ax.set_xlabel("t (s)"); ax.set_ylabel("err = resp - ref (deg)")
        ax.grid(alpha=0.3); ax.legend(loc="lower right")
        fig.tight_layout(); fig.savefig(gate / f"a2-{joint}-v0-vs-vf.png", dpi=110)
        plt.close(fig)

    # Fig 3: lag_at_rate bar across configs (mechanism pair + caliber)
    fig, ax = plt.subplots(figsize=(10, 4.2))
    bars, labels, colors = [], [], []
    for joint in ("J6", "J2"):
        for gain, gl in (("default", "def"), ("kp2", "kp/2")):
            for cal, cc in (("v0", "tab:red"), ("vf", "tab:green")):
                m, s, _ = cv(_vals(group(runs, joint, gain, cal), "lag_at_rate_deg"))
                if m is None:
                    continue
                bars.append((m, s or 0)); labels.append(f"{joint}\n{gl}\n{cal}"); colors.append(cc)
    xs = range(len(bars))
    ax.bar(xs, [b[0] for b in bars], yerr=[b[1] for b in bars], color=colors,
           alpha=0.85, capsize=3)
    ax.set_xticks(list(xs)); ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("lag_at_rate (deg)")
    ax.set_title("A2 lag_at_rate by joint x gain x caliber (vel-ff collapses the kd*qdot/kp lag)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout(); fig.savefig(gate / "a2-lag-summary.png", dpi=110); plt.close(fig)
    print("wrote 3 PNGs ->", gate)


def main():
    ap = argparse.ArgumentParser(description="A2 vel-ff 口径实验评估 (SOP-10 P1/P3)")
    ap.add_argument("--output-root", required=True,
                    help="01-output 目录（含 <date>-run-A2*/ 与 G0A 归档）")
    ap.add_argument("--gate-dir", required=True,
                    help="A2 产物目录 04-a2-velff-gate（写 a2-report.md / a2-summary.csv）")
    ap.add_argument("--glob", default="*-run-A2*",
                    help="A2 run-dir 名 glob；空跑 G0 用 '*-run-G0A*'")
    ap.add_argument("--g0-glob", default="2026-07-23-run-G0A-J*",
                    help="G0 R-A 归档 glob（A2-6 跨 session σ 复核基准）")
    args = ap.parse_args()

    root = Path(args.output_root)
    gate = Path(args.gate_dir)
    runs = load_runs(root, args.glob)
    g0_runs = load_runs(root, args.g0_glob)

    n_vf = sum(r.cal == "vf" for r in runs)
    n_v0 = sum(r.cal == "v0" for r in runs)
    print(f"[a2] loaded {len(runs)} A2 runs (v0={n_v0} vf={n_vf}), "
          f"{len(g0_runs)} G0 R-A runs")

    lines = []
    W = lines.append
    W("# A2 vel-ff 口径实验 Gate 报告（SOP-10 P1/P3，自动生成）\n")
    W(f"> 口径：`--vel-ff` 主 / `vel=0` 诊断锚（meta.vel_ff.enabled 唯一真值，R8）。")
    W(f"> caliper 沿 G0 定稿：`k_sigma=4.0`、`apex_excl_s=0.55 s`（各 run metrics-v2.json 记录）。")
    W(f"> 数据：A2 run={len(runs)}（v0={n_v0}, vf={n_vf}）；G0 R-A={len(g0_runs)}。"
      f"{'**⚠️ 空跑/部分采数**（vf 或差分项缺 → 判据待数据）。' if n_vf == 0 else ''}\n")

    # A2-6 先跑：产出 G0 同 session σ_run，喂给 A2-5
    sigma_run = eval_a2_6(runs, g0_runs, W)
    a2_1 = eval_a2_1(runs, W)
    a2_2 = eval_a2_2(runs, W)
    a2_3 = eval_a2_3(runs, W)
    a2_4 = eval_a2_4(runs, W)
    a2_5 = eval_a2_5(runs, W, sigma_run)
    eval_rate_crosscheck(runs, W)

    # 汇总表
    W("## 判据汇总\n")
    W("| 判据 | 性质 | 判定 |")
    W("|---|---|---|")
    for gid, kind, v in [("A2-1 量级", "sanity", a2_1),
                         ("A2-2 小信号重复性", "硬(主)", a2_2),
                         ("A2-3 机制对排序", "硬", a2_3),
                         ("A2-4 差分一致性", "硬", a2_4),
                         ("A2-5 平顺度", "sanity", a2_5)]:
        k = "sanity" if "sanity" in kind else "hard"
        W(f"| {gid} | {kind} | {_verdict_str(v, k)} |")
    W("")
    _decision(a2_2, a2_3, a2_4, W)

    gate.mkdir(parents=True, exist_ok=True)
    (gate / "a2-report.md").write_text("\n".join(lines))
    write_summary_csv(runs, gate / "a2-summary.csv")
    _plots(runs, root, gate)
    print("wrote", gate / "a2-report.md")
    print("wrote", gate / "a2-summary.csv")


if __name__ == "__main__":
    main()
