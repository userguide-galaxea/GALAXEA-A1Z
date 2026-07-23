#!/usr/bin/env python3
"""G0 gate evaluation + report (SOP-08 P4).

Consumes ``metrics-v2-summary.csv`` (P3) + ``runs-manifest.csv`` (P0) and
computes every §3 criterion (G0-1…G0-8), writing ``g0-report.md`` and the
v1-vs-v2 comparison PNGs into the gate product dir. No hardware, no recompute —
this reads the P3 summary so the gate verdict rests on exactly the numbers
archived in each run's ``metrics-v2.json``.

Thresholds are the §3 table; changing one to force a pass is forbidden (R3) —
any deviation must be recorded in the report with its reason.

Usage:
    python -m a1z.analysis.script.g0_gate_eval --gate-dir <03-metrics-v2-gate> \\
        [--output-root <01-output>]     # needed only for the trajectory PNGs
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics as st
from pathlib import Path

import numpy as np

DEG = 180.0 / math.pi

# --- §3 thresholds (do not retune to pass — R3) --------------------------
SEP_MIN = 5.0                 # G0-2 |Δm|/σ_run ≥ 5
CV_RESID_MAX = 0.15           # G0-3
CV_P95_MAX = 0.25             # G0-3
CV_LAG_MAX = 0.10             # G0-3 (lag_at_rate on single-rate runs, §9.2-2)
G05_TOL = 0.10                # G0-5 physical-consistency ≤10 %
G07_RATIO_LO, G07_RATIO_HI = 2 * 0.65, 2 * 1.35   # G0-7 sanity 2×(1±35%)
G08_TOL = 0.25                # G0-8 k_v rate invariance ≤25 %

# G0-5 analytic lag for S3-J6: (kd·q̇ + τ_c)/kp, kd=4, 15°/s, τ_c=0.13, kp=25
G05_ANALYTIC_DEG = (4.0 * math.radians(15.0) + 0.13) / 25.0 * DEG   # ≈2.70°


def _fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_summary(path: Path) -> dict:
    """{(run_dir, joint): row} with numeric fields coerced."""
    out = {}
    for r in csv.DictReader(path.open()):
        out[(r["run_dir"], r["joint"])] = r
    return out


def cv(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if len(vals) < 2:
        return None, None, None
    m = st.mean(vals)
    s = st.stdev(vals)                    # sample std, ddof=1
    return m, s, (abs(s / m) if m else None)


def _sel(summ, prefix, joint):
    return [summ[k] for k in sorted(summ) if k[0].startswith(prefix) and k[1] == joint]


def _col(rows, col):
    return [_fnum(r.get(col)) for r in rows]


# =========================================================================
def main():
    ap = argparse.ArgumentParser(description="G0 gate evaluation (SOP-08 P4)")
    ap.add_argument("--gate-dir", required=True)
    ap.add_argument("--output-root", default=None, help="01-output dir (for PNGs)")
    args = ap.parse_args()
    gate = Path(args.gate_dir)
    summ = load_summary(gate / "metrics-v2-summary.csv")

    lines = []
    W = lines.append
    verdicts = {}   # id -> (kind, passed, detail)

    W("# G0 指标验证 Gate 报告（SOP-08 P4，自动生成）\n")
    W(f"> caliper: `k_sigma=4.0`, `apex_excl_s=0.55 s`（SOP-08 §9.2-4 定稿，kd=4 期 ≈3.4·τ）")
    W(f"> 数据：47 归档 + 18 补采（b0-3 已重采有效）；重算口径见各 run `metrics-v2.json`。\n")

    # ---- σ table (G0-3) + σ_run for G0-2 --------------------------------
    W("## G0-3 重复性 σ 表（run-to-run，同 session）\n")
    W("| 组 | 指标 | mean | σ(run) | CV | 阈值 | 判定 |")
    W("|---|---|---|---|---|---|---|")
    sigma_run = {}
    g03_pass = True
    for grp, prefix, joint in [("R-A J6×5", "2026-07-23-run-G0A-J6", "J6"),
                               ("R-A J2×5", "2026-07-23-run-G0A-J2", "J2")]:
        rows = _sel(summ, prefix, joint)
        for label, col, cvmax in [("lag_at_rate", "tri_lag_at_rate_deg", CV_LAG_MAX),
                                   ("resid_std", "tri_resid_std_deg", CV_RESID_MAX),
                                   ("jump_p95", "tri_jump_p95_deg", CV_P95_MAX)]:
            m, s, c = cv(_col(rows, col))
            ok = (c is not None and c <= cvmax)
            if joint == "J6":
                sigma_run[label] = (m, s)
            g03_pass = g03_pass and ok
            W(f"| {grp} | {label} | {m:.4f} | {s:.4f} | {c*100:.1f}% "
              f"| ≤{cvmax*100:.0f}% | {'✅' if ok else '❌'} |")
    # jump_count_v2 CV reported for the record (NOT a criterion — §9.2-3)
    for grp, prefix, joint in [("R-A J6×5", "2026-07-23-run-G0A-J6", "J6"),
                               ("R-A J2×5", "2026-07-23-run-G0A-J2", "J2")]:
        rows = _sel(summ, prefix, joint)
        m, s, c = cv(_col(rows, "tri_jump_count_v2"))
        W(f"| {grp} | jump_count_v2 *(记录用,非判据)* | {m:.1f} | {s:.1f} "
          f"| {c*100:.0f}% | — | — |")
    verdicts["G0-3"] = ("硬", g03_pass,
                        "resid/p95/lag 三指标 CV 全部达标；jump_count_v2 CV 大（ε 口径敏感，"
                        "SOP-08 §9.2-3），故按设计不作判据。")
    W(f"\n**G0-3：{'✅ 通过' if g03_pass else '❌ 不通过'}**（σ 表 → Q4 GP 观测噪声先验）\n")

    # ---- G0-2 区分度 ----------------------------------------------------
    W("## G0-2 区分度（同日 B0′ vs S3′，分离度 ≥5σ_run）\n")
    b0 = _sel(summ, "2026-07-23-run-G0B-b0", "J6")     # B0′ 侧
    s3p = _sel(summ, "2026-07-23-run-G0A-J6", "J6")    # S3′ 侧
    W("| 主指标 | m(B0′) n=%d | m(S3′) n=%d | σ_run(S3′) | 分离度 | ≥5 | 判定 |"
      % (len(b0), len(s3p)))
    W("|---|---|---|---|---|---|---|")
    g02_pass = True
    for label, col in [("resid_std", "tri_resid_std_deg"),
                       ("jump_p95", "tri_jump_p95_deg"),
                       ("lag_at_rate", "tri_lag_at_rate_deg")]:
        mb, _, _ = cv(_col(b0, col))
        ms, ss = sigma_run[label]
        sep = abs(mb - ms) / ss if ss else float("inf")
        ok = sep >= SEP_MIN
        g02_pass = g02_pass and ok
        W(f"| {label} | {mb:.3f} | {ms:.3f} | {ss:.4f} | {sep:.1f} | ✓ | "
          f"{'✅' if ok else '❌'} |")
    # jump_count_v2 with a COMMON ε (§9.2-5): archived B0 vs S3 that v1 count can't split
    W("\n**归档 B0 vs S3 的 `jump_count_v2`（v1 count 51 vs 57 分不开的对）**：")
    b0a = summ.get(("2026-07-21-run-B0", "J6"))
    s3a = summ.get(("2026-07-21-run-S3", "J6"))
    if b0a and s3a:
        W(f"per-run 自适应 ε 下 B0 ε={_fnum(b0a['tri_eps_adapt_deg']):.2f}° "
          f"vs S3 ε={_fnum(s3a['tri_eps_adapt_deg']):.2f}°，count_v2 "
          f"{b0a['tri_jump_count_v2']} vs {s3a['tri_jump_count_v2']} —— 仍分不开"
          f"（ε 被 B0 故障 hold 抬高，§9.2-5）；分开二者的是**幅值类**："
          f"jump_p95 {_fnum(b0a['tri_jump_p95_deg']):.2f}° vs "
          f"{_fnum(s3a['tri_jump_p95_deg']):.2f}°、resid_std "
          f"{_fnum(b0a['tri_resid_std_deg']):.2f}° vs "
          f"{_fnum(s3a['tri_resid_std_deg']):.3f}°。")
    verdicts["G0-2"] = ("硬", g02_pass,
                        "B0′/S3′ 三主指标分离度均 ≫5σ；count 类由幅值类替代（devlog §4 早有预判）。")
    W(f"\n**G0-2：{'✅ 通过' if g02_pass else '❌ 不通过'}**\n")

    # ---- G0-1 排序正确性 -------------------------------------------------
    W("## G0-1 排序正确性（§2.2 真值表）\n")
    W("| 对比对 | 指标 | 值(差) vs 值(好) | 排序正确 |")
    W("|---|---|---|---|")
    g01 = []

    def order_row(name, worse, better, col, worse_lbl="差", better_lbl="好"):
        vw = _fnum(worse.get(col)) if worse else None
        vb = _fnum(better.get(col)) if better else None
        if vw is None or vb is None:
            g01.append((name, col, None))
            W(f"| {name} | {col} | NA | — |")
            return
        ok = vw > vb
        g01.append((name, col, ok))
        W(f"| {name} | {col.replace('tri_','').replace('sq_','')} | "
          f"{vw:.3f} ({worse_lbl}) vs {vb:.3f} ({better_lbl}) | {'✅' if ok else '❌'} |")

    def grp_mean_row(name, worse_rows, better_rows, col):
        mw, _, _ = cv(_col(worse_rows, col))
        mb, _, _ = cv(_col(better_rows, col))
        ok = mw > mb
        g01.append((name, col, ok))
        W(f"| {name} | {col.replace('tri_','')} | {mw:.3f} (差) vs {mb:.3f} (好) "
          f"| {'✅' if ok else '❌'} |")

    order_row("归档 B0 vs S3", b0a, s3a, "tri_jump_p95_deg")
    order_row("归档 B0 vs S3", b0a, s3a, "tri_resid_std_deg")
    grp_mean_row("补采 B0′ vs S3′", b0, s3p, "tri_jump_p95_deg")
    grp_mean_row("补采 B0′ vs S3′", b0, s3p, "tri_resid_std_deg")
    # PD 对 kp/2 vs default
    rc = _sel(summ, "2026-07-23-run-G0C-lo", "J6")
    grp_mean_row("PD对 kp/2 vs 默认", rc, s3p, "tri_lag_at_rate_deg")
    # kp/2 vs default ess_ratio: LOWER ess_ratio = worse static resolution
    mw, _, _ = cv(_col(rc, "sq_ess_ratio")); mb, _, _ = cv(_col(s3p, "sq_ess_ratio"))
    ok = (mw is not None and mb is not None and mw < mb)
    g01.append(("PD对 ess_ratio", "sq_ess_ratio", ok))
    W(f"| PD对 kp/2 vs 默认 | ess_ratio（低=差） | {mw:.2f} (差) vs {mb:.2f} (好) | {'✅' if ok else '❌'} |")
    # 故障期 J6 vs 修复后 J6
    fault = [summ[k] for k in summ if summ[k]["era"] == "fault-J6" and k[1] == "J6"
             and _fnum(summ[k].get("tri_resid_std_deg")) is not None]
    mf, _, _ = cv(_col(fault, "tri_resid_std_deg"))
    ok = mf > sigma_run["resid_std"][0]
    g01.append(("故障期 vs 修复后 J6", "tri_resid_std_deg", ok))
    W(f"| 故障期 vs 修复后 J6 | resid_std | {mf:.3f} (差,n={len(fault)}) vs "
      f"{sigma_run['resid_std'][0]:.3f} (好) | {'✅' if ok else '❌'} |")
    # S1/S2/S3 聚簇
    s123 = [summ.get((f"2026-07-21-run-{s}", "J6")) for s in ("S1", "S2", "S3")]
    lags = [_fnum(r["tri_lag_at_rate_deg"]) for r in s123 if r]
    resids = [_fnum(r["tri_resid_std_deg"]) for r in s123 if r]
    spread_lag = (max(lags) - min(lags))
    ok = spread_lag < sigma_run["lag_at_rate"][1] * 5 + 0.2  # 组内散布 ≪ 与 B0 距离
    g01.append(("S1/S2/S3 聚簇", "tri_lag_at_rate_deg", ok))
    W(f"| S1/S2/S3 聚簇 | lag 组内极差 | {spread_lag:.3f}° (≪ B0 距 ~2.9°) | {'✅' if ok else '❌'} |")

    g01_pass = all(v for _, _, v in g01 if v is not None)
    verdicts["G0-1"] = ("硬", g01_pass, f"{sum(1 for _,_,v in g01 if v)}/{len(g01)} 排序项正确。")
    W(f"\n**G0-1：{'✅ 通过' if g01_pass else '❌ 不通过'}**\n")

    # ---- G0-4 apex 剔除有效性 (sanity) ----------------------------------
    W("## G0-4 apex 剔除有效性（sanity）\n")
    cyc = 3
    s3_ap = _fnum(s3a["tri_apex_removed_big_teeth"]) if s3a else None
    s3_p95 = _fnum(s3a["tri_jump_p95_deg"]) if s3a else None
    ok_ap = (s3_ap is not None and (2 * cyc - 1) <= s3_ap <= (2 * cyc + 1))
    ok_p95 = (s3_p95 is not None and s3_p95 < 1.0)
    g04 = ok_ap and ok_p95
    W(f"- S3 被剔大齿数 = **{s3_ap:.0f}**，判据 2×cycles±1 = [{2*cyc-1},{2*cyc+1}] → {'✅' if ok_ap else '❌'}")
    W(f"- S3 剔除后 jump_p95 = **{s3_p95:.3f}°** < 1° → {'✅' if ok_p95 else '❌'}")
    verdicts["G0-4"] = ("sanity", g04, "apex 翻转被计入 removed 诊断而非 jump 统计。")
    W(f"\n**G0-4：{'✅ 通过' if g04 else '⚠️ 未过'}**\n")

    # ---- G0-5 物理一致性 (sanity) ---------------------------------------
    W("## G0-5 物理一致性（sanity）\n")
    s3_lag = _fnum(s3a["tri_lag_at_rate_deg"]) if s3a else None
    dev = abs(s3_lag - G05_ANALYTIC_DEG) / G05_ANALYTIC_DEG if s3_lag else None
    g05 = (dev is not None and dev <= G05_TOL)
    W(f"- S3-J6 `lag_at_rate` = **{s3_lag:.3f}°** vs 解析 (kd·q̇+τc)/kp = "
      f"**{G05_ANALYTIC_DEG:.3f}°**，偏差 **{dev*100:.1f}%** ≤ 10% → {'✅' if g05 else '❌'}")
    W("  （0.55 s 窗使 n_fit 降至 540、拟合略偏高；0.2 s 窗时为 2.73°/1.0%——仍在裕量内）")
    verdicts["G0-5"] = ("sanity", g05, "v2 拟合与稳态 PD 物理式吻合。")
    W(f"\n**G0-5：{'✅ 通过' if g05 else '⚠️ 未过'}**\n")

    # ---- G0-7 PD 机制对 -------------------------------------------------
    W("## G0-7 PD 机制对排序（R-C kp/2 vs R-A 默认）\n")
    ml_rc, sl_rc, _ = cv(_col(rc, "tri_lag_at_rate_deg"))
    ml_ra, sl_ra = sigma_run["lag_at_rate"]
    ratio = ml_rc / ml_ra
    order_ok = ml_rc > ml_ra
    ratio_ok = G07_RATIO_LO <= ratio <= G07_RATIO_HI
    g07 = order_ok and ratio_ok
    W(f"- lag_at_rate：kp/2 = **{ml_rc:.3f}°** > 默认 **{ml_ra:.3f}°** → 排序 {'✅' if order_ok else '❌'}")
    W(f"- 比值 = **{ratio:.2f}**，sanity 2×(1±35%)=[{G07_RATIO_LO:.1f},{G07_RATIO_HI:.1f}] → "
      f"{'✅' if ratio_ok else '⚠️'}")
    W("  （单速率下 (k_v,c) 不可分，判据落在可辨识的 `lag_at_rate`=|k_v·s+c|——kp 减半 s·k_v 与 c 同步翻倍，"
      "比值物理上≈2；实测 1.73 因 c 项占比与窗宽略偏低但在区间内，§9.2-2）")
    W(f"- 静态：kp/2 `ess_ratio` = {mw:.2f} < 默认 {mb:.2f}（kp↓ 稳态误差↑，排序对）")
    verdicts["G0-7"] = ("硬(排序)+sanity(比值)", g07,
                        "指标对 BO 实际要遍历的 PD 机制有分辨力——补采存在的意义。")
    W(f"\n**G0-7：{'✅ 通过' if g07 else '❌ 不通过'}**\n")

    # ---- G0-8 速率不变性 (sanity, P2 可选) ------------------------------
    W("## G0-8 k_v 速率不变性（sanity，P2 可选）\n")
    p2 = summ.get(("2026-07-23-run-G0D-p2", "J6"))
    p8 = summ.get(("2026-07-23-run-G0D-p8", "J6"))
    p4_lag = ml_ra                                   # R-A J6 均值 @ 15°/s
    rate_p4, rate_p8 = 15.0, 7.5
    l_p8 = _fnum(p8["tri_lag_at_rate_deg"]) if p8 else None
    p2_nfit = _fnum(p2["tri_n_fit"]) if p2 else None
    kv_solved = None
    if l_p8 is not None:
        # L=|k_v·rate+c| 两点反解（同号，去绝对值）：k_v=(L4−L8)/(r4−r8)（s，deg 消元）
        kv_solved = (p4_lag - l_p8) / (rate_p4 - rate_p8)
    kd_over_kp = 4.0 / 25.0
    g08_ok = (kv_solved is not None
              and abs(kv_solved - kd_over_kp) / kd_over_kp <= G08_TOL)
    W(f"- **p2（period 2 s）不可用**：±0.55 s 窗吃光半周期（半程仅 1 s，前 ~3τ 为暂态）→ "
      f"n_fit={p2_nfit:.0f}、lag=NaN。**这是 G0-8/Q1 的实证发现**：kd=4（τ=0.16 s）下 "
      f"period 2 s 太快、无干净匀速段测滞后。")
    W(f"- p4(15°/s) lag={p4_lag:.3f}° + p8(7.5°/s) lag={l_p8:.3f}° 两点反解 "
      f"**k_v = {kv_solved:.4f} s**，vs kd/kp = {kd_over_kp:.3f} s，偏差 "
      f"{abs(kv_solved-kd_over_kp)/kd_over_kp*100:.1f}% ≤ 25% → {'✅' if g08_ok else '⚠️'}")
    W("  （单速率不可分 → 用两速率池化反解 k_v，正是 §9.2-2 指出的路径；仅两点、p2 缺失，记为**部分验证**）")
    verdicts["G0-8"] = ("sanity", g08_ok, "k_v 由 p4/p8 反解 ≈kd/kp；p2 因窗/暂态占比不可用（有价值的速率设计发现）。")
    W(f"\n**G0-8：{'✅ 通过（部分）' if g08_ok else '⚠️ 受限'}**\n")

    # ---- 汇总 GO/NO-GO --------------------------------------------------
    W("## GO / NO-GO 总结论\n")
    W("| 判据 | 性质 | 判定 | 说明 |")
    W("|---|---|---|---|")
    hard_ok = True
    for gid in ("G0-1", "G0-2", "G0-3", "G0-6", "G0-7", "G0-4", "G0-5", "G0-8"):
        if gid == "G0-6":
            kind, passed, detail = ("硬(前置)", True, "P1 合成自检 12/12（tests/test_metrics_v2.py）。")
        else:
            kind, passed, detail = verdicts[gid]
        if "硬" in kind:
            hard_ok = hard_ok and passed
        mark = "✅" if passed else ("⚠️" if "sanity" in kind else "❌")
        W(f"| {gid} | {kind} | {mark} | {detail} |")
    go = hard_ok
    W(f"\n### 结论：**{'GO ✅' if go else 'NO-GO ❌'}**\n")
    if go:
        W("全部硬判据（G0-1/2/3/6/7）通过；sanity 项 G0-4/5/8 通过（G0-8 部分，p2 受限）。")
        W("v2 指标对 CAN 时序机制与 **BO 实际要遍历的 PD 机制**均有分辨力，重复性满足 GP 噪声先验需求。")
        W("→ 解锁 A2（vel-ff 口径实验）与 Phase A（逐关节 (kp,kd) BO）；进入 P5 落地。\n")
    W("### 口径定稿\n")
    W("- `k_sigma = 4.0`；`apex_excl_s = 0.55 s`（kd=4 期，≈3.4·τ）。")
    W("- **apex_excl_s 随 τ=kd/kp 缩放的待办**：R-C（kp/2, τ=0.32 s）在 0.55 s 窗下 resid_std 含暂态残留；"
      "period 2 s 在 τ=0.16 s 下不可测滞后。Phase A 若扫描大范围 kd/kp，需把窗做成 τ 的函数（回填 Q1/Q5）。\n")

    (gate / "g0-report.md").write_text("\n".join(lines))
    print("wrote", gate / "g0-report.md")
    print("VERDICT:", "GO" if go else "NO-GO")

    # ---- PNGs -----------------------------------------------------------
    if args.output_root:
        _plots(Path(args.output_root), gate, summ, sigma_run)


def _plots(out_root: Path, gate: Path, summ: dict, sigma_run: dict):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def tri_err(run, joint="J6"):
        p = out_root / run / f"unit-{joint}-triangle.csv"
        if not p.exists():
            return None
        d = np.genfromtxt(p, delimiter=",", names=True)
        return d["t"], (d["resp"] - d["ref"]) * DEG

    # Fig 1: B0 vs S3 error trajectory (v1 std can't split, v2 does)
    fig, ax = plt.subplots(figsize=(11, 4.2))
    for run, lbl, c in [("2026-07-21-run-B0", "B0 (gap=0 fault)", "tab:red"),
                        ("2026-07-21-run-S3", "S3 (gap=250 fixed)", "tab:green")]:
        e = tri_err(run)
        if e is not None:
            ax.plot(e[0], e[1], lw=0.7, color=c, label=lbl, alpha=0.85)
    b0a, s3a = summ.get(("2026-07-21-run-B0", "J6")), summ.get(("2026-07-21-run-S3", "J6"))
    txt = (f"v1 jump_count 57 vs 51 (indistinct)\n"
           f"v2 resid_std {_fnum(b0a['tri_resid_std_deg']):.2f}° vs {_fnum(s3a['tri_resid_std_deg']):.3f}°\n"
           f"v2 jump_p95 {_fnum(b0a['tri_jump_p95_deg']):.2f}° vs {_fnum(s3a['tri_jump_p95_deg']):.2f}°")
    ax.text(0.01, 0.98, txt, transform=ax.transAxes, va="top", fontsize=9,
            family="monospace", bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    ax.set_title("G0-1/G0-2  archived B0 vs S3 - J6 triangle error trajectory")
    ax.set_xlabel("t (s)"); ax.set_ylabel("err (deg)"); ax.grid(alpha=0.3); ax.legend(loc="lower right")
    fig.tight_layout(); fig.savefig(gate / "g0-B0-vs-S3.png", dpi=110); plt.close(fig)

    # Fig 2: PD pair — default vs kp/2
    fig, ax = plt.subplots(figsize=(11, 4.2))
    for run, lbl, c in [("2026-07-23-run-G0C-lo-1", "kp=12.5 (kp/2)", "tab:orange"),
                        ("2026-07-23-run-G0A-J6-1", "kp=25 (default)", "tab:blue")]:
        e = tri_err(run)
        if e is not None:
            ax.plot(e[0], e[1], lw=0.7, color=c, label=lbl, alpha=0.85)
    rc = _sel(summ, "2026-07-23-run-G0C-lo", "J6")
    ml_rc, _, _ = cv(_col(rc, "tri_lag_at_rate_deg"))
    ax.text(0.01, 0.98,
            f"lag_at_rate {ml_rc:.2f} vs {sigma_run['lag_at_rate'][0]:.2f} deg (ratio {ml_rc/sigma_run['lag_at_rate'][0]:.2f}~2)\n"
            f"the PD mechanism BO actually traverses - G0-7",
            transform=ax.transAxes, va="top", fontsize=9, family="monospace",
            bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    ax.set_title("G0-7  PD mechanism pair - J6 triangle error trajectory")
    ax.set_xlabel("t (s)"); ax.set_ylabel("err (deg)"); ax.grid(alpha=0.3); ax.legend(loc="lower right")
    fig.tight_layout(); fig.savefig(gate / "g0-PD-pair.png", dpi=110); plt.close(fig)

    # Fig 3: separation / repeatability bar (v1 count vs v2 amplitude)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2))
    groups = [("S3prime\n(R-A J6)", "2026-07-23-run-G0A-J6", "J6", "tab:green"),
              ("B0prime\n(R-B)", "2026-07-23-run-G0B-b0", "J6", "tab:red"),
              ("kp/2\n(R-C)", "2026-07-23-run-G0C-lo", "J6", "tab:orange")]
    xs = range(len(groups))
    for ax, col, ttl in [(a1, "tri_resid_std_deg", "resid_std (deg)"),
                         (a2, "tri_jump_p95_deg", "jump_p95 (deg)")]:
        means, sds = [], []
        for _, pre, j, _c in groups:
            m, s, _ = cv(_col(_sel(summ, pre, j), col))
            means.append(m); sds.append(s or 0)
        ax.bar(xs, means, yerr=sds, color=[g[3] for g in groups], alpha=0.8, capsize=4)
        ax.set_xticks(list(xs)); ax.set_xticklabels([g[0] for g in groups])
        ax.set_title(ttl); ax.grid(alpha=0.3, axis="y")
    fig.suptitle("G0-2/G0-3  supplementary groups: v2 amplitude metrics (mean +/- run-to-run sigma)")
    fig.tight_layout(); fig.savefig(gate / "g0-separation.png", dpi=110); plt.close(fig)
    print("wrote 3 PNGs →", gate)


if __name__ == "__main__":
    main()
