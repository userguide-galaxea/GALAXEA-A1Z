#!/usr/bin/env python3
"""E7 事后追算（devlog 2026-08-01 计划 E7 / 2026-08-03 计划 F1）。

用 refine session 归档的多组 (J_joint, J_ee) 离线核算层间 Spearman ρ，
事后补验 A3-1/A3-2 前提中**可离线计算**的部分：

- 可计算：ρ(J_joint, J_ee)——同一 benchmark、十余组不同增益下两个代理
  层的一致性。ρ 高 = EE 主导权重不扭曲优化方向；ρ 低 = 两层带不同信息，
  J_total 与 L2 任务背离风险升高（缓释：退回关节代价主导，升版本）。
- 不可计算（如实记录）：以 L2 任务真值为基准的正式 A3-1/A3-2——现有 L2
  真值只有 Pdef/PphA 两点人工定性结论，Spearman 需 ≥3 点数值真值（即被
  简化的正式 A3 gate 的 5 组 × L2 矩阵）。两点方向性核对作旁证。

数据通道：trial meta.json 的 ``cost_breakdown``（refine v10+ 含原始
``j_joint`` / ``j_ee`` 字段），可行 trial = verdict "ok" 且 cost < PENALTY。

用法:
    python -m a1z.analysis.script.e7_rho_backfill \
        --session $TEST_LOG_ROOT/02-a1z/02-para-opt/2026-08-03-run-opt-eeRefine-J456
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Optional, Tuple

from a1z.analysis.optimize.cost_spec import PENALTY_COST
from a1z.analysis.script.a3_gate_eval import spearman_rho

RHO_CONSISTENT = 0.8   # ≥: 两代理层一致，EE 主导权重不扭曲方向
RHO_MIXED = 0.3        # <: 背离风险高，建议退回关节代价主导


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="E7 ρ backfill (refine session)")
    ap.add_argument("--session", required=True, type=str,
                    help="refine session 目录（含 trials/t*/meta.json）")
    ap.add_argument("--a3-pdef-jee", type=float, default=0.9999,
                    help="A3 Pdef 组 J_ee（E5 回放值，用于两点方向核对）")
    ap.add_argument("--a3-ppha-jee", type=float, default=0.7108,
                    help="A3 PphA 组 J_ee（E5 回放值）")
    return ap.parse_args()


def load_pairs(session: Path) -> List[Tuple[int, float, float]]:
    """(trial_id, j_joint, j_ee) for feasible refine trials."""
    pairs: List[Tuple[int, float, float]] = []
    for meta_path in sorted((session / "trials").glob("t*/meta.json")):
        meta = json.loads(meta_path.read_text())
        bd = meta.get("cost_breakdown") or {}
        j_joint = bd.get("j_joint")
        j_ee = bd.get("j_ee")
        if (meta.get("watchdog_verdict") == "ok"
                and meta.get("cost", PENALTY_COST) < PENALTY_COST
                and j_joint is not None and j_ee is not None):
            pairs.append((int(meta["trial_id"]), float(j_joint), float(j_ee)))
    return pairs


def main() -> int:
    args = _parse_args()
    session = Path(args.session)
    if not (session / "trials").exists():
        print(f"ERROR: no trials/ under {session}")
        return 1

    pairs = load_pairs(session)
    if len(pairs) < 3:
        print(f"ERROR: only {len(pairs)} feasible (j_joint, j_ee) pairs "
              f"— need >= 3 for Spearman")
        return 1

    xs = [p[1] for p in pairs]
    ys = [p[2] for p in pairs]
    rho = spearman_rho(xs, ys)

    lines: List[str] = []
    W = lines.append
    W("# E7 ρ 追算报告（层间一致性，J_joint vs J_ee）")
    W("")
    W(f"- session: `{session}`")
    W(f"- 可行 trial 数: {len(pairs)}（verdict=ok 且 cost < PENALTY）")
    W("")
    W("| trial | J_joint | J_ee |")
    W("|---|---|---|")
    for tid, jj, je in pairs:
        W(f"| t{tid:03d} | {jj:.4f} | {je:.4f} |")
    W("")
    W(f"## Spearman ρ(J_joint, J_ee) = {rho:.3f}")
    W("")
    if rho is not None and rho >= RHO_CONSISTENT:
        verdict = (f"ρ ≥ {RHO_CONSISTENT}：两代理层一致——EE 主导权重（0.65）"
                   f"不扭曲优化方向，A3-2 前提的可计算部分成立。")
    elif rho is not None and rho >= RHO_MIXED:
        verdict = (f"{RHO_MIXED} ≤ ρ < {RHO_CONSISTENT}：部分一致——J_total 优化"
                   f"方向需以 L2 锚点复核（E6 收官 verify + 人工评估）。")
    else:
        verdict = (f"ρ < {RHO_MIXED}：两层信息背离——J_total 方向与关节层冲突"
                   f"风险高，建议退回关节代价主导（改 W_EE_TOTAL/W_JT_TOTAL，"
                   f"升版本）。")
    W(f"**判定**：{verdict}")
    W("")
    W("## 两点方向性旁证（A3 Pdef → PphA，人工 L2 结论：PphA 达标更优）")
    W("")
    W(f"- J_ee: {args.a3_pdef_jee:.4f} → {args.a3_ppha_jee:.4f}（改善 ✅）")
    W("- J_joint 侧：PphA 5/6 关节 RMSE 改善（devlog 2026-08-01 Phase A 收官 ✅）")
    W("- 两层方向均与 L2 真值一致（两点，仅作方向旁证）。")
    W("")
    W("## 不可计算部分（如实记录）")
    W("")
    W("以 L2 数值真值（fail_rate / 插孔时间）为基准的正式 A3-1/A3-2 无法追算：")
    W("现有 L2 真值仅 Pdef/PphA 两点人工定性结论（简化 A3），Spearman 需 ≥3 点")
    W("数值真值。如需硬判定，补正式 A3 gate 的 5 组 × L2 矩阵后用")
    W("`a3_gate_eval.py`（已支持 run_dir_ee 通道，G6）重跑。")

    report = session / "e7-rho-report.md"
    report.write_text("\n".join(lines))
    print(f"[e7] ρ(J_joint, J_ee) = {rho:.3f} over {len(pairs)} pairs")
    print(f"[e7] {verdict}")
    print(f"[e7] Report: {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
