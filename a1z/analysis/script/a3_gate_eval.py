#!/usr/bin/env python3
"""A3 Goodhart Gate 评估（SOP-11 §5）— 代理有效性验证。

读取 5 组参数（P_bad/P_default/P_hand/P_phaseA/P_aggr）各自的 L0 run 产物
（``metrics-v2.json`` + ``meta.json``）加人工 L2 插孔真值（CSV / 命令行传入），
计算 Spearman 秩相关检验 A3-1~A3-4 判据，输出 ``a3-report.md`` + ``a3-summary.csv``。

无硬件、无 scipy——Spearman 用 stdlib 手写（纯排名 + Pearson on ranks）。

用法:
    python -m a1z.analysis.script.a3_gate_eval \\
        --groups-csv /path/to/a3_groups.csv \\
        --output-dir /path/to/output

a3_groups.csv 格式 (每行一个参数组):
    group,run_dir,l2_fail_rate,l2_insert_time_s
    P_bad,/path/to/run-opt-Pbad,0.80,12.5
    P_default,/path/to/run-opt-Pdef,0.40,6.0
    ...

可选列:
    run_dir_square  — verify.sh 的 step 腿 run 目录（L0 两腿拆开的组）
    run_dir_ee      — EE 跟踪 run 目录（含 ee-traj-{ref,response}.csv）；
                      提供后启用 J_ee 通道与真实 A3-2 判定（G6，2026-08-03 F1）
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics as st
from pathlib import Path
from typing import Dict, List, Optional, Tuple

DEG = 180.0 / math.pi

# --- A3 判据阈值（SOP-11 §5.3 — 不可为通过而回调）---
A3_1_RHO_MIN = 0.80       # 代理方向正确: ρ ≥ 0.8
A3_4_SIGMA_MULT = 3.0     # 区分度: |cost_diff| >> 组内 sigma


# ---------------------------------------------------------------------------
# Spearman rank correlation (stdlib only, no scipy)
# ---------------------------------------------------------------------------
def _rank(xs: List[float]) -> List[float]:
    """Average-rank transform (handles ties)."""
    n = len(xs)
    indexed = sorted(range(n), key=lambda i: xs[i])
    ranks = [0.0] * n
    i = 0
    while i < n:
        j = i + 1
        while j < n and xs[indexed[j]] == xs[indexed[i]]:
            j += 1
        avg_rank = sum(range(i + 1, j + 1)) / (j - i)
        for k in range(i, j):
            ranks[indexed[k]] = avg_rank
        i = j
    return ranks


def spearman_rho(xs: List[float], ys: List[float]) -> Optional[float]:
    """Spearman rank correlation coefficient.

    Returns None if fewer than 3 observations (meaningless).
    """
    if len(xs) != len(ys) or len(xs) < 3:
        return None
    rx = _rank(xs)
    ry = _rank(ys)
    n = len(rx)
    mx = sum(rx) / n
    my = sum(ry) / n
    cov = sum((rx[i] - mx) * (ry[i] - my) for i in range(n))
    sx = math.sqrt(sum((r - mx) ** 2 for r in rx))
    sy = math.sqrt(sum((r - my) ** 2 for r in ry))
    if sx < 1e-12 or sy < 1e-12:
        return None
    return cov / (sx * sy)


# ---------------------------------------------------------------------------
# Run data loading
# ---------------------------------------------------------------------------
def _fnum(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def load_cost_from_run(run_dir: Path, square_run_dir: Optional[Path] = None) -> Optional[Dict]:
    """Load J_joint proxy cost from a run's meta/metrics/result files.

    ``square_run_dir``: optional second run directory to take square-wave
    metrics from.  verify.sh (SOP-12) emits triangle and square legs as
    separate runs, so a group's cost needs one tri run + one step run.

    Source priority: meta.json (optimizer trial format) → metrics-v2.json
    (offline recompute) → result.json (inline v2 metrics — verify.sh legs
    only have this one until recompute_metrics runs).
    """
    # Try meta.json first (optimizer trial format)
    meta_path = run_dir / "meta.json"
    if meta_path.exists():
        with open(meta_path) as f:
            meta = json.load(f)
        cost = _fnum(meta.get("cost"))
        bd = meta.get("cost_breakdown", {})
        if cost is not None:
            return {"cost": cost, "breakdown": bd, "source": "meta.json"}

    # Minimal cost computation (weights from cost_spec)
    from a1z.analysis.optimize.cost_spec import compute_joint_cost

    # Resolve joint: meta (optimizer trial format) → single-entry metrics
    # map → default J6.  Standard run_test meta.json carries no "joint" key.
    joint = None
    if meta_path.exists():
        with open(meta_path) as f:
            meta = json.load(f)
        j1 = meta.get("joint")
        if j1 is not None:
            joint = j1 - 1

    def _single_joint_key(m: Dict) -> Optional[int]:
        keys = list(m.keys())
        if len(keys) == 1:
            return int(keys[0].lstrip("J")) - 1
        return None

    # --- Source 1: metrics-v2.json (offline recompute) ---------------------
    # Nested per joint: mv2["joints"]["J{n}"]["triangle"|"square"], native
    # units (deg / ms / pct) — same fields eval_runner uses.
    mv2_path = run_dir / "metrics-v2.json"
    if mv2_path.exists():
        with open(mv2_path) as f:
            mv2 = json.load(f)
        sq_mv2 = mv2
        if square_run_dir is not None:
            sq_path = Path(square_run_dir) / "metrics-v2.json"
            if sq_path.exists():
                with open(sq_path) as f:
                    sq_mv2 = json.load(f)
        joints_map = mv2.get("joints", {})
        j = joint if joint is not None else _single_joint_key(joints_map)
        if j is None:
            j = 5  # default to J6 (0-based) if ambiguous
        jm = joints_map.get(f"J{j + 1}", {})
        tri = jm.get("triangle", {})
        sq_jm = sq_mv2.get("joints", {}).get(f"J{j + 1}", {})
        step_sum = sq_jm.get("square", {}) or jm.get("square", {})
        lag_deg = _fnum(tri.get("lag_at_rate_deg"))
        if lag_deg is not None:
            ts_ms = _fnum(step_sum.get("ts_max_ms"))
            if ts_ms is None:
                ts_ms = 2000.0  # ms — eval_runner never-settles proxy
            cost, bd = compute_joint_cost(
                j,
                lag_deg,
                ts_ms,
                _fnum(tri.get("resid_std_deg")) or 0.0,
                _fnum(step_sum.get("ess_max_deg")) or 0.0,
                _fnum(step_sum.get("overshoot_max_pct"))
                or _fnum(step_sum.get("overshoot_pct")) or 0.0,
            )
            return {"cost": cost, "breakdown": bd, "source": "metrics-v2.json"}

    # --- Source 2: result.json (inline v2 metrics, always present) ---------
    # unit_tests/J{n}/{triangle,square}; field choices mirror eval_runner
    # (v2 dict's ts_max_ms / ess_max_deg / overshoot_max_pct).
    res_path = run_dir / "result.json"
    if not res_path.exists():
        return None
    with open(res_path) as f:
        result = json.load(f)
    sq_result = result
    if square_run_dir is not None:
        sq_res = Path(square_run_dir) / "result.json"
        if sq_res.exists():
            with open(sq_res) as f:
                sq_result = json.load(f)
    ut = result.get("unit_tests", {})
    j = joint if joint is not None else _single_joint_key(ut)
    if j is None:
        j = 5  # default to J6 (0-based) if ambiguous
    jm = ut.get(f"J{j + 1}", {})
    tri_v2 = jm.get("triangle", {}).get("v2", {})
    lag_deg = _fnum(tri_v2.get("lag_at_rate_deg"))
    if lag_deg is None:
        return None
    sq = sq_result.get("unit_tests", {}).get(f"J{j + 1}", {}).get("square", {})
    sq_v2 = sq.get("v2", {}) or sq
    ts_ms = _fnum(sq_v2.get("ts_max_ms"))
    if ts_ms is None:
        ts_ms = 2000.0  # ms — eval_runner never-settles proxy
    cost, bd = compute_joint_cost(
        j,
        lag_deg,
        ts_ms,
        _fnum(tri_v2.get("resid_std_deg")) or 0.0,
        _fnum(sq_v2.get("ess_max_deg")) or 0.0,
        _fnum(sq_v2.get("overshoot_max_pct")) or 0.0,
    )
    return {"cost": cost, "breakdown": bd, "source": "result.json"}


# ---------------------------------------------------------------------------
# J_ee channel (G6, 2026-08-03 F1): cost from an EE tracking run's archives
# ---------------------------------------------------------------------------
_PLANE_NORMAL = {"xz": (0.0, 1.0, 0.0), "yz": (1.0, 0.0, 0.0),
                 "xy": (0.0, 0.0, 1.0)}


def load_ee_cost_from_run(ee_run_dir: Path) -> Optional[Dict]:
    """Compute J_ee (cost_spec compute_ee_cost) from an EE run's archived
    ``ee-traj-{ref,response}.csv``.  Returns ``{"cost", "metrics", "source"}``
    or None when the archives are missing.

    The trajectory plane (for the jitter-projection normal) is read from the
    run's meta.json (``ee_traj.plane``), default xz.  tail_frac=0.1 matches
    the no-hold archive口径 (devlog 2026-08-01 E1 smoke / E5 replay).
    """
    import numpy as np

    from a1z.analysis.metrics import ee_refine_metrics
    from a1z.analysis.optimize.cost_spec import compute_ee_cost
    from a1z.analysis.report import read_ee_pose_csv

    ref_csv = ee_run_dir / "ee-traj-ref.csv"
    resp_csv = ee_run_dir / "ee-traj-response.csv"
    if not ref_csv.exists() or not resp_csv.exists():
        return None

    plane = "xz"
    meta_path = ee_run_dir / "meta.json"
    if meta_path.exists():
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            plane = (meta.get("ee_traj") or {}).get("plane", plane)
        except Exception:
            pass
    normal = np.array(_PLANE_NORMAL.get(plane, (0.0, 1.0, 0.0)), dtype=float)

    t_ref, T_ref = read_ee_pose_csv(ref_csv)
    t_resp, T_resp = read_ee_pose_csv(resp_csv)
    n = min(len(T_ref), len(T_resp))
    if n < 10:
        return None
    T_ref, T_resp = T_ref[:n], T_resp[:n]
    fs = 1.0 / float(np.median(np.diff(t_ref[:n])))
    m = ee_refine_metrics(T_ref, T_resp, fs, normal, tail_frac=0.1)
    cost, _ = compute_ee_cost(m)
    return {"cost": cost, "metrics": m, "source": "ee-traj CSV"}


# ---------------------------------------------------------------------------
# Main gate logic
# ---------------------------------------------------------------------------
def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="A3 Goodhart Gate evaluation (SOP-11 §5)")
    ap.add_argument("--groups-csv", required=True, type=str,
                    help="Path to CSV defining parameter groups + L2 truth")
    ap.add_argument("--output-dir", required=True, type=str,
                    help="Directory for a3-report.md and a3-summary.csv")
    return ap.parse_args()


def main() -> None:
    args = _parse_args()
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load groups
    groups: List[Dict] = []
    with open(args.groups_csv) as f:
        reader = csv.DictReader(f)
        for row in reader:
            g = {
                "name": row["group"],
                "run_dir": Path(row["run_dir"]),
                "l2_fail_rate": _fnum(row.get("l2_fail_rate")),
                "l2_insert_time_s": _fnum(row.get("l2_insert_time_s")),
            }
            # verify.sh (SOP-12) splits a group's L0 into separate tri/step
            # runs; the optional run_dir_square column points at the step leg.
            sq_dir = row.get("run_dir_square") or None
            cost_data = load_cost_from_run(
                g["run_dir"], Path(sq_dir) if sq_dir else None)
            g["j_joint"] = cost_data["cost"] if cost_data else None
            g["breakdown"] = cost_data.get("breakdown", {}) if cost_data else {}
            # J_ee channel (G6): optional run_dir_ee column with EE archives
            ee_dir = row.get("run_dir_ee") or None
            ee_data = load_ee_cost_from_run(Path(ee_dir)) if ee_dir else None
            g["j_ee"] = ee_data["cost"] if ee_data else None
            groups.append(g)

    names = [g["name"] for g in groups]
    j_joint_vals = [g["j_joint"] for g in groups]
    j_ee_vals = [g.get("j_ee") for g in groups]
    fail_rates = [g["l2_fail_rate"] for g in groups]

    # --- A3-1: Spearman(J_joint, fail_rate) ≥ 0.8 ---
    valid_jf = [(jj, fr) for jj, fr in zip(j_joint_vals, fail_rates)
                if jj is not None and fr is not None]
    rho_joint = None
    if len(valid_jf) >= 3:
        rho_joint = spearman_rho([x[0] for x in valid_jf], [x[1] for x in valid_jf])
    a3_1_pass = (rho_joint is not None and rho_joint >= A3_1_RHO_MIN)

    # --- A3-2: ρ(J_ee, task) > ρ(J_joint, task) ---
    # Fair comparison: both rhos computed on the SAME groups (those carrying
    # j_joint + j_ee + L2 truth).  PENDING when fewer than 3 complete groups.
    both = [(jj, je, fr) for jj, je, fr in zip(j_joint_vals, j_ee_vals, fail_rates)
            if jj is not None and je is not None and fr is not None]
    rho_ee = None
    rho_joint_paired = None
    a3_2_pass = False
    a3_2_pending = True
    if len(both) >= 3:
        rho_ee = spearman_rho([b[1] for b in both], [b[2] for b in both])
        rho_joint_paired = spearman_rho([b[0] for b in both], [b[2] for b in both])
        if rho_ee is not None and rho_joint_paired is not None:
            a3_2_pass = rho_ee > rho_joint_paired
            a3_2_pending = False

    # --- A3-3: no anti-correlated cost component ---
    # Check each breakdown term against fail_rate
    anti_corr_items: List[str] = []
    breakdown_keys = set()
    for g in groups:
        breakdown_keys.update(g.get("breakdown", {}).keys())
    for key in sorted(breakdown_keys):
        vals = [g["breakdown"].get(key) for g in groups]
        valid_kf = [(v, fr) for v, fr in zip(vals, fail_rates)
                    if v is not None and fr is not None]
        if len(valid_kf) >= 3:
            rho_k = spearman_rho([x[0] for x in valid_kf], [x[1] for x in valid_kf])
            if rho_k is not None and rho_k < -0.3:
                anti_corr_items.append(f"{key}(rho={rho_k:.2f})")
    a3_3_pass = len(anti_corr_items) == 0

    # --- A3-4: P_bad vs P_hand gap >> sigma ---
    p_bad_cost = None
    p_hand_cost = None
    for g in groups:
        if "bad" in g["name"].lower():
            p_bad_cost = g["j_joint"]
        if "hand" in g["name"].lower():
            p_hand_cost = g["j_joint"]
    a3_4_pass = False
    a3_4_detail = "insufficient data"
    if p_bad_cost is not None and p_hand_cost is not None:
        gap = abs(p_bad_cost - p_hand_cost)
        valid_costs = [jj for jj in j_joint_vals if jj is not None]
        if len(valid_costs) >= 2:
            sigma = st.stdev(valid_costs)
            a3_4_pass = gap > A3_4_SIGMA_MULT * sigma if sigma > 0 else gap > 0
            a3_4_detail = f"gap={gap:.4f}, sigma={sigma:.4f}, ratio={gap / sigma:.1f}" if sigma > 0 else f"gap={gap:.4f}"

    # --- Write report ---
    lines: List[str] = []
    W = lines.append
    W("# A3 Goodhart Gate Report")
    W("")
    W(f"> Generated: {__import__('datetime').datetime.now().isoformat()}")
    W(f"> Ref: SOP-11 §5")
    W("")
    W("## Parameter Groups")
    W("")
    W("| Group | J_joint | J_ee | L2 fail_rate | L2 insert_time_s |")
    W("|---|---|---|---|---|")
    def _fmt(v, spec):
        return f"{v:{spec}}" if v is not None else "N/A"

    for g in groups:
        W(f"| {g['name']} | {_fmt(g['j_joint'], '.4f')} "
          f"| {_fmt(g.get('j_ee'), '.4f')} "
          f"| {_fmt(g['l2_fail_rate'], '.2f')} "
          f"| {_fmt(g['l2_insert_time_s'], '.1f')} |")
    W("")
    W("## Gate Verdicts")
    W("")
    W(f"| # | Criterion | Value | Pass | Required |")
    W("|---|---|---|---|---|")
    W(f"| A3-1 | ρ(J_joint, fail_rate) | "
      f"{_fmt(rho_joint, '.3f')} | "
      f"{'PASS' if a3_1_pass else 'FAIL'} | ≥{A3_1_RHO_MIN} |")
    a3_2_status = ("PENDING" if a3_2_pending
                   else ("PASS" if a3_2_pass else "FAIL"))
    W(f"| A3-2 | ρ(J_ee, task) > ρ(J_joint, task) | "
      f"rho_ee={'N/A' if rho_ee is None else f'{rho_ee:.3f}'} vs "
      f"rho_joint={_fmt(rho_joint_paired, '.3f')} (同组配对) | "
      f"{a3_2_status} | hard |")
    W(f"| A3-3 | No anti-correlated terms | "
      f"{'none' if not anti_corr_items else ','.join(anti_corr_items)} | "
      f"{'PASS' if a3_3_pass else 'FAIL'} | hard |")
    W(f"| A3-4 | P_bad vs P_hand gap >> σ | {a3_4_detail} | "
      f"{'PASS' if a3_4_pass else 'FAIL'} | sanity |")
    W("")

    # Decision (SOP-11 §5.4)
    W("## Decision")
    W("")
    if a3_1_pass and a3_2_pass:
        W("**A3-1 + A3-2 PASS** → EE 权重 > 关节权重成立，按 §2.3 合成代价冻结，进 Phase A。")
    elif a3_1_pass and a3_2_pending:
        W("**A3-1 PASS, A3-2 PENDING** → 完整配对（J_joint + J_ee + L2 真值）的组 <3；"
          "补齐 `run_dir_ee` 与 L0 双腿 run 后重跑本脚本方可判定 EE 权重前提。")
    elif a3_1_pass and not a3_2_pass:
        W("**A3-1 PASS, A3-2 FAIL** → 退回 Q4 原设计：EE 降为 L1/L2 验证、关节代价主导内环。")
    elif not a3_1_pass:
        W("**A3-1 FAIL** → 代理设计错误，回 Q3/§2 重设 EE 测试。")
    if anti_corr_items:
        W(f"\n**A3-3 FAIL** → 从代价函数剔除反号项: {', '.join(anti_corr_items)}，记入 cost_spec 版本注记。")
    W("")

    report_path = out_dir / "a3-report.md"
    with open(report_path, "w") as f:
        f.write("\n".join(lines))
    print(f"[a3_gate] Report: {report_path}")

    # Write CSV summary
    csv_path = out_dir / "a3-summary.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["group", "j_joint", "j_ee", "l2_fail_rate", "l2_insert_time_s"])
        for g in groups:
            w.writerow([
                g["name"],
                f"{g['j_joint']:.6f}" if g["j_joint"] is not None else "",
                f"{g.get('j_ee'):.6f}" if g.get("j_ee") is not None else "",
                f"{g['l2_fail_rate']:.4f}" if g["l2_fail_rate"] is not None else "",
                f"{g['l2_insert_time_s']:.2f}" if g["l2_insert_time_s"] is not None else "",
            ])
    print(f"[a3_gate] Summary CSV: {csv_path}")

    # Overall verdict
    overall = "PASS" if (a3_1_pass and a3_3_pass) else "FAIL"
    print(f"[a3_gate] Overall: {overall} (A3-1={'P' if a3_1_pass else 'F'} "
          f"A3-2={a3_2_status} "
          f"A3-3={'P' if a3_3_pass else 'F'} "
          f"A3-4={'P' if a3_4_pass else 'F'})")


if __name__ == "__main__":
    main()
