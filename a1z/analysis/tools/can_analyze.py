#!/usr/bin/env python3
"""CAN 抓包分析工具：读取 can_capture.py 产出的 CSV，按 CAN ID 统计帧率/到达间隔，
并给出目标关节 vs 基线关节的分岔判读。

背景：01-docs/01-devlogs/2026-07-20.md §5 "下一步" 第 1 条——J6 在 SDK 侧观测到的位置
反馈更新率仅 ~6Hz（正常应接近 SDK 250Hz 控制环），但不确定根因在总线层还是 SDK 内部。
本工具把抓包 CSV 里同一份数据按 CAN ID 分组算帧率，与其余关节（baseline）对比，得到
两分岔判读：
  - 目标 ID 帧率显著低于 baseline → 总线上确实缺帧
    → 查命令/请求侧（电机没发，或轮询请求没送到电机；可配合
      `python tools/motor_diag.py --probe <joint>` 进一步探测该电机）
  - 目标 ID 帧率与 baseline 相当（未显著更低）→ 总线帧本身正常
    → 查 driver RX 分发/过滤（SDK `motor_chain` 的 250Hz 轮询预算分配、按 CAN ID 的
      分发/过滤逻辑是否把该 ID 挤掉）

用法（CLI）：
    # 基本用法：打印逐 ID 帧率表（不做分岔判读）
    python -m a1z.analysis.tools.can_analyze capture.csv

    # 指定怀疑丢帧的目标关节 + 对照基线关节（默认基线 = 抓包里除目标外的全部 ID）
    python -m a1z.analysis.tools.can_analyze capture.csv --target 6 --baseline 1,2,3,4,5

    # 判读阈值：目标/基线帧率比值低于此值才判"总线缺帧"分支（默认 0.5）
    python -m a1z.analysis.tools.can_analyze capture.csv --target 6 --ratio-threshold 0.5

    # SDK 控制环频率参照（仅用于展示，默认 250，见 script/run_test.py meta.hardware.sdk_loop_hz）
    python -m a1z.analysis.tools.can_analyze capture.csv --target 6 --expected-hz 250

    # 把逐 ID 统计 + 判读结果导出为 JSON（便于贴进 devlog / meta.json 附录）
    python -m a1z.analysis.tools.can_analyze capture.csv --target 6 --json-out stats.json

--json-out 的相对路径解析规则同 can_capture.py 的 --out（见
01-docs/04-sops/07-测试数据管理-SOP.md）：设置了 TEST_LOG_ROOT 时，相对路径落到
$TEST_LOG_ROOT/02-a1z/output/ 下；未设置时相对当前工作目录（绝对路径始终原样）。
输入的 capture_csv 是已存在文件的路径，按当前工作目录/绝对路径正常打开，不做重解析。
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List, Optional

from a1z.analysis.tools.can_capture import (
    JOINT_NAMES, fmt_id, parse_ids, resolve_output_path,
)


def _percentile(sorted_vals: List[float], pct: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * (pct / 100.0)
    lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
    frac = k - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def load_capture(path: Path) -> Dict[int, List[float]]:
    """CSV → {can_id: [t_rel_s, ...]}（按到达顺序，已经是时间升序）。"""
    times: Dict[int, List[float]] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            can_id = int(row["can_id_hex"], 16)
            times.setdefault(can_id, []).append(float(row["t_rel_s"]))
    return times


def id_stats(t: List[float]) -> dict:
    """单个 CAN ID 的帧率 / 到达间隔统计。"""
    n = len(t)
    if n < 2:
        return {"count": n, "span_s": 0.0, "rate_hz": 0.0,
                "dt_mean_ms": float("nan"), "dt_median_ms": float("nan"),
                "dt_p90_ms": float("nan"), "dt_max_ms": float("nan")}
    span = t[-1] - t[0]
    diffs_ms = sorted((b - a) * 1000.0 for a, b in zip(t, t[1:]))
    return {
        "count": n,
        "span_s": span,
        "rate_hz": (n - 1) / span if span > 0 else float("nan"),
        "dt_mean_ms": sum(diffs_ms) / len(diffs_ms),
        "dt_median_ms": _percentile(diffs_ms, 50),
        "dt_p90_ms": _percentile(diffs_ms, 90),
        "dt_max_ms": diffs_ms[-1],
    }


def _print_table(stats: Dict[int, dict], expected_hz: float) -> None:
    print(f"\n逐 CAN ID 帧率统计（参照 SDK 控制环 {expected_hz:.0f} Hz）:")
    header = f"{'ID':16s} {'帧数':>6s} {'时长(s)':>8s} {'帧率(Hz)':>9s} {'占预期%':>8s} " \
             f"{'均值(ms)':>9s} {'p90(ms)':>9s} {'最大间隔(ms)':>12s}"
    print(header)
    print("-" * len(header))
    for can_id in sorted(stats):
        s = stats[can_id]
        pct = 100.0 * s["rate_hz"] / expected_hz if expected_hz > 0 else float("nan")
        print(
            f"{fmt_id(can_id):16s} {s['count']:6d} {s['span_s']:8.2f} "
            f"{s['rate_hz']:9.2f} {pct:7.1f}% "
            f"{s['dt_mean_ms']:9.1f} {s['dt_p90_ms']:9.1f} {s['dt_max_ms']:12.1f}"
        )


def verdict(
    stats: Dict[int, dict],
    target: int,
    baseline_ids: List[int],
    ratio_threshold: float,
) -> dict:
    target_rate = stats.get(target, {"rate_hz": 0.0, "count": 0})["rate_hz"]
    baseline_rates = [stats[i]["rate_hz"] for i in baseline_ids if i in stats]
    baseline_rate = sum(baseline_rates) / len(baseline_rates) if baseline_rates else float("nan")
    ratio = (target_rate / baseline_rate) if baseline_rate and baseline_rate > 0 else float("nan")

    on_bus = ratio == ratio and ratio >= ratio_threshold  # ratio==ratio 排除 NaN
    branch = "on_bus_stale_in_sdk" if on_bus else "missing_from_bus"

    print("\n" + "=" * 60)
    print("分岔判读")
    print("=" * 60)
    print(f"目标 {fmt_id(target)}: 帧率 = {target_rate:.2f} Hz（抓包实测，来自总线，与 SDK 无关）")
    if baseline_rates:
        print(f"基线 {'+'.join(fmt_id(i) for i in baseline_ids if i in stats)} 均值: "
              f"帧率 = {baseline_rate:.2f} Hz")
        print(f"比值 target/baseline = {ratio:.3f}  (判读阈值 {ratio_threshold:.2f})")
    else:
        print("基线 ID 均未出现在抓包数据中，无法与 baseline 比较——仅按目标 ID 是否存在判读。")

    print()
    if branch == "missing_from_bus":
        print("判读: 【帧不在总线上】——目标 ID 的帧率显著低于基线（或抓包里完全没有）。")
        print("  → 查命令/请求侧: 电机没发送反馈帧，或 SDK 对该关节的轮询/请求没有送达电机。")
        print("    下一步建议:")
        print(f"      python tools/motor_diag.py --probe {target}   # 单独探测该电机是否在线/响应")
        print("      检查 SDK motor_chain 对该关节的发送逻辑是否被跳过/延迟")
    else:
        print("判读: 【帧在总线上，SDK 状态没更新】——目标 ID 帧率与基线相当，总线本身不缺帧。")
        print("  → 查 driver RX 分发/过滤: SDK 250Hz 轮询预算分配、按 CAN ID 的分发/过滤逻辑")
        print("    是否把该 ID 挤掉（参考 devlog 2026-07-20 §5 下一步第 2 条）。")
    print("=" * 60)

    return {
        "target_id": target, "target_rate_hz": target_rate,
        "baseline_ids": baseline_ids, "baseline_rate_hz": baseline_rate,
        "ratio": ratio, "ratio_threshold": ratio_threshold, "branch": branch,
    }


def build_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="分析 can_capture.py 抓包 CSV：逐 ID 帧率 + 分岔判读")
    ap.add_argument("capture_csv", type=Path, help="can_capture.py --out 产出的 CSV")
    ap.add_argument("--target", type=int, default=None,
                     help="怀疑丢帧的目标关节号/CAN ID（1-7；不给则只打印统计表，不判读）")
    ap.add_argument("--baseline", default=None,
                     help="对照基线关节号，逗号分隔（默认 = 抓包里除 target 外的全部 ID）")
    ap.add_argument("--ratio-threshold", type=float, default=0.5,
                     help="target/baseline 帧率比值低于此值才判'总线缺帧'分支（默认 0.5）")
    ap.add_argument("--expected-hz", type=float, default=250.0,
                     help="SDK 控制环频率参照，仅用于展示占比（默认 250）")
    ap.add_argument("--json-out", type=Path, default=None, help="导出统计+判读为 JSON")
    return ap.parse_args()


def main() -> None:
    args = build_args()
    if not args.capture_csv.exists():
        raise SystemExit(f"找不到抓包文件: {args.capture_csv}")

    times = load_capture(args.capture_csv)
    if not times:
        raise SystemExit("抓包文件里没有任何有效帧。")

    stats = {can_id: id_stats(t) for can_id, t in times.items()}
    _print_table(stats, args.expected_hz)

    result: dict = {"ids": {f"0x{k:02X}": v for k, v in stats.items()}}

    if args.target is not None:
        baseline_ids = (parse_ids(args.baseline) if args.baseline
                         else sorted(set(stats) - {args.target}))
        result["verdict"] = verdict(stats, args.target, sorted(baseline_ids),
                                     args.ratio_threshold)

    if args.json_out:
        json_out = resolve_output_path(args.json_out)
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(json.dumps(result, indent=2, ensure_ascii=False))
        print(f"\n[can_analyze] 已写入 {json_out}")


if __name__ == "__main__":
    main()
