#!/usr/bin/env python3
"""离线回放回归：hf_osc 阶跃沿豁免（v9）对历史 trial/verify 数据的判定对比。

用真实 ``TickWatchdog`` 代码路径（``make_tick_watchdog`` + B1 标定 JSON）
逐拍喂入归档 CSV，比较豁免关闭（= v8 行为）与豁免开启（= v9 行为）的
hf_osc 判定差异，并与 trial meta.json 中记录的在线 verdict 对照。

用法::

    python -m a1z.analysis.script.replay_hf_step_exempt \
        --session <session-dir> [--verify-csv CSV ...] \
        [--calib <watchdog_thresholds.json>]

预期（devlog 2026-08-01 归因分析）：v8 口径下 t001/t006/t013/t043 的
hf_osc 违例全部是方波沿瞬态，v9 口径下不再 trip；t042 等可行 trial 两种
口径均不 trip；豁免窗口外若仍有超阈窗则单独列出（真实振荡不可豁免）。
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from a1z.analysis.optimize.watchdog import make_tick_watchdog

JOINT1 = 6  # J6 session
J = JOINT1 - 1


def _row_to_vecs(row: Dict[str, str]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    ref = np.zeros(6)
    resp = np.zeros(6)
    eff = np.zeros(6)
    ref[J] = float(row["ref"])
    resp[J] = float(row["resp"])
    eff[J] = float(row["eff"])
    return ref, resp, eff


def replay(csv_path: Path, calib: Optional[Path], exempt: bool
           ) -> Tuple[bool, str, List[Tuple[float, float]]]:
    """Feed one CSV through TickWatchdog.  Returns (tripped, reason,
    over_windows) where over_windows lists (t, hf_rms) of every over-threshold
    evaluation *outside* exemption periods (v9) or all of them (v8)."""
    kw = {} if exempt else {"hf_step_exempt_deg": None}
    wd = make_tick_watchdog(J, calib, **kw)
    overs: List[Tuple[float, float]] = []
    reason = ""
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        t = float(row["t"])
        ref, resp, eff = _row_to_vecs(row)
        ok, r = wd.check(t, ref, resp, eff)
        # record over-threshold evaluations for audit (mirror check() logic)
        if (len(wd._eff_buf) >= wd.hf_window
                and wd._hf_baseline_rms
                and t >= wd._hf_exempt_until):
            import math
            rms = math.sqrt(sum(d * d for d in wd._eff_buf) / len(wd._eff_buf))
            if rms > wd.theta_hf_scale * wd._hf_baseline_rms:
                overs.append((t, rms))
        if not ok:
            reason = r
            return True, reason, overs
    return False, "", overs


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", type=Path, default=None,
                    help="BO session dir with trials/tNNN/{meta.json,unit-*.csv}")
    ap.add_argument("--verify-csv", type=Path, nargs="*", default=[],
                    help="extra verify CSVs to replay (label = parent dir name)")
    ap.add_argument("--calib", type=Path, default=None,
                    help="watchdog thresholds JSON (B1 calib)")
    args = ap.parse_args()

    print(f"{'run':60s} {'leg':9s} {'online':10s} {'v8-replay':10s} "
          f"{'v9-replay':10s} v9-over-outside-exempt")
    n_changed = 0
    n_masked_real = 0

    def do(csv_path: Path, label: str, leg: str, online: str) -> None:
        nonlocal n_changed, n_masked_real
        t8, r8, _ = replay(csv_path, args.calib, exempt=False)
        t9, r9, overs9 = replay(csv_path, args.calib, exempt=True)
        v8 = "TRIP" if t8 else "ok"
        v9 = "TRIP" if t9 else "ok"
        if v8 != v9:
            n_changed += 1
        if overs9 and not t9:
            n_masked_real += 1
        over_s = ",".join(f"{t:.2f}s/{r:.3f}" for t, r in overs9[:4])
        print(f"{label:60s} {leg:9s} {online:10s} {v8:10s} {v9:10s} {over_s}")
        if t9:
            print(f"{'':60s} v9 reason: {r9}")

    if args.session:
        trials = sorted((args.session / "trials").glob("t*"))
        for td in trials:
            meta = json.loads((td / "meta.json").read_text())
            online = meta.get("watchdog_verdict", "?")
            online_s = "ok" if online == "ok" else "VIOLATED"
            for leg in ("triangle", "square"):
                p = td / f"unit-J{JOINT1}-{leg}.csv"
                if p.exists():
                    do(p, td.name, leg, online_s if leg == "square" else "")
    for p in args.verify_csv:
        do(p, p.parent.name, p.stem.split("-")[-1], "")

    print(f"\nchanged verdicts (v8→v9): {n_changed}")
    print(f"runs with over-threshold windows OUTSIDE exemption (audit): "
          f"{n_masked_real}")


if __name__ == "__main__":
    main()
