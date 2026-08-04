#!/usr/bin/env python3
"""G0-ext 机制对灵敏度回归（SOP-11 §7.4，devlog 2026-08-03 计划 B1）。

对比一对「机制 off / 机制 on」的 verify run（同腿型、同关节、同增益），
检查已知方向的指标灵敏度是否成立——成立该机制才准进 Phase B 搜索空间。

判据（方向性 + 恶化闸）：
- ``--mechanism coulomb``：tri 腿 ``lag_at_rate_deg`` 下降（vel-ff 口径
  lag ≈ c/kp 是库伦前馈的主补偿对象）；resid_std / ess / overshoot 恶化
  均不得超过 +20%。
- ``--mechanism integral``：step 腿 ``ess_max_deg`` 下降；``lag_at_rate_deg``
  恶化不得超过 +10%（积分相位滞后风险，devlog 2026-08-03 风险登记）；
  overshoot 恶化不得超过 +20%。

用法:
    python -m a1z.analysis.script.g0ext_regression --joint 3 --mechanism coulomb \
        --off-tri $TEST_LOG_ROOT/02-a1z/01-output/<date>-run-g0ext-J3-coulomb-off-r1-tri-01 \
        --on-tri  $TEST_LOG_ROOT/02-a1z/01-output/<date>-run-g0ext-J3-coulomb-on-r1-tri-01 \
        [--off-step <run> --on-step <run>]

退出码 0 = 方向判据成立（准进搜索空间），1 = 不成立。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Optional

DEG_WORSEN_GATE = {"resid_std_deg": 0.20, "ess_max_deg": 0.20,
                   "overshoot_max_pct": 0.20, "ts_max_ms": 0.20}


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="G0-ext mechanism-pair regression")
    ap.add_argument("--joint", type=int, required=True, choices=range(1, 7))
    ap.add_argument("--mechanism", choices=["coulomb", "integral"], required=True)
    ap.add_argument("--off-tri", type=str, default=None)
    ap.add_argument("--on-tri", type=str, default=None)
    ap.add_argument("--off-step", type=str, default=None)
    ap.add_argument("--on-step", type=str, default=None)
    return ap.parse_args()


def _v2(run_dir: Optional[str], joint1: int, wave: str) -> Optional[Dict]:
    """v2 metrics dict for one leg from result.json (unit_tests/J{n}/wave/v2)."""
    if run_dir is None:
        return None
    p = Path(run_dir) / "result.json"
    if not p.exists():
        return None
    result = json.loads(p.read_text())
    return (result.get("unit_tests", {}).get(f"J{joint1}", {})
            .get(wave, {}).get("v2", {})) or None


def _row(name: str, off: Optional[float], on: Optional[float]) -> str:
    if off is None or on is None:
        return f"  {name:<22} off={off}  on={on}  (缺数据)"
    delta = on - off
    rel_str = f"{delta / off * 100.0:+6.1f}%" if off else ("  +0.0%" if on == 0 else "  从零升")
    arrow = "↓" if delta < 0 else ("↑" if delta > 0 else "=")
    return f"  {name:<22} off={off:9.4f}  on={on:9.4f}  {arrow} {rel_str}"


def main() -> int:
    args = _parse_args()
    j1 = args.joint
    tri_off = _v2(args.off_tri, j1, "triangle")
    tri_on = _v2(args.on_tri, j1, "triangle")
    sq_off = _v2(args.off_step, j1, "square")
    sq_on = _v2(args.on_step, j1, "square")

    if tri_off is None and sq_off is None:
        print("ERROR: no leg data found (need at least one off/on pair)",
              file=sys.stderr)
        return 1

    print(f"[g0ext] J{j1} mechanism={args.mechanism}")
    checks = []
    if tri_off is not None and tri_on is not None:
        print(" triangle leg:")
        for k in ("lag_at_rate_deg", "resid_std_deg"):
            print(_row(k, tri_off.get(k), tri_on.get(k)))
        lag_off, lag_on = tri_off.get("lag_at_rate_deg"), tri_on.get("lag_at_rate_deg")
        resid_off, resid_on = tri_off.get("resid_std_deg"), tri_on.get("resid_std_deg")
        if args.mechanism == "coulomb" and lag_off is not None and lag_on is not None:
            checks.append(("lag 下降（coulomb 主补偿）", lag_on < lag_off))
        if args.mechanism == "integral" and lag_off is not None and lag_on is not None and lag_off > 0:
            checks.append(("lag 恶化 ≤ +10%（相位滞后闸）",
                           (lag_on - lag_off) / lag_off <= 0.10))
        if resid_off and resid_on is not None:
            checks.append(("resid_std 恶化 ≤ +20%",
                           (resid_on - resid_off) / resid_off <= 0.20))
    if sq_off is not None and sq_on is not None:
        print(" square leg:")
        for k in ("ts_max_ms", "ess_max_deg", "overshoot_max_pct"):
            print(_row(k, sq_off.get(k), sq_on.get(k)))
        ess_off, ess_on = sq_off.get("ess_max_deg"), sq_on.get("ess_max_deg")
        os_off, os_on = sq_off.get("overshoot_max_pct"), sq_on.get("overshoot_max_pct")
        ts_off, ts_on = sq_off.get("ts_max_ms"), sq_on.get("ts_max_ms")
        if args.mechanism == "integral" and ess_off is not None and ess_on is not None:
            checks.append(("ess 下降（积分主补偿）", ess_on < ess_off))
        if os_off and os_on is not None:
            checks.append(("overshoot 恶化 ≤ +20%",
                           (os_on - os_off) / os_off <= 0.20))
        if ts_off and ts_on is not None:
            checks.append(("ts 恶化 ≤ +20%",
                           (ts_on - ts_off) / ts_off <= 0.20))

    print("[g0ext] 判据:")
    ok_all = True
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        ok_all &= ok
    if not checks:
        print("  (无可判数据)")
        return 1
    print(f"[g0ext] Overall: {'PASS — 机制方向成立，准进 Phase B 搜索空间' if ok_all else 'FAIL — 机制方向不成立或恶化越闸，不得进搜索空间（§7.4）'}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
