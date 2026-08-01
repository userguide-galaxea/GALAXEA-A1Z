#!/usr/bin/env python3
"""B1 watchdog 阈值标定（SOP-11 §15.1.7）。

消费 B1 run（norm = 默认增益，hi = 故意高 kp 激发振荡）的 ``unit-J*-triangle.csv``，
对 ``eff`` 列做滑动窗（默认 50 拍 = 0.5 s @100 Hz）差分 RMS，并额外计算响应
速度/加速度（devlog 2026-07-29：J2 等低 sensed-inertia 大关节的“运动学暴力”
在力矩通道上不可见，必须用 q̇/q̈ 通道）。

按关节比较 norm/hi 两组的 per-run 极值分布：

- ``theta_hf``   = norm 组最大值与 hi 组最小值的中点（eff 差分 RMS 分离点）
- ``theta_vel``  = norm 组最大值与 hi 组最小值的中点（|resp_vel − ref_vel|）
- ``theta_acc``  = norm 组最大值与 hi 组最小值的中点（|resp_acc| p99）
- ``theta_pos``  = norm 组 ``|ref-resp|`` 全程最大值的 2×

分布不交叠时成立；交叠或缺 hi 组时回退为 norm 最大值 × 裕量并在 JSON 标记。

分组唯一真值 = ``meta.pd.override.kp``（null → norm，SOP-10 R8：不从 run-id 猜）。

产物：
    watchdog_thresholds.json   —— v4，每关节 theta_hf/theta_vel/theta_acc/theta_pos
                               acc 通道改为与在线 watchdog 一致的 kin_window RMS 最大值
    watchdog-calib-<J>.png     —— 三通道 norm vs hi 分布 + 阈值线（快查）

无硬件、不重算指标——只读 run 目录内的 CSV 与 meta.json。

用法：
    python -m a1z.analysis.script.calibrate_watchdog \
        --runs $TEST_LOG_ROOT/02-a1z/01-output/<date>-run-B1-* \
        --output <gate-dir>/watchdog_thresholds.json
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import statistics as st
import sys
from pathlib import Path

import numpy as np

DEG = 180.0 / math.pi

# 缺 hi 组 / 分布交叠时的回退裕量（在 JSON 中显式标记，非静默）
FALLBACK_MULT = 1.5
NO_HI_MULT = 3.0


def _window_rms(x: np.ndarray, window: int) -> np.ndarray:
    """Sliding-window RMS (right-aligned, same semantics as TickWatchdog)."""
    if x.size < window:
        return np.array([])
    sq = np.concatenate([[0.0], np.cumsum(x * x)])
    return np.sqrt((sq[window:] - sq[:-window]) / window)


# ---------------------------------------------------------------------------
# Run 载入
# ---------------------------------------------------------------------------
class Run:
    __slots__ = ("dir", "joint", "group", "kp_applied",
                 "hf_rms_max", "hf_rms_med", "pos_err_max_rad", "n_ticks",
                 "vel_err_max", "resp_acc_window_rms_max", "resp_acc_p99", "resp_acc_rms")

    def __init__(self, rundir: Path, window: int, kin_window: int):
        self.dir = rundir.name
        meta = json.loads((rundir / "meta.json").read_text())
        applied = (meta.get("pd", {}).get("applied", {}) or {})
        joints = list(applied.keys())
        if not joints:
            raise ValueError("meta.pd.applied 为空，无法确定激励关节")
        self.joint = joints[0]
        j = int(self.joint[1:]) - 1
        kp_vec = applied[self.joint].get("kp")
        self.kp_applied = float(kp_vec[j]) if kp_vec else None
        # 分组真值：pd.override.kp null → norm（默认增益），否则 hi
        self.group = "norm" if (meta.get("pd", {}).get("override", {}) or {}
                                ).get("kp") is None else "hi"

        csv_path = rundir / f"unit-{self.joint}-triangle.csv"
        if not csv_path.exists():
            raise ValueError(f"缺 {csv_path.name}")
        d = np.genfromtxt(csv_path, delimiter=",", names=True)
        eff = np.atleast_1d(d["eff"])
        ref = np.atleast_1d(d["ref"])
        resp = np.atleast_1d(d["resp"])
        t = np.atleast_1d(d["t"])
        self.n_ticks = int(eff.size)
        if self.n_ticks < window + 1:
            raise ValueError(f"采样数 {self.n_ticks} < 窗 {window}+1（estop 过早？）")

        # --- HF effort channel (existing) ---
        diff = np.diff(eff)
        cs = np.concatenate([[0.0], np.cumsum(diff * diff)])
        win_rms = np.sqrt((cs[window:] - cs[:-window]) / window)
        self.hf_rms_max = float(np.max(win_rms))
        self.hf_rms_med = float(np.median(win_rms))
        self.pos_err_max_rad = float(np.max(np.abs(resp - ref)))

        # --- Kinematic channels (new) ---
        # np.gradient gives central differences; boundary effects are negligible
        # for per-run max/p99 metrics.
        resp_vel = np.gradient(resp, t)
        ref_vel = np.gradient(ref, t)
        resp_acc = np.gradient(resp_vel, t)

        vel_err = np.abs(resp_vel - ref_vel)
        self.vel_err_max = float(np.max(_window_rms(vel_err, kin_window))) \
            if self.n_ticks >= kin_window else 0.0

        abs_acc = np.abs(resp_acc)
        self.resp_acc_window_rms_max = float(np.max(_window_rms(abs_acc, kin_window))) \
            if self.n_ticks >= kin_window else 0.0
        self.resp_acc_p99 = float(np.percentile(abs_acc, 99))
        self.resp_acc_rms = float(np.sqrt(np.mean(abs_acc * abs_acc)))


def load_runs(patterns, window: int, kin_window: int):
    runs = []
    for pat in patterns:
        for p in sorted(glob.glob(pat)):
            rd = Path(p)
            if not rd.is_dir():
                continue
            try:
                runs.append(Run(rd, window, kin_window))
            except Exception as e:  # noqa: BLE001 — one bad run must not sink the batch
                print(f"[WARN] skip {rd.name}: {type(e).__name__}: {e}")
    return runs


# ---------------------------------------------------------------------------
# 标定辅助
# ---------------------------------------------------------------------------
def _separation_threshold(
    norm_vals, hi_vals, *,
    fallback_mult: float = FALLBACK_MULT,
    no_hi_mult: float = NO_HI_MULT,
) -> Tuple[float, Optional[bool], Optional[str]]:
    """Return (theta, separated, warning) using norm-max / hi-min midpoint rule."""
    norm_max = max(norm_vals)
    if not hi_vals:
        return norm_max * no_hi_mult, None, f"缺 hi 组，回退 norm_max×{no_hi_mult}"
    hi_min = min(hi_vals)
    separated = hi_min > norm_max
    if separated:
        return 0.5 * (norm_max + hi_min), True, None
    return norm_max * fallback_mult, False, (
        f"norm/hi 交叠（norm_max={norm_max:.4f} ≥ hi_min={hi_min:.4f}），"
        f"回退 norm_max×{fallback_mult}")


# ---------------------------------------------------------------------------
# 标定
# ---------------------------------------------------------------------------
def calibrate_joint(joint: str, runs, window: int, kin_window: int,
                    pos_mult: float) -> dict:
    norm = [r for r in runs if r.group == "norm"]
    hi = [r for r in runs if r.group == "hi"]
    out = {
        "window_ticks": window,
        "kin_window": kin_window,
        "norm_runs": [r.dir for r in norm],
        "hi_runs": [r.dir for r in hi],
    }
    if not norm:
        out["error"] = "无 norm 组 run，无法标定"
        return out

    # --- HF effort channel ---
    out["norm_hf_rms_max"] = {r.dir: r.hf_rms_max for r in norm}
    out["hi_hf_rms_max"] = {r.dir: r.hf_rms_max for r in hi}
    hf_base = st.median(r.hf_rms_med for r in norm)
    out["hf_baseline_rms"] = hf_base
    theta_hf, hf_sep, hf_warn = _separation_threshold(
        [r.hf_rms_max for r in norm], [r.hf_rms_max for r in hi])
    out["theta_hf"] = theta_hf
    out["separated_hf"] = hf_sep
    out["theta_hf_scale_implied"] = theta_hf / hf_base if hf_base > 0 else None

    # --- Velocity-error channel ---
    out["norm_vel_err_max"] = {r.dir: r.vel_err_max for r in norm}
    out["hi_vel_err_max"] = {r.dir: r.vel_err_max for r in hi}
    vel_base = st.median(r.vel_err_max for r in norm)
    out["vel_baseline_rms"] = vel_base
    theta_vel, vel_sep, vel_warn = _separation_threshold(
        [r.vel_err_max for r in norm], [r.vel_err_max for r in hi])
    out["theta_vel_deg_s"] = theta_vel * DEG
    out["separated_vel"] = vel_sep

    # --- Acceleration channel ---
    # 与 online TickWatchdog 保持一致：用 kin_window 滑动窗对 |q̈_resp| 做 RMS 后的最大值，
    # 而不是单点 p99。原 p99 作为参考保留。
    out["norm_resp_acc_window_rms_max"] = {r.dir: r.resp_acc_window_rms_max for r in norm}
    out["hi_resp_acc_window_rms_max"] = {r.dir: r.resp_acc_window_rms_max for r in hi}
    out["norm_resp_acc_p99"] = {r.dir: r.resp_acc_p99 for r in norm}
    out["hi_resp_acc_p99"] = {r.dir: r.resp_acc_p99 for r in hi}
    acc_base = st.median(r.resp_acc_rms for r in norm)
    out["acc_baseline_rms"] = acc_base
    theta_acc, acc_sep, acc_warn = _separation_threshold(
        [r.resp_acc_window_rms_max for r in norm],
        [r.resp_acc_window_rms_max for r in hi])
    out["theta_acc_deg_s2"] = theta_acc * DEG
    out["separated_acc"] = acc_sep

    # --- Position channel ---
    pos_max = max(r.pos_err_max_rad for r in norm)
    out["norm_pos_err_max_deg"] = pos_max * DEG
    out["theta_pos_rad"] = pos_max * pos_mult
    out["theta_pos_deg"] = pos_max * pos_mult * DEG

    # --- Warnings ---
    warnings = [w for w in (hf_warn, vel_warn, acc_warn) if w]
    if warnings:
        out["warning"] = "; ".join(warnings)

    return out


# ---------------------------------------------------------------------------
# 绘图（快查：三通道 norm vs hi 分布 + 阈值线）
# ---------------------------------------------------------------------------
def _plot_channel(ax, jruns, joint, cal, channel: str, metric: str,
                  theta_key: str, ylabel: str, deg_scale: float = 1.0):
    """Scatter one channel: norm vs hi with threshold line.

    ``deg_scale`` converts the stored per-run metric (rad/s or rad/s2) to the
    y-axis unit (deg/s or deg/s2) so the scatter points sit on the same scale
    as the threshold line.
    """
    for grp, x, c in (("norm", 0, "tab:blue"), ("hi", 1, "tab:red")):
        ys = [getattr(r, metric) * deg_scale for r in jruns if r.group == grp]
        ax.scatter([x] * len(ys), ys, color=c, zorder=3,
                   label=f"{grp} (n={len(ys)})")
        for r in jruns:
            if r.group == grp:
                ax.annotate(r.dir.split("-run-")[-1], (x, getattr(r, metric) * deg_scale),
                            textcoords="offset points", xytext=(6, -3),
                            fontsize=7, color=c)
    theta = cal[theta_key]
    ax.axhline(theta, color="k", ls="--", lw=1, label=f"{theta_key}={theta:.4f}")
    ax.set_xticks([0, 1]); ax.set_xticklabels(["norm", "hi"])
    ax.set_ylabel(ylabel)
    sep = cal.get(f"separated_{channel}")
    title = (f"{joint} {channel}: "
             f"{'separated' if sep else 'OVERLAP/fallback' if sep is False else 'no hi'}")
    ax.set_title(title, fontsize=10)
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=7)


def _plots(joints_cal: dict, runs, out_dir: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        print(f"[calib] plots skipped ({e})")
        return
    for joint, cal in joints_cal.items():
        if "error" in cal:
            continue
        jruns = [r for r in runs if r.joint == joint]
        fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
        _plot_channel(axes[0], jruns, joint, cal, "hf", "hf_rms_max",
                      "theta_hf", "eff diff-RMS (Nm/tick)", deg_scale=1.0)
        _plot_channel(axes[1], jruns, joint, cal, "vel", "vel_err_max",
                      "theta_vel_deg_s", "vel-error RMS (deg/s)", deg_scale=DEG)
        _plot_channel(axes[2], jruns, joint, cal, "acc", "resp_acc_window_rms_max",
                      "theta_acc_deg_s2", "|resp_acc| window-RMS (deg/s2)", deg_scale=DEG)
        fig.tight_layout()
        fig.savefig(out_dir / f"watchdog-calib-{joint}.png", dpi=110)
        plt.close(fig)
    print(f"[calib] wrote {len(joints_cal)} PNGs -> {out_dir}")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="B1 watchdog 阈值标定 (SOP-11 §15.1.7)")
    ap.add_argument("--runs", nargs="+", required=True,
                    help="B1 run 目录（可 glob，shell 展开后多个实参）")
    ap.add_argument("--output", required=True, help="watchdog_thresholds.json 输出路径")
    ap.add_argument("--window", type=int, default=50,
                    help="HF 滑动窗拍数（默认 50 = 0.5 s @100 Hz）")
    ap.add_argument("--kin-window", type=int, default=5,
                    help="运动学通道滑动窗拍数（默认 5，与 TickWatchdog 一致）")
    ap.add_argument("--pos-mult", type=float, default=2.0,
                    help="theta_pos = norm 组 |ref-resp| 最大值 × 此系数（SOP 默认 2）")
    args = ap.parse_args()

    runs = load_runs(args.runs, args.window, args.kin_window)
    if not runs:
        sys.exit("[calib] 没有可用 run")
    joints = sorted({r.joint for r in runs})
    print(f"[calib] loaded {len(runs)} runs, joints={joints}, "
          f"norm={sum(r.group == 'norm' for r in runs)}, hi={sum(r.group == 'hi' for r in runs)}")

    joints_cal = {}
    for joint in joints:
        cal = calibrate_joint(joint, [r for r in runs if r.joint == joint],
                              args.window, args.kin_window, args.pos_mult)
        joints_cal[joint] = cal
        if "error" in cal:
            print(f"[{joint}] ERROR: {cal['error']}")
            continue
        print(f"[{joint}] "
              f"theta_hf={cal['theta_hf']:.4f} Nm/tick (sep={cal['separated_hf']})  "
              f"theta_vel={cal['theta_vel_deg_s']:.2f} deg/s (sep={cal['separated_vel']})  "
              f"theta_acc={cal['theta_acc_deg_s2']:.1f} deg/s2 (sep={cal['separated_acc']})  "
              f"theta_pos={cal['theta_pos_deg']:.2f} deg")
        if "warning" in cal:
            print(f"[{joint}] WARNING: {cal['warning']}")

    doc = {
        "version": "b1-watchdog-calib-v4",
        "window_ticks": args.window,
        "kin_window": args.kin_window,
        "pos_mult": args.pos_mult,
        "runs": [r.dir for r in runs],
        "joints": joints_cal,
    }
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(doc, ensure_ascii=False, indent=2))
    print("[calib] wrote", out_path)

    _plots(joints_cal, runs, out_path.parent)


if __name__ == "__main__":
    main()
