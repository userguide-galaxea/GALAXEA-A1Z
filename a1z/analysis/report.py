"""Output writers: run directory, JSON, CSV, PNG (matplotlib Agg).

All plotting uses the non-interactive Agg backend so the module runs headless
(no DISPLAY on the robot host). Nothing here touches hardware; the runner hands
it plain numpy arrays and dicts. Layout follows SOP-03 §2.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DEG = 180.0 / math.pi


class RunDir:
    """Resolves and creates ``output/<date>-run-<id>/``, refusing to overwrite."""

    def __init__(self, output_root: Path, date: str, run_id: str):
        self.root = Path(output_root) / f"{date}-run-{run_id}"
        if self.root.exists():
            raise FileExistsError(
                f"run dir already exists: {self.root} (pick a fresh --run id)")
        self.root.mkdir(parents=True)

    def path(self, name: str) -> Path:
        return self.root / name


def _default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    raise TypeError(f"not JSON-serializable: {type(o)}")


def write_json(path: Path, obj: dict) -> None:
    with open(path, "w") as fp:
        json.dump(obj, fp, indent=2, ensure_ascii=False, default=_default)


# ---------------------------------------------------------------------------
# CSV writers
# ---------------------------------------------------------------------------
def write_joint_csv(path: Path, t: np.ndarray, q: np.ndarray) -> None:
    """(t, q1..q6) rows. q is (N,6)."""
    with open(path, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["t"] + [f"q{i}" for i in range(1, 7)])
        for k in range(len(t)):
            w.writerow([f"{t[k]:.6f}"] + [f"{v:.6f}" for v in q[k]])


def write_unit_csv(path: Path, t: np.ndarray, ref: np.ndarray, resp: np.ndarray,
                   eff: Optional[np.ndarray] = None) -> None:
    """(t, ref, resp, err[, eff]) — angles rad, effort Nm, single joint."""
    with open(path, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["t", "ref", "resp", "err"] + (["eff"] if eff is not None else []))
        for k in range(len(t)):
            row = [f"{t[k]:.6f}", f"{ref[k]:.6f}", f"{resp[k]:.6f}",
                   f"{resp[k] - ref[k]:.6f}"]
            if eff is not None:
                row.append(f"{eff[k]:.6f}")
            w.writerow(row)


def _pose_row(T: np.ndarray):
    """(px,py,pz,qw,qx,qy,qz) from a 4x4 homogeneous transform."""
    p = T[:3, 3]
    R = T[:3, :3]
    # rotation matrix -> quaternion (w,x,y,z), numerically stable branch
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    return p[0], p[1], p[2], qw, qx, qy, qz


def write_ee_pose_csv(path: Path, t: np.ndarray, T: np.ndarray) -> None:
    with open(path, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["t", "px", "py", "pz", "qw", "qx", "qy", "qz"])
        for k in range(len(t)):
            row = _pose_row(T[k])
            w.writerow([f"{t[k]:.6f}"] + [f"{v:.6f}" for v in row])


def write_ee_error_csv(path: Path, t: np.ndarray, dp: np.ndarray,
                       e_pos: np.ndarray, e_ang: np.ndarray) -> None:
    with open(path, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["t", "ex", "ey", "ez", "e_pos", "e_ang"])
        for k in range(len(t)):
            w.writerow([f"{t[k]:.6f}", f"{dp[k, 0]:.6f}", f"{dp[k, 1]:.6f}",
                        f"{dp[k, 2]:.6f}", f"{e_pos[k]:.6f}", f"{e_ang[k]:.6f}"])


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def plot_unit(path: Path, joint1: int,
              sq: Optional[dict], tri: Optional[dict]) -> None:
    """Two stacked panels (square / triangle): ref+resp+err, with metric text.

    ``sq`` / ``tri`` are dicts {t, ref, resp, metrics} or None if that waveform
    wasn't run. If the dict carries ``eff`` (Nm), it is drawn on a right-hand
    axis — during a stall, eff ramping = fighting friction/contact, eff≈0 =
    command path (SOP-03 §5.4).
    """
    panels = [(("square", sq)), (("triangle", tri))]
    panels = [(name, d) for name, d in panels if d is not None]
    n = len(panels) or 1
    fig, axes = plt.subplots(n, 1, figsize=(11, 3.4 * n), squeeze=False)
    for ax, (name, d) in zip(axes[:, 0], panels):
        t = d["t"]
        ax.plot(t, d["ref"] * DEG, label="ref", lw=1.4)
        ax.plot(t, d["resp"] * DEG, label="resp", lw=1.0)
        ax.plot(t, (d["resp"] - d["ref"]) * DEG, label="err", lw=0.9, alpha=0.7)
        handles, labels = ax.get_legend_handles_labels()
        if d.get("eff") is not None:
            ax_eff = ax.twinx()
            ax_eff.plot(t, d["eff"], label="eff", lw=0.8, color="tab:purple",
                        alpha=0.55)
            ax_eff.set_ylabel("eff (Nm)", color="tab:purple", fontsize=8)
            ax_eff.tick_params(axis="y", labelcolor="tab:purple", labelsize=8)
            h2, l2 = ax_eff.get_legend_handles_labels()
            handles += h2
            labels += l2
        ax.set_title(f"J{joint1} {name}")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("deg")
        ax.grid(alpha=0.3)
        ax.legend(handles, labels, loc="upper right", fontsize=8)
        m = d.get("metrics", {})
        ax.text(0.01, 0.98, _fmt_metrics(name, m), transform=ax.transAxes,
                va="top", ha="left", fontsize=8, family="monospace",
                bbox=dict(boxstyle="round", fc="white", alpha=0.7))
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _fmt_metrics(name: str, m: dict) -> str:
    v2 = m.get("v2") or {}
    if name == "square":
        base = (f"n={m.get('n_steps', 0)}  "
                f"σmax={_f(m.get('overshoot_max_pct'))}%  "
                f"t_s max={_f(m.get('ts_max_ms'))}ms  "
                f"e_ss max={_f(m.get('ess_max_deg'))}°")
        if v2:
            base += (f"\n[v2] t_s(from ref)={_f(v2.get('ts_v2_max_ms'))}ms  "
                     f"ess/floor={_f(v2.get('ess_ratio'))}")
        return base
    base = (f"range={_f(m.get('err_range_deg'))}°  "
            f"std={_f(m.get('err_std_deg'))}°  "
            f"jumps={m.get('jump_count', 0)} ({_f(m.get('jump_rate_hz'))}Hz)  "
            f"mean={_f(m.get('jump_mean_deg'))}° max={_f(m.get('jump_max_deg'))}°")
    if v2:
        base += (f"\n[v2] lag={_f(v2.get('lag_at_rate_deg'))}°  "
                 f"resid_std={_f(v2.get('resid_std_deg'))}°  "
                 f"jumpP95={_f(v2.get('jump_p95_deg'))}° (n{v2.get('jump_count_v2', 0)}, "
                 f"ε{_f(v2.get('eps_adapt_deg'))}°)")
    return base


def _f(v) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "–"
    return f"{v:.2f}"


def plot_joints_traj(path: Path, t: np.ndarray, ref: np.ndarray, resp: np.ndarray) -> None:
    """6 panels: J_i reference vs response."""
    fig, axes = plt.subplots(6, 1, figsize=(11, 12), sharex=True)
    for i in range(6):
        ax = axes[i]
        ax.plot(t, ref[:, i] * DEG, label="ref", lw=1.3)
        ax.plot(t, resp[:, i] * DEG, label="resp", lw=1.0)
        ax.set_ylabel(f"J{i + 1}")
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("joint trajectories: reference vs response")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_joints_error(path: Path, t: np.ndarray, ref: np.ndarray, resp: np.ndarray,
                      per_joint_notes: List[str]) -> None:
    """6 panels: J_i error, each annotated with RMSE (+ step metrics if any)."""
    fig, axes = plt.subplots(6, 1, figsize=(11, 12), sharex=True)
    for i in range(6):
        ax = axes[i]
        err = (resp[:, i] - ref[:, i]) * DEG
        ax.plot(t, err, lw=0.9, color="tab:red")
        ax.axhline(0, color="k", lw=0.5, alpha=0.4)
        ax.set_ylabel(f"J{i + 1}")
        ax.grid(alpha=0.3)
        ax.text(0.01, 0.95, per_joint_notes[i], transform=ax.transAxes,
                va="top", ha="left", fontsize=8, family="monospace",
                bbox=dict(boxstyle="round", fc="white", alpha=0.7))
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("joint error trajectories")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_ee_traj(path: Path, plane: str, T_ref: np.ndarray, T_resp: np.ndarray,
                 t: np.ndarray, e_pos: np.ndarray, e_ang: np.ndarray,
                 rmse_txt: str) -> None:
    """Panel 1: ref vs response in the trajectory plane; panel 2: error curves."""
    axes_idx = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}[plane]
    labels = ["x", "y", "z"]
    a, b = axes_idx
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))
    pr = T_ref[:, :3, 3] * 1000
    ps = T_resp[:, :3, 3] * 1000
    ax1.plot(pr[:, a], pr[:, b], label="ref", lw=1.6)
    ax1.plot(ps[:, a], ps[:, b], label="resp", lw=1.0)
    ax1.set_xlabel(f"{labels[a]} (mm)")
    ax1.set_ylabel(f"{labels[b]} (mm)")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9)
    ax1.set_title(f"EE trajectory ({plane} plane)")
    ax2.plot(t, e_pos * 1000, label="|pos err| (mm)", color="tab:red")
    ax2.plot(t, e_ang * DEG, label="ang err (deg)", color="tab:blue", alpha=0.8)
    ax2.set_xlabel("t (s)")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9)
    ax2.set_title("EE tracking error")
    ax2.text(0.01, 0.98, rmse_txt, transform=ax2.transAxes, va="top", ha="left",
             fontsize=9, family="monospace",
             bbox=dict(boxstyle="round", fc="white", alpha=0.75))
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_ik_preview(path: Path, t: np.ndarray, q_ref: np.ndarray,
                    limits: np.ndarray) -> None:
    """Dry-run: joint reference from IK with limit bands (SOP-03 §4.4)."""
    fig, axes = plt.subplots(6, 1, figsize=(11, 12), sharex=True)
    for i in range(6):
        ax = axes[i]
        ax.plot(t, q_ref[:, i] * DEG, lw=1.2)
        ax.axhline(limits[i, 0] * DEG, color="r", ls="--", lw=0.7, alpha=0.6)
        ax.axhline(limits[i, 1] * DEG, color="r", ls="--", lw=0.7, alpha=0.6)
        ax.set_ylabel(f"J{i + 1}")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("t (s)")
    fig.suptitle("IK dry-run: joint reference vs effective limits")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
