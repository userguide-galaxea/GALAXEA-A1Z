"""Offline end-effector pipeline: build reference, batch IK, gate, FK.

Pure kinematics — imports :class:`a1z.robots.kinematics.Kinematics` (Pinocchio)
but NOT the robot / CAN stack, so ``run_test.py --dry-run`` can validate a
trajectory without touching hardware (SOP-03 §4.4).
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from a1z.robots.kinematics import Kinematics
from a1z.analysis import safety
from a1z.analysis.signals import make_ee_traj

DEG = 180.0 / math.pi

# Bundled URDF (mirrors get_robot._DEFAULT_URDF_PATH without importing it).
DEFAULT_URDF_PATH = str(
    Path(__file__).parent.parent / "robot_models" / "a1z" / "A1Z_G1Z.urdf")
DEFAULT_EE_FRAME = "arm_link6"


def make_kinematics(urdf_path: Optional[str] = None,
                    ee_frame: Optional[str] = None) -> Kinematics:
    return Kinematics(urdf_path or DEFAULT_URDF_PATH,
                      end_effector_frame=ee_frame or DEFAULT_EE_FRAME)


def ee_frame_name(kin: Kinematics) -> str:
    """Resolve the actual EE frame name (for meta.json)."""
    try:
        return kin._model.frames[kin._frame_id].name
    except Exception:
        return DEFAULT_EE_FRAME


def urdf_limits(kin: Kinematics) -> np.ndarray:
    """(6,2) [lo,hi] rad from the loaded model."""
    return np.stack([kin._q_lower[:6], kin._q_upper[:6]], axis=1)


class GateResult:
    """Outcome of the dry-run gate (SOP-03 §4.4)."""

    def __init__(self):
        self.ik_convergence_rate = 1.0
        self.limit_ok = True
        self.speed_ok = True
        self.max_tick_speed_deg = 0.0
        self.n_unconverged = 0
        self.n_limit_viol = 0
        self.notes: list[str] = []

    @property
    def passed(self) -> bool:
        return (self.ik_convergence_rate >= 1.0 and self.limit_ok and self.speed_ok)

    def summary(self) -> str:
        return (f"IK conv={self.ik_convergence_rate * 100:.1f}%  "
                f"limit_ok={self.limit_ok}  speed_ok={self.speed_ok}  "
                f"max Δq/tick={self.max_tick_speed_deg:.2f}°"
                + ("" if not self.notes else "\n  " + "\n  ".join(self.notes)))


def build_reference(kin: Kinematics, q_nom: np.ndarray, *, kind: str, plane: str,
                    radius: float, period: float, cycles: int,
                    sample_hz: int) -> Tuple[np.ndarray, np.ndarray]:
    """Return (t (N,), T_ref (N,4,4)) sampled at ``sample_hz``, anchored at FK(q_nom)."""
    T_anchor = kin.fk(np.asarray(q_nom, dtype=float))
    traj = make_ee_traj(kind, T_anchor, plane=plane, radius=radius,
                        period=period, cycles=cycles)
    n = int(round(traj.duration * sample_hz)) + 1
    t = np.arange(n) / sample_hz
    T_ref = np.array([traj.sample(float(tt)) for tt in t])
    return t, T_ref


def solve_ik_path(kin: Kinematics, T_ref: np.ndarray, q_nom: np.ndarray,
                  *, pos_threshold: float, ori_threshold: float,
                  max_iters: int) -> Tuple[np.ndarray, np.ndarray]:
    """Warm-started IK for every pose. Returns (q_ref (N,6), converged (N,) bool)."""
    n = len(T_ref)
    q_ref = np.zeros((n, 6))
    converged = np.zeros(n, dtype=bool)
    q_prev = np.asarray(q_nom, dtype=float).copy()
    for k in range(n):
        ok, q = kin.ik(T_ref[k], init_q=q_prev, pos_threshold=pos_threshold,
                       ori_threshold=ori_threshold, max_iters=max_iters)
        q_ref[k] = q[:6]
        converged[k] = ok
        if ok:
            q_prev = q[:6].copy()
    return q_ref, converged


def gate(q_ref: np.ndarray, converged: np.ndarray, *, sample_hz: int,
         buffer_rad: float, urdf_lim: np.ndarray,
         max_tick_deg: float = 2.0) -> GateResult:
    """Run the three dry-run checks (convergence / limits / per-tick speed).

    ``max_tick_deg`` gates against IK branch jumps (elbow flips), which are tens
    of degrees per tick; a smooth 3 cm/s Cartesian trajectory produces <1°/tick.
    2.0°/tick (200°/s @100 Hz) sits well below every joint's hardware velocity
    cap yet an order of magnitude under a real branch discontinuity.
    """
    g = GateResult()
    n = len(q_ref)
    g.n_unconverged = int(np.sum(~converged))
    g.ik_convergence_rate = float(np.sum(converged)) / n if n else 0.0
    if g.n_unconverged:
        g.notes.append(f"{g.n_unconverged}/{n} IK points did not converge")

    lim = safety.effective_limits(buffer_rad, urdf_lim)
    inside = np.array([safety.within_limits(q_ref[k], lim).all() for k in range(n)])
    g.n_limit_viol = int(np.sum(~inside))
    g.limit_ok = g.n_limit_viol == 0
    if not g.limit_ok:
        g.notes.append(f"{g.n_limit_viol}/{n} joint refs exceed effective limits")

    if n >= 2:
        dq = np.abs(np.diff(q_ref, axis=0)) * DEG
        g.max_tick_speed_deg = float(np.max(dq))
        g.speed_ok = g.max_tick_speed_deg <= max_tick_deg
        if not g.speed_ok:
            g.notes.append(
                f"max Δq/tick {g.max_tick_speed_deg:.2f}° > {max_tick_deg}° "
                f"(IK branch jump?)")
    return g


def fk_path(kin: Kinematics, q: np.ndarray) -> np.ndarray:
    """FK for every joint config. Returns (N,4,4)."""
    return np.array([kin.fk(q[k]) for k in range(len(q))])
