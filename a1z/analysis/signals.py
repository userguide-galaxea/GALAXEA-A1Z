"""Reference/excitation trajectory generators (pure numpy, no hardware).

Three families, all offline-unit-testable:

* :class:`WaveTrajectory` — per-joint square/triangle excitation confined to a
  safe absolute-angle window. Ported verbatim (semantics) from the DK1 leader
  ``leader_ref.py`` so joint dynamic-response unit tests share one waveform
  definition with the historical PD-excitation experiments (SOP-02).
* :class:`EECircleTrajectory` / :class:`EELineTrajectory` — Cartesian
  end-effector reference poses ``T_ref(t)`` (4x4 homogeneous), orientation held
  fixed at the anchor pose. Fed to IK offline (dry-run gate, SOP-03 §4.4) then
  streamed to the arm.

Run ``python -m a1z.analysis.signals`` for a self-check that prints each
waveform's effective window and each EE trajectory's extent.
"""
from __future__ import annotations

import math

import numpy as np

from a1z.analysis.safety import (
    PRECONDITIONS_DEG,
    SAFE_RANGES_DEG,
    clamp_wave_window,
)

DEG = 180.0 / math.pi


# ---------------------------------------------------------------------------
# Joint excitation: square / triangle in a safe absolute window
# ---------------------------------------------------------------------------
class WaveTrajectory:
    """Periodic square/triangle excitation for ONE joint, absolute-window safe.

    Targets are ABSOLUTE canonical joint angles confined to
    ``SAFE_RANGES_DEG[joint]`` inset by ``margin_deg``. The waveform swings
    +/-``amp_deg`` around the inset-window centre, auto-capped to the inset
    window, so narrow windows shrink the amplitude instead of violating range.

    ``q_start`` is the full 6-dof pose to slow-move to before the waveform
    begins: preconditioned joints at ``PRECONDITIONS_DEG``, the excited joint at
    the waveform low point, all others held at ``q0``. Only the excitation
    segment (``hold_pre .. hold_pre + cycles*period``) is meant to be recorded;
    the leading/trailing holds provide steady-state and noise-floor windows.
    """

    def __init__(
        self,
        q0: np.ndarray,
        joint: int,                 # 0-based
        kind: str,                  # "square" | "triangle"
        *,
        amp_deg: float = 15.0,
        margin_deg: float = 5.0,
        period: float = 4.0,
        cycles: int = 3,
        hold_pre: float = 1.0,
        hold_post: float = 1.5,
        edge_rate_deg_s: float = 240.0,
    ):
        if kind not in ("square", "triangle"):
            raise ValueError(f"unknown wave kind: {kind}")
        j1 = joint + 1
        if j1 not in SAFE_RANGES_DEG:
            raise ValueError(f"no safe range for J{j1}")

        self.lo, self.hi = clamp_wave_window(j1, amp_deg=amp_deg, margin_deg=margin_deg)
        self.j = int(joint)
        self.kind = kind
        self.period = float(period)
        self.cycles = int(cycles)
        self.hold_pre = float(hold_pre)
        self.hold_post = float(hold_post)
        self.edge_rate = math.radians(edge_rate_deg_s)   # square slew cap (rad/s)

        q = np.asarray(q0, dtype=float).copy()[:6]
        for pj1, deg in PRECONDITIONS_DEG.get(j1, {}).items():
            q[pj1 - 1] = math.radians(deg)
        q[joint] = self.lo                        # waveform starts at low point
        self.q_start = q

    @property
    def duration(self) -> float:
        return self.hold_pre + self.cycles * self.period + self.hold_post

    @property
    def excite_window(self) -> tuple[float, float]:
        """(t_start, t_end) of the excitation segment (excludes holds)."""
        return self.hold_pre, self.hold_pre + self.cycles * self.period

    def _val(self, t: float) -> float:
        tt = t - self.hold_pre
        if tt < 0.0 or tt >= self.cycles * self.period:
            return self.lo
        if self.kind == "square":
            # Slew-limited square: each half-period the target flips lo<->hi, and
            # the value ramps toward it at edge_rate instead of jumping. A hard
            # ~30 deg step x kp would be a torque slam (SOP-03 §3.2); the ramp
            # (240 deg/s over 30 deg = 125 ms) stays a near-step for t_s while
            # bounding the command derivative. Half-period (>=1 s here) always
            # long enough to reach and hold the plateau.
            half = self.period / 2.0
            k = int(tt // half)
            going_up = (k % 2 == 0)
            start_val = self.lo if going_up else self.hi
            target = self.hi if going_up else self.lo
            elapsed = tt - k * half
            span = self.edge_rate * elapsed
            if going_up:
                return min(start_val + span, target)
            return max(start_val - span, target)
        # triangle: lo -> hi -> lo, linear
        ph = (tt % self.period) / self.period
        frac = 2.0 * ph if ph < 0.5 else 2.0 * (1.0 - ph)
        return self.lo + (self.hi - self.lo) * frac

    def sample(self, t: float) -> np.ndarray:
        """Full (6,) canonical target at time t (rad)."""
        q = self.q_start.copy()
        q[self.j] = self._val(t)
        return q


def make_wave(kind: str, q0: np.ndarray, joint: int, **kw) -> WaveTrajectory:
    return WaveTrajectory(q0, joint, kind, **kw)


# ---------------------------------------------------------------------------
# End-effector Cartesian reference trajectories (orientation held fixed)
# ---------------------------------------------------------------------------
def _ramp(t: float, dur: float) -> float:
    """Cosine ease-in/out phase weight in [0,1] over a ramp of length ``dur``.

    Used to fade the trajectory in from rest and out to rest so neither the
    Cartesian velocity nor (after IK) the joint velocity steps at the ends.
    """
    if dur <= 0.0:
        return 1.0
    x = min(max(t / dur, 0.0), 1.0)
    return 0.5 * (1.0 - math.cos(math.pi * x))


_PLANE_AXES = {
    "xy": (0, 1),
    "xz": (0, 2),
    "yz": (1, 2),
}


class EECircleTrajectory:
    """Circle in a base-frame plane, centred so ``t=0`` sits on the anchor pose.

    The circle lies in ``plane`` (two of x/y/z), radius ``radius`` (m), one lap
    per ``period`` s, ``cycles`` laps. Orientation is frozen at ``T_anchor``'s
    rotation for the whole trajectory. A half-period cosine ramp fades the
    angular sweep in at the start and out at the end.
    """

    def __init__(
        self,
        T_anchor: np.ndarray,
        *,
        plane: str = "xz",
        radius: float = 0.04,
        period: float = 8.0,
        cycles: int = 2,
    ):
        if plane not in _PLANE_AXES:
            raise ValueError(f"unknown plane: {plane} (want one of {list(_PLANE_AXES)})")
        self.T0 = np.asarray(T_anchor, dtype=float).copy()
        self.R = self.T0[:3, :3].copy()
        self.p0 = self.T0[:3, 3].copy()
        self.plane = plane
        self.a1, self.a2 = _PLANE_AXES[plane]
        self.radius = float(radius)
        self.period = float(period)
        self.cycles = int(cycles)
        # Circle centre is offset from the anchor along +a1 so that the point at
        # angle 0 coincides with the anchor position (start on the arm's pose).
        self._centre = self.p0.copy()
        self._centre[self.a1] -= self.radius
        self._ramp = 0.5 * self.period

    @property
    def duration(self) -> float:
        return self.cycles * self.period

    def _angle(self, t: float) -> float:
        # Ramp the *phase rate* in/out so velocity is zero at both ends.
        w_in = _ramp(t, self._ramp)
        w_out = _ramp(self.duration - t, self._ramp)
        base = 2.0 * math.pi * t / self.period
        return base * w_in * w_out

    def sample(self, t: float) -> np.ndarray:
        theta = self._angle(t)
        p = self._centre.copy()
        p[self.a1] += self.radius * math.cos(theta)
        p[self.a2] += self.radius * math.sin(theta)
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = p
        return T


class EELineTrajectory:
    """Straight reciprocating line +/-``radius`` along one plane axis from anchor."""

    def __init__(
        self,
        T_anchor: np.ndarray,
        *,
        plane: str = "xz",
        radius: float = 0.04,
        period: float = 8.0,
        cycles: int = 2,
    ):
        if plane not in _PLANE_AXES:
            raise ValueError(f"unknown plane: {plane}")
        self.T0 = np.asarray(T_anchor, dtype=float).copy()
        self.R = self.T0[:3, :3].copy()
        self.p0 = self.T0[:3, 3].copy()
        self.axis = _PLANE_AXES[plane][0]
        self.radius = float(radius)
        self.period = float(period)
        self.cycles = int(cycles)
        self._ramp = 0.5 * self.period

    @property
    def duration(self) -> float:
        return self.cycles * self.period

    def sample(self, t: float) -> np.ndarray:
        w_in = _ramp(t, self._ramp)
        w_out = _ramp(self.duration - t, self._ramp)
        # One full back-and-forth per period; sin so it starts and ends at anchor.
        offset = self.radius * math.sin(2.0 * math.pi * t / self.period) * w_in * w_out
        p = self.p0.copy()
        p[self.axis] += offset
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = p
        return T


def make_ee_traj(kind: str, T_anchor: np.ndarray, **kw):
    if kind == "circle":
        return EECircleTrajectory(T_anchor, **kw)
    if kind == "line":
        return EELineTrajectory(T_anchor, **kw)
    raise ValueError(f"unknown ee trajectory kind: {kind}")


if __name__ == "__main__":  # tiny smoke print
    q0 = np.zeros(6)
    print("=== joint waves (default amp=15, margin=5) ===")
    for kind in ("square", "triangle"):
        for j in range(6):
            w = make_wave(kind, q0, joint=j)
            print(f"{kind:8s} J{j + 1} dur={w.duration:5.1f}s  "
                  f"wave=[{w.lo * DEG:+6.1f},{w.hi * DEG:+6.1f}]deg  "
                  f"start={np.round(w.q_start * DEG, 1)}")
    print("=== ee trajectories (anchor = identity @ [0.4,0,0.3]) ===")
    T = np.eye(4); T[:3, 3] = [0.4, 0.0, 0.3]
    for kind in ("circle", "line"):
        tr = make_ee_traj(kind, T, plane="xz", radius=0.04, period=8.0, cycles=2)
        pts = np.array([tr.sample(t)[:3, 3] for t in np.linspace(0, tr.duration, 200)])
        span = (pts.max(axis=0) - pts.min(axis=0)) * 1000
        print(f"{kind:8s} dur={tr.duration:5.1f}s  bbox(mm)=[{span[0]:.1f},"
              f"{span[1]:.1f},{span[2]:.1f}]  start={np.round(pts[0] * 1000, 1)}mm")
