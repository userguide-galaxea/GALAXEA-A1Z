"""Hardware execution layer — the ONLY module that opens the CAN bus.

Two runners share a common 100 Hz stream-and-sample loop, RAM buffering, and a
slow-move transit discipline (SOP-03 §3.3 / §4.5): enter slow, excite/track,
exit slow, always return to zero even on Ctrl+C or error (finally-guarded).

* :class:`JointUnitTestRunner` — per-joint square+triangle excitation.
* :class:`EETrackingRunner` — Cartesian trajectory tracking via IK/FK.

Transit uses the SDK's own :meth:`ArmRobot.move_joints` (min-jerk, limit- and
estop-checked, ``max_jump_rad`` guarded); excitation uses
:meth:`ArmRobot.command_joint_state` streaming at 100 Hz.
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from a1z.robots.get_robot import get_a1z_robot, _JOINT_LIMITS
from a1z.analysis import ee_pipeline as ee
from a1z.analysis import safety
from a1z.analysis.signals import WaveTrajectory

DEG = 180.0 / math.pi
_ZERO6 = np.zeros(6)


def _rad_speed(deg_per_s: float) -> float:
    return math.radians(deg_per_s)


class _Base:
    """Shared hardware lifecycle: build robot, transit, stream, safe shutdown."""

    def __init__(self, can_channel: str, *, sample_hz: int = 100,
                 transit_speed_deg_s: float = 15.0,
                 kp: Optional[np.ndarray] = None, kd: Optional[np.ndarray] = None):
        # Cross-check the mirrored SDK limits against the live constant so a
        # future SDK edit can't silently desync the safety gate.
        safety.assert_matches_sdk_limits(_JOINT_LIMITS)
        self.can_channel = can_channel
        self.sample_hz = sample_hz
        self.dt = 1.0 / sample_hz
        self.transit_speed = _rad_speed(transit_speed_deg_s)
        self.kp = kp
        self.kd = kd
        # Robot/CAN bus are created lazily in start() so offline paths
        # (dry-run gate, IK solve) never open the bus.
        self.robot = None
        self._started = False

    # --- lifecycle ---
    def start(self) -> None:
        # Position-hold + PD (NOT zero-gravity): we want the arm to track.
        self.robot = get_a1z_robot(can_channel=self.can_channel,
                                   zero_gravity_mode=False, with_gripper=False)
        self.robot.start()
        self._started = True

    def shutdown(self) -> None:
        if self._started:
            try:
                self.robot.stop()
            finally:
                try:
                    self.robot._bus.shutdown()
                except Exception:
                    pass
        self._started = False

    def effective_gains(self) -> Tuple[np.ndarray, np.ndarray]:
        info = self.robot.get_robot_info()
        kp = info["default_kp"].copy()
        kd = info["default_kd"].copy()
        if self.kp is not None:
            kp = self.kp.copy()
        if self.kd is not None:
            kd = self.kd.copy()
        return kp, kd

    # --- transit (slow) ---
    def transit_to(self, q_target: np.ndarray, *, max_jump_rad: float = 3.5) -> None:
        """Slow min-jerk move to a full 6-dof pose at the transit speed.

        Always uses DEFAULT gains (test gains apply only to excitation). Raises
        on limit / far-jump violation so callers abort into safe shutdown.
        """
        self.robot.move_joints(np.asarray(q_target, dtype=float)[:6],
                               speed=self.transit_speed, max_jump_rad=max_jump_rad)
        # Verify arrival (SOP-01 §4.1 fail-safe: > 3° off → abort).
        time.sleep(0.05)
        meas = self.robot.get_joint_pos()[:6]
        off = np.abs(meas - np.asarray(q_target, dtype=float)[:6]) * DEG
        if np.any(off > 3.0):
            bad = "; ".join(f"J{i+1}={off[i]:.1f}°" for i in np.flatnonzero(off > 3.0))
            raise RuntimeError(f"transit arrival check failed (>3°): {bad}")

    def return_to_zero(self) -> None:
        """Slow move back to the zero configuration (best-effort, no raise)."""
        if not self._started or self.robot.is_estopped:
            return
        try:
            self.robot.move_joints(_ZERO6.copy(), speed=self.transit_speed,
                                   max_jump_rad=6.5)
        except Exception as e:  # noqa: BLE001
            print(f"[runner] return_to_zero skipped: {e}")

    # --- streaming excitation/tracking (records only this segment) ---
    def stream(self, sample_fn, duration: float, kp: np.ndarray, kd: np.ndarray
               ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Stream ``sample_fn(t)->(6,)`` at sample_hz for ``duration`` seconds.

        Returns (t (M,), ref (M,6), resp (M,6)). RAM-buffered; no disk IO in loop.
        Aborts (returns partial) if estop latches mid-stream.
        """
        t_buf: List[float] = []
        ref_buf: List[np.ndarray] = []
        resp_buf: List[np.ndarray] = []
        js: Dict[str, np.ndarray] = {"vel": _ZERO6.copy(), "kp": kp, "kd": kd}
        t0 = time.monotonic()
        while True:
            tick = time.monotonic()
            t = tick - t0
            if t > duration:
                break
            if self.robot.is_estopped:
                print("[runner] stream aborted: estop latched")
                break
            q_ref = np.asarray(sample_fn(t), dtype=float)[:6]
            js["pos"] = q_ref
            self.robot.command_joint_state(js)
            q_resp = self.robot.get_joint_pos()[:6]
            t_buf.append(t)
            ref_buf.append(q_ref)
            resp_buf.append(q_resp.copy())
            time.sleep(max(0.0, self.dt - (time.monotonic() - tick)))
        return (np.array(t_buf), np.array(ref_buf), np.array(resp_buf))


class JointUnitTestRunner(_Base):
    """Per-joint square + triangle excitation with full transit discipline."""

    def __init__(self, can_channel: str, *, amp_deg: float = 15.0,
                 margin_deg: float = 5.0, period: float = 4.0, cycles: int = 3,
                 hold_pre: float = 1.0, hold_post: float = 1.5,
                 edge_rate_deg_s: float = 240.0, waves=("square", "triangle"),
                 **base_kw):
        super().__init__(can_channel, **base_kw)
        self.amp_deg = amp_deg
        self.margin_deg = margin_deg
        self.period = period
        self.cycles = cycles
        self.hold_pre = hold_pre
        self.hold_post = hold_post
        self.edge_rate_deg_s = edge_rate_deg_s
        self.waves = tuple(waves)

    def run_joint(self, joint1: int) -> Dict[str, dict]:
        """Excite joint ``joint1`` (1-based). Returns {wave: {t, ref, resp}}.

        Full timeline (§3.3): preconditions → waveform start → [square, triangle]
        → retract joint → retract preconditions → zero. finally-guarded.
        """
        j = joint1 - 1
        kp, kd = self.effective_gains()
        # Build both wave trajectories up front (shared q_start / preconditions).
        trajs = {name: WaveTrajectory(_ZERO6, j, name, amp_deg=self.amp_deg,
                                      margin_deg=self.margin_deg, period=self.period,
                                      cycles=self.cycles, hold_pre=self.hold_pre,
                                      hold_post=self.hold_post,
                                      edge_rate_deg_s=self.edge_rate_deg_s)
                 for name in self.waves}
        q_start = trajs[self.waves[0]].q_start
        pre = safety.PRECONDITIONS_DEG.get(joint1, {})

        out: Dict[str, dict] = {}
        try:
            # Stage A: preconditioned joints to posture (excited joint stays home).
            if pre:
                q_pre = _ZERO6.copy()
                for pj1, deg in pre.items():
                    q_pre[pj1 - 1] = math.radians(deg)
                print(f"[J{joint1}] Stage A: preconditions {pre}")
                self.transit_to(q_pre)
            # Stage B: excited joint to waveform low point.
            print(f"[J{joint1}] Stage B: to waveform start "
                  f"{np.round(q_start * DEG, 1)}")
            self.transit_to(q_start)
            # Excitation segment(s).
            for name in self.waves:
                tr = trajs[name]
                print(f"[J{joint1}] excite {name} ({tr.duration:.1f}s, gains "
                      f"kp[{j}]={kp[j]:.1f} kd[{j}]={kd[j]:.2f})")
                t, ref, resp = self.stream(tr.sample, tr.duration, kp, kd)
                out[name] = dict(t=t, ref=ref, resp=resp, traj=tr)
                # brief settle between waves at low point
                if len(self.waves) > 1:
                    self.transit_to(q_start)
        finally:
            # Reverse retract: excited joint home → drop preconditions → zero.
            print(f"[J{joint1}] retract + return to zero")
            self.return_to_zero()
        return out


class EETrackingRunner(_Base):
    """Cartesian trajectory tracking: transit to q_nom, stream q_ref, FK responses."""

    def __init__(self, can_channel: str, *, q_nom_deg=(-20, 35, -25, -25, 0, 0),
                 ee_kind: str = "circle", plane: str = "xz", radius: float = 0.04,
                 period: float = 8.0, cycles: int = 2,
                 pos_threshold: float = 1e-4, ori_threshold: float = 1e-3,
                 max_iters: int = 500, settle_s: float = 1.0, **base_kw):
        super().__init__(can_channel, **base_kw)
        self.q_nom = np.radians(np.asarray(q_nom_deg, dtype=float))
        self.ee_kind = ee_kind
        self.plane = plane
        self.radius = radius
        self.period = period
        self.cycles = cycles
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.max_iters = max_iters
        self.settle_s = settle_s
        self.kin = ee.make_kinematics()

    def solve_offline(self):
        """Build reference + IK + gate (no hardware). Returns dict for dry-run/live."""
        t, T_ref = ee.build_reference(
            self.kin, self.q_nom, kind=self.ee_kind, plane=self.plane,
            radius=self.radius, period=self.period, cycles=self.cycles,
            sample_hz=self.sample_hz)
        q_ref, conv = ee.solve_ik_path(
            self.kin, T_ref, self.q_nom, pos_threshold=self.pos_threshold,
            ori_threshold=self.ori_threshold, max_iters=self.max_iters)
        g = ee.gate(q_ref, conv, sample_hz=self.sample_hz, buffer_rad=safety.LIMIT_BUFFER_RAD,
                    urdf_lim=ee.urdf_limits(self.kin))
        return dict(t=t, T_ref=T_ref, q_ref=q_ref, converged=conv, gate=g)

    def run(self, offline: dict) -> dict:
        """Stream the pre-solved q_ref and capture the joint response.

        Requires ``offline`` from :meth:`solve_offline` with a PASSED gate.
        Returns dict with t, q_ref, q_resp, T_ref, T_resp.
        """
        if not offline["gate"].passed:
            raise RuntimeError("EE gate not passed — refusing to drive hardware")
        kp, kd = self.effective_gains()
        q_ref = offline["q_ref"]
        t_ref = offline["t"]
        n = len(q_ref)

        def sample_fn(t: float) -> np.ndarray:
            k = min(int(round(t * self.sample_hz)), n - 1)
            return q_ref[k]

        try:
            print(f"[ee] transit to q_nom {np.round(self.q_nom * DEG, 1)}")
            self.transit_to(self.q_nom)
            time.sleep(self.settle_s)
            print(f"[ee] track {t_ref[-1]:.1f}s ({self.ee_kind} r={self.radius*1000:.0f}mm)")
            t, ref_q, resp_q = self.stream(sample_fn, float(t_ref[-1]), kp, kd)
            time.sleep(self.settle_s)
        finally:
            print("[ee] return to zero")
            self.return_to_zero()
        # FK both reference (from IK'd q) and response for the recorded window.
        T_resp = ee.fk_path(self.kin, resp_q)
        T_ref_meas = ee.fk_path(self.kin, ref_q)
        return dict(t=t, q_ref=ref_q, q_resp=resp_q, T_ref=T_ref_meas, T_resp=T_resp)
