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
                 kp: Optional[np.ndarray] = None, kd: Optional[np.ndarray] = None,
                 inter_cmd_gap_us: Optional[float] = None,
                 integral_level: str = "K0",
                 integral_joints: Optional[list] = None,
                 integral_overrides: Optional[dict] = None,
                 coulomb_ff=None):
        # Cross-check the mirrored SDK limits against the live constant so a
        # future SDK edit can't silently desync the safety gate.
        safety.assert_matches_sdk_limits(_JOINT_LIMITS)
        self.can_channel = can_channel
        self.sample_hz = sample_hz
        self.dt = 1.0 / sample_hz
        self.transit_speed = _rad_speed(transit_speed_deg_s)
        self.kp = kp
        self.kd = kd
        # CAN pacing gap forwarded verbatim to get_a1z_robot (SOP-06: the SDK
        # reads no env var; None = SDK default 250 µs, 0 = unpaced/B0 regime).
        self.inter_cmd_gap_us = inter_cmd_gap_us
        # Error-integral feedforward level + config, forwarded to get_a1z_robot
        # (SOP-09 §7 mechanism-pair companion run; K0 = no integrator).
        self.integral_level = integral_level
        self.integral_joints = integral_joints
        self.integral_overrides = integral_overrides
        # Coulomb friction feedforward: ndarray (legacy hard-sign) or
        # CoulombConfig (tanh-smoothed, SOP-11 §7.2).  None = disabled.
        self.coulomb_ff = coulomb_ff
        # Robot/CAN bus are created lazily in start() so offline paths
        # (dry-run gate, IK solve) never open the bus.
        self.robot = None
        self._started = False

    # --- lifecycle ---
    def start(self) -> None:
        # Position-hold + PD (NOT zero-gravity): we want the arm to track.
        self.robot = get_a1z_robot(can_channel=self.can_channel,
                                   zero_gravity_mode=False, with_gripper=False,
                                   inter_cmd_gap_us=self.inter_cmd_gap_us,
                                   integral_level=self.integral_level,
                                   integral_joints=self.integral_joints,
                                   integral_overrides=self.integral_overrides,
                                   coulomb_ff=self.coulomb_ff)
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
        elif self.robot is not None:
            # robot.start() may have raised after opening the bus (e.g.
            # incomplete startup feedback) — still close the CAN interface so
            # the OS does not warn "SocketcanBus was not properly shut down".
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
    def _joint_diag(self, joints, target=None) -> str:
        """Per-joint telemetry snapshot attached to arrival-check failures so a
        stall's cause is a readout, not a guess. Reports, per offending joint:
        measured pos, commanded pos, target, effort, temp, error code, plus the
        commanded kp/kd and global estop state. Distinguishes:
          * cmd≈meas, eff≈0        → command never reached target (command path)
          * cmd=target, meas lags, eff high → torque-limited (friction/collision)
          * kp≈0 / estop set        → safety latch zeroed the position gain."""
        try:
            st = self.robot.get_joint_state()
        except Exception:  # noqa: BLE001
            return ""
        cmd = getattr(self.robot, "_command", None)
        estopped = getattr(self.robot, "is_estopped", None)
        parts = []
        for i in joints:
            seg = f"J{i+1}"
            if "pos" in st:
                seg += f" meas={st['pos'][i] * DEG:.1f}°"
            if cmd is not None:
                seg += f" cmd={cmd.pos[i] * DEG:.1f}° kp={cmd.kp[i]:.0f} kd={cmd.kd[i]:.2f}"
            if target is not None:
                seg += f" tgt={np.asarray(target, float)[i] * DEG:.1f}°"
            if "eff" in st:
                seg += f" eff={st['eff'][i]:.2f}"
            if "temp_mos" in st:
                seg += f" mos={st['temp_mos'][i]:.0f}C"
            if "error_codes" in st:
                seg += f" err=0x{int(st['error_codes'][i]):x}"
            parts.append(seg)
        tail = f" estop={estopped}" if estopped is not None else ""
        return " | ".join(parts) + tail

    def transit_to(self, q_target: np.ndarray, *, max_jump_rad: float = 3.5) -> None:
        """Slow min-jerk move to a full 6-dof pose at the transit speed.

        Always uses DEFAULT gains (test gains apply only to excitation). Raises
        on limit / far-jump violation so callers abort into safe shutdown.
        """
        q_target = np.asarray(q_target, dtype=float)
        self.robot.move_joints(q_target[:6], speed=self.transit_speed,
                               max_jump_rad=max_jump_rad)
        # Verify arrival (SOP-01 §4.1 fail-safe: > 3° off → abort). One recheck
        # after a short extra wait before failing: J6 (CAN id 0x06, lowest bus
        # priority) has shown single-sample "stuck" reads with near-zero
        # effort (2026-07-20 run-66) — consistent with a stale/dropped
        # feedback frame rather than an actual stall, since a real stall
        # would still show the PD fighting the error (high eff). A recheck
        # that clears is logged, not silently swallowed, so it stays visible
        # for the CAN-delivery investigation (devlog §3.3/§3.4).
        for attempt in range(2):
            time.sleep(0.05 if attempt == 0 else 0.15)
            meas = self.robot.get_joint_pos()[:6]
            off = np.abs(meas - q_target[:6]) * DEG
            if not np.any(off > 3.0):
                if attempt > 0:
                    print(f"[runner] transit arrival recheck cleared after retry "
                          f"(initial read was stale/glitched)")
                return
        offenders = np.flatnonzero(off > 3.0)
        bad = "; ".join(f"J{i+1}={off[i]:.1f}°" for i in offenders)
        diag = self._joint_diag(offenders, target=q_target[:6])
        msg = f"transit arrival check failed (>3°) after recheck: {bad}"
        if diag:
            msg += f"  [{diag}]"
        raise RuntimeError(msg)

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
    def stream(self, sample_fn, duration: float, kp: np.ndarray, kd: np.ndarray,
               vel_fn=None, watchdog=None,
               ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Stream ``sample_fn(t)->(6,)`` at sample_hz for ``duration`` seconds.

        Returns (t (M,), ref (M,6), resp (M,6), eff (M,6)). RAM-buffered; no
        disk IO in loop. Aborts (returns partial) if estop latches mid-stream.

        Position and effort come from one ``get_joint_state()`` call so both are
        the same lock-held state snapshot. Effort is the stick-slip vs
        command-path discriminator (SOP-03 §5.4): during a stall, eff ramping
        up means the motor is fighting friction/contact; eff≈0 means the
        command never produced torque (CAN delivery / gain latch).

        ``vel_fn``: optional ``vel_fn(t)->(6,)`` velocity feedforward q̇_ref
        (SOP-10 §1.1 I2). When None (default) the commanded velocity stays
        ``_ZERO6`` every tick — this path is byte-for-byte the pre-vel-ff
        behaviour (the only opt-in switch that changes what the follower sees,
        devlog Q7). When given, ``js['vel']`` is refreshed each tick so the
        follower's ``kd·(v_des−v)`` term tracks the reference slope, matching
        the real teleop通路.

        ``watchdog``: optional object with
        ``check(t, ref, resp, eff, *, vel=None) -> (ok, reason)``.  When
        ``ok`` is False the stream breaks immediately (SOP-11 §4.1). The
        watchdog records its trip state internally (``wd.tripped`` /
        ``wd.reason``); the caller inspects it after ``stream()`` returns.
        Measured joint velocity is passed as the ``vel`` keyword when the
        watchdog accepts it.  Default ``None`` preserves byte-identical
        behaviour for existing callers.
        """
        t_buf: List[float] = []
        ref_buf: List[np.ndarray] = []
        resp_buf: List[np.ndarray] = []
        eff_buf: List[np.ndarray] = []
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
            if vel_fn is not None:
                js["vel"] = np.asarray(vel_fn(t), dtype=float)[:6]
            self.robot.command_joint_state(js)
            st = self.robot.get_joint_state()  # copies under one state lock
            t_buf.append(t)
            ref_buf.append(q_ref)
            resp_buf.append(st["pos"][:6])
            eff_buf.append(st["eff"][:6])
            if watchdog is not None:
                wd_ok, wd_reason = watchdog.check(
                    t, q_ref, st["pos"][:6], st["eff"][:6],
                    vel=st.get("vel", _ZERO6)[:6],
                )
                if not wd_ok:
                    print(f"[runner] stream aborted: watchdog {wd_reason}")
                    break
            time.sleep(max(0.0, self.dt - (time.monotonic() - tick)))
        return (np.array(t_buf), np.array(ref_buf), np.array(resp_buf),
                np.array(eff_buf))


class JointUnitTestRunner(_Base):
    """Per-joint square + triangle excitation with full transit discipline."""

    def __init__(self, can_channel: str, *, amp_deg: float = 15.0,
                 margin_deg: float = 5.0, period: float = 4.0,
                 period_triangle: float | None = None, cycles: int = 3,
                 hold_pre: float = 1.0, hold_post: float = 1.5,
                 edge_rate_deg_s: float = 240.0, waves=("square", "triangle"),
                 vel_ff: bool = False, vel_blend_s: float = 0.15,
                 **base_kw):
        super().__init__(can_channel, **base_kw)
        self.amp_deg = amp_deg
        self.margin_deg = margin_deg
        self.period = period
        # None => triangle shares --period with square (old behavior). Set to
        # decouple triangle's sweep rate for the scan-rate diagnostic (SOP-03
        # §10.16 / devlog 2026-07-20 §3.x): halving triangle period tests
        # whether jump amplitude scales with slope (sparse-update artifact)
        # vs stays put (friction/PD stick-slip).
        self.period_triangle = period_triangle
        self.cycles = cycles
        self.hold_pre = hold_pre
        self.hold_post = hold_post
        self.edge_rate_deg_s = edge_rate_deg_s
        self.waves = tuple(waves)
        # vel-ff (SOP-10): opt-in analytic q̇_ref feedforward, TRIANGLE only
        # (square slew ≈240°/s would blow the 4.0 rad/s cap, §1.3). vel_blend_s
        # is threaded into every WaveTrajectory so the apex crossover is smooth.
        self.vel_ff = bool(vel_ff)
        self.vel_blend_s = vel_blend_s

    def run_joint(self, joint1: int) -> Dict[str, dict]:
        """Excite joint ``joint1`` (1-based). Returns {wave: {t, ref, resp}}.

        Full timeline (§3.3): preconditions → waveform start → [square, triangle]
        → retract joint → retract preconditions → zero. finally-guarded.
        """
        j = joint1 - 1
        kp, kd = self.effective_gains()
        # Build both wave trajectories up front (shared q_start / preconditions).
        # triangle may run at its own sweep period (self.period_triangle) while
        # square keeps self.period — see __init__ docstring note.
        def _period_for(name):
            if name == "triangle" and self.period_triangle is not None:
                return self.period_triangle
            return self.period
        trajs = {name: WaveTrajectory(_ZERO6, j, name, amp_deg=self.amp_deg,
                                      margin_deg=self.margin_deg, period=_period_for(name),
                                      cycles=self.cycles, hold_pre=self.hold_pre,
                                      hold_post=self.hold_post,
                                      edge_rate_deg_s=self.edge_rate_deg_s,
                                      vel_blend_s=self.vel_blend_s)
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
                # vel-ff派发 (SOP-10 §1.1 I3): TRIANGLE only, and only when the
                # runner was built with vel_ff=True. square never gets a vel_fn
                # (its ~240°/s slew would exceed the 4.0 rad/s cap — code-level
                # written here, not a convention, §1.3). None → vel=0 (现状口径).
                vel_fn = (tr.sample_vel
                          if (self.vel_ff and name == "triangle") else None)
                vtag = "  vel-ff" if vel_fn is not None else ""
                print(f"[J{joint1}] excite {name} ({tr.duration:.1f}s, gains "
                      f"kp[{j}]={kp[j]:.1f} kd[{j}]={kd[j]:.2f}){vtag}")
                t, ref, resp, eff = self.stream(tr.sample, tr.duration, kp, kd,
                                                vel_fn=vel_fn)
                out[name] = dict(t=t, ref=ref, resp=resp, eff=eff, traj=tr)
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
                 period: float = 8.0, cycles: int = 2, hold_s: float = 0.0,
                 pos_threshold: float = 1e-4, ori_threshold: float = 1e-3,
                 max_iters: int = 500, settle_s: float = 1.0, **base_kw):
        super().__init__(can_channel, **base_kw)
        self.q_nom = np.radians(np.asarray(q_nom_deg, dtype=float))
        self.ee_kind = ee_kind
        self.plane = plane
        self.radius = radius
        self.period = period
        self.cycles = cycles
        self.hold_s = hold_s
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
            sample_hz=self.sample_hz, hold_s=self.hold_s)
        q_ref, conv = ee.solve_ik_path(
            self.kin, T_ref, self.q_nom, pos_threshold=self.pos_threshold,
            ori_threshold=self.ori_threshold, max_iters=self.max_iters)
        g = ee.gate(q_ref, conv, sample_hz=self.sample_hz, buffer_rad=safety.LIMIT_BUFFER_RAD,
                    urdf_lim=ee.urdf_limits(self.kin))
        return dict(t=t, T_ref=T_ref, q_ref=q_ref, converged=conv, gate=g)

    def run(self, offline: dict, watchdog=None) -> dict:
        """Stream the pre-solved q_ref and capture the joint response.

        Requires ``offline`` from :meth:`solve_offline` with a PASSED gate.
        Returns dict with t, q_ref, q_resp, T_ref, T_resp.  ``watchdog`` is an
        optional tick-level checker forwarded to ``stream()`` (E-segment
        refine sessions attach a MultiTickWatchdog as the vel_abs/pos/eff
        backstop, SOP-11 §12.1); ``None`` preserves the legacy behaviour.
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
            t, ref_q, resp_q, eff_q = self.stream(sample_fn, float(t_ref[-1]), kp, kd,
                                                  watchdog=watchdog)
            time.sleep(self.settle_s)
        finally:
            print("[ee] return to zero")
            self.return_to_zero()
        # FK both reference (from IK'd q) and response for the recorded window.
        T_resp = ee.fk_path(self.kin, resp_q)
        T_ref_meas = ee.fk_path(self.kin, ref_q)
        return dict(t=t, q_ref=ref_q, q_resp=resp_q, q_eff=eff_q,
                    T_ref=T_ref_meas, T_resp=T_resp)
