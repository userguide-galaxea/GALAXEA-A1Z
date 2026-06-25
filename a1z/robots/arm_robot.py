"""A1Z arm robot implementation with gravity compensation."""

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import can
import numpy as np

from a1z.dynamics.gravity_model import GravityModel
from a1z.motor_drivers.motor_b_driver import MixedMotorChain

logger = logging.getLogger(__name__)

# Hard saturation for commanded feedforward velocity/acceleration, applied in
# the control loop regardless of who produced the command. Legitimate
# minimum-jerk profiles stay below ~2 rad/s and ~16 rad/s²; anything beyond
# is a planner bug and must not reach inverse dynamics (inertia torque scales
# with acc and would otherwise spike the torque safety stop) or the motor
# velocity setpoint.
_MAX_CMD_VEL_RAD_S = 4.0
_MAX_CMD_ACC_RAD_S2 = 20.0

# Bounds on PD gains supplied through command_joint_state. The motor protocol
# layer also clips to its hardware range (kp_max=500, kd_max=5), but we cap
# earlier so our own PD math never operates on absurd gains.
_MAX_CMD_KP = 200.0
_MAX_CMD_KD = 5.0

# How many consecutive streaming frames may be rejected for out-of-limits
# before we engage the soft estop. Catches a broken IK upstream flooding the
# controller with junk instead of silently holding the last good command.
_MAX_REJECTED_STREAM_FRAMES = 5


@dataclass
class JointState:
    """Joint state for all DOFs."""

    pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    eff: np.ndarray = field(default_factory=lambda: np.zeros(6))
    error_codes: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=int))
    temp_mos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    temp_rotor: np.ndarray = field(default_factory=lambda: np.zeros(6))


@dataclass
class JointCommand:
    """Joint command for all DOFs."""

    pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(6))
    kp: np.ndarray = field(default_factory=lambda: np.zeros(6))
    kd: np.ndarray = field(default_factory=lambda: np.zeros(6))
    torque_ff: np.ndarray = field(default_factory=lambda: np.zeros(6))


class ArmRobot:
    """A1Z 6-DOF arm robot with gravity compensation.

    Manages a MixedMotorChain (MotorA + MotorB), a Pinocchio gravity model,
    and runs a background control loop for gravity compensation + PD control.
    """

    def __init__(
        self,
        motor_chain: MixedMotorChain,
        bus: can.BusABC,
        gravity_model: GravityModel,
        num_joints: int = 6,
        gravity_comp_factor: float = 1.0,
        zero_gravity_mode: bool = True,
        joint_sign: Optional[np.ndarray] = None,
        gravity_torque_scale: Optional[np.ndarray] = None,
        max_gravity_torque: Optional[np.ndarray] = None,
        torque_clip: Optional[np.ndarray] = None,
        default_kp: Optional[np.ndarray] = None,
        default_kd: Optional[np.ndarray] = None,
        joint_limits: Optional[List[Tuple[float, float]]] = None,
        control_freq_hz: int = 250,
        min_freq_hz: float = 80.0,
        motor_a_kt: float = 2.8,
        # --- runtime safety (P0) ---
        runtime_limit_buffer_rad: float = 0.15,
        vel_limit: Optional[np.ndarray] = None,
        temp_mos_warn_c: float = 70.0,
        temp_mos_estop_c: float = 85.0,
        temp_rotor_warn_c: float = 75.0,
        temp_rotor_estop_c: float = 90.0,
        stale_feedback_warn_s: float = 0.05,
        stale_feedback_estop_s: float = 0.2,
    ):
        self._motor_chain = motor_chain
        self._bus = bus
        self._gravity_model = gravity_model
        self._num_joints = num_joints
        self.gravity_comp_factor = gravity_comp_factor
        self.zero_gravity_mode = zero_gravity_mode

        self._joint_sign = joint_sign if joint_sign is not None else np.ones(num_joints)
        self._gravity_torque_scale = gravity_torque_scale if gravity_torque_scale is not None else np.ones(num_joints)
        self._max_gravity_torque = max_gravity_torque if max_gravity_torque is not None else np.full(num_joints, 50.0)
        self._torque_clip = torque_clip if torque_clip is not None else np.full(num_joints, 50.0)
        self._default_kp = default_kp if default_kp is not None else np.array([30.0, 30.0, 30.0, 20.0, 5.0, 5.0])
        self._default_kd = default_kd if default_kd is not None else np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
        self._joint_limits = joint_limits
        self._control_freq_hz = control_freq_hz
        self._control_period_s = 1.0 / control_freq_hz
        self._min_freq_hz = min_freq_hz

        self._state = JointState(
            pos=np.zeros(num_joints),
            vel=np.zeros(num_joints),
            eff=np.zeros(num_joints),
        )
        self._command = JointCommand(
            pos=np.zeros(num_joints),
            vel=np.zeros(num_joints),
            kp=np.zeros(num_joints),
            kd=np.zeros(num_joints),
            torque_ff=np.zeros(num_joints),
        )
        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._recording: bool = False
        self._record_buffer: List[Tuple[float, np.ndarray]] = []
        self._record_lock = threading.Lock()
        self._record_last_t: float = 0.0
        self._record_period: float = 1.0 / 50.0

        self._last_clip_warn_t: float = 0.0

        # --- runtime safety state ---
        self._runtime_limit_buffer_rad = runtime_limit_buffer_rad
        # Default per-joint velocity caps: ~70% of each motor's hardware vel_max
        # (MotorA ±18 → 12; joint4 MotorB ±10 → 7; joints 5,6 MotorB ±30 → 20).
        self._vel_limit = (
            vel_limit.copy() if vel_limit is not None
            else np.array([12.0, 12.0, 12.0, 7.0, 20.0, 20.0])
        )
        self._temp_mos_warn_c = temp_mos_warn_c
        self._temp_mos_estop_c = temp_mos_estop_c
        self._temp_rotor_warn_c = temp_rotor_warn_c
        self._temp_rotor_estop_c = temp_rotor_estop_c
        self._stale_warn_s = stale_feedback_warn_s
        self._stale_estop_s = stale_feedback_estop_s
        self._last_feedback_t: float = 0.0
        self._last_temp_warn_t: float = 0.0
        self._last_stale_warn_t: float = 0.0
        self._estop_latch = threading.Event()
        # Consecutive streaming frames rejected for out-of-limit positions.
        # Reset on any accepted frame; triggers estop at threshold.
        self._rejected_frames: int = 0
        self._last_reject_warn_t: float = 0.0

    def num_dofs(self) -> int:
        return self._num_joints

    def start(
        self,
        initial_kp: Optional[np.ndarray] = None,
        initial_kd: Optional[np.ndarray] = None,
    ) -> None:
        """Enable motors and start the control loop.

        Args:
            initial_kp: Override kp gains for startup.
            initial_kd: Override kd gains for startup.
        """
        logger.info("Enabling motors...")
        self._motor_chain.enable_all()

        # MotorA does not return feedback on enable alone — it needs at least one
        # MIT command first.  Send a zero-gain probe (kp=0, tiny kd, zero torque)
        # so the motor responds without applying any position correction, then wait
        # for the replies before reading the actual initial position.
        _zero = np.zeros(self._num_joints)
        _probe_kd = np.full(self._num_joints, 0.05)
        self._motor_chain.send_commands(_zero, _zero, _zero, _probe_kd, _zero)
        time.sleep(0.05)

        # Read initial state
        self._read_state()
        logger.info(f"Initial joint positions: {np.round(self._state.pos, 3)} rad")

        if self._joint_limits is not None:
            self._check_joint_limits(self._state.pos)

        # Set initial command
        with self._command_lock:
            self._command.pos = self._state.pos.copy()
            if initial_kp is not None:
                self._command.kp = initial_kp.copy()
            elif not self.zero_gravity_mode:
                self._command.kp = self._default_kp.copy()
            else:
                self._command.kp = np.zeros(self._num_joints)

            if initial_kd is not None:
                self._command.kd = initial_kd.copy()
            elif not self.zero_gravity_mode:
                self._command.kd = self._default_kd.copy()
            else:
                self._command.kd = self._default_kd.copy() * 0.5

        logger.info(f"Initial kp={np.round(self._command.kp, 1)}, kd={np.round(self._command.kd, 2)}")

        self._stop_event.clear()
        self._running = True
        self._estop_latch.clear()
        # Initialize so the first control loop iteration doesn't immediately
        # flag the bus as stale before any messages have been drained.
        self._last_feedback_t = time.time()
        self._thread = threading.Thread(target=self._control_loop, name="arm_control_loop", daemon=True)
        self._thread.start()
        logger.info(f"Control loop started at {self._control_freq_hz} Hz")

    def stop(self) -> None:
        """Stop the control loop and disable all motors."""
        logger.info("Stopping control loop...")
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            # Normal path: control loop disables motors before returning.
            # disable_all() below is a safety net in case join times out.
            self._thread.join(timeout=2.0)
        self._running = False
        # Safety net: disable again from main thread in case the control thread
        # was killed before its own disable_all() completed (e.g. join timeout).
        self._motor_chain.disable_all()
        logger.info("All motors disabled.")

    def _accept_or_reject_stream(
        self, pos: np.ndarray
    ) -> Optional[np.ndarray]:
        """Run streaming clip, count rejections, engage estop on a flood.

        Returns the (possibly tolerance-clipped) safe position to write to
        the command, or None if the caller must skip updating the command
        (caller holds the previous valid command). After
        :data:`_MAX_REJECTED_STREAM_FRAMES` consecutive rejections the soft
        estop is engaged to stop a broken upstream from flooding garbage.
        """
        safe = self._clip_joint_pos(pos)
        if safe is None:
            self._rejected_frames += 1
            if self._rejected_frames >= _MAX_REJECTED_STREAM_FRAMES:
                logger.error(
                    f"{self._rejected_frames} consecutive streaming frames "
                    "rejected — engaging soft estop"
                )
                self.estop()
            return None
        self._rejected_frames = 0
        return safe

    def command_joint_pos(self, pos: np.ndarray) -> None:
        """Set target joint angles (rad) with default PD gains."""
        if self._estop_latch.is_set():
            logger.warning("command_joint_pos rejected: robot is in estop")
            return
        arm_pos = self._accept_or_reject_stream(pos[:self._num_joints])
        if arm_pos is None:
            return
        with self._command_lock:
            self._command.pos = arm_pos.copy()
            self._command.kp = self._default_kp.copy()
            self._command.kd = self._default_kd.copy()
            self._command.torque_ff = np.zeros(self._num_joints)

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Set target joint state.

        Args:
            joint_state: Dict with keys 'pos', 'vel', and optionally 'kp', 'kd'.

        Out-of-range pos / vel / kp / kd reject the entire frame (previous
        command is held). Silent clipping of garbage feedforward would let
        a bad upstream silently distort the trajectory or PD response, which
        is harder to diagnose than a refused frame plus an error log.
        """
        if self._estop_latch.is_set():
            logger.warning("command_joint_state rejected: robot is in estop")
            return
        pos = self._accept_or_reject_stream(joint_state["pos"])
        if pos is None:
            return

        vel = np.asarray(joint_state["vel"], dtype=np.float64)
        if np.any(np.abs(vel) > _MAX_CMD_VEL_RAD_S):
            offenders = "; ".join(
                f"joint{i + 1}={vel[i]:.2f}"
                for i in np.flatnonzero(np.abs(vel) > _MAX_CMD_VEL_RAD_S)
            )
            logger.error(
                f"command_joint_state rejected: vel exceeds "
                f"{_MAX_CMD_VEL_RAD_S} rad/s ({offenders})"
            )
            return

        kp = np.asarray(joint_state.get("kp", self._default_kp), dtype=np.float64)
        if np.any(kp < 0) or np.any(kp > _MAX_CMD_KP):
            logger.error(
                f"command_joint_state rejected: kp out of [0, {_MAX_CMD_KP}] "
                f"({np.round(kp, 2).tolist()})"
            )
            return
        kd = np.asarray(joint_state.get("kd", self._default_kd), dtype=np.float64)
        if np.any(kd < 0) or np.any(kd > _MAX_CMD_KD):
            logger.error(
                f"command_joint_state rejected: kd out of [0, {_MAX_CMD_KD}] "
                f"({np.round(kd, 3).tolist()})"
            )
            return

        with self._command_lock:
            self._command.pos = pos.copy()
            self._command.vel = vel.copy()
            self._command.kp = kp.copy()
            self._command.kd = kd.copy()

    def get_joint_pos(self) -> np.ndarray:
        with self._state_lock:
            return self._state.pos.copy()

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        with self._state_lock:
            return {
                "pos": self._state.pos.copy(),
                "vel": self._state.vel.copy(),
                "eff": self._state.eff.copy(),
                "error_codes": self._state.error_codes.copy(),
                "temp_mos": self._state.temp_mos.copy(),
                "temp_rotor": self._state.temp_rotor.copy(),
            }

    def get_observations(self) -> Dict[str, np.ndarray]:
        state = self.get_joint_state()
        return {
            "joint_pos": state["pos"],
            "joint_vel": state["vel"],
            "joint_eff": state["eff"],
            "joint_error_codes": state["error_codes"],
            "joint_temp_mos": state["temp_mos"],
            "joint_temp_rotor": state["temp_rotor"],
        }

    def get_robot_info(self) -> Dict[str, Any]:
        return {
            "num_joints": self._num_joints,
            "default_kp": self._default_kp.copy(),
            "default_kd": self._default_kd.copy(),
            "joint_limits": self._joint_limits,
            "gravity_comp_factor": self.gravity_comp_factor,
            "control_freq_hz": self._control_freq_hz,
        }

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_estopped(self) -> bool:
        return self._estop_latch.is_set()

    def estop(self) -> None:
        """Engage soft emergency stop.

        Atomically zeros position gain, halves the velocity damping, clears
        any feedforward torque, and pins the command position to the current
        measured position. Gravity compensation keeps running so the arm
        does not collapse under load.

        Subsequent command_joint_pos / command_joint_state calls are silently
        rejected until :meth:`release` is called. An in-flight
        :meth:`move_joints` exits its interpolation loop on the next step
        and returns early.
        """
        if self._estop_latch.is_set():
            return
        logger.warning("[ArmRobot] ESTOP engaged — commands suspended")
        with self._state_lock:
            cur_pos = self._state.pos.copy()
        with self._command_lock:
            self._command.pos = cur_pos
            self._command.vel = np.zeros(self._num_joints)
            self._command.acc = np.zeros(self._num_joints)
            self._command.kp = np.zeros(self._num_joints)
            self._command.kd = self._default_kd.copy() * 0.5
            self._command.torque_ff = np.zeros(self._num_joints)
        self._estop_latch.set()

    def release(self) -> None:
        """Release the estop latch and resume with default PD at current pose."""
        if not self._estop_latch.is_set():
            return
        with self._state_lock:
            cur_pos = self._state.pos.copy()
        with self._command_lock:
            self._command.pos = cur_pos
            self._command.vel = np.zeros(self._num_joints)
            self._command.acc = np.zeros(self._num_joints)
            self._command.kp = self._default_kp.copy()
            self._command.kd = self._default_kd.copy()
            self._command.torque_ff = np.zeros(self._num_joints)
        self._estop_latch.clear()
        logger.info("[ArmRobot] ESTOP released")

    def move_joints(
        self,
        target_pos: np.ndarray,
        speed: float = 0.5,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        max_jump_rad: Optional[float] = None,
    ) -> None:
        """Smoothly interpolate to target position at the given speed (rad/s).

        Blocks until the target is reached or close enough.
        Uses minimum-jerk (5th-order polynomial) interpolation so velocity and
        acceleration are zero at both endpoints, eliminating start/stop jolts.

        Args:
            target_pos: Target joint angles (rad). Must be within joint limits
                (small overshoot up to 0.05 rad is tolerated and clipped).
            speed: Max joint speed (rad/s).
            kp/kd: Optional PD gain overrides.
            max_jump_rad: If set, refuse to move when any joint is farther than
                this from the target. Useful to reject far-away IK solutions
                (e.g. elbow-flipped branches) that would sweep the arm across
                the workspace.

        Raises:
            ValueError: If the target exceeds joint limits beyond tolerance,
                or violates max_jump_rad.
        """
        if self._estop_latch.is_set():
            logger.warning("move_joints rejected: robot is in estop")
            return
        # Minimum-jerk peak velocity is 1.875 × average. Reject upfront so we
        # never generate a feedforward that the _update assert would refuse.
        if speed <= 0:
            raise ValueError(f"move_joints speed must be > 0, got {speed}")
        if speed * 1.875 > _MAX_CMD_VEL_RAD_S:
            raise ValueError(
                f"move_joints speed {speed:.2f} rad/s exceeds feedforward "
                f"cap: peak vel {speed * 1.875:.2f} > {_MAX_CMD_VEL_RAD_S} rad/s"
            )

        target_pos = self._validate_joint_pos(target_pos)
        # Safety check uses the *measured* position — this is what max_jump_rad
        # actually protects (e.g. catching elbow-flipped IK against reality).
        measured_pos = self.get_joint_pos()

        if max_jump_rad is not None:
            jumps = np.abs(target_pos - measured_pos)
            if np.any(jumps > max_jump_rad):
                offenders = "; ".join(
                    f"joint{i + 1}: {jumps[i]:.3f} rad"
                    for i in np.flatnonzero(jumps > max_jump_rad)
                )
                raise ValueError(
                    f"Target too far from current position "
                    f"(max_jump_rad={max_jump_rad}), refusing to move: {offenders}"
                )

        # Trajectory start uses the *last commanded* position so back-to-back
        # move_joints calls keep command-space continuity. Using the measured
        # position here would inject a backwards step equal to the PD tracking
        # error between consecutive moves, which the PD loop sees as a jerk.
        with self._command_lock:
            current_pos = self._command.pos.copy()

        kp = kp if kp is not None else self._default_kp
        kd = kd if kd is not None else self._default_kd

        max_dist = np.max(np.abs(target_pos - current_pos))
        if max_dist < 0.001:
            return

        # Minimum-jerk peak acc ≈ 5.7735·delta/duration². A re-command of an
        # already-reached pose (tiny delta) or a high-speed short-distance
        # move would otherwise spike past the feedforward cap and trip the
        # _update assert. Clamp duration from below by:
        #   (a) the fixed MIN_DURATION (prevents re-command twitch),
        #   (b) sqrt(6·delta/MAX_ACC) — slight over-estimate of the 5.7735
        #       coefficient gives ~4% acc headroom for float epsilon.
        # The vel cap is already guaranteed by the speed pre-check above.
        _MIN_MOVE_DURATION_S = 0.3
        _acc_dur = float(np.sqrt(6.0 * max_dist / _MAX_CMD_ACC_RAD_S2))
        duration = max(max_dist / speed, _acc_dur, _MIN_MOVE_DURATION_S)
        dt = self._control_period_s
        steps = max(1, int(duration / dt))
        delta = target_pos - current_pos

        for step in range(1, steps + 1):
            if self._estop_latch.is_set():
                logger.warning("move_joints aborted mid-trajectory: estop engaged")
                return
            t = step / steps
            # minimum-jerk profile: pos and vel are zero at t=0 and t=1
            alpha = 10*t**3 - 15*t**4 + 6*t**5
            alpha_dot = (30*t**2 - 60*t**3 + 30*t**4) / duration
            alpha_ddot = (60*t - 180*t**2 + 120*t**3) / duration**2
            with self._command_lock:
                self._command.pos = current_pos + alpha * delta
                self._command.vel = alpha_dot * delta
                self._command.acc = alpha_ddot * delta
                self._command.kp = kp.copy()
                self._command.kd = kd.copy()
            time.sleep(dt)

        with self._command_lock:
            self._command.pos = target_pos.copy()
            self._command.vel = np.zeros(self._num_joints)
            self._command.acc = np.zeros(self._num_joints)

    def start_recording(self, sample_hz: int = 50) -> None:
        """Start recording joint positions (during gravity-comp teaching).

        Args:
            sample_hz: Recording sample rate in Hz (default 50).
        """
        if not self._running:
            raise RuntimeError("Robot not running. Call start() first.")
        with self._record_lock:
            self._record_buffer = []
            self._record_period = 1.0 / max(1, sample_hz)
            self._record_last_t = 0.0
            self._recording = True
        logger.info(f"Recording started at {sample_hz} Hz")

    def stop_recording(self) -> List[Tuple[float, np.ndarray]]:
        """Stop recording and return the trajectory.

        Returns:
            List of (timestamp_s, joint_positions_rad) tuples with timestamps
            relative to the start of the recording.
        """
        with self._record_lock:
            self._recording = False
            raw = list(self._record_buffer)
        if not raw:
            logger.info("Recording stopped: 0 frames")
            return []
        t0 = raw[0][0]
        traj = [(t - t0, pos.copy()) for t, pos in raw]
        logger.info(f"Recording stopped: {len(traj)} frames, {traj[-1][0]:.2f}s")
        return traj

    def play_trajectory(
        self,
        trajectory: List[Tuple[float, np.ndarray]],
        speed_factor: float = 1.0,
    ) -> None:
        """Play back a recorded trajectory.

        Args:
            trajectory: List of (timestamp_s, joint_positions_rad) as returned
                by stop_recording() or load_recording().
            speed_factor: >1 speeds up, <1 slows down (default 1.0 = real time).
        """
        if not trajectory:
            raise ValueError("Empty trajectory")
        if not self._running:
            raise RuntimeError("Robot not running. Call start() first.")
        if speed_factor <= 0:
            raise ValueError("speed_factor must be > 0")

        t0_play = time.time()
        for t_rec, pos in trajectory:
            t_target = t0_play + t_rec / speed_factor
            self.command_joint_pos(pos)
            sleep_t = t_target - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)

    @staticmethod
    def save_recording(
        trajectory: List[Tuple[float, np.ndarray]],
        path: str,
    ) -> None:
        """Save a trajectory to a JSON file.

        Args:
            trajectory: As returned by stop_recording().
            path: Output file path (e.g. "teach.json").
        """
        data = {
            "version": 1,
            "num_joints": len(trajectory[0][1]) if trajectory else 6,
            "frames": [[t, pos.tolist()] for t, pos in trajectory],
        }
        with open(path, "w") as f:
            json.dump(data, f)
        logger.info(f"Saved {len(trajectory)} frames to {path}")

    @staticmethod
    def load_recording(path: str) -> List[Tuple[float, np.ndarray]]:
        """Load a trajectory from a JSON file saved by save_recording().

        Returns:
            List of (timestamp_s, joint_positions_rad) tuples.
        """
        with open(path) as f:
            data = json.load(f)
        traj = [(float(t), np.array(pos, dtype=np.float64)) for t, pos in data["frames"]]
        logger.info(f"Loaded {len(traj)} frames from {path}")
        return traj

    # --- Control loop ---

    def _control_loop(self) -> None:
        _FREQ_CHECK_INTERVAL = 2.0  # check frequency every 2s
        _MAX_SLOW_PERIODS = 3  # emergency stop after 3 consecutive slow periods (6s)

        last_check_time = time.time()
        iteration_count = 0
        consecutive_slow = 0

        while not self._stop_event.is_set():
            loop_start = time.time()
            try:
                self._update()
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                logger.error("Emergency stop!")
                self._motor_chain.disable_all()
                self._running = False
                return

            iteration_count += 1
            now = time.time()

            # Frequency monitoring and protection
            elapsed_since_check = now - last_check_time
            if elapsed_since_check >= _FREQ_CHECK_INTERVAL:
                freq = iteration_count / elapsed_since_check
                logger.info(f"Control loop frequency: {freq:.1f} Hz")

                if freq < self._min_freq_hz:
                    consecutive_slow += 1
                    logger.warning(
                        f"Control loop too slow: {freq:.1f} Hz < {self._min_freq_hz} Hz "
                        f"({consecutive_slow}/{_MAX_SLOW_PERIODS})"
                    )
                    if consecutive_slow >= _MAX_SLOW_PERIODS:
                        logger.error(
                            f"Frequency below {self._min_freq_hz} Hz for "
                            f"{consecutive_slow * _FREQ_CHECK_INTERVAL:.0f}s — emergency stop!"
                        )
                        self._motor_chain.disable_all()
                        self._running = False
                        return
                else:
                    consecutive_slow = 0

                last_check_time = now
                iteration_count = 0

            elapsed = time.time() - loop_start
            sleep_time = self._control_period_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Send zero-torque first so motors cache a safe state, then disable immediately
        # from this thread — guarantees disable frames follow the last command in-order
        # on the CAN bus with no race against the main thread.
        _zeros = np.zeros(self._num_joints)
        try:
            self._motor_chain.send_commands(_zeros, _zeros, _zeros, _zeros, _zeros)
        except Exception:
            pass
        self._motor_chain.disable_all()
        self._running = False

    def _update(self) -> None:
        """Single control step: read state -> compute gravity -> send commands."""
        t_now = time.time()

        # 1) Read current joint state
        self._read_state()

        # 1.5) Runtime safety checks. Hard faults raise; the control loop's
        # try/except catches and triggers emergency disable. Warnings are
        # rate-limited and logged in-place.
        self._check_runtime_safety()

        # 2) Sample for teaching recording
        if self._recording and t_now - self._record_last_t >= self._record_period:
            with self._state_lock:
                pos_snap = self._state.pos.copy()
            with self._record_lock:
                if self._recording:
                    self._record_buffer.append((t_now, pos_snap))
            self._record_last_t = t_now

        # 3) Get current command
        with self._command_lock:
            cmd = JointCommand(
                pos=self._command.pos.copy(),
                vel=self._command.vel.copy(),
                acc=self._command.acc.copy(),
                kp=self._command.kp.copy(),
                kd=self._command.kd.copy(),
                torque_ff=self._command.torque_ff.copy(),
            )

        # 4) Full inverse dynamics: gravity + Coriolis + inertia
        # When vel=0 and acc=0 (static hold) this degenerates to pure gravity comp.
        # Hard assertions, not silent clips: if vel/acc ever exceed the
        # feedforward caps here, an upstream validation in command_joint_state
        # or move_joints failed and we'd rather emergency-stop than push a
        # distorted trajectory the caller can't see.
        if np.any(np.abs(cmd.vel) > _MAX_CMD_VEL_RAD_S):
            raise RuntimeError(
                f"Feedforward vel out of bounds in _update: "
                f"{np.round(cmd.vel, 2).tolist()} "
                f"(cap ±{_MAX_CMD_VEL_RAD_S} rad/s)"
            )
        if np.any(np.abs(cmd.acc) > _MAX_CMD_ACC_RAD_S2):
            raise RuntimeError(
                f"Feedforward acc out of bounds in _update: "
                f"{np.round(cmd.acc, 2).tolist()} "
                f"(cap ±{_MAX_CMD_ACC_RAD_S2} rad/s²)"
            )
        with self._state_lock:
            q = self._state.pos.copy()

        tau_id = self._gravity_model.compute_inverse_dynamics(q, cmd.vel, cmd.acc)

        # Safety check
        if np.any(np.abs(tau_id) > self._max_gravity_torque):
            raise RuntimeError(
                f"Inverse dynamics torques too large! tau={np.round(tau_id, 2)} Nm. "
                f"Max allowed: {self._max_gravity_torque} Nm."
            )

        # 5) Combine torques (in URDF frame), then convert to motor frame
        tau_id_scaled = tau_id * self._gravity_torque_scale
        torques_urdf = cmd.torque_ff + tau_id_scaled * self.gravity_comp_factor
        motor_torques = np.clip(torques_urdf * self._joint_sign, -self._torque_clip, self._torque_clip)

        # 6) Send commands to motor chain (convert position/velocity to motor frame)
        self._motor_chain.send_commands(
            pos=cmd.pos * self._joint_sign,
            vel=cmd.vel * self._joint_sign,
            kp=cmd.kp,
            kd=cmd.kd,
            torque=motor_torques,
        )

    def _read_state(self) -> None:
        """Read all motor feedback and update internal state."""
        count = self._motor_chain.drain_and_update(self._bus)
        if count > 0:
            self._last_feedback_t = time.time()
        temp_mos, temp_rotor = self._motor_chain.get_temperatures()
        with self._state_lock:
            self._state.pos = self._motor_chain.get_positions() * self._joint_sign
            self._state.vel = self._motor_chain.get_velocities() * self._joint_sign
            self._state.eff = self._motor_chain.get_efforts() * self._joint_sign
            self._state.error_codes = self._motor_chain.get_error_codes()
            self._state.temp_mos = temp_mos
            self._state.temp_rotor = temp_rotor

    # --- Safety ---

    def _check_runtime_safety(self) -> None:
        """Run all per-cycle safety checks. Raises on hard fault.

        Order is intentional: bus/health checks first so a downed bus is
        caught before motion checks are evaluated against stale data.
        Motion-based checks (joint limits, velocity) are skipped while
        estop is latched because the user is expected to move the arm
        by hand in that state.
        """
        self._check_feedback_stale()
        self._check_motor_errors()
        self._check_motor_temps()
        if self._estop_latch.is_set():
            return
        self._check_runtime_joint_limits()
        self._check_velocity_limits()

    def _check_runtime_joint_limits(self) -> None:
        if self._joint_limits is None:
            return
        pos = self._state.pos
        buf = self._runtime_limit_buffer_rad
        for i, (lo, hi) in enumerate(self._joint_limits):
            if pos[i] < lo - buf or pos[i] > hi + buf:
                raise RuntimeError(
                    f"Runtime joint limit violated: joint{i + 1}={pos[i]:.3f} rad "
                    f"outside [{lo:.3f}, {hi:.3f}] (buffer={buf:.2f})"
                )

    def _check_motor_errors(self) -> None:
        # MotorB codes from MOTOR_B_ERROR_CODES; MotorA codes share the same
        # convention for 0x0 (disabled) and 0x1 (normal). Anything else is a
        # hardware fault — over voltage/current, thermal cutout, comm loss, etc.
        # We can't distinguish "no feedback parsed yet" from a real disabled
        # report, so a disabled code (0x0) is only flagged when feedback is
        # arriving fresh — which is already guaranteed by _check_feedback_stale
        # being called first.
        errs = self._state.error_codes
        bad = (errs != 0x1) & (errs != 0x0)
        if np.any(bad):
            from a1z.motor_drivers.utils import MotorErrorCode
            idx = int(np.argmax(bad))
            code = int(errs[idx])
            raise RuntimeError(
                f"Motor fault on joint{idx + 1}: error_code=0x{code:X} "
                f"({MotorErrorCode.get_error_message(code)})"
            )

    def _check_motor_temps(self) -> None:
        t_mos = self._state.temp_mos
        t_rotor = self._state.temp_rotor
        if np.any(t_mos > self._temp_mos_estop_c):
            idx = int(np.argmax(t_mos))
            raise RuntimeError(
                f"MOS over-temperature on joint{idx + 1}: {t_mos[idx]:.1f}°C "
                f"(limit {self._temp_mos_estop_c:.1f}°C)"
            )
        if np.any(t_rotor > self._temp_rotor_estop_c):
            idx = int(np.argmax(t_rotor))
            raise RuntimeError(
                f"Motor coil over-temperature on joint{idx + 1}: "
                f"{t_rotor[idx]:.1f}°C (limit {self._temp_rotor_estop_c:.1f}°C)"
            )
        now = time.time()
        if now - self._last_temp_warn_t > 1.0:
            hot_mos = t_mos > self._temp_mos_warn_c
            hot_rotor = t_rotor > self._temp_rotor_warn_c
            if np.any(hot_mos) or np.any(hot_rotor):
                parts = []
                for i in np.flatnonzero(hot_mos):
                    parts.append(f"joint{i + 1} MOS={t_mos[i]:.1f}°C")
                for i in np.flatnonzero(hot_rotor):
                    parts.append(f"joint{i + 1} coil={t_rotor[i]:.1f}°C")
                logger.warning("Motor temperature warning: " + "; ".join(parts))
                self._last_temp_warn_t = now

    def _check_velocity_limits(self) -> None:
        v = self._state.vel
        absv = np.abs(v)
        over = absv > self._vel_limit
        if np.any(over):
            idx = int(np.argmax(absv - self._vel_limit))
            raise RuntimeError(
                f"Joint velocity limit exceeded on joint{idx + 1}: "
                f"{v[idx]:.2f} rad/s (limit ±{self._vel_limit[idx]:.2f})"
            )

    def _check_feedback_stale(self) -> None:
        now = time.time()
        age = now - self._last_feedback_t
        if age > self._stale_estop_s:
            raise RuntimeError(
                f"CAN feedback stale for {age * 1000:.0f}ms "
                f"(limit {self._stale_estop_s * 1000:.0f}ms) — bus may be down"
            )
        if age > self._stale_warn_s and now - self._last_stale_warn_t > 1.0:
            logger.warning(f"CAN feedback stale: {age * 1000:.0f}ms")
            self._last_stale_warn_t = now

    def _clip_joint_pos(
        self, pos: np.ndarray, tol_rad: float = 0.05
    ) -> Optional[np.ndarray]:
        """Clip small overshoots, reject genuinely out-of-limit commands.

        Used by the streaming command entry points (teleop / trajectory replay
        loops) where raising on a single noisy frame would tank the entire
        session. The behavior splits the violation by magnitude:

        - Within tol_rad of a limit: clipped silently (feedback noise, teach
          recordings resting at the boundary).
        - Beyond tol_rad: command is *rejected*. Returns None so the caller
          knows to keep the previous valid command instead of driving the arm
          to a geometrically unrelated clipped pose.

        Rejections are logged at error level on every occurrence (no rate
        limiting) because a junk IK solution is a serious upstream bug. Small
        clips are rate-limited at 1 Hz to avoid spamming.
        """
        pos = pos.copy()
        if self._joint_limits is None:
            return pos
        rejected: List[str] = []
        clipped: List[str] = []
        for i, (lo, hi) in enumerate(self._joint_limits):
            if pos[i] < lo - tol_rad or pos[i] > hi + tol_rad:
                rejected.append(
                    f"joint{i + 1}={pos[i]:.3f} outside [{lo:.3f}, {hi:.3f}]"
                )
            elif pos[i] < lo or pos[i] > hi:
                clipped.append(
                    f"joint{i + 1}: {pos[i]:.3f} -> [{lo:.3f}, {hi:.3f}]"
                )
                pos[i] = np.clip(pos[i], lo, hi)
        if rejected:
            logger.error(
                "Streaming command REJECTED (out of limits): "
                + "; ".join(rejected)
            )
            return None
        if clipped:
            now = time.time()
            if now - self._last_clip_warn_t >= 1.0:
                self._last_clip_warn_t = now
                logger.warning(
                    "Joint command minor overshoot, clipped: "
                    + "; ".join(clipped)
                )
        return pos

    def _validate_joint_pos(
        self, pos: np.ndarray, tolerance_rad: float = 0.05
    ) -> np.ndarray:
        """Validate target against joint limits: clip within tolerance, raise beyond.

        Small overshoots (e.g. recorded waypoints resting slightly past a soft
        limit) are silently clipped; anything beyond tolerance_rad indicates an
        upstream error (bad IK solution, wrong units) and raises ValueError
        instead of executing a geometrically unrelated clipped configuration.
        """
        pos = pos.copy()
        if self._joint_limits is None:
            return pos
        violations = []
        for i, (lo, hi) in enumerate(self._joint_limits):
            if pos[i] < lo - tolerance_rad or pos[i] > hi + tolerance_rad:
                violations.append(
                    f"joint{i + 1}: {pos[i]:.3f} rad outside [{lo:.3f}, {hi:.3f}]"
                )
            pos[i] = np.clip(pos[i], lo, hi)
        if violations:
            raise ValueError(
                "Target joint position out of limits, refusing to move: "
                + "; ".join(violations)
            )
        return pos

    def _check_joint_limits(self, pos: np.ndarray, buffer_rad: float = 0.1) -> None:
        if self._joint_limits is None:
            return
        for i, (lo, hi) in enumerate(self._joint_limits):
            if pos[i] < lo - buffer_rad or pos[i] > hi + buffer_rad:
                logger.warning(
                    f"Joint {i} position {pos[i]:.3f} rad is outside limits "
                    f"[{lo:.3f}, {hi:.3f}] (buffer={buffer_rad})"
                )

    def __del__(self):
        if self._running:
            self.stop()
