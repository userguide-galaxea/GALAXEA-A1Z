"""A1Z arm robot implementation with gravity compensation."""

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


@dataclass
class JointState:
    """Joint state for all DOFs."""

    pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    eff: np.ndarray = field(default_factory=lambda: np.zeros(6))


@dataclass
class JointCommand:
    """Joint command for all DOFs."""

    pos: np.ndarray = field(default_factory=lambda: np.zeros(6))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(6))
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
        gravity_torque_sign: Optional[np.ndarray] = None,
        gravity_torque_scale: Optional[np.ndarray] = None,
        max_gravity_torque: Optional[np.ndarray] = None,
        torque_clip: Optional[np.ndarray] = None,
        default_kp: Optional[np.ndarray] = None,
        default_kd: Optional[np.ndarray] = None,
        joint_limits: Optional[List[Tuple[float, float]]] = None,
        control_freq_hz: int = 250,
        min_freq_hz: float = 80.0,
        motor_a_kt: float = 2.8,
    ):
        self._motor_chain = motor_chain
        self._bus = bus
        self._gravity_model = gravity_model
        self._num_joints = num_joints
        self.gravity_comp_factor = gravity_comp_factor
        self.zero_gravity_mode = zero_gravity_mode

        self._gravity_torque_sign = gravity_torque_sign if gravity_torque_sign is not None else np.ones(num_joints)
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
        self._thread = threading.Thread(target=self._control_loop, name="arm_control_loop", daemon=True)
        self._thread.start()
        logger.info(f"Control loop started at {self._control_freq_hz} Hz")

    def stop(self) -> None:
        """Stop the control loop and disable motors with smooth ramp-down."""
        logger.info("Stopping control loop...")
        try:
            if self._running:
                with self._command_lock:
                    hold_pos = self._command.pos.copy()
                t0 = time.time()
                while time.time() - t0 < 0.3:
                    alpha = max(0.0, 1.0 - (time.time() - t0) / 0.3)
                    self.gravity_comp_factor *= alpha
                    with self._command_lock:
                        self._command.pos = hold_pos
                        self._command.vel = np.zeros(self._num_joints)
                        self._command.kp = np.zeros(self._num_joints)
                        self._command.kd = self._default_kd.copy() * 1.5
                        self._command.torque_ff = np.zeros(self._num_joints)
                    time.sleep(0.01)
        except Exception:
            pass

        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._running = False
        self._motor_chain.disable_all()
        logger.info("All motors disabled.")

    def command_joint_pos(self, pos: np.ndarray) -> None:
        """Set target joint angles (rad) with default PD gains."""
        pos = self._clip_joint_pos(pos)
        with self._command_lock:
            self._command.pos = pos.copy()
            self._command.kp = self._default_kp.copy()
            self._command.kd = self._default_kd.copy()
            self._command.torque_ff = np.zeros(self._num_joints)

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Set target joint state.

        Args:
            joint_state: Dict with keys 'pos', 'vel', and optionally 'kp', 'kd'.
        """
        pos = self._clip_joint_pos(joint_state["pos"])
        with self._command_lock:
            self._command.pos = pos.copy()
            self._command.vel = joint_state["vel"].copy()
            self._command.kp = joint_state.get("kp", self._default_kp).copy()
            self._command.kd = joint_state.get("kd", self._default_kd).copy()

    def get_joint_pos(self) -> np.ndarray:
        with self._state_lock:
            return self._state.pos.copy()

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        with self._state_lock:
            return {
                "pos": self._state.pos.copy(),
                "vel": self._state.vel.copy(),
                "eff": self._state.eff.copy(),
            }

    def get_observations(self) -> Dict[str, np.ndarray]:
        return self.get_joint_state()

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

    def move_joints(
        self,
        target_pos: np.ndarray,
        speed: float = 0.5,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
    ) -> None:
        """Smoothly interpolate to target position at the given speed (rad/s).

        Blocks until the target is reached or close enough.
        """
        target_pos = self._clip_joint_pos(target_pos)
        current_pos = self.get_joint_pos()
        kp = kp if kp is not None else self._default_kp
        kd = kd if kd is not None else self._default_kd

        max_dist = np.max(np.abs(target_pos - current_pos))
        if max_dist < 0.001:
            return

        duration = max_dist / speed
        dt = self._control_period_s
        steps = max(1, int(duration / dt))

        for step in range(1, steps + 1):
            alpha = step / steps
            interp_pos = current_pos + alpha * (target_pos - current_pos)
            with self._command_lock:
                self._command.pos = interp_pos
                self._command.kp = kp.copy()
                self._command.kd = kd.copy()
            time.sleep(dt)

        with self._command_lock:
            self._command.pos = target_pos.copy()

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

    def _update(self) -> None:
        """Single control step: read state -> compute gravity -> send commands."""
        # 1) Read current joint state
        self._read_state()

        # 2) Get current command
        with self._command_lock:
            cmd = JointCommand(
                pos=self._command.pos.copy(),
                vel=self._command.vel.copy(),
                kp=self._command.kp.copy(),
                kd=self._command.kd.copy(),
                torque_ff=self._command.torque_ff.copy(),
            )

        # 3) Compute gravity compensation
        with self._state_lock:
            q_raw = self._state.pos.copy()

        q_urdf = q_raw * self._gravity_torque_sign
        tau_g = self._gravity_model.compute_gravity_torque(q_urdf)

        # Safety check
        if np.any(np.abs(tau_g) > self._max_gravity_torque):
            raise RuntimeError(
                f"Gravity torques too large! tau_g={np.round(tau_g, 2)} Nm. "
                f"Max allowed: {self._max_gravity_torque} Nm."
            )

        # 4) Combine torques
        tau_g_signed = tau_g * self._gravity_torque_sign * self._gravity_torque_scale
        motor_torques = cmd.torque_ff + tau_g_signed * self.gravity_comp_factor
        motor_torques = np.clip(motor_torques, -self._torque_clip, self._torque_clip)

        # 5) Send commands to motor chain
        self._motor_chain.send_commands(
            pos=cmd.pos,
            vel=cmd.vel,
            kp=cmd.kp,
            kd=cmd.kd,
            torque=motor_torques,
        )

    def _read_state(self) -> None:
        """Read all motor feedback and update internal state."""
        self._motor_chain.drain_and_update(self._bus)
        with self._state_lock:
            self._state.pos = self._motor_chain.get_positions()
            self._state.vel = self._motor_chain.get_velocities()
            self._state.eff = self._motor_chain.get_efforts()

    # --- Safety ---

    def _clip_joint_pos(self, pos: np.ndarray) -> np.ndarray:
        pos = pos.copy()
        if self._joint_limits is not None:
            for i, (lo, hi) in enumerate(self._joint_limits):
                pos[i] = np.clip(pos[i], lo, hi)
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
