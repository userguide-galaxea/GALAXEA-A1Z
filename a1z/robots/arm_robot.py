"""A1Z arm robot implementation with gravity compensation."""

import enum
import errno
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
from a1z.robots.gripper import Gripper
from a1z.robots.integrator import IntegralConfig, JointErrorIntegrator

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

# Recoverable software/transport failures enter a latched MIT hold after this
# window. They must not call disable_all(): this arm has no brake, so a
# communication/software exception is not evidence that dropping motor torque
# is safer. Only confirmed hardware safety violations use hard disable.
_RECOVERABLE_CONTROL_ERROR_TIMEOUT_S = 0.2


class ControlState(enum.Enum):
    """Lifecycle/safety state of the SDK control loop."""

    STOPPED = "STOPPED"
    RUNNING = "RUNNING"
    SOFT_ESTOP = "SOFT_ESTOP"
    COMMAND_HOLD = "COMMAND_HOLD"
    FAULT_HOLD = "FAULT_HOLD"
    HARD_DISABLED = "HARD_DISABLED"
    HARD_DISABLE_UNCONFIRMED = "HARD_DISABLE_UNCONFIRMED"


class RecoverableControlFault(RuntimeError):
    """A transport, feedback, or computation fault recoverable via MIT hold."""

    def __init__(
        self,
        message: str,
        *,
        fault_code: str = "CONTROL_RECOVERABLE",
        hold_frame_sent: bool = False,
    ):
        super().__init__(message)
        self.fault_code = fault_code
        self.hold_frame_sent = hold_frame_sent


class HardSafetyFault(RuntimeError):
    """A hardware/runtime safety fault that requires motor disable."""


def _is_transient_can_error(exc: BaseException) -> bool:
    """Return True only for retryable SocketCAN TX queue/interruption errors."""
    if not isinstance(exc, can.CanOperationError):
        return False
    error_code = getattr(exc, "error_code", None)
    if error_code in {
        errno.EAGAIN,
        errno.EWOULDBLOCK,
        errno.EINTR,
        errno.ENOBUFS,
    }:
        return True
    message = str(exc).lower()
    return "transmit buffer full" in message or "tx buffer full" in message


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


@dataclass
class _MotorCommandFrame:
    """One fully queued six-axis MIT frame in motor coordinates."""

    pos: np.ndarray
    vel: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    torque: np.ndarray


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
        default_kp: Optional[np.ndarray] = np.array([30.0, 30.0, 30.0, 20.0, 5.0, 5.0]),
        default_kd: Optional[np.ndarray] = np.array([5.0, 1.0, 1.0, 0.5, 0.5, 0.5]),
        joint_limits: Optional[List[Tuple[float, float]]] = None,
        gripper: Optional[Gripper] = None,
        control_freq_hz: int = 250,
        min_freq_hz: float = 80.0,
        motor_a_kt: float = 2.8,
        integral_config: Optional[IntegralConfig] = None,
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
        self.gripper: Optional[Gripper] = gripper
        self._control_freq_hz = control_freq_hz
        self._control_period_s = 1.0 / control_freq_hz
        self._min_freq_hz = min_freq_hz

        # --- error-integral feedforward (SOP-09 S2; None = 逐字节不变的现状) ---
        # Serves only the streaming command_joint_state tracking path; every
        # non-streaming entry / fault resets it (see _reset_integral_state).
        self._integral_config: Optional[IntegralConfig] = integral_config
        self._integrator: Optional[JointErrorIntegrator] = (
            JointErrorIntegrator(integral_config, self._control_period_s)
            if integral_config is not None
            else None
        )
        self._last_tau_i = np.zeros(num_joints)

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
        self._last_sent_command = JointCommand(
            pos=np.zeros(num_joints),
            vel=np.zeros(num_joints),
            acc=np.zeros(num_joints),
            kp=np.zeros(num_joints),
            kd=np.zeros(num_joints),
            torque_ff=np.zeros(num_joints),
        )
        self._has_sent_command = False
        # Serializes one six-axis send with the bookkeeping that defines
        # "last fully queued to SocketCAN". A Leader hold/latch waits on this
        # barrier, so it cannot return while an older command is still in
        # flight.
        self._send_lock = threading.Lock()
        self._last_safe_hold_frame: Optional[_MotorCommandFrame] = None
        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        # Serialize the complete start/stop transaction. Without this lock,
        # two callers could both observe STOPPED, or stop() could clear a new
        # session's shared stop event while the previous thread still exits.
        self._lifecycle_lock = threading.Lock()
        # Sticky for this object. A control thread that ever fails to stop on
        # time must never be followed by a new session on the same locks/event,
        # even if its late cleanup later reports STOPPED.
        self._restart_forbidden = threading.Event()

        self._recording: bool = False
        self._record_buffer: List[Tuple[float, np.ndarray]] = []
        self._record_lock = threading.Lock()
        self._record_last_t: float = 0.0
        self._record_period: float = 1.0 / 50.0

        self._gripper_free_drive: bool = False

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
        # ``_commands_blocked`` is set before a safety command is written.
        # Command entry points check it again while holding _command_lock, so
        # an in-flight upstream command cannot overwrite an estop/fault hold.
        self._commands_blocked = threading.Event()
        # A stopped object must not accept a target before start() has
        # established a fresh current-pose hold.
        self._commands_blocked.set()
        self._control_state_lock = threading.Lock()
        self._control_state = ControlState.STOPPED
        self._fault_code = ""
        self._fault_reason = ""
        self._fault_since_monotonic = 0.0
        self._control_generation = 0
        self._last_successful_update_t = 0.0
        self._last_successful_arm_tx_t = 0.0
        self._control_started_t = 0.0
        # Rate-limits the warning for measured-position outside soft limits.
        # Fires at 1 Hz max so a teleop session parked at a limit doesn't
        # flood the log.
        self._last_limit_warn_t: float = 0.0

    def num_dofs(self) -> int:
        return self._num_joints + (1 if self.gripper is not None else 0)

    def _reset_control_session(self) -> None:
        """Discard command/RX evidence from an earlier start/stop session."""
        with self._send_lock:
            with self._command_lock:
                self._last_sent_command = JointCommand(
                    pos=np.zeros(self._num_joints),
                    vel=np.zeros(self._num_joints),
                    acc=np.zeros(self._num_joints),
                    kp=np.zeros(self._num_joints),
                    kd=np.zeros(self._num_joints),
                    torque_ff=np.zeros(self._num_joints),
                )
                self._has_sent_command = False
                self._last_safe_hold_frame = None
        self._last_feedback_t = 0.0
        self._last_successful_update_t = 0.0
        self._last_successful_arm_tx_t = 0.0
        self._control_started_t = 0.0

        reset_feedback = getattr(
            self._motor_chain,
            "reset_feedback_health",
            None,
        )
        if callable(reset_feedback):
            reset_feedback()

        # Frames already queued before this start cannot prove that the new
        # zero-gain probe reached a motor. Discard them before probing.
        recv = getattr(self._bus, "recv", None)
        if callable(recv):
            for _ in range(1024):
                if recv(timeout=0.0) is None:
                    break

    def _require_complete_startup_feedback(self) -> None:
        """Reject closed-loop startup until this session has all six joints."""
        seen, age = self._joint_feedback_health()
        unhealthy = (~seen) | (age > self._stale_estop_s)
        if not np.any(unhealthy):
            return
        details = "; ".join(
            (
                f"joint{i + 1}=missing"
                if not seen[i]
                else f"joint{i + 1}={age[i] * 1000:.0f}ms"
            )
            for i in np.flatnonzero(unhealthy)
        )
        raise RuntimeError(
            "Startup probe did not obtain fresh feedback from every arm joint "
            f"({details}); refusing to apply position gains"
        )

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
        startup_kp = (
            self._require_gain_vector(initial_kp, "initial_kp", _MAX_CMD_KP)
            if initial_kp is not None
            else None
        )
        startup_kd = (
            self._require_gain_vector(initial_kd, "initial_kd", _MAX_CMD_KD)
            if initial_kd is not None
            else None
        )
        with self._lifecycle_lock:
            with self._control_state_lock:
                old_thread_alive = (
                    self._thread is not None and self._thread.is_alive()
                )
                if (
                    self._running
                    or old_thread_alive
                    or self._restart_forbidden.is_set()
                    or self._control_state != ControlState.STOPPED
                ):
                    raise RuntimeError(
                        "ArmRobot.start() requires STOPPED state with no "
                        "live control thread and no sticky lifecycle fault; "
                        f"current state={self._control_state.value}, "
                        "restart_forbidden="
                        f"{self._restart_forbidden.is_set()}. Recreate the "
                        "robot object after a hard/lifecycle fault"
                    )
                # Reject public commands until startup has admitted complete,
                # current-session feedback and queued its verified pose hold.
                self._commands_blocked.set()

            self._reset_control_session()
            try:
                logger.info("Enabling motors...")
                self._motor_chain.enable_all()
                if self.gripper is not None:
                    self.gripper.enable()
                    self.gripper.home()
                    # Route gripper CAN feedback through drain_and_update so
                    # last_feedback stays fresh.
                    self._motor_chain.register_external_motor(
                        self.gripper._motor
                    )

                # MotorA does not return feedback on enable alone — it needs at
                # least one MIT command first. Send a zero-position-gain probe
                # so motors answer without applying a position correction.
                zeros = np.zeros(self._num_joints)
                probe_kd = np.full(self._num_joints, 0.05)
                self._motor_chain.send_commands(
                    zeros,
                    zeros,
                    zeros,
                    probe_kd,
                    zeros,
                )
                time.sleep(0.05)

                self._read_state()
                self._require_complete_startup_feedback()
                # Fresh feedback alone is not enough to admit closed-loop
                # gains: reject a motor that already reports disabled/faulted,
                # over-temperature, or implausible velocity.
                self._check_motor_errors()
                self._check_motor_temps()
                self._check_velocity_limits()
                logger.info(
                    f"Initial joint positions: "
                    f"{np.round(self._state.pos, 3)} rad"
                )

                if self._joint_limits is not None:
                    self._check_joint_limits(self._state.pos)

                # Only fresh, complete feedback may become a position target.
                with self._command_lock:
                    self._command.pos = self._state.pos.copy()
                    self._command.vel = np.zeros(self._num_joints)
                    self._command.acc = np.zeros(self._num_joints)
                    self._command.torque_ff = np.zeros(self._num_joints)
                    if startup_kp is not None:
                        self._command.kp = startup_kp
                    elif not self.zero_gravity_mode:
                        self._command.kp = self._default_kp.copy()
                    else:
                        self._command.kp = np.zeros(self._num_joints)

                    if startup_kd is not None:
                        self._command.kd = startup_kd
                    elif not self.zero_gravity_mode:
                        self._command.kd = self._default_kd.copy()
                    else:
                        self._command.kd = self._default_kd.copy() * 0.5

                # Queue one verified current-pose hold before the background
                # thread starts. This establishes the only frame stale-feedback
                # recovery is allowed to replay.
                with self._state_lock:
                    startup_q = self._state.pos.copy()
                with self._send_lock:
                    startup_cmd = self._snapshot_command()
                    self._send_command_and_cache_hold_locked(
                        startup_cmd,
                        startup_q,
                    )

                logger.info(
                    f"Initial kp={np.round(self._command.kp, 1)}, "
                    f"kd={np.round(self._command.kd, 2)}"
                )

                # Thread construction and start are part of the same hardware
                # transaction. Commands remain blocked until start() returns.
                self._thread = threading.Thread(
                    target=self._control_loop,
                    name="arm_control_loop",
                    daemon=True,
                )
                self._stop_event.clear()
                self._estop_latch.clear()
                now = time.monotonic()
                self._last_feedback_t = now
                self._control_started_t = now
                self._running = True
                with self._control_state_lock:
                    self._control_state = ControlState.RUNNING
                    self._fault_code = ""
                    self._fault_reason = ""
                    self._control_generation += 1
                    self._fault_since_monotonic = 0.0
                self._thread.start()
                with self._control_state_lock:
                    if (
                        not self._running
                        or not self._thread.is_alive()
                        or self._control_state != ControlState.RUNNING
                    ):
                        raise RuntimeError(
                            "control thread faulted while start() was "
                            "completing"
                        )
                    self._commands_blocked.clear()
            except Exception as exc:
                self._commands_blocked.set()
                self._stop_event.set()
                thread = self._thread
                if thread is not None and thread.is_alive():
                    thread.join(timeout=2.0)
                thread_timed_out = (
                    thread is not None and thread.is_alive()
                )
                if thread_timed_out:
                    self._restart_forbidden.set()
                disable_errors = self._disable_outputs()
                thread_alive = (
                    thread is not None and thread.is_alive()
                )
                if not thread_alive:
                    self._running = False

                with self._control_state_lock:
                    prior_state = self._control_state
                    prior_code = self._fault_code
                    prior_reason = self._fault_reason

                if thread_timed_out:
                    detail = (
                        "; ".join(disable_errors)
                        if disable_errors
                        else (
                            "disable frames queued, but the old control "
                            "thread may still transmit"
                            if thread_alive
                            else "the control thread exited only after the "
                            "startup timeout"
                        )
                    )
                    self._set_fault_state(
                        (
                            ControlState.HARD_DISABLE_UNCONFIRMED
                            if thread_alive or disable_errors
                            else ControlState.HARD_DISABLED
                        ),
                        "STARTUP_CONTROL_THREAD_STUCK",
                        f"startup failed ({type(exc).__name__}: {exc}); "
                        f"{detail}; use physical emergency stop and recreate "
                        "the robot object",
                    )
                    raise RuntimeError(
                        f"{exc}; startup control thread did not stop; "
                        "use physical emergency stop and recreate the robot "
                        "object"
                    ) from exc

                if disable_errors:
                    detail = "; ".join(disable_errors)
                    self._set_fault_state(
                        ControlState.HARD_DISABLE_UNCONFIRMED,
                        "STARTUP_DISABLE_UNCONFIRMED",
                        f"startup failed ({type(exc).__name__}: {exc}); "
                        f"disable transmission unconfirmed ({detail}); "
                        "use physical emergency stop",
                    )
                    raise RuntimeError(
                        f"{exc}; startup disable transmission unconfirmed "
                        f"({detail}); use physical emergency stop"
                    ) from exc

                if prior_state in (
                    ControlState.HARD_DISABLED,
                    ControlState.HARD_DISABLE_UNCONFIRMED,
                ):
                    self._set_fault_state(
                        ControlState.HARD_DISABLED,
                        prior_code,
                        prior_reason.split(
                            "; disable transmission unconfirmed",
                            1,
                        )[0],
                    )
                else:
                    self._set_fault_state(ControlState.STOPPED, "", "")
                raise

            logger.info(
                f"Control loop started at {self._control_freq_hz} Hz"
            )

    def stop(self) -> None:
        """Stop the control loop and disable all motors."""
        with self._lifecycle_lock:
            logger.info("Stopping control loop...")
            self._commands_blocked.set()
            self._stop_event.set()
            thread = self._thread
            if thread is not None and thread.is_alive():
                # Normal path: the control loop disables motors before
                # returning. Never make the object restartable until this
                # thread has actually exited.
                thread.join(timeout=2.0)

            if thread is not None and thread.is_alive():
                # Set this before disable I/O: the old thread may finish and
                # race its cleanup while the main thread is transmitting the
                # fallback disable frames.
                self._restart_forbidden.set()
                disable_errors = self._disable_outputs()
                thread_alive = thread.is_alive()
                detail = (
                    "; ".join(disable_errors)
                    if disable_errors
                    else (
                        "disable frames queued, but the live control thread "
                        "may still transmit"
                        if thread_alive
                        else "the control thread exited only after the stop "
                        "timeout"
                    )
                )
                if not thread_alive:
                    self._running = False
                self._set_fault_state(
                    (
                        ControlState.HARD_DISABLE_UNCONFIRMED
                        if thread_alive or disable_errors
                        else ControlState.HARD_DISABLED
                    ),
                    "CONTROL_THREAD_STOP_TIMEOUT",
                    f"control thread did not stop within 2.0s; {detail}; "
                    "use physical emergency stop and recreate the robot object",
                )
                logger.critical(
                    "Control thread did not stop within 2.0s; use physical "
                    "emergency stop and recreate the robot object"
                )
                return

            self._running = False
            # Safety net: disable again from the main thread after the
            # control-loop shutdown transaction completed.
            disable_errors = self._disable_outputs()
            with self._control_state_lock:
                prior_state = self._control_state
                prior_code = self._fault_code
                prior_reason = self._fault_reason
            if disable_errors:
                detail = "; ".join(disable_errors)
                self._set_fault_state(
                    ControlState.HARD_DISABLE_UNCONFIRMED,
                    "STOP_DISABLE_UNCONFIRMED",
                    f"shutdown disable transmission unconfirmed ({detail}); "
                    "use physical emergency stop",
                )
                logger.critical(
                    f"Shutdown disable transmission unconfirmed: {detail}; "
                    "use physical emergency stop"
                )
            elif (
                prior_state == ControlState.HARD_DISABLED
                or (
                    prior_state == ControlState.HARD_DISABLE_UNCONFIRMED
                    and prior_code != "STOP_DISABLE_UNCONFIRMED"
                )
            ):
                confirmed_reason = prior_reason.split(
                    "; disable transmission unconfirmed",
                    1,
                )[0]
                self._set_fault_state(
                    ControlState.HARD_DISABLED,
                    prior_code,
                    confirmed_reason,
                )
                logger.info(
                    "Hard-fault disable frames retransmitted; hard fault "
                    "remains latched."
                )
            else:
                self._set_fault_state(ControlState.STOPPED, "", "")
                logger.info("All motor disable frames transmitted.")

    def command_gripper(self, value: float) -> None:
        """Set gripper target position.

        Args:
            value: Normalized position in [0.0, 1.0]. 0.0 = closed, 1.0 = fully open.

        Raises:
            RuntimeError: If no gripper was attached at construction.
        """
        if self.gripper is None:
            raise RuntimeError("No gripper attached. Pass gripper= to get_a1z_robot().")
        self._command_gripper_if_allowed(value)

    def _command_gripper_if_allowed(self, value: float) -> bool:
        """Atomically gate an integrated-gripper target against fault entry."""
        with self._control_state_lock:
            if self._commands_blocked.is_set():
                logger.warning("command_gripper rejected: robot is in a fault")
                return False
            self.gripper.command(value)
            return True

    def get_gripper_pos(self) -> Optional[float]:
        """Return current commanded gripper position [0.0=closed, 1.0=fully open], or None if no gripper."""
        if self.gripper is None:
            return None
        return self.gripper.get_pos()

    def set_gripper_free_drive(self, enabled: bool) -> None:
        """Toggle gripper free-drive (zero-torque) mode for hand teaching.

        When enabled, the control loop sends gripper.free_drive_step() instead
        of step(), so the jaw produces no torque and the operator can open/close
        it manually while feedback continues to stream.
        """
        if self.gripper is None:
            return
        self._gripper_free_drive = bool(enabled)

    def _coerce_joint_vector(
        self,
        value: Any,
        name: str,
        *,
        allowed_sizes: Optional[Tuple[int, ...]] = None,
    ) -> Optional[np.ndarray]:
        """Convert an SDK command field to a finite one-dimensional vector."""
        try:
            vector = np.asarray(value, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            logger.error(f"{name} rejected: cannot convert to float array ({exc})")
            return None
        sizes = allowed_sizes or (self._num_joints,)
        if vector.ndim != 1 or vector.size not in sizes:
            expected = " or ".join(str(size) for size in sizes)
            logger.error(
                f"{name} rejected: expected a 1-D vector of size {expected}, "
                f"got shape {vector.shape}"
            )
            return None
        if not np.all(np.isfinite(vector)):
            logger.error(f"{name} rejected: contains NaN/Inf ({vector.tolist()})")
            return None
        return vector

    def _require_gain_vector(
        self,
        value: Any,
        name: str,
        max_value: float,
    ) -> np.ndarray:
        """Validate a public gain override before it reaches shared state."""
        vector = self._coerce_joint_vector(value, name)
        if vector is None:
            raise ValueError(
                f"{name} must be a finite {self._num_joints}-element vector"
            )
        if np.any(vector < 0) or np.any(vector > max_value):
            raise ValueError(
                f"{name} must be within [0, {max_value}], got "
                f"{np.round(vector, 3).tolist()}"
            )
        return vector.copy()

    def _accept_or_reject_stream(
        self, pos: np.ndarray
    ) -> Optional[np.ndarray]:
        """Thin wrapper around :meth:`_clip_joint_pos` for streaming callers.

        Returns the (possibly tolerance-clipped) safe position, or None if
        the frame is out of soft limits beyond the small clip tolerance. In
        the ``None`` case the caller must skip updating the command so the
        previous valid command is held — the arm parks near the limit and
        resumes tracking as soon as the upstream sends an in-range frame
        again (this is the teleop path: master pulled past a soft limit,
        then dragged back).

        Rejection logging is handled inside ``_clip_joint_pos`` itself.
        This wrapper deliberately does NOT engage estop on repeated
        rejects — teleop naturally produces short bursts of out-of-range
        frames, and locking the arm out would strand the user.
        """
        vector = self._coerce_joint_vector(pos, "joint position")
        if vector is None:
            return None
        return self._clip_joint_pos(vector)

    def command_joint_pos(self, pos: np.ndarray) -> None:
        """Set target joint angles (rad) with default PD gains.

        Accepts either a 6-element arm-only array or a 7-element array where
        pos[6] is the gripper normalized position in [0.0, 1.0].
        """
        if self._commands_blocked.is_set():
            logger.warning("command_joint_pos rejected: robot is in estop")
            return
        # Keep the historical 7-element compatibility even when no integrated
        # gripper is attached; in that case the final value is ignored.
        allowed_sizes = (self._num_joints, self._num_joints + 1)
        target = self._coerce_joint_vector(
            pos,
            "command_joint_pos",
            allowed_sizes=allowed_sizes,
        )
        if target is None:
            return
        arm_pos = self._accept_or_reject_stream(target[:self._num_joints])
        if arm_pos is None:
            return
        with self._command_lock:
            if self._commands_blocked.is_set():
                logger.warning("command_joint_pos rejected: robot entered a fault")
                return
            self._command.pos = arm_pos.copy()
            self._command.vel = np.zeros(self._num_joints)
            self._command.acc = np.zeros(self._num_joints)
            self._command.kp = self._default_kp.copy()
            self._command.kd = self._default_kd.copy()
            self._command.torque_ff = np.zeros(self._num_joints)
        self._reset_integral_state()
        if (
            target.size == self._num_joints + 1
            and self.gripper is not None
        ):
            self._command_gripper_if_allowed(
                float(np.clip(target[self._num_joints], 0.0, 1.0))
            )

    def command_joint_state(self, joint_state: Dict[str, np.ndarray]) -> None:
        """Set target joint state.

        Args:
            joint_state: Dict with keys 'pos', 'vel', and optionally 'kp', 'kd',
                'acc', 'torque_ff'.

        Out-of-range pos / vel / kp / kd / acc / torque_ff reject the entire
        frame (previous command is held). Silent clipping of garbage
        feedforward would let a bad upstream silently distort the trajectory or
        PD response, which is harder to diagnose than a refused frame plus an
        error log. Any feedforward field NOT supplied this frame (acc,
        torque_ff) is explicitly zeroed — no stale carry-over from a prior
        command_joint_pos / move_joints (devlog 2026-07-22 Q9 hazard).
        """
        if self._commands_blocked.is_set():
            logger.warning("command_joint_state rejected: robot is in estop")
            return
        if not isinstance(joint_state, dict) or "pos" not in joint_state or "vel" not in joint_state:
            logger.error("command_joint_state rejected: required keys are 'pos' and 'vel'")
            return
        pos = self._accept_or_reject_stream(joint_state["pos"])
        if pos is None:
            return

        vel = self._coerce_joint_vector(joint_state["vel"], "joint velocity")
        if vel is None:
            return
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

        kp = self._coerce_joint_vector(
            joint_state.get("kp", self._default_kp),
            "joint kp",
        )
        if kp is None:
            return
        if np.any(kp < 0) or np.any(kp > _MAX_CMD_KP):
            logger.error(
                f"command_joint_state rejected: kp out of [0, {_MAX_CMD_KP}] "
                f"({np.round(kp, 2).tolist()})"
            )
            return
        kd = self._coerce_joint_vector(
            joint_state.get("kd", self._default_kd),
            "joint kd",
        )
        if kd is None:
            return
        if np.any(kd < 0) or np.any(kd > _MAX_CMD_KD):
            logger.error(
                f"command_joint_state rejected: kd out of [0, {_MAX_CMD_KD}] "
                f"({np.round(kd, 3).tolist()})"
            )
            return

        # Optional feedforward keys — reject-frame on out-of-range, else zero.
        acc_in = joint_state.get("acc")
        if acc_in is None:
            acc = np.zeros(self._num_joints)
        else:
            acc = np.asarray(acc_in, dtype=np.float64)
            if np.any(np.abs(acc) > _MAX_CMD_ACC_RAD_S2):
                offenders = "; ".join(
                    f"joint{i + 1}={acc[i]:.2f}"
                    for i in np.flatnonzero(np.abs(acc) > _MAX_CMD_ACC_RAD_S2)
                )
                logger.error(
                    f"command_joint_state rejected: acc exceeds "
                    f"{_MAX_CMD_ACC_RAD_S2} rad/s² ({offenders})"
                )
                return

        tff_in = joint_state.get("torque_ff")
        if tff_in is None:
            torque_ff = np.zeros(self._num_joints)
        else:
            torque_ff = np.asarray(tff_in, dtype=np.float64)
            if np.any(np.abs(torque_ff) > self._torque_clip):
                offenders = "; ".join(
                    f"joint{i + 1}={torque_ff[i]:.2f}"
                    for i in np.flatnonzero(np.abs(torque_ff) > self._torque_clip)
                )
                logger.error(
                    f"command_joint_state rejected: torque_ff exceeds per-joint "
                    f"torque_clip ({offenders})"
                )
                return

        with self._command_lock:
            if self._commands_blocked.is_set():
                logger.warning("command_joint_state rejected: robot entered a fault")
                return
            self._command.pos = pos.copy()
            self._command.vel = vel.copy()
            self._command.acc = acc.copy()
            self._command.kp = kp.copy()
            self._command.kd = kd.copy()
            self._command.torque_ff = torque_ff.copy()

    def get_joint_pos(self) -> np.ndarray:
        with self._state_lock:
            arm_pos = self._state.pos.copy()
        if self.gripper is not None:
            return np.append(arm_pos, self.gripper.get_feedback_norm())
        return arm_pos

    def get_joint_state(self) -> Dict[str, np.ndarray]:
        with self._state_lock:
            return {
                "pos": self._state.pos.copy(),
                "vel": self._state.vel.copy(),
                "eff": self._state.eff.copy(),
                "tau_i": self._last_tau_i.copy(),
                "error_codes": self._state.error_codes.copy(),
                "temp_mos": self._state.temp_mos.copy(),
                "temp_rotor": self._state.temp_rotor.copy(),
            }

    def get_command_state(self) -> Dict[str, np.ndarray]:
        """Return a defensive snapshot of the command accepted by the SDK."""
        with self._command_lock:
            return {
                "pos": self._command.pos.copy(),
                "vel": self._command.vel.copy(),
                "acc": self._command.acc.copy(),
                "kp": self._command.kp.copy(),
                "kd": self._command.kd.copy(),
                "torque_ff": self._command.torque_ff.copy(),
            }

    def get_observations(self) -> Dict[str, np.ndarray]:
        state = self.get_joint_state()
        obs: Dict[str, np.ndarray] = {
            "joint_pos": state["pos"],
            "joint_vel": state["vel"],
            "joint_eff": state["eff"],
            "joint_error_codes": state["error_codes"],
            "joint_temp_mos": state["temp_mos"],
            "joint_temp_rotor": state["temp_rotor"],
        }
        if self.gripper is not None:
            obs["gripper_pos"] = np.array([self.gripper.get_feedback_norm()])
        return obs

    def get_robot_info(self) -> Dict[str, Any]:
        return {
            "num_joints": self._num_joints,
            "default_kp": self._default_kp.copy(),
            "default_kd": self._default_kd.copy(),
            "joint_limits": self._joint_limits,
            "gravity_comp_factor": self.gravity_comp_factor,
            "control_freq_hz": self._control_freq_hz,
            "integral": (
                self._integral_config.as_info()
                if self._integral_config is not None
                else {
                    "level": "K0",
                    "ki": np.zeros(self._num_joints).tolist(),
                    "tau_i_max": np.zeros(self._num_joints).tolist(),
                    "t_leak_s": None,
                    "e_db_deg": np.zeros(self._num_joints).tolist(),
                    "qd_freeze": None,
                    "enable_mask": [False] * self._num_joints,
                }
            ),
        }

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_estopped(self) -> bool:
        return self._commands_blocked.is_set()

    def _joint_feedback_health(
        self,
        now: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return per-joint ``(seen, age_s)`` with a legacy-chain fallback."""
        if now is None:
            now = time.monotonic()
        get_health = getattr(self._motor_chain, "get_feedback_health", None)
        if callable(get_health):
            seen, age = get_health(now)
            return (
                np.asarray(seen, dtype=bool),
                np.asarray(age, dtype=np.float64),
            )
        # Compatibility fallback for custom MotorChain implementations. The
        # built-in MixedMotorChain always supplies true per-joint health.
        seen = np.full(
            self._num_joints,
            self._last_feedback_t > 0,
            dtype=bool,
        )
        age = np.full(
            self._num_joints,
            max(0.0, now - self._last_feedback_t),
            dtype=np.float64,
        )
        return seen, age

    def get_fault_status(self) -> Dict[str, Any]:
        """Return a thread-safe snapshot of the current SDK safety state."""
        feedback_seen, feedback_age = self._joint_feedback_health()
        with self._control_state_lock:
            since = self._fault_since_monotonic
            return {
                "state": self._control_state.value,
                "code": self._fault_code,
                "reason": self._fault_reason,
                "age_s": (
                    max(0.0, time.monotonic() - since)
                    if since > 0
                    else 0.0
                ),
                "last_successful_update_age_s": (
                    max(0.0, time.monotonic() - self._last_successful_update_t)
                    if self._last_successful_update_t > 0
                    else None
                ),
                "last_successful_arm_tx_age_s": (
                    max(0.0, time.monotonic() - self._last_successful_arm_tx_t)
                    if self._last_successful_arm_tx_t > 0
                    else None
                ),
                "disable_transmission_confirmed": (
                    True
                    if self._control_state == ControlState.HARD_DISABLED
                    else (
                        False
                        if self._control_state
                        == ControlState.HARD_DISABLE_UNCONFIRMED
                        else None
                    )
                ),
                "restart_allowed": not self._restart_forbidden.is_set(),
                "feedback_seen": feedback_seen.tolist(),
                "feedback_age_s": feedback_age.tolist(),
            }

    def hold_last_command(self) -> bool:
        """Keep the last fully queued position target and clear motion FF.

        This is the short input-loss behavior used by teleoperation.  It does
        not latch a fault. Any newer command that has only been accepted in
        memory is replaced by the last target/PD gains successfully sent.
        """
        with self._send_lock:
            with self._command_lock:
                if self._commands_blocked.is_set():
                    return False
                self._write_last_sent_hold_locked()
        return True

    def _write_last_sent_hold_locked(self) -> None:
        """Write a zero-feedforward hold; caller owns ``_command_lock``."""
        if self._has_sent_command:
            self._command.pos = self._last_sent_command.pos.copy()
            self._command.kp = self._last_sent_command.kp.copy()
            self._command.kd = self._last_sent_command.kd.copy()
        self._command.vel = np.zeros(self._num_joints)
        self._command.acc = np.zeros(self._num_joints)
        self._command.torque_ff = np.zeros(self._num_joints)

    def latch_last_command(
        self,
        reason: str = "upstream command source unavailable",
        fault_code: str = "UPSTREAM_COMMUNICATION",
    ) -> bool:
        """Latch the last fully queued target while control stays active.

        This is intended for a persistent Leader/input loss. Unlike
        :meth:`estop`, it restores the last transmitted position and PD gains;
        velocity, acceleration and torque feedforward are cleared. New
        commands remain blocked until :meth:`release` succeeds.
        """
        with self._control_state_lock:
            if self._control_state in (
                ControlState.HARD_DISABLED,
                ControlState.HARD_DISABLE_UNCONFIRMED,
                ControlState.STOPPED,
                ControlState.FAULT_HOLD,
            ):
                return False
            if self._control_state == ControlState.COMMAND_HOLD:
                return self._fault_code == fault_code
            if self._control_state == ControlState.SOFT_ESTOP:
                return False
            self._commands_blocked.set()
            with self._send_lock:
                with self._command_lock:
                    self._write_last_sent_hold_locked()
            self._control_state = ControlState.COMMAND_HOLD
            self._fault_code = fault_code
            self._fault_reason = reason
            self._control_generation += 1
            self._fault_since_monotonic = time.monotonic()
            self._estop_latch.set()
        logger.warning(
            f"[ArmRobot] last-command hold latched ({fault_code}) — "
            "new commands suspended, MIT position hold remains active"
        )
        return True

    def _set_fault_state(
        self,
        state: ControlState,
        code: str,
        reason: str,
    ) -> None:
        with self._control_state_lock:
            if state in (
                ControlState.HARD_DISABLED,
                ControlState.HARD_DISABLE_UNCONFIRMED,
            ):
                self._restart_forbidden.set()
            self._control_state = state
            self._fault_code = code
            self._fault_reason = reason
            self._control_generation += 1
            self._fault_since_monotonic = (
                0.0
                if state in (ControlState.RUNNING, ControlState.STOPPED)
                else time.monotonic()
            )

    def _write_safe_command(self, *, position_hold: bool) -> None:
        with self._state_lock:
            cur_pos = self._state.pos.copy()
        with self._command_lock:
            self._command.pos = cur_pos
            self._command.vel = np.zeros(self._num_joints)
            self._command.acc = np.zeros(self._num_joints)
            self._command.kp = (
                self._default_kp.copy()
                if position_hold
                else np.zeros(self._num_joints)
            )
            self._command.kd = (
                self._default_kd.copy()
                if position_hold
                else self._default_kd.copy() * 0.5
            )
            self._command.torque_ff = np.zeros(self._num_joints)

    def estop(
        self,
        reason: str = "manual soft estop",
        fault_code: str = "MANUAL_ESTOP",
    ) -> bool:
        """Engage soft emergency stop.

        Atomically zeros position gain, halves the velocity damping, clears
        any feedforward torque, and pins the command position to the current
        measured position. Gravity compensation keeps running so the arm
        does not collapse under load.

        Subsequent command_joint_pos / command_joint_state / command_gripper
        calls are silently rejected until :meth:`release` is called. An
        in-flight :meth:`move_joints` exits its interpolation loop on the
        next step and returns early.
        """
        with self._control_state_lock:
            if self._control_state in (
                ControlState.HARD_DISABLED,
                ControlState.HARD_DISABLE_UNCONFIRMED,
                ControlState.STOPPED,
                ControlState.FAULT_HOLD,
            ):
                return False
            if self._control_state == ControlState.SOFT_ESTOP:
                return self._fault_code == fault_code
            self._commands_blocked.set()
            with self._send_lock:
                self._write_safe_command(position_hold=False)
                self._control_state = ControlState.SOFT_ESTOP
                self._fault_code = fault_code
                self._fault_reason = reason
                self._control_generation += 1
                self._fault_since_monotonic = time.monotonic()
                self._estop_latch.set()
        self._reset_integral_state()
        logger.warning(
            f"[ArmRobot] soft ESTOP engaged ({fault_code}) — "
            "commands suspended, gravity compensation remains active"
        )
        return True

    def hold_position(
        self,
        reason: str = "recoverable control fault",
        fault_code: str = "CONTROL_FAULT",
    ) -> bool:
        """Latch a strict MIT position hold while the control loop stays alive."""
        with self._control_state_lock:
            if self._control_state in (
                ControlState.HARD_DISABLED,
                ControlState.HARD_DISABLE_UNCONFIRMED,
                ControlState.STOPPED,
            ):
                return False
            already_holding = (
                self._control_state == ControlState.FAULT_HOLD
                and self._fault_code == fault_code
            )
            if not already_holding:
                self._commands_blocked.set()
                with self._send_lock:
                    self._write_safe_command(position_hold=True)
                    self._control_state = ControlState.FAULT_HOLD
                    self._fault_code = fault_code
                    self._fault_reason = reason
                    self._control_generation += 1
                    self._fault_since_monotonic = time.monotonic()
                    self._estop_latch.set()
        if not already_holding:
            self._reset_integral_state()
            logger.error(
                f"[ArmRobot] fault hold engaged ({fault_code}): {reason}"
            )
        return True

    def release(self, expected_fault_code: Optional[str] = None) -> bool:
        """Release a soft estop/fault hold at the current measured pose.

        Hard-disabled or feedback-stale robots cannot be released through this
        method.  ``expected_fault_code`` lets an integration layer avoid
        accidentally releasing an SDK-internal fault while recovering an
        unrelated upstream fault.
        """
        with self._control_state_lock:
            state = self._control_state
            fault_code = self._fault_code
            if state == ControlState.RUNNING:
                return True
            if state not in (
                ControlState.SOFT_ESTOP,
                ControlState.COMMAND_HOLD,
                ControlState.FAULT_HOLD,
            ):
                logger.error(f"[ArmRobot] release rejected from state {state.value}")
                return False
            if expected_fault_code is not None and fault_code != expected_fault_code:
                logger.error(
                    f"[ArmRobot] release rejected: expected fault "
                    f"{expected_fault_code}, active fault is {fault_code}"
                )
                return False
            if not self._running:
                logger.error("[ArmRobot] release rejected: control loop is not running")
                return False
            now = time.monotonic()
            if (
                self._last_successful_update_t <= 0
                or now - self._last_successful_update_t
                > self._stale_estop_s
                or (
                    self._fault_since_monotonic > 0
                    and self._last_successful_update_t
                    < self._fault_since_monotonic
                )
            ):
                age_ms = (
                    float("inf")
                    if self._last_successful_update_t <= 0
                    else (now - self._last_successful_update_t) * 1000.0
                )
                logger.error(
                    "[ArmRobot] release rejected: no recent complete, "
                    "fault-free control update after this fault "
                    f"({age_ms:.0f}ms)"
                )
                return False
            feedback_seen, feedback_age = self._joint_feedback_health()
            unhealthy = (~feedback_seen) | (feedback_age > self._stale_estop_s)
            if np.any(unhealthy):
                details = "; ".join(
                    (
                        f"joint{i + 1}=never-seen"
                        if not feedback_seen[i]
                        else f"joint{i + 1}={feedback_age[i] * 1000:.0f}ms"
                    )
                    for i in np.flatnonzero(unhealthy)
                )
                logger.error(
                    f"[ArmRobot] release rejected: incomplete/stale feedback "
                    f"({details})"
                )
                return False
            with self._send_lock:
                with self._state_lock:
                    cur_pos = self._state.pos.copy()
                with self._command_lock:
                    self._command.pos = cur_pos
                    self._command.vel = np.zeros(self._num_joints)
                    self._command.acc = np.zeros(self._num_joints)
                    self._command.kp = self._default_kp.copy()
                    self._command.kd = self._default_kd.copy()
                    self._command.torque_ff = np.zeros(self._num_joints)
                self._control_state = ControlState.RUNNING
                self._fault_code = ""
                self._fault_reason = ""
                self._control_generation += 1
                self._fault_since_monotonic = 0.0
                self._estop_latch.clear()
                self._commands_blocked.clear()
        self._reset_integral_state()
        logger.info("[ArmRobot] ESTOP released")
        return True

    def _disable_outputs(self) -> List[str]:
        """Best-effort disable and return outputs whose frame was not sent."""
        errors = []
        try:
            self._motor_chain.disable_all()
        except Exception as exc:
            errors.append(f"arm: {exc}")
        if self.gripper is not None:
            try:
                self.gripper.disable()
            except Exception as exc:
                errors.append(f"gripper: {exc}")
        return errors

    def _hard_disable(self, fault_code: str, reason: str) -> None:
        """Disable motors for a confirmed hard fault, exactly once per state."""
        with self._control_state_lock:
            if self._control_state in (
                ControlState.HARD_DISABLED,
                ControlState.HARD_DISABLE_UNCONFIRMED,
            ):
                return
            self._commands_blocked.set()
            self._restart_forbidden.set()
            self._control_state = ControlState.HARD_DISABLED
            self._fault_code = fault_code
            self._fault_reason = reason
            self._control_generation += 1
            self._fault_since_monotonic = time.monotonic()
        logger.error(f"[ArmRobot] hard fault ({fault_code}): {reason}")
        disable_errors = self._disable_outputs()
        if disable_errors:
            detail = "; ".join(disable_errors)
            with self._control_state_lock:
                self._control_state = ControlState.HARD_DISABLE_UNCONFIRMED
                self._control_generation += 1
                self._fault_reason = (
                    f"{reason}; disable transmission unconfirmed ({detail}); "
                    "use physical emergency stop"
                )
            logger.critical(
                "[ArmRobot] hard disable transmission unconfirmed: "
                f"{detail}; use physical emergency stop"
            )
        self._running = False

    def move_joints(
        self,
        target_pos: np.ndarray,
        speed: float = 0.5,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        max_jump_rad: Optional[float] = None,
    ) -> None:
        """Smoothly interpolate to target position at the given speed (rad/s).

        Accepts either a 6-element arm-only array or a 7-element array where
        target_pos[6] is the gripper normalized position in [0.0, 1.0].
        Gripper command is applied immediately; arm interpolation runs at speed.
        Blocks until the arm target is reached or close enough.

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
        if self._commands_blocked.is_set():
            logger.warning("move_joints rejected: robot is in estop")
            return
        # Minimum-jerk peak velocity is 1.875 × average. Reject upfront so we
        # never generate a feedforward that the _update assert would refuse.
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError(f"move_joints speed must be > 0, got {speed}")
        if speed * 1.875 > _MAX_CMD_VEL_RAD_S:
            raise ValueError(
                f"move_joints speed {speed:.2f} rad/s exceeds feedforward "
                f"cap: peak vel {speed * 1.875:.2f} > {_MAX_CMD_VEL_RAD_S} rad/s"
            )
        allowed_sizes = (self._num_joints, self._num_joints + 1)
        target_vector = self._coerce_joint_vector(
            target_pos,
            "move_joints target",
            allowed_sizes=allowed_sizes,
        )
        if target_vector is None:
            raise ValueError("move_joints target must be a finite joint vector")
        gripper_target: Optional[float] = None
        if (
            target_vector.size == self._num_joints + 1
            and self.gripper is not None
        ):
            gripper_target = float(
                np.clip(target_vector[self._num_joints], 0.0, 1.0)
            )
        target_pos = target_vector[:self._num_joints]

        target_pos = self._validate_joint_pos(target_pos)
        kp_cmd = self._require_gain_vector(
            self._default_kp if kp is None else kp,
            "move_joints kp",
            _MAX_CMD_KP,
        )
        kd_cmd = self._require_gain_vector(
            self._default_kd if kd is None else kd,
            "move_joints kd",
            _MAX_CMD_KD,
        )
        # Safety check uses the *measured* position — this is what max_jump_rad
        # actually protects (e.g. catching elbow-flipped IK against reality).
        measured_pos = self.get_joint_pos()[:self._num_joints]

        if max_jump_rad is not None:
            if not np.isfinite(max_jump_rad) or max_jump_rad < 0:
                raise ValueError(
                    f"max_jump_rad must be finite and >= 0, got {max_jump_rad}"
                )
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
            if self._commands_blocked.is_set():
                logger.warning("move_joints rejected while entering a fault")
                return
            current_pos = self._command.pos.copy()

        if gripper_target is not None:
            self._command_gripper_if_allowed(gripper_target)

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
            if self._commands_blocked.is_set():
                logger.warning("move_joints aborted mid-trajectory: estop engaged")
                return
            t = step / steps
            # minimum-jerk profile: pos and vel are zero at t=0 and t=1
            alpha = 10*t**3 - 15*t**4 + 6*t**5
            alpha_dot = (30*t**2 - 60*t**3 + 30*t**4) / duration
            alpha_ddot = (60*t - 180*t**2 + 120*t**3) / duration**2
            with self._command_lock:
                if self._commands_blocked.is_set():
                    logger.warning("move_joints aborted while entering a fault")
                    return
                self._command.pos = current_pos + alpha * delta
                self._command.vel = alpha_dot * delta
                self._command.acc = alpha_ddot * delta
                self._command.kp = kp_cmd.copy()
                self._command.kd = kd_cmd.copy()
            time.sleep(dt)

        with self._command_lock:
            if self._commands_blocked.is_set():
                return
            self._command.pos = target_pos.copy()
            self._command.vel = np.zeros(self._num_joints)
            self._command.acc = np.zeros(self._num_joints)
        self._reset_integral_state()

    def set_gravity_mode(self, enabled: bool) -> None:
        """Switch between zero-gravity (floating) and position-hold mode.

        In zero-gravity mode kp=0 so the arm follows gravity compensation only.
        In position-hold mode the default PD gains are restored.
        """
        with self._command_lock:
            if self._commands_blocked.is_set():
                logger.warning("set_gravity_mode rejected: robot is in a fault")
                return
            if enabled:
                self._command.kp = np.zeros(self._num_joints)
                self._command.kd = self._default_kd.copy() * 0.5
            else:
                self._command.kp = self._default_kp.copy()
                self._command.kd = self._default_kd.copy()
        self.zero_gravity_mode = enabled
        self._reset_integral_state()

    def _reset_integral_state(self) -> None:
        """Zero the error integrator (None-safe). Called by every non-streaming
        command entry and fault path so integral only serves the streaming
        command_joint_state tracker (SOP-09 W4)."""
        if self._integrator is not None:
            self._integrator.reset()
        with self._state_lock:
            self._last_tau_i = np.zeros(self._num_joints)

    def set_integral_config(self, cfg: Optional[IntegralConfig]) -> None:
        """Atomically swap the integral config (or disable with None) and reset.

        Rebuilds the integrator under the command lock so a mid-run 档位切换
        never mixes old accumulator state with new gains.
        """
        with self._command_lock:
            self._integral_config = cfg
            self._integrator = (
                JointErrorIntegrator(cfg, self._control_period_s)
                if cfg is not None else None
            )
        self._reset_integral_state()

    def reset_integral(self) -> None:
        """Public: zero the integral accumulator without changing config."""
        self._reset_integral_state()

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

        Switches to position-hold mode for the duration of playback.  After
        the last waypoint the arm stays at that position under PD control.

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

    def _handle_control_exception(
        self,
        exc: BaseException,
        recoverable_since: Optional[float],
    ) -> Tuple[bool, Optional[float]]:
        """Apply fault policy and return ``(continue_loop, error_since)``."""
        now = time.monotonic()

        if isinstance(exc, HardSafetyFault):
            self._hard_disable("SAFETY_HARD_FAULT", str(exc))
            return False, recoverable_since

        if isinstance(exc, can.CanError):
            code = (
                "CAN_TX_TRANSIENT"
                if _is_transient_can_error(exc)
                else "CAN_TRANSPORT"
            )
        elif isinstance(exc, RecoverableControlFault):
            code = exc.fault_code
        else:
            code = "INTERNAL_ERROR"

        first_error = recoverable_since is None
        if first_error:
            recoverable_since = now
            logger.warning(
                f"[ArmRobot] recoverable control error ({code}); "
                f"holding last command while retrying: {type(exc).__name__}: {exc}"
            )
        self.hold_last_command()
        if not getattr(exc, "hold_frame_sent", False):
            try:
                self._resend_last_safe_hold()
            except Exception as resend_exc:
                if first_error:
                    logger.error(
                        "[ArmRobot] cached MIT hold resend failed: "
                        f"{type(resend_exc).__name__}: {resend_exc}"
                    )
        if now - recoverable_since >= _RECOVERABLE_CONTROL_ERROR_TIMEOUT_S:
            self.hold_position(
                reason=(
                    f"{type(exc).__name__}: {exc} "
                    f"(no successful control update for "
                    f"{now - recoverable_since:.3f}s)"
                ),
                fault_code=f"{code}_PERSISTENT",
            )
        return True, recoverable_since

    def _control_loop(self) -> None:
        _FREQ_CHECK_INTERVAL = 2.0  # check frequency every 2s
        _MAX_SLOW_PERIODS = 3  # enter position hold after 3 slow periods (6s)

        last_check_time = time.monotonic()
        iteration_count = 0
        consecutive_slow = 0
        recoverable_since: Optional[float] = None

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                self._update()
            except Exception as exc:
                keep_running, recoverable_since = self._handle_control_exception(
                    exc,
                    recoverable_since,
                )
                if not keep_running:
                    return
            else:
                recoverable_since = None

            iteration_count += 1
            now = time.monotonic()

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
                        reason = (
                            f"Frequency below {self._min_freq_hz} Hz for "
                            f"{consecutive_slow * _FREQ_CHECK_INTERVAL:.0f}s"
                        )
                        self.hold_position(
                            reason=reason,
                            fault_code="CONTROL_FREQUENCY_LOW",
                        )
                        consecutive_slow = 0
                else:
                    consecutive_slow = 0

                last_check_time = now
                iteration_count = 0

            elapsed = time.monotonic() - loop_start
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
        disable_errors = self._disable_outputs()
        self._commands_blocked.set()
        final_state = self._finalize_control_thread_shutdown(disable_errors)
        if disable_errors:
            detail = "; ".join(disable_errors)
            logger.critical(
                f"Control-loop shutdown disable transmission unconfirmed: "
                f"{detail}; use physical emergency stop"
            )
        elif final_state == ControlState.HARD_DISABLED:
            logger.error(
                "Control thread exited after a non-restartable lifecycle or "
                "hard fault; disable transmission completed"
            )

    def _finalize_control_thread_shutdown(
        self,
        disable_errors: List[str],
    ) -> ControlState:
        """Atomically finalize a control thread without clearing sticky faults."""
        with self._control_state_lock:
            self._running = False
            prior_state = self._control_state
            prior_code = self._fault_code
            prior_reason = self._fault_reason
            sticky_hard_fault = (
                prior_state
                in (
                    ControlState.HARD_DISABLED,
                    ControlState.HARD_DISABLE_UNCONFIRMED,
                )
                or self._restart_forbidden.is_set()
            )
            if disable_errors:
                detail = "; ".join(disable_errors)
                self._restart_forbidden.set()
                self._control_state = ControlState.HARD_DISABLE_UNCONFIRMED
                self._fault_code = (
                    prior_code
                    if sticky_hard_fault and prior_code
                    else "STOP_DISABLE_UNCONFIRMED"
                )
                prefix = (
                    prior_reason
                    if sticky_hard_fault and prior_reason
                    else "control-loop shutdown"
                )
                self._fault_reason = (
                    f"{prefix}; final disable transmission unconfirmed "
                    f"({detail}); use physical emergency stop"
                )
                self._fault_since_monotonic = (
                    self._fault_since_monotonic or time.monotonic()
                )
            elif sticky_hard_fault:
                # A thread that previously timed out is no longer live. A
                # successful final disable upgrades transmission confirmation,
                # but never makes this object restartable.
                self._restart_forbidden.set()
                self._control_state = ControlState.HARD_DISABLED
                self._fault_code = (
                    prior_code or "CONTROL_THREAD_LIFECYCLE_FAULT"
                )
                self._fault_reason = (
                    prior_reason.split(
                        "; final disable transmission unconfirmed",
                        1,
                    )[0]
                    if prior_reason
                    else "control thread exited after a lifecycle timeout; "
                    "recreate the robot object"
                )
                self._fault_since_monotonic = (
                    self._fault_since_monotonic or time.monotonic()
                )
            else:
                self._control_state = ControlState.STOPPED
                self._fault_code = ""
                self._fault_reason = ""
                self._fault_since_monotonic = 0.0
            self._control_generation += 1
            return self._control_state

    def _snapshot_command(self) -> JointCommand:
        with self._command_lock:
            return JointCommand(
                pos=self._command.pos.copy(),
                vel=self._command.vel.copy(),
                acc=self._command.acc.copy(),
                kp=self._command.kp.copy(),
                kd=self._command.kd.copy(),
                torque_ff=self._command.torque_ff.copy(),
            )

    def _static_hold_motor_torque(self, q: np.ndarray) -> np.ndarray:
        """Compute clipped gravity-only torque in motor coordinates."""
        zeros = np.zeros(self._num_joints)
        try:
            compute_gravity = getattr(
                self._gravity_model,
                "compute_gravity_torque",
                None,
            )
            if callable(compute_gravity):
                tau_hold = compute_gravity(q)
            else:
                tau_hold = self._gravity_model.compute_inverse_dynamics(
                    q,
                    zeros,
                    zeros,
                )
        except Exception as exc:
            raise RecoverableControlFault(
                f"Static hold dynamics computation failed: {exc}"
            ) from exc
        tau_hold = np.asarray(tau_hold, dtype=np.float64)
        if (
            tau_hold.shape != (self._num_joints,)
            or not np.all(np.isfinite(tau_hold))
        ):
            raise RecoverableControlFault(
                "Static hold dynamics returned an invalid vector: "
                f"shape={tau_hold.shape}, values={tau_hold.tolist()}"
            )
        if np.any(np.abs(tau_hold) > self._max_gravity_torque):
            raise RecoverableControlFault(
                f"Static hold torques too large: "
                f"{np.round(tau_hold, 2).tolist()} Nm"
            )
        torques_urdf = (
            tau_hold
            * self._gravity_torque_scale
            * self.gravity_comp_factor
        )
        return np.clip(
            torques_urdf * self._joint_sign,
            -self._torque_clip,
            self._torque_clip,
        )

    def _send_command_and_cache_hold_locked(
        self,
        cmd: JointCommand,
        q: np.ndarray,
    ) -> None:
        """Queue one command and its gravity-only fallback; owns send barrier."""
        if not all(
            np.all(np.isfinite(vector))
            for vector in (
                cmd.pos,
                cmd.vel,
                cmd.acc,
                cmd.kp,
                cmd.kd,
                cmd.torque_ff,
            )
        ):
            raise RecoverableControlFault(
                "Non-finite value reached the internal joint command"
            )
        if np.any(np.abs(cmd.vel) > _MAX_CMD_VEL_RAD_S):
            raise RecoverableControlFault(
                f"Feedforward vel out of bounds in _update: "
                f"{np.round(cmd.vel, 2).tolist()} "
                f"(cap ±{_MAX_CMD_VEL_RAD_S} rad/s)"
            )
        if np.any(np.abs(cmd.acc) > _MAX_CMD_ACC_RAD_S2):
            raise RecoverableControlFault(
                f"Feedforward acc out of bounds in _update: "
                f"{np.round(cmd.acc, 2).tolist()} "
                f"(cap ±{_MAX_CMD_ACC_RAD_S2} rad/s²)"
            )
        if not np.all(np.isfinite(q)):
            raise HardSafetyFault(
                f"Non-finite motor feedback position: {q.tolist()}"
            )

        # Error-integral feedforward (SOP-09 W2). e = q_des − q_meas, both
        # in URDF frame. Only accumulates on the streaming tracker and never
        # while estopped; steps at the 250 Hz control rate. tau_i (already
        # clamped to ±tau_i_max inside the integrator) joins the torque sum
        # below — ahead of × joint_sign and torque_clip, so it is frame-correct
        # and the global clip stays the mechanical backstop for a runaway.
        integrator = self._integrator
        if integrator is not None and not self._estop_latch.is_set():
            tau_i = integrator.step(cmd.pos - q, cmd.vel)
        else:
            tau_i = np.zeros(self._num_joints)
        with self._state_lock:
            self._last_tau_i = tau_i.copy()

        try:
            tau_id = self._gravity_model.compute_inverse_dynamics(
                q,
                cmd.vel,
                cmd.acc,
            )
        except Exception as exc:
            raise RecoverableControlFault(
                f"Inverse dynamics computation failed: {exc}"
            ) from exc
        tau_id = np.asarray(tau_id, dtype=np.float64)
        if (
            tau_id.shape != (self._num_joints,)
            or not np.all(np.isfinite(tau_id))
        ):
            raise RecoverableControlFault(
                "Inverse dynamics returned an invalid vector: "
                f"shape={tau_id.shape}, values={tau_id.tolist()}"
            )
        if np.any(np.abs(tau_id) > self._max_gravity_torque):
            raise RecoverableControlFault(
                f"Inverse dynamics torques too large! "
                f"tau={np.round(tau_id, 2).tolist()} Nm. "
                f"Max allowed: "
                f"{np.asarray(self._max_gravity_torque).tolist()} Nm."
            )

        tau_id_scaled = tau_id * self._gravity_torque_scale
        torques_urdf = (
            cmd.torque_ff
            + tau_i
            + tau_id_scaled * self.gravity_comp_factor
        )
        motor_torques = np.clip(
            torques_urdf * self._joint_sign,
            -self._torque_clip,
            self._torque_clip,
        )
        if (
            np.any(cmd.vel)
            or np.any(cmd.acc)
            or np.any(cmd.torque_ff)
        ):
            safe_hold_torque = self._static_hold_motor_torque(q)
        else:
            safe_hold_torque = motor_torques.copy()
        safe_frame = _MotorCommandFrame(
            pos=cmd.pos * self._joint_sign,
            vel=np.zeros(self._num_joints),
            kp=cmd.kp.copy(),
            kd=cmd.kd.copy(),
            torque=safe_hold_torque,
        )

        self._motor_chain.send_commands(
            pos=cmd.pos * self._joint_sign,
            vel=cmd.vel * self._joint_sign,
            kp=cmd.kp,
            kd=cmd.kd,
            torque=motor_torques,
        )
        with self._command_lock:
            self._last_sent_command = JointCommand(
                pos=cmd.pos.copy(),
                vel=cmd.vel.copy(),
                acc=cmd.acc.copy(),
                kp=cmd.kp.copy(),
                kd=cmd.kd.copy(),
                torque_ff=cmd.torque_ff.copy(),
            )
            self._has_sent_command = True
            self._last_safe_hold_frame = safe_frame
        self._last_successful_arm_tx_t = time.monotonic()

    def _resend_last_safe_hold(self) -> None:
        """Resend the last fully queued target without using stale dynamics."""
        with self._send_lock:
            with self._command_lock:
                if (
                    not self._has_sent_command
                    or self._last_safe_hold_frame is None
                ):
                    raise RecoverableControlFault(
                        "No successfully queued safe MIT hold frame is available"
                    )
                self._write_last_sent_hold_locked()
                frame = _MotorCommandFrame(
                    pos=self._last_safe_hold_frame.pos.copy(),
                    vel=self._last_safe_hold_frame.vel.copy(),
                    kp=self._last_safe_hold_frame.kp.copy(),
                    kd=self._last_safe_hold_frame.kd.copy(),
                    torque=self._last_safe_hold_frame.torque.copy(),
                )
            self._motor_chain.send_commands(
                pos=frame.pos,
                vel=frame.vel,
                kp=frame.kp,
                kd=frame.kd,
                torque=frame.torque,
            )
            self._last_successful_arm_tx_t = time.monotonic()

    def _step_integrated_gripper(self) -> None:
        """Send one integrated-gripper frame after the arm chain."""
        if self.gripper is None:
            return
        gap = self._motor_chain.inter_cmd_gap_s
        if gap > 0:
            time.sleep(gap)
        if self._gripper_free_drive:
            self.gripper.free_drive_step()
        else:
            self.gripper.step()

    def _update(self) -> None:
        """Single control step: read state -> compute gravity -> send commands."""
        with self._control_state_lock:
            cycle_generation = self._control_generation
        t_now = time.time()
        self._read_state()

        # A stale RX path must not depend on stale-state dynamics. Replay the
        # gravity-only motor frame cached during the last fresh-feedback arm send.
        # Confirmed hardware faults still raise before any further MIT command.
        feedback_fault = self._check_runtime_safety()
        if feedback_fault is not None:
            self._resend_last_safe_hold()
            self._step_integrated_gripper()
            feedback_fault.hold_frame_sent = True
            raise feedback_fault

        if self._recording and t_now - self._record_last_t >= self._record_period:
            with self._state_lock:
                pos_snap = self._state.pos.copy()
            if self.gripper is not None:
                pos_snap = np.append(
                    pos_snap,
                    self.gripper.get_feedback_norm(),
                )
            with self._record_lock:
                if self._recording:
                    self._record_buffer.append((t_now, pos_snap))
            self._record_last_t = t_now

        with self._state_lock:
            q = self._state.pos.copy()
        with self._send_lock:
            cmd = self._snapshot_command()
            self._send_command_and_cache_hold_locked(cmd, q)
        self._step_integrated_gripper()
        self._mark_complete_update_if_current(cycle_generation)

    def _mark_complete_update_if_current(self, cycle_generation: int) -> bool:
        """Commit a full cycle only if no safety transition interrupted it."""
        with self._control_state_lock:
            if cycle_generation != self._control_generation:
                return False
            self._last_successful_update_t = time.monotonic()
            return True

    def _read_state(self) -> None:
        """Read all motor feedback and update internal state."""
        count = self._motor_chain.drain_and_update(self._bus)
        if count > 0:
            self._last_feedback_t = time.monotonic()
        temp_mos, temp_rotor = self._motor_chain.get_temperatures()
        with self._state_lock:
            self._state.pos = self._motor_chain.get_positions() * self._joint_sign
            self._state.vel = self._motor_chain.get_velocities() * self._joint_sign
            self._state.eff = self._motor_chain.get_efforts() * self._joint_sign
            self._state.error_codes = self._motor_chain.get_error_codes()
            self._state.temp_mos = temp_mos
            self._state.temp_rotor = temp_rotor

    # --- Safety ---

    def _check_runtime_safety(self) -> Optional[RecoverableControlFault]:
        """Run all per-cycle safety checks.

        Order is intentional: bus/health checks first so a downed bus is
        caught before motion checks are evaluated against stale data.
        Communication freshness raises a recoverable fault (keep MIT hold);
        confirmed motor/temperature/velocity hazards raise a hard fault.
        Motion-based checks (joint limits, velocity) are skipped while
        estop is latched because the user is expected to move the arm
        by hand in that state.
        """
        feedback_fault: Optional[RecoverableControlFault] = None
        try:
            self._check_feedback_stale()
        except RecoverableControlFault as exc:
            feedback_fault = exc
        self._check_motor_errors()
        self._check_motor_temps()
        with self._control_state_lock:
            soft_estop = self._control_state == ControlState.SOFT_ESTOP
        if soft_estop:
            return feedback_fault
        self._check_runtime_joint_limits()
        self._check_velocity_limits()
        return feedback_fault

    def _check_runtime_joint_limits(self) -> None:
        """Warn — do not estop — when measured position drifts out of limits.

        Teleop naturally parks joints at the soft limits (master lead pulled
        past the follower's reachable range). Historically this raised and
        tripped the emergency disable, locking the follower out once the
        master was dragged back into range. Now we only log at 1 Hz. Real
        hardware faults still hard-disable through the velocity, motor-error,
        and temperature checkers. Stale feedback remains a recoverable hold.
        """
        if self._joint_limits is None:
            return
        pos = self._state.pos
        buf = self._runtime_limit_buffer_rad
        offenders: List[str] = []
        for i, (lo, hi) in enumerate(self._joint_limits):
            if pos[i] < lo - buf or pos[i] > hi + buf:
                offenders.append(
                    f"joint{i + 1}={pos[i]:.3f} rad outside [{lo:.3f}, {hi:.3f}]"
                )
        if offenders:
            now = time.monotonic()
            if now - self._last_limit_warn_t > 1.0:
                logger.warning(
                    "Measured joint position outside soft limits: "
                    + "; ".join(offenders)
                )
                self._last_limit_warn_t = now

    def _check_motor_errors(self) -> None:
        # Both motor protocols use 0x0=disabled and 0x1=normal. Per-joint
        # freshness is checked first, so a fresh 0x0 is a real unexpected
        # torque-off report rather than an uninitialized state array.
        errs = self._state.error_codes
        feedback_seen, _ = self._joint_feedback_health()
        bad = feedback_seen & (errs != 0x1)
        if np.any(bad):
            from a1z.motor_drivers.utils import MotorErrorCode
            idx = int(np.argmax(bad))
            code = int(errs[idx])
            raise HardSafetyFault(
                f"Motor fault on joint{idx + 1}: error_code=0x{code:X} "
                f"({MotorErrorCode.get_error_message(code)})"
            )

    def _check_motor_temps(self) -> None:
        t_mos = self._state.temp_mos
        t_rotor = self._state.temp_rotor
        if np.any(t_mos > self._temp_mos_estop_c):
            idx = int(np.argmax(t_mos))
            raise HardSafetyFault(
                f"MOS over-temperature on joint{idx + 1}: {t_mos[idx]:.1f}°C "
                f"(limit {self._temp_mos_estop_c:.1f}°C)"
            )
        if np.any(t_rotor > self._temp_rotor_estop_c):
            idx = int(np.argmax(t_rotor))
            raise HardSafetyFault(
                f"Motor coil over-temperature on joint{idx + 1}: "
                f"{t_rotor[idx]:.1f}°C (limit {self._temp_rotor_estop_c:.1f}°C)"
            )
        now = time.monotonic()
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
            raise HardSafetyFault(
                f"Joint velocity limit exceeded on joint{idx + 1}: "
                f"{v[idx]:.2f} rad/s (limit ±{self._vel_limit[idx]:.2f})"
            )

    def _check_feedback_stale(self) -> None:
        now = time.monotonic()
        seen, age = self._joint_feedback_health(now)
        startup_age = (
            max(0.0, now - self._control_started_t)
            if self._control_started_t > 0
            else float("inf")
        )
        unhealthy = (seen & (age > self._stale_estop_s)) | (
            (~seen) & (startup_age > self._stale_estop_s)
        )
        if np.any(unhealthy):
            details = "; ".join(
                (
                    f"joint{i + 1}=never-seen"
                    if not seen[i]
                    else f"joint{i + 1}={age[i] * 1000:.0f}ms"
                )
                for i in np.flatnonzero(unhealthy)
            )
            raise RecoverableControlFault(
                "CAN feedback incomplete/stale "
                f"({details}; limit {self._stale_estop_s * 1000:.0f}ms)",
                fault_code="CAN_FEEDBACK_STALE",
            )
        warning = (seen & (age > self._stale_warn_s)) | (~seen)
        if np.any(warning) and now - self._last_stale_warn_t > 1.0:
            details = "; ".join(
                (
                    f"joint{i + 1}=waiting"
                    if not seen[i]
                    else f"joint{i + 1}={age[i] * 1000:.0f}ms"
                )
                for i in np.flatnonzero(warning)
            )
            logger.warning(f"CAN feedback delayed: {details}")
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
