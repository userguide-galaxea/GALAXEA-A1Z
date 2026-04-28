"""Gripper control for A1Z — MotorB at CAN ID 0x07.

Physical stroke: -2.87 rad (open) → +2.87 rad (closed).
External interface uses normalized values: 0.0 = closed, 1.0 = fully open.

Zero calibration
----------------
Flash zero is written at the travel midpoint by tools/gripper_set_zero.py:
  1. Velocity-home to the mechanical close stop.
  2. Drive half-travel toward open.
  3. Write flash zero (0xFE + 0xAA broadcast).

Result: open_rad=-2.87, close_rad=+2.87, both within (-π, π).
No homing required at startup — direct position commands are always correct
after power cycle.
"""
import logging
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges

logger = logging.getLogger(__name__)

GRIPPER_CLOSE_RAD: float = 2.87
GRIPPER_OPEN_RAD: float = -2.87
GRIPPER_KP: float = 10.0
GRIPPER_KD: float = 0.5
GRIPPER_CAN_ID: int = 7

GRIPPER_MOTOR_RANGES = MotorBRanges(
    pos_min=-12.5,
    pos_max=12.5,
    vel_min=-30,
    vel_max=30,
    torque_min=-10,
    torque_max=10,
    kp_min=0,
    kp_max=500,
    kd_min=0,
    kd_max=5,
)


class GripperForceLimiter:
    """Detects gripper clogging and limits holding torque.

    When the gripper stalls against an object (high average torque, low speed),
    it switches to torque-limited mode: instead of chasing the commanded
    position, it backs off to whatever position produces ``max_torque``.
    The adjusted position is smoothed with an EMA to avoid jitter.

    Unclogging: when the command moves toward open, or torque drops below 0.2 Nm.
    """

    def __init__(
        self,
        max_torque: float,
        kp: float,
        clog_torque_threshold: float = 0.5,
        clog_speed_threshold: float = 0.3,
        average_window_s: float = 0.1,
    ) -> None:
        self._max_torque = max_torque
        self._kp = kp
        self._clog_torque_threshold = clog_torque_threshold
        self._clog_speed_threshold = clog_speed_threshold
        self._average_window_s = average_window_s
        self._is_clogged = False
        self._adjusted_raw: Optional[float] = None
        self._effort_history: deque = deque(maxlen=1000)

    def update(self, gripper_state: dict) -> float:
        """Compute adjusted raw position command.

        Args:
            gripper_state: dict with keys:
                target_raw   – commanded position in radians
                current_raw  – feedback position in radians
                target_norm  – commanded normalized [0=close, 1=open]
                current_norm – feedback normalized [0=close, 1=open]
                current_vel  – feedback velocity (rad/s)
                current_eff  – feedback torque (Nm)
                last_cmd_raw – last position sent to motor (radians)

        Returns:
            Adjusted raw position to send to the motor.
        """
        # Record effort sample
        now = time.time()
        self._effort_history.append((now, gripper_state["current_eff"]))

        # Compute rolling average effort
        history = list(self._effort_history)
        ts = np.array([h[0] for h in history])
        efforts = np.array([h[1] for h in history])
        valid = ts > now - self._average_window_s
        avg_effort = float(np.abs(np.mean(efforts[valid]))) if valid.any() else 0.0

        # Clog / unclog state machine
        if self._is_clogged:
            # Unclog if command direction is toward open, or torque has dropped
            if gripper_state["target_norm"] > gripper_state["current_norm"] or avg_effort < 0.2:
                self._is_clogged = False
        elif avg_effort > self._clog_torque_threshold and abs(gripper_state["current_vel"]) < self._clog_speed_threshold:
            self._is_clogged = True

        if not self._is_clogged:
            self._adjusted_raw = gripper_state["current_raw"]
            return gripper_state["target_raw"]

        # Clogged: compute position that yields max_torque
        # sign: positive = closing direction (close_rad > open_rad, so closing = increasing raw)
        command_sign = float(np.sign(gripper_state["target_raw"] - gripper_state["current_raw"]))
        if command_sign == 0.0:
            command_sign = 1.0
        zero_eff_pos = gripper_state["last_cmd_raw"] - command_sign * abs(gripper_state["current_eff"]) / self._kp
        target_raw = zero_eff_pos + command_sign * self._max_torque / self._kp

        # EMA smoothing
        a = 0.1
        if self._adjusted_raw is None:
            self._adjusted_raw = target_raw
        self._adjusted_raw = (1 - a) * self._adjusted_raw + a * target_raw
        return self._adjusted_raw


class Gripper:
    """Controls a single MotorB as a gripper.

    Usage::

        gripper = Gripper(motor, max_torque=2.5)
        gripper.enable()
        gripper.home()     # drive to open; call before main loop
        gripper.command(0.0)   # closed
        gripper.command(1.0)   # fully open
        # ... called from ArmRobot control loop:
        gripper.step()
        gripper.disable()
    """

    def __init__(
        self,
        motor: MotorB,
        open_rad: float = GRIPPER_OPEN_RAD,
        close_rad: float = GRIPPER_CLOSE_RAD,
        kp: float = GRIPPER_KP,
        kd: float = GRIPPER_KD,
        max_torque: float = -1.0,
        clog_torque_threshold: float = 0.3,
    ) -> None:
        self._motor = motor
        self._open_rad = open_rad
        self._close_rad = close_rad
        self._kp = kp
        self._kd = kd
        self._cmd_norm = 1.0  # start open
        self._last_cmd_raw: float = open_rad
        self._lock = threading.Lock()
        self._limiter: Optional[GripperForceLimiter] = (
            GripperForceLimiter(max_torque=max_torque, kp=kp,
                                clog_torque_threshold=clog_torque_threshold)
            if max_torque > 0 else None
        )

    def enable(self) -> None:
        self._motor.enable()

    def disable(self) -> None:
        self._motor.disable()

    def home(self, timeout: float = 3.0) -> None:
        """Drive gripper to open position and wait for arrival.

        Flash zero is pre-calibrated at the travel midpoint, so no velocity
        homing is needed — a direct position command is always safe after
        power cycle.

        Args:
            timeout: Maximum seconds to wait (default 3 s).
        """
        bus = self._motor.bus
        t0 = time.time()
        logger.info("Gripper init: driving to open (%+.3f rad) ...", self._open_rad)
        while time.time() - t0 < timeout:
            self._motor.send_mit_command(
                pos=self._open_rad, vel=0.0, kp=self._kp, kd=self._kd, torque=0.0
            )
            msg = bus.recv(timeout=0.01)
            if msg is not None and int(msg.arbitration_id) == self._motor.motor_id:
                fb = self._motor.parse_feedback(msg)
                if fb is not None:
                    self._motor.last_feedback = fb
            fb = self._motor.last_feedback
            if fb is not None and abs(fb.position - self._open_rad) < 0.1:
                logger.info(
                    "Gripper init: open at %+.3f rad (%.1fs).", fb.position, time.time() - t0
                )
                break
        with self._lock:
            self._cmd_norm = 1.0
        self._last_cmd_raw = self._open_rad

    def command(self, value: float) -> None:
        """Set gripper target position.

        Args:
            value: Normalized position in [0.0, 1.0].
                   0.0 = fully closed, 1.0 = fully open.
        """
        with self._lock:
            self._cmd_norm = float(np.clip(value, 0, 1))

    def get_pos(self) -> float:
        """Return the current commanded position in [0.0, 1.0]."""
        with self._lock:
            return self._cmd_norm

    def get_feedback_norm(self) -> float:
        """Return feedback-based normalized position in [0.0, 1.0].

        Falls back to commanded norm when no feedback is available.
        """
        fb = self._motor.last_feedback
        if fb is None:
            return self.get_pos()
        span = self._open_rad - self._close_rad  # negative (open < close)
        norm = (fb.position - self._close_rad) / span
        return float(np.clip(norm, 0.0, 1.0))

    def step(self) -> None:
        """Send one MIT command to the gripper. Call once per control tick."""
        with self._lock:
            norm = self._cmd_norm
        target_raw = self._close_rad + norm * (self._open_rad - self._close_rad)

        if self._limiter is not None:
            fb = self._motor.last_feedback
            if fb is not None:
                span = self._open_rad - self._close_rad
                current_norm = float(np.clip((fb.position - self._close_rad) / span, 0.0, 1.0))
                state = {
                    "target_raw": target_raw,
                    "current_raw": fb.position,
                    "target_norm": norm,
                    "current_norm": current_norm,
                    "current_vel": fb.velocity,
                    "current_eff": fb.torque,
                    "last_cmd_raw": self._last_cmd_raw,
                }
                target_raw = self._limiter.update(state)

        self._last_cmd_raw = target_raw
        self._motor.send_mit_command(
            pos=target_raw, vel=0, kp=self._kp, kd=self._kd, torque=0.0
        )
