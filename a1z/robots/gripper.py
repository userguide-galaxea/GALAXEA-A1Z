"""Gripper control for A1Z — MotorB at CAN ID 0x07.

Physical stroke: -2.87 rad (open) → +2.87 rad (closed).
External interface uses normalized values: 0.0 = closed, 1.0 = fully open.

Control mode
------------
Uses the motor's force-position hybrid mode (mode 4, CAN ID 0x300+motor_id).
The motor drives toward the target position while clamping phase current to
``i_des × max_phase_current``.  When the gripper contacts an object the current
limit kicks in and holds at constant force — no position error accumulates, no
software estimation needed.

``max_torque`` (Nm) is converted to ``i_des`` via the motor's peak output torque
(MOTOR_PEAK_TORQUE_NM = 11 Nm at 0.8 overcurrent).

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
from typing import Optional

import numpy as np

from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges

logger = logging.getLogger(__name__)

# ── Gripper hardware configuration ──────────────────────────────────────────
# Adjust these two values to match the actual mechanical travel of your gripper.
# Run tools/gripper_set_zero.py --half-travel <GRIPPER_CLOSE_RAD> to calibrate.
GRIPPER_CLOSE_RAD: float = 2.87   # rad, fully closed (positive mechanical stop)
GRIPPER_OPEN_RAD: float  = -2.87  # rad, fully open   (negative mechanical stop)
# ────────────────────────────────────────────────────────────────────────────
GRIPPER_MAX_VEL: float = 10.0   # rad/s speed limit during normal operation
GRIPPER_HOME_VEL: float = 5.0   # rad/s speed limit during homing
GRIPPER_HOME_TORQUE_NM: float = 0.5  # torque limit during homing (Nm)
GRIPPER_CAN_ID: int = 7

# Peak output torque at 0.8 overcurrent setting (from datasheet)
MOTOR_PEAK_TORQUE_NM: float = 11.0

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


class Gripper:
    """Controls a single MotorB as a gripper using force-position hybrid mode.

    Usage::

        gripper = Gripper(motor, max_torque=2.0)
        gripper.enable()
        gripper.home()       # drive to open; call before main loop
        gripper.command(0.0) # closed
        gripper.command(1.0) # fully open
        # ... called from ArmRobot control loop:
        gripper.step()
        gripper.disable()

    The motor is switched to force-position hybrid mode (mode 4) on
    ``enable()``.  ``max_torque`` limits the gripping force: the gripper closes
    until the object resists with that force, then holds without crushing it.
    """

    def __init__(
        self,
        motor: MotorB,
        open_rad: float = GRIPPER_OPEN_RAD,
        close_rad: float = GRIPPER_CLOSE_RAD,
        max_torque: float = 0.5,
        max_vel: float = GRIPPER_MAX_VEL,
    ) -> None:
        self._motor = motor
        self._open_rad = open_rad
        self._close_rad = close_rad
        self._max_vel = max_vel
        self._i_des = float(np.clip(max_torque / MOTOR_PEAK_TORQUE_NM, 0.0, 1.0))
        self._cmd_norm = 1.0  # start open
        self._lock = threading.Lock()

    def enable(self) -> None:
        self._motor.clear_error()
        self._motor.enable()
        # Switch to force-position hybrid mode so hardware enforces torque limit.
        self._motor.set_ctrl_mode(4)
        # After mode switch the motor clears p_des to 0 (travel midpoint = flash zero).
        # Read actual position and feed it as first hybrid frame to prevent a brief
        # snap toward midpoint before home() takes over.
        if self._motor.last_feedback is None:
            msg = self._motor.bus.recv(timeout=0.05)
            if msg is not None and int(msg.arbitration_id) == self._motor.motor_id:
                fb = self._motor.parse_feedback(msg)
                if fb is not None:
                    self._motor.last_feedback = fb
        fb = self._motor.last_feedback
        if fb is not None:
            self._motor.send_hybrid_command(pos=fb.position, vel=0.0, i_des=0.0)

    def disable(self) -> None:
        self._motor.disable()

    def home(self, timeout: float = 1.5) -> bool:
        """Drive gripper to open position and wait for arrival.

        Args:
            timeout: Maximum seconds to wait (default 3 s).

        Returns:
            True if gripper reached open position, False if timed out.
        """
        bus = self._motor.bus
        t0 = time.time()
        i_home = GRIPPER_HOME_TORQUE_NM / MOTOR_PEAK_TORQUE_NM
        logger.info("Gripper init: driving to open (%+.3f rad) ...", self._open_rad)
        reached = False
        while time.time() - t0 < timeout:
            self._motor.send_hybrid_command(
                pos=self._open_rad, vel=GRIPPER_HOME_VEL, i_des=i_home
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
                reached = True
                break
        if not reached:
            logger.warning(
                "Gripper home timed out after %.1fs, pos=%s rad.",
                timeout,
                f"{self._motor.last_feedback.position:+.3f}" if self._motor.last_feedback else "?",
            )
        with self._lock:
            self._cmd_norm = 1.0
        return reached

    def command(self, value: float) -> None:
        """Set gripper target position.

        Args:
            value: Normalized position in [0.0, 1.0].
                   0.0 = fully closed, 1.0 = fully open.
        """
        with self._lock:
            self._cmd_norm = float(np.clip(value, 0.0, 1.0))

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
        """Send one hybrid command to the gripper. Call once per control tick."""
        with self._lock:
            norm = self._cmd_norm
        target_raw = self._close_rad + norm * (self._open_rad - self._close_rad)
        self._motor.send_hybrid_command(
            pos=target_raw, vel=self._max_vel, i_des=self._i_des
        )

    def free_drive_step(self) -> None:
        """Send a zero-current hybrid frame so the user can move the gripper by hand.

        i_des=0 disables phase current, so the motor produces no torque and the
        jaw can be opened/closed manually while still streaming feedback. The
        commanded position is kept at the current feedback so that returning to
        normal step() does not snap the jaw.
        """
        fb = self._motor.last_feedback
        pos = fb.position if fb is not None else self._open_rad
        self._motor.send_hybrid_command(pos=pos, vel=0.0, i_des=0.0)
        with self._lock:
            self._cmd_norm = float(np.clip(
                (pos - self._close_rad) / (self._open_rad - self._close_rad),
                0.0, 1.0,
            ))
