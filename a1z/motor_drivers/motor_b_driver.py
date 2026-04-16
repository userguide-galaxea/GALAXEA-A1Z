"""MotorB CAN driver (MIT mixed control) and MixedMotorChain.

MotorB MIT command bit layout (64 bits):
    pos(16) | vel(12) | kp(12) | kd(12) | torque(12)

MotorB Feedback layout:
    error(4, high nibble byte0) | pos(16) | vel(12) | torque(12) |
    temp_mos(8) | temp_rotor(8)
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol, runtime_checkable

import can
import numpy as np

from a1z.motor_drivers.motor_a_driver import MotorA, MotorAFeedback, MotorARanges
from a1z.motor_drivers.utils import float_to_uint, uint_to_float

logger = logging.getLogger(__name__)


MOTOR_B_ERROR_CODES = {
    0x0: "disabled",
    0x1: "normal",
    0x8: "over voltage",
    0x9: "under voltage",
    0xA: "over current",
    0xB: "mos over temperature",
    0xC: "motor coil over temperature",
    0xD: "communication lost",
    0xE: "overload",
    0xF: "position out of range",
}


@dataclass
class MotorBRanges:
    """MotorB physical ranges."""

    pos_min: float = -12.5
    pos_max: float = 12.5
    vel_min: float = -30.0
    vel_max: float = 30.0
    torque_min: float = -10.0
    torque_max: float = 10.0
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0


@dataclass
class MotorBFeedback:
    """MotorB feedback data."""

    motor_id: int = 0
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    error: int = 0
    error_message: str = ""
    temperature_mos: float = 0.0
    temperature_rotor: float = 0.0


class MotorB:
    """Single MotorB CAN driver (MIT mixed control)."""

    def __init__(self, motor_id: int, bus: can.BusABC, ranges: Optional[MotorBRanges] = None):
        self.motor_id = motor_id
        self.bus = bus
        self.ranges = ranges or MotorBRanges()
        self.last_feedback: Optional[MotorBFeedback] = None

    def enable(self) -> None:
        """Send motor enable command (0xFC)."""
        data = bytes([0xFF] * 7 + [0xFC])
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.01)

    def disable(self) -> None:
        """Send motor disable command (0xFD)."""
        data = bytes([0xFF] * 7 + [0xFD])
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.01)

    def clear_error(self) -> None:
        """Clear motor error (0xFB)."""
        data = bytes([0xFF] * 7 + [0xFB])
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        time.sleep(0.01)

    def send_mit_command(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        torque: float,
    ) -> None:
        """Send MIT mixed-control command.

        Args:
            pos: Target position (rad).
            vel: Target velocity (rad/s).
            kp:  Position gain.
            kd:  Velocity gain.
            torque: Feedforward torque (Nm).
        """
        r = self.ranges
        pos_u16 = float_to_uint(pos, r.pos_min, r.pos_max, 16)
        vel_u12 = float_to_uint(vel, r.vel_min, r.vel_max, 12)
        kp_u12 = float_to_uint(kp, r.kp_min, r.kp_max, 12)
        kd_u12 = float_to_uint(kd, r.kd_min, r.kd_max, 12)
        tor_u12 = float_to_uint(torque, r.torque_min, r.torque_max, 12)

        data = bytearray(8)
        data[0] = (pos_u16 >> 8) & 0xFF
        data[1] = pos_u16 & 0xFF
        data[2] = (vel_u12 >> 4) & 0xFF
        data[3] = ((vel_u12 & 0xF) << 4) | ((kp_u12 >> 8) & 0xF)
        data[4] = kp_u12 & 0xFF
        data[5] = (kd_u12 >> 4) & 0xFF
        data[6] = ((kd_u12 & 0xF) << 4) | ((tor_u12 >> 8) & 0xF)
        data[7] = tor_u12 & 0xFF

        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def parse_feedback(self, msg: can.Message) -> Optional[MotorBFeedback]:
        """Parse MotorB feedback CAN frame."""
        if msg is None or len(msg.data) < 8:
            return None

        data = msg.data
        r = self.ranges

        error_int = (data[0] & 0xF0) >> 4
        error_message = MOTOR_B_ERROR_CODES.get(error_int, f"unknown({error_int})")

        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        t_int = ((data[4] & 0xF) << 8) | data[5]
        temp_mos = float(data[6])
        temp_rotor = float(data[7])

        position = uint_to_float(p_int, r.pos_min, r.pos_max, 16)
        velocity = uint_to_float(v_int, r.vel_min, r.vel_max, 12)
        torque = uint_to_float(t_int, r.torque_min, r.torque_max, 12)

        return MotorBFeedback(
            motor_id=msg.arbitration_id,
            position=position,
            velocity=velocity,
            torque=torque,
            error=error_int,
            error_message=error_message,
            temperature_mos=temp_mos,
            temperature_rotor=temp_rotor,
        )


@runtime_checkable
class MotorChain(Protocol):
    """Protocol for a chain of motors providing unified position/velocity/torque access."""

    def num_motors(self) -> int: ...
    def enable_all(self) -> None: ...
    def disable_all(self) -> None: ...
    def get_positions(self) -> np.ndarray: ...
    def get_velocities(self) -> np.ndarray: ...
    def get_efforts(self) -> np.ndarray: ...
    def send_commands(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
        torque: np.ndarray,
    ) -> None: ...


class MixedMotorChain:
    """Manages MotorA + MotorB motors as a unified motor chain.

    Joints are ordered: MotorA motors first (indices 0..n_motor_a-1),
    then MotorB motors (indices n_motor_a..n_motor_a+n_motor_b-1).
    The total number of joints is n_motor_a + n_motor_b.
    """

    def __init__(
        self,
        motor_a_list: List[MotorA],
        motor_b_list: List[MotorB],
        motor_a_joint_indices: List[int],
        motor_b_joint_indices: List[int],
        motor_a_kt: float = 2.8,
    ):
        """
        Args:
            motor_a_list: List of MotorA instances.
            motor_b_list: List of MotorB instances.
            motor_a_joint_indices: Joint indices corresponding to MotorA motors.
            motor_b_joint_indices: Joint indices corresponding to MotorB motors.
            motor_a_kt: Torque constant for MotorA current->torque conversion.
        """
        self._motor_a_list = motor_a_list
        self._motor_b_list = motor_b_list
        self._motor_a_joint_indices = motor_a_joint_indices
        self._motor_b_joint_indices = motor_b_joint_indices
        self._motor_a_kt = motor_a_kt
        self._n = len(motor_a_list) + len(motor_b_list)

        # Build motor_id -> (type, motor, joint_idx) lookup
        self._motor_id_map: Dict[int, tuple] = {}
        for i, motor in enumerate(motor_a_list):
            self._motor_id_map[motor.motor_id] = ("motor_a", motor, motor_a_joint_indices[i])
        for i, motor in enumerate(motor_b_list):
            self._motor_id_map[motor.motor_id] = ("motor_b", motor, motor_b_joint_indices[i])

        self._positions = np.zeros(self._n)
        self._velocities = np.zeros(self._n)
        self._efforts = np.zeros(self._n)

    def num_motors(self) -> int:
        return self._n

    def enable_all(self) -> None:
        for motor in self._motor_a_list:
            motor.enable()
        for motor in self._motor_b_list:
            motor.enable()

    def disable_all(self) -> None:
        for motor in self._motor_a_list:
            try:
                motor.disable()
            except Exception:
                pass
        for motor in self._motor_b_list:
            try:
                motor.disable()
            except Exception:
                pass

    def drain_and_update(self, bus: can.BusABC, timeout: float = 0.001, max_messages: int = 0) -> int:
        """Drain all pending CAN messages from the bus, dispatching to the correct motor parser.

        Args:
            bus: CAN bus to read from.
            timeout: Maximum time (s) to spend draining. Default 1ms.
            max_messages: Maximum messages to read per call. 0 means 2 * num_motors.

        Returns:
            Number of messages processed.
        """
        if max_messages <= 0:
            max_messages = self._n * 2
        count = 0
        t_end = time.time() + timeout
        while count < max_messages and time.time() < t_end:
            msg = bus.recv(timeout=0.0)
            if msg is None:
                break
            self._dispatch_feedback(msg)
            count += 1

        # Update state arrays from last_feedback
        for i, motor in enumerate(self._motor_a_list):
            idx = self._motor_a_joint_indices[i]
            fb = motor.last_feedback
            if fb is not None:
                self._positions[idx] = fb.position
                self._velocities[idx] = fb.velocity
                self._efforts[idx] = fb.current * self._motor_a_kt

        for i, motor in enumerate(self._motor_b_list):
            idx = self._motor_b_joint_indices[i]
            fb = motor.last_feedback
            if fb is not None:
                self._positions[idx] = fb.position
                self._velocities[idx] = fb.velocity
                self._efforts[idx] = fb.torque

        return count

    def _dispatch_feedback(self, msg: can.Message) -> None:
        """Route a CAN message to the correct motor parser."""
        mid = int(msg.arbitration_id)
        entry = self._motor_id_map.get(mid)
        if entry is None:
            return
        motor_type, motor, joint_idx = entry
        fb = motor.parse_feedback(msg)
        if fb is not None:
            motor.last_feedback = fb

    def get_positions(self) -> np.ndarray:
        return self._positions.copy()

    def get_velocities(self) -> np.ndarray:
        return self._velocities.copy()

    def get_efforts(self) -> np.ndarray:
        return self._efforts.copy()

    def send_commands(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
        torque: np.ndarray,
        motor_a_mode: int = 0,
    ) -> None:
        """Send MIT commands to all motors.

        Args:
            pos: Target positions (rad), shape (n,).
            vel: Target velocities (rad/s), shape (n,).
            kp: Position gains, shape (n,).
            kd: Velocity gains, shape (n,).
            torque: Feedforward torques (Nm), shape (n,).
            motor_a_mode: MotorA MIT mode field (default 0).
        """
        for i, motor in enumerate(self._motor_a_list):
            idx = self._motor_a_joint_indices[i]
            motor.send_mit_command(
                pos=float(pos[idx]),
                vel=float(vel[idx]),
                kp=float(kp[idx]),
                kd=float(kd[idx]),
                torque=float(torque[idx]),
                mode=motor_a_mode,
            )

        for i, motor in enumerate(self._motor_b_list):
            idx = self._motor_b_joint_indices[i]
            motor.send_mit_command(
                pos=float(pos[idx]),
                vel=float(vel[idx]),
                kp=float(kp[idx]),
                kd=float(kd[idx]),
                torque=float(torque[idx]),
            )
