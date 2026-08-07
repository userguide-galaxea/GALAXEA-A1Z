"""MotorB CAN driver (MIT mixed control) and MixedMotorChain.

MotorB MIT command bit layout (64 bits):
    pos(16) | vel(12) | kp(12) | kd(12) | torque(12)

MotorB Feedback layout:
    error(4, high nibble byte0) | pos(16) | vel(12) | torque(12) |
    temp_mos(8) | temp_rotor(8)

Force-position hybrid command (mode 4) frame layout (CAN ID 0x300+motor_id):
    p_des(float32-LE) | v_des(uint16-LE, ×100 → rad/s) | i_des(uint16-LE, ×10000 → fraction)
"""

import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Protocol, runtime_checkable

import can
import numpy as np

from a1z.motor_drivers.motor_a_driver import MotorA
from a1z.motor_drivers.utils import float_to_uint, uint_to_float

logger = logging.getLogger(__name__)

_CAN_SEND_TIMEOUT_S = 0.0


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
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)
        time.sleep(0.01)

    def disable(self) -> None:
        """Send motor disable command (0xFD)."""
        data = bytes([0xFF] * 7 + [0xFD])
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)
        time.sleep(0.01)

    def clear_error(self) -> None:
        """Clear motor error (0xFB)."""
        data = bytes([0xFF] * 7 + [0xFB])
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)
        time.sleep(0.01)

    def write_register(self, reg_id: int, value: int | float, is_float: bool = False) -> None:
        """Write a value to a motor register via the 0x7FF broadcast frame.

        Args:
            reg_id:   Register address (see manual register table).
            value:    Value to write.
            is_float: True for float32 registers, False for uint32.
        """
        data = bytearray(8)
        data[0] = self.motor_id & 0xFF
        data[1] = (self.motor_id >> 8) & 0xFF
        data[2] = 0x55  # write command
        data[3] = reg_id & 0xFF
        if is_float:
            struct.pack_into("<f", data, 4, float(value))
        else:
            struct.pack_into("<I", data, 4, int(value))
        msg = can.Message(arbitration_id=0x7FF, data=bytes(data), is_extended_id=False)
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)
        time.sleep(0.01)

    def set_ctrl_mode(self, mode: int) -> None:
        """Switch control mode in RAM (lost on power cycle).

        Modes: 1=MIT, 2=position-speed cascade, 3=speed, 4=force-position hybrid.
        """
        self.write_register(0x0A, mode, is_float=False)

    def send_hybrid_command(self, pos: float, vel: float, i_des: float) -> None:
        """Send force-position hybrid command (mode 4).

        The motor drives toward ``pos`` at up to ``vel`` rad/s while clamping
        the phase current to ``i_des`` × max-phase-current.  When the gripper
        contacts an object the current limit kicks in and the motor holds with
        constant force, regardless of the position error.

        Args:
            pos:   Target position (rad).
            vel:   Speed limit (rad/s), clamped to [0, 100].
            i_des: Torque current fraction [0.0, 1.0] of max phase current.
                   Use ``max_torque_nm / MOTOR_PEAK_TORQUE_NM`` to convert.
        """
        p_bytes = struct.pack("<f", pos)
        v_int = int(np.clip(vel * 100.0, 0, 10000))
        i_int = int(np.clip(i_des * 10000.0, 0, 10000))
        data = p_bytes + v_int.to_bytes(2, "little") + i_int.to_bytes(2, "little")
        msg = can.Message(
            arbitration_id=0x300 + self.motor_id, data=data, is_extended_id=False
        )
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)

    def set_zero_ram(self) -> None:
        """Set current position as zero in RAM only (0xFE). Does not write flash.

        Use after mechanically homing to a known stop so the motor's multi-turn
        counter is re-anchored without overwriting the factory flash calibration.
        """
        data = bytes([0xFF] * 7 + [0xFE])
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)
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
        self.bus.send(msg, timeout=_CAN_SEND_TIMEOUT_S)

    def parse_feedback(self, msg: can.Message) -> Optional[MotorBFeedback]:
        """Parse MotorB feedback CAN frame.

        byte0 低半字节必须是本电机 ID(达妙布局 ID | ERR<<4);不匹配的帧
        (如命令帧回显、未定义的 0x7F 告警帧)直接拒绝,防止 payload 被
        误当位置解出量程端点值(反馈跳变/伪错误码)。"""
        if msg is None or len(msg.data) < 8:
            return None

        data = msg.data
        r = self.ranges

        if (data[0] & 0x0F) != (self.motor_id & 0x0F):
            return None

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

    @staticmethod
    def error_is_fault(code: int) -> bool:
        """Return whether a DaMiao/MotorB status value is unusable.

        This field carries enable state as well as faults: only 0x1 means the
        motor is enabled and healthy after startup; 0x0 means disabled.
        """
        return int(code) != 0x1

    @staticmethod
    def describe_error(code: int) -> str:
        """Describe a DaMiao/MotorB status value."""
        code = int(code)
        name = MOTOR_B_ERROR_CODES.get(code, f"unknown({code})")
        return f"error_code=0x{code:X} ({name})"


@runtime_checkable
class MotorChain(Protocol):
    """Protocol for a chain of motors providing unified position/velocity/torque access."""

    def num_motors(self) -> int: ...
    def enable_all(self) -> None: ...
    def disable_all(self) -> None: ...
    def get_positions(self) -> np.ndarray: ...
    def get_velocities(self) -> np.ndarray: ...
    def get_efforts(self) -> np.ndarray: ...
    def classify_error_codes(self, codes: np.ndarray) -> np.ndarray: ...
    def describe_error_code(self, joint_idx: int, code: int) -> str: ...
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
        inter_cmd_gap_s: float = 0.0,
    ):
        """
        Args:
            motor_a_list: List of MotorA instances.
            motor_b_list: List of MotorB instances.
            motor_a_joint_indices: Joint indices corresponding to MotorA motors.
            motor_b_joint_indices: Joint indices corresponding to MotorB motors.
            motor_a_kt: Torque constant for MotorA current->torque conversion.
            inter_cmd_gap_s: Delay (s) inserted before each command frame send
                after the first in ``send_commands`` (covers MotorA->MotorB and
                MotorB->MotorB boundaries uniformly). Default 0.0 = bit-identical
                to the pre-experiment back-to-back burst. See SOP-05: pacing the
                burst gives the last-commanded motor's answer a clear bus slot,
                fixing the J6 feedback/target-latch starvation.
        """
        self._motor_a_list = motor_a_list
        self._motor_b_list = motor_b_list
        self._motor_a_joint_indices = motor_a_joint_indices
        self._motor_b_joint_indices = motor_b_joint_indices
        self._motor_a_kt = motor_a_kt
        self._inter_cmd_gap_s = inter_cmd_gap_s
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
        # Per-arm-joint freshness only. External motors such as the gripper
        # are routed for parsing but must never make a missing arm joint look
        # healthy.
        self._feedback_seen = np.zeros(self._n, dtype=bool)
        self._last_feedback_monotonic = np.zeros(self._n)
        self._feedback_time_lock = threading.Lock()

    def num_motors(self) -> int:
        return self._n

    def enable_all(self) -> None:
        for motor in self._motor_a_list:
            motor.enable()
        for motor in self._motor_b_list:
            # 达妙电机的控制模式存在 flash 里，装机调试时可能被改成非 MIT
            # 模式（症状：使能/反馈正常但 MIT 帧完全被忽略，电机"假死"，
            # 2026-08 J4/4340 实测）。每次启动统一写回 MIT（RAM-only，不写
            # flash），不依赖电机里存的模式。夹爪不走这里——它自己设 mode 4。
            motor.set_ctrl_mode(1)
            motor.enable()

    def disable_all(self) -> None:
        # Send twice for both motor types — a single frame arriving immediately
        # after an MIT command can be missed on a busy bus.
        # motor.disable() already includes a 10 ms inter-frame gap.
        motors = [
            (f"MotorA[{motor.motor_id}]", motor)
            for motor in self._motor_a_list
        ] + [
            (f"MotorB[{motor.motor_id}]", motor)
            for motor in self._motor_b_list
        ]
        sent_once = set()
        last_error: Dict[int, BaseException] = {}
        for _ in range(2):
            for index, (_name, motor) in enumerate(motors):
                try:
                    motor.disable()
                    sent_once.add(index)
                except Exception as exc:
                    last_error[index] = exc
        missing = [
            f"{name}: {last_error.get(index, 'no disable frame accepted')}"
            for index, (name, _motor) in enumerate(motors)
            if index not in sent_once
        ]
        if missing:
            raise RuntimeError("; ".join(missing))

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
        # SocketCAN 回显的自己发送的命令帧(is_rx=False)不是反馈,直接丢弃 —
        # 命令帧与反馈同 CAN ID,误当反馈解会污染位置/错误码。
        if not getattr(msg, "is_rx", True):
            return
        mid = int(msg.arbitration_id)
        entry = self._motor_id_map.get(mid)
        if entry is None:
            return
        motor_type, motor, joint_idx = entry
        fb = motor.parse_feedback(msg)
        if fb is None:
            return
        if isinstance(motor, MotorA) and not fb.valid_position:
            # 非类型 1 返回报文(错误上报/配置/查询/抱闸):只取错误码,
            # 不覆盖 last_feedback(位置/温度保持上一帧有效值),也不刷新
            # 反馈新鲜度 —— 它不是有效的位置反馈。
            if fb.error:
                motor.last_reported_error = fb.error
            return
        motor.last_feedback = fb
        if joint_idx >= 0:
            with self._feedback_time_lock:
                self._feedback_seen[joint_idx] = True
                self._last_feedback_monotonic[joint_idx] = time.monotonic()

    def register_external_motor(self, motor: "MotorB") -> None:
        """Register a motor for CAN feedback routing without adding it to the joint chain.

        Use for motors (e.g. gripper) that share the bus but are not part of the
        6-DOF joint chain. Their feedback frames are dispatched to motor.last_feedback
        by drain_and_update so callers can read torque/position.
        """
        self._motor_id_map[motor.motor_id] = ("motor_b", motor, -1)

    def get_positions(self) -> np.ndarray:
        return self._positions.copy()

    def get_velocities(self) -> np.ndarray:
        return self._velocities.copy()

    def get_efforts(self) -> np.ndarray:
        return self._efforts.copy()

    def get_feedback_health(self, now: Optional[float] = None) -> tuple:
        """Return ``(seen, age_s)`` for each arm joint.

        External/unknown CAN frames are deliberately excluded. Unseen joints
        have an infinite age so callers cannot mistake another motor's traffic
        for complete arm feedback.
        """
        if now is None:
            now = time.monotonic()
        with self._feedback_time_lock:
            seen = self._feedback_seen.copy()
            last = self._last_feedback_monotonic.copy()
        age = np.full(self._n, np.inf)
        age[seen] = np.maximum(0.0, now - last[seen])
        return seen, age

    def reset_feedback_health(self) -> None:
        """Start a new feedback session without trusting cached motor frames.

        ``ArmRobot.start()`` calls this before its zero-gain probe. Clearing
        both freshness and each motor's parsed cache prevents a stop/start
        cycle from treating the previous session's pose as fresh startup data.
        """
        with self._feedback_time_lock:
            self._feedback_seen.fill(False)
            self._last_feedback_monotonic.fill(0.0)
        self._positions.fill(0.0)
        self._velocities.fill(0.0)
        self._efforts.fill(0.0)
        for motor in self._motor_a_list:
            motor.last_feedback = None
        for motor in self._motor_b_list:
            motor.last_feedback = None
        for _motor_type, motor, joint_idx in self._motor_id_map.values():
            if joint_idx < 0:
                motor.last_feedback = None

    def get_joint_motor_types(self) -> List[str]:
        """Return ``motor_a``/``motor_b`` for each arm-joint index."""
        out = [""] * self._n
        for idx in self._motor_a_joint_indices:
            out[idx] = "motor_a"
        for idx in self._motor_b_joint_indices:
            out[idx] = "motor_b"
        return out


    @property
    def inter_cmd_gap_s(self) -> float:
        """Pacing gap (s) inserted before each per-tick command frame after the first.

        Read-only view of the gap ``send_commands`` applies between MotorA->MotorB
        and MotorB->MotorB boundaries. Exposed so callers pacing an extra per-tick
        frame outside this chain (e.g. the gripper's 0x07 frame in ArmRobot) can
        use the same gap from a single source of truth (SOP-06 §1.5).
        """
        return self._inter_cmd_gap_s

    def get_error_codes(self) -> np.ndarray:
        """Return per-joint motor error codes (int array, length n).

        Values retain their native protocol semantics: ENCOS/MotorA uses
        0=no-error while DaMiao/MotorB uses 0=disabled and 1=enabled.
        Call :meth:`classify_error_codes` instead of comparing the returned
        values against one shared constant. Motors without feedback yet return
        0; callers must also check per-joint feedback freshness.
        """
        out = np.zeros(self._n, dtype=int)
        for i, motor in enumerate(self._motor_a_list):
            idx = self._motor_a_joint_indices[i]
            fb = motor.last_feedback
            if fb is not None:
                out[idx] = int(fb.error)
            # 错误上报帧(非类型 1)的错误码单独锁存,优先于常规反馈帧里的值
            if motor.last_reported_error:
                out[idx] = int(motor.last_reported_error)
        for i, motor in enumerate(self._motor_b_list):
            idx = self._motor_b_joint_indices[i]
            fb = motor.last_feedback
            if fb is not None:
                out[idx] = int(fb.error)
        return out

    def classify_error_codes(self, codes: np.ndarray) -> np.ndarray:
        """Classify raw per-joint status values using each motor's protocol."""
        codes = np.asarray(codes)
        if codes.shape != (self._n,):
            raise ValueError(
                f"error code shape must be ({self._n},), got {codes.shape}"
            )

        is_fault = np.zeros(self._n, dtype=bool)
        for i, motor in enumerate(self._motor_a_list):
            idx = self._motor_a_joint_indices[i]
            is_fault[idx] = motor.error_is_fault(codes[idx])
        for i, motor in enumerate(self._motor_b_list):
            idx = self._motor_b_joint_indices[i]
            is_fault[idx] = motor.error_is_fault(codes[idx])
        return is_fault

    def describe_error_code(self, joint_idx: int, code: int) -> str:
        """Describe a raw status value using the joint's motor protocol."""
        for i, motor in enumerate(self._motor_a_list):
            if self._motor_a_joint_indices[i] == joint_idx:
                return motor.describe_error(code)
        for i, motor in enumerate(self._motor_b_list):
            if self._motor_b_joint_indices[i] == joint_idx:
                return motor.describe_error(code)
        return f"error_code=0x{int(code):X} (unknown joint {joint_idx})"

    def get_temperatures(self) -> tuple:
        """Return (temp_mos, temp_rotor) arrays in °C, length n.

        MotorA reports motor-coil temperature in its ``temperature`` field —
        we surface that as the rotor temperature for naming consistency with
        MotorB. Motors without feedback yet read 0.0.
        """
        temp_mos = np.zeros(self._n)
        temp_rotor = np.zeros(self._n)
        for i, motor in enumerate(self._motor_a_list):
            idx = self._motor_a_joint_indices[i]
            fb = motor.last_feedback
            if fb is not None:
                temp_mos[idx] = float(fb.temperature_mos)
                temp_rotor[idx] = float(fb.temperature)
        for i, motor in enumerate(self._motor_b_list):
            idx = self._motor_b_joint_indices[i]
            fb = motor.last_feedback
            if fb is not None:
                temp_mos[idx] = float(fb.temperature_mos)
                temp_rotor[idx] = float(fb.temperature_rotor)
        return temp_mos, temp_rotor

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
        gap = self._inter_cmd_gap_s
        first = True

        for i, motor in enumerate(self._motor_a_list):
            if not first and gap > 0:
                time.sleep(gap)
            first = False
            idx = self._motor_a_joint_indices[i]
            motor.send_mit_command(
                pos=float(pos[idx]),
                vel=float(vel[idx]),
                kp=float(kp[idx]),
                kd=float(kd[idx]),
                torque=float(torque[idx]),
                mode=motor_a_mode,
            )

        for i in range(len(self._motor_b_list)):
            if not first and gap > 0:
                time.sleep(gap)
            first = False
            motor = self._motor_b_list[i]
            idx = self._motor_b_joint_indices[i]
            motor.send_mit_command(
                pos=float(pos[idx]),
                vel=float(vel[idx]),
                kp=float(kp[idx]),
                kd=float(kd[idx]),
                torque=float(torque[idx]),
            )
