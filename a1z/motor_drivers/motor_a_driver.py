"""MotorA CAN driver (MIT mixed control).

Bit layout (64 bits, big-endian, MSB first):
    mode:   uint3
    kp:     uint12   (0..4095 -> kp_min..kp_max)
    kd:     uint9    (0..511  -> kd_min..kd_max)
    pos:    uint16   (0..65535 -> pos_min..pos_max)
    vel:    uint12   (0..4095  -> vel_min..vel_max)
    torque: uint12   (0..4095  -> torque_min..torque_max)

Feedback layout (64 bits, big-endian):
    report_type: uint3
    error_code:  uint5
    pos:         uint16
    vel:         uint12
    current:     uint12
    motor_temp:  uint8
    mos_temp:    uint8
"""

import time
from dataclasses import dataclass
from typing import Optional

import can

from a1z.motor_drivers.utils import float_to_uint, uint_to_float


@dataclass
class MotorARanges:
    """MotorA physical ranges."""

    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0
    pos_min: float = -12.5
    pos_max: float = 12.5
    vel_min: float = -18.0
    vel_max: float = 18.0
    torque_min: float = -90.0
    torque_max: float = 90.0
    current_fb_min: float = -30.0
    current_fb_max: float = 30.0


@dataclass
class MotorAFeedback:
    """MotorA feedback data."""

    motor_id: int = 0
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    error: int = 0
    temperature: float = 0.0      # motor coil temperature (°C)
    temperature_mos: float = 0.0  # MOS temperature (°C)


def pack_motor_a_mit(
    mode: int,
    kp_u12: int,
    kd_u9: int,
    pos_u16: int,
    vel_u12: int,
    torque_u12: int,
) -> bytes:
    """Pack MotorA MIT command into 8 bytes (big-endian bitfield).

    Byte layout (aligned with CAN-H7 send_motor_ctrl_cmd):
        data[0] = (mode << 5) | (kp >> 7)
        data[1] = ((kp & 0x7F) << 1) | (kd >> 8)
        data[2] = kd & 0xFF
        data[3] = pos >> 8
        data[4] = pos & 0xFF
        data[5] = vel >> 4
        data[6] = ((vel & 0x0F) << 4) | (torque >> 8)
        data[7] = torque & 0xFF
    """
    payload = 0
    payload = (payload << 3) | (mode & 0x7)
    payload = (payload << 12) | (kp_u12 & 0xFFF)
    payload = (payload << 9) | (kd_u9 & 0x1FF)
    payload = (payload << 16) | (pos_u16 & 0xFFFF)
    payload = (payload << 12) | (vel_u12 & 0xFFF)
    payload = (payload << 12) | (torque_u12 & 0xFFF)
    return payload.to_bytes(8, byteorder="big", signed=False)


class MotorA:
    """Single MotorA CAN driver."""

    def __init__(
        self,
        motor_id: int,
        bus: can.BusABC,
        ranges: Optional[MotorARanges] = None,
    ):
        self.motor_id = motor_id
        self.bus = bus
        self.ranges = ranges or MotorARanges()
        self.last_feedback: Optional[MotorAFeedback] = None

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

    def send_mit_command(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        torque: float,
        mode: int = 0,
    ) -> None:
        """Send MIT mixed-control command.

        Args:
            pos: Target position (rad).
            vel: Target velocity (rad/s).
            kp:  Position gain.
            kd:  Velocity gain.
            torque: Feedforward torque (Nm).
            mode: MotorA mode field (uint3), usually 0.
        """
        r = self.ranges
        kp_u12 = float_to_uint(kp, r.kp_min, r.kp_max, 12)
        kd_u9 = float_to_uint(kd, r.kd_min, r.kd_max, 9)
        pos_u16 = float_to_uint(pos, r.pos_min, r.pos_max, 16)
        vel_u12 = float_to_uint(vel, r.vel_min, r.vel_max, 12)
        torque_u12 = float_to_uint(torque, r.torque_min, r.torque_max, 12)

        data = pack_motor_a_mit(mode, kp_u12, kd_u9, pos_u16, vel_u12, torque_u12)
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def parse_feedback(self, msg: can.Message) -> Optional[MotorAFeedback]:
        """Parse MotorA feedback CAN frame.

        Bit layout (big-endian):
            report_type(3) | error_code(5) | pos(16) | vel(12) |
            current(12) | motor_temp(8) | mos_temp(8)
        """
        if msg is None or len(msg.data) < 8:
            return None

        data = msg.data
        r = self.ranges

        frame = int.from_bytes(data, byteorder="big", signed=False)
        error_code = (frame >> 56) & 0x1F
        pos_raw = (frame >> 40) & 0xFFFF
        vel_raw = (frame >> 28) & 0xFFF
        curr_raw = (frame >> 16) & 0xFFF
        motor_temp_raw = (frame >> 8) & 0xFF
        mos_temp_raw = frame & 0xFF

        position = uint_to_float(pos_raw, r.pos_min, r.pos_max, 16)
        velocity = uint_to_float(vel_raw, r.vel_min, r.vel_max, 12)
        current = uint_to_float(curr_raw, r.current_fb_min, r.current_fb_max, 12)
        # Temperature encoding: raw = actual_°C * 2 + 50
        temperature = (motor_temp_raw - 50) / 2
        temperature_mos = (mos_temp_raw - 50) / 2

        return MotorAFeedback(
            motor_id=msg.arbitration_id,
            position=position,
            velocity=velocity,
            current=current,
            error=error_code,
            temperature=temperature,
            temperature_mos=temperature_mos,
        )
