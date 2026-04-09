"""Motor driver shared data structures and conversion utilities."""

import enum
from dataclasses import dataclass
from typing import List


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """Convert unsigned int to float, given range and number of bits."""
    span = x_max - x_min
    x_int = max(0, min(int(x_int), (1 << bits) - 1))
    return (x_int * span / ((1 << bits) - 1)) + x_min


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Convert a float to an unsigned int, given range and number of bits."""
    span = x_max - x_min
    x = min(x, x_max)
    x = max(x, x_min)
    return int((x - x_min) * ((1 << bits) - 1) / span)


@dataclass
class MotorConstants:
    """Physical parameter ranges for a motor type."""

    POSITION_MAX: float = 12.5
    POSITION_MIN: float = -12.5

    VELOCITY_MAX: float = 45
    VELOCITY_MIN: float = -45

    TORQUE_MAX: float = 54
    TORQUE_MIN: float = -54

    CURRENT_MAX: float = 1.0
    CURRENT_MIN: float = -1.0
    KT: float = 1.0

    KP_MAX: float = 500.0
    KP_MIN: float = 0.0
    KD_MAX: float = 5.0
    KD_MIN: float = 0.0


@dataclass
class MotorInfo:
    """Runtime motor state information."""

    id: int
    error_code: int
    target_torque: int = 0
    vel: float = 0.0
    eff: float = 0
    pos: float = 0
    voltage: float = -1
    temp_mos: float = -1
    temp_rotor: float = -1


@dataclass
class FeedbackFrameInfo:
    """Parsed feedback frame from a motor."""

    id: int
    error_code: int
    error_message: str
    position: float
    velocity: float
    torque: float
    temperature_mos: float
    temperature_rotor: float


class MotorErrorCode:
    """Motor error codes."""

    disabled = 0x0
    normal = 0x1
    over_voltage = 0x8
    under_voltage = 0x9
    over_current = 0xA
    mosfet_over_temperature = 0xB
    motor_over_temperature = 0xC
    loss_communication = 0xD
    overload = 0xE

    motor_error_code_dict = {
        0x0: "disabled",
        0x1: "normal",
        0x8: "over voltage",
        0x9: "under voltage",
        0xA: "over current",
        0xB: "mosfet over temperature",
        0xC: "motor over temperature",
        0xD: "loss communication",
        0xE: "overload",
    }

    @classmethod
    def get_error_message(cls, error_code: int) -> str:
        return cls.motor_error_code_dict.get(int(error_code), f"Unknown error code: {error_code}")


class MotorType:
    """Known motor type identifiers and their default constants."""

    MOTOR_B_4310 = "MOTOR_B_4310"
    MOTOR_B_4340 = "MOTOR_B_4340"
    MOTOR_B_8009 = "MOTOR_B_8009"
    MOTOR_B_3507 = "MOTOR_B_3507"
    MOTOR_A_4315 = "MOTOR_A_4315"

    @classmethod
    def get_motor_constants(cls, motor_type: str) -> MotorConstants:
        if motor_type == cls.MOTOR_A_4315:
            return MotorConstants(
                POSITION_MAX=12.5,
                POSITION_MIN=-12.5,
                VELOCITY_MAX=18,
                VELOCITY_MIN=-18,
                TORQUE_MAX=80,
                TORQUE_MIN=-80,
                CURRENT_MAX=30.0,
                CURRENT_MIN=-30.0,
                KT=2.8,
                KP_MAX=500.0,
                KP_MIN=0.0,
                KD_MAX=5.0,
                KD_MIN=0.0,
            )
        elif motor_type == cls.MOTOR_B_4310:
            return MotorConstants(
                POSITION_MAX=12.5,
                POSITION_MIN=-12.5,
                VELOCITY_MAX=30,
                VELOCITY_MIN=-30,
                TORQUE_MAX=10,
                TORQUE_MIN=-10,
            )
        elif motor_type == cls.MOTOR_B_4340:
            return MotorConstants(
                POSITION_MAX=12.5,
                POSITION_MIN=-12.5,
                VELOCITY_MAX=10,
                VELOCITY_MIN=-10,
                TORQUE_MAX=28,
                TORQUE_MIN=-28,
            )
        elif motor_type == cls.MOTOR_B_8009:
            return MotorConstants(
                POSITION_MAX=12.5,
                POSITION_MIN=-12.5,
                VELOCITY_MAX=45,
                VELOCITY_MIN=-45,
                TORQUE_MAX=54,
                TORQUE_MIN=-54,
            )
        elif motor_type == cls.MOTOR_B_3507:
            return MotorConstants(
                POSITION_MAX=12.5,
                POSITION_MIN=-12.5,
                VELOCITY_MAX=50,
                VELOCITY_MIN=-50,
                TORQUE_MAX=5,
                TORQUE_MIN=-5,
            )
        else:
            raise ValueError(f"Motor type '{motor_type}' not recognized.")


class AutoNameEnum(enum.Enum):
    def _generate_next_value_(name: str, start: int, count: int, last_values: List[str]) -> str:
        return name


class ReceiveMode(AutoNameEnum):
    """CAN receive ID mode for different motor firmware variants."""

    p16 = enum.auto()
    same = enum.auto()
    zero = enum.auto()
    plus_one = enum.auto()

    def get_receive_id(self, motor_id: int) -> int:
        if self == ReceiveMode.p16:
            return motor_id + 16
        elif self == ReceiveMode.same:
            return motor_id
        elif self == ReceiveMode.zero:
            return 0
        elif self == ReceiveMode.plus_one:
            return motor_id + 1
        else:
            raise NotImplementedError(f"receive_mode: {self} not recognized")

    def to_motor_id(self, receive_id: int) -> int:
        if self == ReceiveMode.p16:
            return receive_id - 16
        elif self == ReceiveMode.same:
            return receive_id
        elif self == ReceiveMode.zero:
            return 0
        else:
            raise NotImplementedError(f"receive_mode: {self} not recognized")
