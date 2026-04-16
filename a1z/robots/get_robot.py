"""Factory function for creating an A1Z ArmRobot."""

import os
from pathlib import Path
from typing import Optional

import can
import numpy as np

from a1z.dynamics.gravity_model import GravityModel
from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges, MixedMotorChain
from a1z.motor_drivers.motor_a_driver import MotorA, MotorARanges
from a1z.robots.arm_robot import ArmRobot

# Default URDF path (bundled inside the package)
_DEFAULT_URDF_PATH = str(Path(__file__).parent.parent / "robot_models" / "a1z" / "A1Z_Flange.urdf")

# Default A1Z configuration
_NUM_JOINTS = 6
_MOTOR_A_JOINT_INDICES = [0, 1, 2]
_MOTOR_B_JOINT_INDICES = [3, 4, 5]
_MOTOR_A_IDS = [0x01, 0x02, 0x03]
_MOTOR_B_IDS = [0x04, 0x05, 0x06]

_JOINT_LIMITS = [
    (-2.094, 2.094),   # arm_joint1
    (0.0,    3.142),   # arm_joint2
    (-3.142, 0.0),     # arm_joint3
    (-1.484, 1.484),   # arm_joint4
    (-1.484, 1.484),   # arm_joint5
    (-2.007, 2.007),   # arm_joint6
]

_DEFAULT_KP = np.array([30.0, 30.0, 30.0, 20.0, 5.0, 5.0])
_DEFAULT_KD = np.array([1.0,  1.0,  1.0,  0.5,  0.5,  0.5])
_JOINT_SIGN = np.array([1.0, 1.0, -1.0, 1.0, 1.0, 1.0])
_GRAVITY_TORQUE_SCALE = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
_MAX_GRAVITY_TORQUE = np.array([50.0, 50.0, 50.0, 24.0, 10.0, 10.0])
_TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])

# MotorA ranges (EC-A4315-P2-36)
_MOTOR_A_RANGES = MotorARanges(
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
    pos_min=-12.5, pos_max=12.5,
    vel_min=-18.0, vel_max=18.0,
    torque_min=-70.0, torque_max=70.0,
    current_fb_min=-30.0, current_fb_max=30.0,
)
_MOTOR_A_KT = 2.8

# MotorB default ranges (4310)
_MOTOR_B_RANGES_DEFAULT = MotorBRanges(
    pos_min=-12.5, pos_max=12.5,
    vel_min=-30.0, vel_max=30.0,
    torque_min=-10.0, torque_max=10.0,
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
)

# Joint 3 (arm_joint4) uses higher torque range
_MOTOR_B_RANGES_JOINT3 = MotorBRanges(
    pos_min=-12.5, pos_max=12.5,
    vel_min=-30.0, vel_max=30.0,
    torque_min=-28.0, torque_max=28.0,
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
)


def get_a1z_robot(
    can_channel: str = "can0",
    gravity_comp_factor: float = 1.0,
    zero_gravity_mode: bool = True,
    control_freq_hz: int = 250,
    min_freq_hz: float = 80.0,
    urdf_path: Optional[str] = None,
    default_kp: Optional[np.ndarray] = None,
    default_kd: Optional[np.ndarray] = None,
) -> ArmRobot:
    """Create and return a configured A1Z ArmRobot.

    Args:
        can_channel: CAN interface name (e.g. 'can0').
        gravity_comp_factor: Gravity compensation scale (0=off, 1=full).
        zero_gravity_mode: True for zero-gravity (floating) mode, False for
                           position hold with PD + gravity comp.
        control_freq_hz: Control loop frequency in Hz.
        min_freq_hz: Minimum acceptable control frequency. Emergency stop if
                     frequency stays below this for 3 consecutive check periods.
        urdf_path: Override URDF path.
        default_kp: Override default position gains.
        default_kd: Override default velocity gains.

    Returns:
        Configured ArmRobot instance (call .start() to begin control).
    """
    urdf = urdf_path or _DEFAULT_URDF_PATH

    # Open CAN bus
    bus = can.interface.Bus(
        channel=can_channel,
        bustype="socketcan",
        bitrate=1_000_000,
    )

    # Create MotorA motors
    motor_a_list = [
        MotorA(motor_id=mid, bus=bus, ranges=_MOTOR_A_RANGES)
        for mid in _MOTOR_A_IDS
    ]

    # Create MotorB motors with per-joint ranges
    motor_b_ranges_by_joint = {3: _MOTOR_B_RANGES_JOINT3}
    motor_b_list = []
    for i, mid in enumerate(_MOTOR_B_IDS):
        joint_idx = _MOTOR_B_JOINT_INDICES[i]
        ranges = motor_b_ranges_by_joint.get(joint_idx, _MOTOR_B_RANGES_DEFAULT)
        motor_b_list.append(MotorB(motor_id=mid, bus=bus, ranges=ranges))

    # Build motor chain
    motor_chain = MixedMotorChain(
        motor_a_list=motor_a_list,
        motor_b_list=motor_b_list,
        motor_a_joint_indices=_MOTOR_A_JOINT_INDICES,
        motor_b_joint_indices=_MOTOR_B_JOINT_INDICES,
        motor_a_kt=_MOTOR_A_KT,
    )

    # Load gravity model
    gravity_model = GravityModel(urdf)

    return ArmRobot(
        motor_chain=motor_chain,
        bus=bus,
        gravity_model=gravity_model,
        num_joints=_NUM_JOINTS,
        gravity_comp_factor=gravity_comp_factor,
        zero_gravity_mode=zero_gravity_mode,
        joint_sign=_JOINT_SIGN,
        gravity_torque_scale=_GRAVITY_TORQUE_SCALE,
        max_gravity_torque=_MAX_GRAVITY_TORQUE,
        torque_clip=_TORQUE_CLIP,
        default_kp=default_kp if default_kp is not None else _DEFAULT_KP,
        default_kd=default_kd if default_kd is not None else _DEFAULT_KD,
        joint_limits=_JOINT_LIMITS,
        control_freq_hz=control_freq_hz,
        min_freq_hz=min_freq_hz,
        motor_a_kt=_MOTOR_A_KT,
    )
