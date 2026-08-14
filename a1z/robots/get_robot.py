"""Factory function for creating an A1Z ArmRobot."""

import os
from pathlib import Path
from typing import Optional

import can
import numpy as np

from a1z.dynamics.gravity_model import GravityModel
from a1z.motor_drivers.can_backend import open_can_bus
from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges, MixedMotorChain
from a1z.motor_drivers.motor_a_driver import MotorA, MotorARanges
from a1z.robots.arm_robot import ArmRobot
from a1z.robots.gripper import Gripper, GRIPPER_CAN_ID, GRIPPER_MOTOR_RANGES

# Default URDF path (bundled inside the package)
# Keep the arm-only A1Z_Flange.urdf as the default so existing users without a
# gripper are not affected. The G1Z variant is selected automatically when
# ``with_gripper=True``.
_DEFAULT_URDF_PATH = str(Path(__file__).parent.parent / "robot_models" / "a1z" / "A1Z_Flange.urdf")
_GRIPPER_URDF_PATH = str(Path(__file__).parent.parent / "robot_models" / "a1z" / "A1Z_G1Z.urdf")

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

# Physical limits remain identical to the URDF. Every joint boundary carries a
# 0.05 rad calibration/noise margin (feedback noise, arms resting at the
# boundary — e.g. J2 resting slightly below 0, J3 slightly above 0). Stored on
# ArmRobot as-is; the limit-checking policy that consumes them is ported in a
# later stage, so they change no behavior yet.
_JOINT_LIMIT_LOWER_TOLERANCE_RAD = np.full(_NUM_JOINTS, 0.05)
_JOINT_LIMIT_UPPER_TOLERANCE_RAD = np.full(_NUM_JOINTS, 0.05)

_DEFAULT_KP = np.array([146.8988, 62.9454, 89.2416, 120.0, 40.0, 100.0])
_DEFAULT_KD = np.array([5.0, 5.0, 5.0, 2.0776, 1.5059, 1.2553])
_JOINT_SIGN = np.array([1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
_GRAVITY_TORQUE_SCALE = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
_MAX_GRAVITY_TORQUE = np.array([50.0, 50.0, 50.0, 24.0, 10.0, 10.0])
_TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])

# MotorA ranges
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
    vel_min=-10.0, vel_max=10.0,
    torque_min=-28.0, torque_max=28.0,
    kp_min=0.0, kp_max=500.0,
    kd_min=0.0, kd_max=5.0,
)


# CAN command pacing. The per-tick command burst is spaced so the
# last-commanded motor's answer slot is never occupied by its predecessor's
# answer, fixing the feedback/target-latch starvation observed on the wrist
# joint at the end of the burst.
_DEFAULT_INTER_CMD_GAP_US = 250.0
_MAX_INTER_CMD_GAP_US = 500.0  # 6 paced boundaries x worst-case sleep must stay under the 4 ms tick


def _resolve_inter_cmd_gap_us(param: Optional[float]) -> float:
    """Resolve the inter-command pacing gap in µs.

    ``param`` (the ``inter_cmd_gap_us`` argument) when not None, else the
    hard-coded 250 µs product default. Range-checked at construction
    (fail-fast) so a misconfigured gap refuses to build the robot rather than
    degrading mid-run. The gap is hard-coded/parameter-only — the SDK reads no
    environment variable; pass ``inter_cmd_gap_us=0`` for the legacy
    back-to-back burst.
    """
    gap = param if param is not None else _DEFAULT_INTER_CMD_GAP_US
    if not (0.0 <= gap <= _MAX_INTER_CMD_GAP_US):
        raise ValueError(
            f"inter_cmd_gap_us={gap} outside [0, {_MAX_INTER_CMD_GAP_US}] µs"
        )
    return gap


def get_a1z_robot(
    can_channel: str = "can0",
    gravity_comp_factor: float = 1.0,
    zero_gravity_mode: bool = True,
    control_freq_hz: int = 250,
    min_freq_hz: float = 80.0,
    urdf_path: Optional[str] = None,
    default_kp: Optional[np.ndarray] = None,
    default_kd: Optional[np.ndarray] = None,
    with_gripper: bool = False,
    gripper_max_torque: float = 2.0,
    inter_cmd_gap_us: Optional[float] = None,
    motor_a_use_new_enable_protocol: bool = False,
    bustype: Optional[str] = None,
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
        with_gripper: If True, attach a Gripper at CAN ID 0x07 and use the
                      A1Z_G1Z.urdf model.
        gripper_max_torque: Maximum gripping torque (Nm). Default 2.0 Nm.
                            Passed to Gripper as i_des = max_torque / 11.0.
        inter_cmd_gap_us: Inter-command CAN pacing gap in microseconds,
            inserted before each per-tick command frame after the first. None
            (default) uses the hard-coded product default 250 µs, which frees
            the last-commanded motor's answer slot. Pass an explicit value to
            retune, or 0 to disable pacing (legacy back-to-back burst).
            Range-checked to [0, 500] µs at construction (ValueError
            otherwise). The SDK reads no environment variable for this.
        motor_a_use_new_enable_protocol: If True, use the 0x7FF config-frame
            enable/disable protocol for MotorA (for newer firmware). Default
            False keeps the legacy per-motor 0xFC/0xFD frames.
        bustype: Force a python-can backend (e.g. 'socketcan', 'gs_usb',
                 'pcan'). None auto-detects based on OS.

    Returns:
        Configured ArmRobot instance (call .start() to begin control).
    """
    if urdf_path is not None:
        urdf = urdf_path
    elif with_gripper:
        urdf = _GRIPPER_URDF_PATH
    else:
        urdf = _DEFAULT_URDF_PATH

    # Open CAN bus (auto-detects platform: socketcan on Linux, gs_usb on macOS/Windows)
    bus = open_can_bus(channel=can_channel, bitrate=1_000_000, bustype=bustype)

    # Create MotorA motors
    motor_a_list = [
        MotorA(
            motor_id=mid,
            bus=bus,
            ranges=_MOTOR_A_RANGES,
            use_new_enable_protocol=motor_a_use_new_enable_protocol,
        )
        for mid in _MOTOR_A_IDS
    ]

    # Create MotorB motors with per-joint ranges
    motor_b_ranges_by_joint = {3: _MOTOR_B_RANGES_JOINT3}
    motor_b_list = []
    for i, mid in enumerate(_MOTOR_B_IDS):
        joint_idx = _MOTOR_B_JOINT_INDICES[i]
        ranges = motor_b_ranges_by_joint.get(joint_idx, _MOTOR_B_RANGES_DEFAULT)
        motor_b_list.append(MotorB(motor_id=mid, bus=bus, ranges=ranges))

    # Build motor chain. The chain default stays 0.0 (mechanism-neutral); the
    # pacing policy lives in this factory.
    inter_cmd_gap_s = _resolve_inter_cmd_gap_us(inter_cmd_gap_us) * 1e-6
    motor_chain = MixedMotorChain(
        motor_a_list=motor_a_list,
        motor_b_list=motor_b_list,
        motor_a_joint_indices=_MOTOR_A_JOINT_INDICES,
        motor_b_joint_indices=_MOTOR_B_JOINT_INDICES,
        motor_a_kt=_MOTOR_A_KT,
        inter_cmd_gap_s=inter_cmd_gap_s,
    )

    # Load gravity model
    gravity_model = GravityModel(urdf)

    gripper = None
    if with_gripper:
        gripper_motor = MotorB(motor_id=GRIPPER_CAN_ID, bus=bus, ranges=GRIPPER_MOTOR_RANGES)
        gripper = Gripper(gripper_motor, max_torque=gripper_max_torque)

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
        joint_limit_lower_tolerance_rad=_JOINT_LIMIT_LOWER_TOLERANCE_RAD,
        joint_limit_upper_tolerance_rad=_JOINT_LIMIT_UPPER_TOLERANCE_RAD,
        gripper=gripper,
        control_freq_hz=control_freq_hz,
        min_freq_hz=min_freq_hz,
        motor_a_kt=_MOTOR_A_KT,
        # J6 firmware feedback is intentionally/sometimes sparse on both
        # SocketCAN and userspace gs_usb (observed gaps near 200 ms). Keep
        # J1-J5 strict while allowing that known cadence on the low-torque
        # wrist joint.
        stale_feedback_warn_s=np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.5]),
        stale_feedback_estop_s=np.array([0.2, 0.2, 0.2, 0.2, 0.2, 2.0]),
    )
