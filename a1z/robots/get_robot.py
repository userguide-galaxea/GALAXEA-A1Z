"""Factory function for creating an A1Z ArmRobot."""

from pathlib import Path
from typing import Optional

import can
import numpy as np

from a1z.dynamics.gravity_model import GravityModel
from a1z.motor_drivers.motor_b_driver import MotorB, MotorBRanges, MixedMotorChain
from a1z.motor_drivers.motor_a_driver import MotorA, MotorARanges
from a1z.robots.arm_robot import ArmRobot
from a1z.robots.gripper import Gripper, GRIPPER_CAN_ID, GRIPPER_MOTOR_RANGES
from a1z.robots.integrator import IntegralConfig

# Default URDF path (bundled inside the package)
_DEFAULT_URDF_PATH = str(Path(__file__).parent.parent / "robot_models" / "a1z" / "A1Z_G1Z.urdf")

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

# Physical limits remain identical to the URDF.  Every joint boundary carries a
# 0.05 rad calibration/noise margin: commands within the margin of a limit are
# accepted but clipped onto it (feedback noise, arms resting at the boundary —
# e.g. J2 resting slightly below 0, J3 slightly above 0).  Commands beyond the
# margin are still rejected outright.
_JOINT_LIMIT_LOWER_TOLERANCE_RAD = np.full(_NUM_JOINTS, 0.05)
_JOINT_LIMIT_UPPER_TOLERANCE_RAD = np.full(_NUM_JOINTS, 0.05)

# _DEFAULT_KP = np.array([100.0, 60.0, 40.0, 120.0, 10.0, 25.0])
# _DEFAULT_KD = np.array([4.9,  4.5,  5.0,  2.0,  0.5,  4])


_DEFAULT_KP = np.array([146.9, 62.95, 89.24, 120.0, 40.0, 100.0])
_DEFAULT_KD = np.array([5.0, 5.0, 5.0, 2.078, 1.506, 1.255])


_JOINT_SIGN = np.array([1.0, 1.0, -1.0, 1.0, -1.0, 1.0])
_GRAVITY_TORQUE_SCALE = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
_MAX_GRAVITY_TORQUE = np.array([50.0, 50.0, 50.0, 24.0, 10.0, 10.0])
_TORQUE_CLIP = np.array([70.0, 70.0, 70.0, 27.0, 10.0, 10.0])

# Per-joint Coulomb-friction estimate τ̂_c (Nm), the calibration anchor for the
# error-integral clamp τ_I,max = 1.2·τ̂_c (SOP-09 §3 / P0-6). NaN joints are
# unstandardised → IntegralConfig.from_level auto-disables them (enable-mask
# implementation).
#
# J1/J2/J3/J5 backfilled from the G0E multi-rate regression (SOP-09 §10.2,
# devlog 2026-07-24): per-joint clean 3-rate triangle sets (p2/p4/p8, default PD,
# gap 250 µs), regressed via regress_tau_c.py (R²: J1 0.97, J2 0.97, J3 0.89,
# J5 0.66). J6 ≈ 0.13 Nm is measured (SOP-05 §6 / devlog 2026-07-21 §4-2), with
# the G0E-口径 regression (0.1125) as confirmation — left at the measured 0.13.
#
# J4 = 0.66 is a LOW-SPEED (Stribeck-regime) anchor, not the linear intercept.
# Its 4-rate sweep (+p16) is monotonic in the WRONG direction for the Coulomb+
# viscous model — friction RISES as speed falls (β_v<0, R²=0.66): the sweep sits
# on the Stribeck downslope, so the linear intercept (0.646) over-extrapolates.
# The integral only acts below qd_freeze=0.15 rad/s, exactly the p8/p16 regime
# where the direction-antisymmetric friction is measured directly at 0.62–0.66;
# 0.66 is that measured low-speed value. The pipeline uses τ̂_c only as a scalar
# magnitude (coulomb_ff bound 1.5·τ̂_c, τ_I,max clamp), never β_v, and explicitly
# does not need a Stribeck curve (devlog 2026-07-22 Q8(3)) — so a magnitude-correct
# anchor is sufficient. Enable per-joint via a P0-7-style smoke test (SOP-09 §10.2).
_TAU_C_HAT = np.array([1.033, 0.3665, 0.6371, 0.66, 0.2355, 0.2925])


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

# CAN command pacing (SOP-05 / SOP-06). The per-tick command burst is spaced so
# the last-commanded motor's answer slot is never occupied by its predecessor's
# answer, fixing the J6 feedback/target-latch starvation (staircase fault).
_DEFAULT_INTER_CMD_GAP_US = 250.0   # validated: SOP-05 §6.4 (wire-verified, 0 missed answers)
_MAX_INTER_CMD_GAP_US = 500.0       # 6 paced boundaries × worst-case sleep must stay < 4 ms tick


def _resolve_inter_cmd_gap_us(param: Optional[float]) -> float:
    """Resolve the inter-command pacing gap in µs.

    ``param`` (the ``inter_cmd_gap_us`` argument) when not None, else the
    hard-coded 250 µs product default. Range-checked at construction (fail-fast)
    so a misconfigured gap refuses to build the robot rather than degrading
    mid-run. The gap is hard-coded/parameter-only — the SDK reads no environment
    variable; to retune or disable pacing, pass ``inter_cmd_gap_us`` (``0`` =
    legacy back-to-back burst).
    """
    gap = param if param is not None else _DEFAULT_INTER_CMD_GAP_US
    if not (0.0 <= gap <= _MAX_INTER_CMD_GAP_US):
        raise ValueError(
            f"inter_cmd_gap_us={gap} outside [0, {_MAX_INTER_CMD_GAP_US}] µs "
            f"(see SOP-06 §3 timing budget)")
    return gap


def get_a1z_robot(
    can_channel: str = "can0",
    transport: str = "socketcan",
    arm_side: str = "left",
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
    integral_level: str = "K1",
    integral_joints: Optional[list] = None,
    integral_overrides: Optional[dict] = None,
    coulomb_ff: Optional[np.ndarray] = None,
) -> ArmRobot:
    """Create and return a configured A1Z ArmRobot.

    Args:
        can_channel: CAN interface name (e.g. 'can0'). SocketCAN transport only.
        transport: Low-level command transport:
                   "socketcan" (default, direct CAN via python-can);
                   "g4ros" (lemo main board, via the embedded g4spi_node's
                   ROS2 topics — requires rclpy, the lemo_main_board message
                   package, and the node running; keeps leader-arm topics
                   available). With "g4ros" every CAN frame is forwarded by
                   its ID (frame-wise firmware protocol), including the
                   0x7FF management broadcasts (MotorA enable/disable/
                   set-zero, gripper mode-4 register write).
        arm_side: g4ros transport only: 'left' or 'right' arm
                  (the left_*/right_* topic prefix).
        gravity_comp_factor: Gravity compensation scale (0=off, 1=full).
        zero_gravity_mode: True for zero-gravity (floating) mode, False for
                           position hold with PD + gravity comp.
        control_freq_hz: Control loop frequency in Hz.
        min_freq_hz: Minimum acceptable control frequency. Emergency stop if
                     frequency stays below this for 3 consecutive check periods.
        urdf_path: Override URDF path.
        default_kp: Override default position gains.
        default_kd: Override default velocity gains.
        with_gripper: If True, attach a Gripper at CAN ID 0x07.
        gripper_max_torque: Maximum gripping torque (Nm). Default 2.0 Nm.
                            Passed to Gripper as i_des = max_torque / 11.0.
        inter_cmd_gap_us: Inter-command CAN pacing gap in microseconds, inserted
                          before each per-tick command frame after the first
                          (SOP-05/SOP-06). None (default) uses the hard-coded
                          product default 250 µs, which frees the last-commanded
                          motor's answer slot and fixes the J6 feedback/target-latch
                          starvation. Pass an explicit value to retune, or ``0`` to
                          disable pacing (legacy back-to-back burst). Range-checked
                          to [0, 500] µs at construction (ValueError otherwise). The
                          SDK reads no environment variable for this.
        integral_level: Error-integral feedforward level (SOP-09 §3):
                          "K0" (ki=0 — no integrator) / "K1" (default, gentle
                          t_wind≈2s) / "K2" / "K3". τ̂_c anchor from _TAU_C_HAT.
        integral_joints: 1-based joint list to enable (e.g. [6] or [4,5,6]); None =
                          all calibrated joints. Uncalibrated (NaN τ̂_c) joints stay
                          disabled regardless.
        integral_overrides: Optional dict forwarded to IntegralConfig.from_level
                          (keys: t_leak_s, e_db_deg, qd_freeze, t_wind_s,
                          clamp_scale).
        coulomb_ff: Per-joint Coulomb friction feedforward amplitude (Nm),
                          shape (6,). Applied as sign(e)·coulomb_ff in the torque
                          sum (S1 strategy layer). None = disabled (default).

    Returns:
        Configured ArmRobot instance (call .start() to begin control).
    """
    urdf = urdf_path or _DEFAULT_URDF_PATH

    # Error-integral feedforward (SOP-09). K0 → None so the default path builds
    # no integrator and the control loop stays byte-for-byte identical to PD +
    # gravity comp; any active level constructs a per-joint leaky integrator.
    integral_config: Optional[IntegralConfig] = None
    if integral_level != "K0":
        integral_config = IntegralConfig.from_level(
            integral_level,
            _TAU_C_HAT,
            joints=integral_joints,
            **(integral_overrides or {}),
        )

    # Open command transport
    if transport == "g4ros":
        from a1z.motor_drivers.ros_topic_bus import RosTopicBus

        bus = RosTopicBus(arm_side=arm_side)
    elif transport == "socketcan":
        bus = can.interface.Bus(
            channel=can_channel,
            bustype="socketcan",
            bitrate=1_000_000,
        )
    else:
        raise ValueError(
            f"transport must be 'socketcan' or 'g4ros', got {transport!r}"
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
    #
    # SOP-05/SOP-06 CAN command pacing (default-on, hard-coded default 250 µs).
    #   inter_cmd_gap_us param : microseconds slept before each per-tick command
    #     frame after the first (see _resolve_inter_cmd_gap_us). None uses the
    #     250 µs default; a gap >= ~150 us frees the last-commanded motor's answer
    #     slot, fixing the J6 feedback/target-latch starvation (SOP-05 / devlog
    #     2026-07-21). Pass 0 to disable (legacy burst). No env var is read.
    _inter_cmd_gap_s = _resolve_inter_cmd_gap_us(inter_cmd_gap_us) * 1e-6
    motor_chain = MixedMotorChain(
        motor_a_list=motor_a_list,
        motor_b_list=motor_b_list,
        motor_a_joint_indices=_MOTOR_A_JOINT_INDICES,
        motor_b_joint_indices=_MOTOR_B_JOINT_INDICES,
        motor_a_kt=_MOTOR_A_KT,
        inter_cmd_gap_s=_inter_cmd_gap_s,
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
        integral_config=integral_config,
        coulomb_ff=coulomb_ff,
    )
