"""Shared test scaffolding for the A1Z SDK offline test suite.

The helpers below consolidate the test doubles that were previously copied
per-file (silent CAN bus, mixed MotorA/MotorB chain, protocol frame builders)
and the "command-entry robot" construction used by the clip/clear-semantics
tests. Existing test files keep their local copies untouched; new tests
should import these helpers or request the fixtures instead of re-rolling
their own doubles.

Everything here runs fully offline: no CAN interface is opened, no control
thread is started, no hardware is touched.

Usage (plain imports work in both pytest-style and unittest-style files)::

    from tests.conftest import FakeBus, make_mixed_chain, motor_a_frame, can_msg

or as pytest fixtures::

    def test_something(fake_bus, mixed_chain, command_robot):
        ...

See tests/README.md for the full conventions.
"""

import can
import numpy as np
import pytest

from a1z.motor_drivers.motor_a_driver import MotorA
from a1z.motor_drivers.motor_b_driver import MixedMotorChain, MotorB
from a1z.robots.arm_robot import ArmRobot, ControlState


# ---------------------------------------------------------------------------
# CAN bus double
# ---------------------------------------------------------------------------
class FakeBus:
    """Silent CAN bus double: accepts every send, never delivers feedback."""

    def send(self, _msg, timeout=None):
        del timeout

    def recv(self, timeout=0.0):
        del timeout
        return None


# ---------------------------------------------------------------------------
# Protocol frame builders (ENCOS MotorA / DaMiao MotorB feedback layouts)
# ---------------------------------------------------------------------------
def motor_a_frame(report_type, error, pos_raw,
                  vel_raw=2048, curr_raw=2048, t_motor=100, t_mos=100):
    """Pack a MotorA (ENCOS) 8-byte feedback payload (report type 1 = position)."""
    frame = ((report_type & 0x7) << 61) | ((error & 0x1F) << 56) \
        | ((pos_raw & 0xFFFF) << 40) | ((vel_raw & 0xFFF) << 28) \
        | ((curr_raw & 0xFFF) << 16) | ((t_motor & 0xFF) << 8) | (t_mos & 0xFF)
    return frame.to_bytes(8, "big")


def motor_b_frame(motor_id, err, pos_raw,
                  vel_raw=2048, tor_raw=2048, t_mos=40, t_rotor=40):
    """Pack a MotorB (DaMiao) 8-byte feedback payload (byte0 = ERR<<4 | ID)."""
    data = bytearray(8)
    data[0] = ((err & 0xF) << 4) | (motor_id & 0xF)
    data[1] = (pos_raw >> 8) & 0xFF
    data[2] = pos_raw & 0xFF
    data[3] = (vel_raw >> 4) & 0xFF
    data[4] = ((vel_raw & 0xF) << 4) | ((tor_raw >> 8) & 0xF)
    data[5] = tor_raw & 0xFF
    data[6] = t_mos
    data[7] = t_rotor
    return bytes(data)


def can_msg(mid, data, is_rx=True):
    """Build a python-can Message; ``is_rx=False`` marks a command echo frame."""
    return can.Message(arbitration_id=mid, data=data,
                       is_extended_id=False, is_rx=is_rx)


# ---------------------------------------------------------------------------
# Motor-chain factory (6-joint layout: joints 0-2 MotorA, joints 3-5 MotorB)
# ---------------------------------------------------------------------------
def make_mixed_chain(bus=None, inter_cmd_gap_s=0.0):
    """Build a 3+3 MixedMotorChain on a silent bus without touching hardware."""
    bus = bus if bus is not None else FakeBus()
    return MixedMotorChain(
        motor_a_list=[MotorA(motor_id=i, bus=bus) for i in (1, 2, 3)],
        motor_b_list=[MotorB(motor_id=i, bus=bus) for i in (4, 5, 6)],
        motor_a_joint_indices=[0, 1, 2],
        motor_b_joint_indices=[3, 4, 5],
        inter_cmd_gap_s=inter_cmd_gap_s,
    )


# ---------------------------------------------------------------------------
# Command-entry robot factory (no chain / bus / gravity model, no threads)
# ---------------------------------------------------------------------------
DEFAULT_JOINT_LIMITS = [
    (-2.094, 2.094),   # J1
    (0.0, 3.142),      # J2
    (-3.142, 0.0),     # J3
    (-1.484, 1.484),   # J4
    (-1.484, 1.484),   # J5
    (-2.007, 2.007),   # J6
]
DEFAULT_LIMIT_TOLERANCE = np.full(6, 0.05)


def make_command_robot(**overrides):
    """Build an ArmRobot for command-entry / clip tests (no hardware paths).

    The command entry points touch neither the motor chain nor the bus, so
    they are left as None. The command gate is opened (``_commands_blocked``
    cleared, state forced to RUNNING) so isolated boundary checks can run
    without a startup sequence.
    """
    kwargs = dict(
        motor_chain=None,
        bus=None,
        gravity_model=None,
        joint_limits=DEFAULT_JOINT_LIMITS,
        joint_limit_lower_tolerance_rad=DEFAULT_LIMIT_TOLERANCE,
        joint_limit_upper_tolerance_rad=DEFAULT_LIMIT_TOLERANCE,
    )
    kwargs.update(overrides)
    robot = ArmRobot(**kwargs)
    robot._commands_blocked.clear()
    robot._control_state = ControlState.RUNNING
    return robot


# ---------------------------------------------------------------------------
# pytest fixtures (thin wrappers over the helpers above)
# ---------------------------------------------------------------------------
@pytest.fixture
def fake_bus():
    return FakeBus()


@pytest.fixture
def mixed_chain():
    return make_mixed_chain()


@pytest.fixture
def command_robot():
    return make_command_robot()
