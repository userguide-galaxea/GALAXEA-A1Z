"""Protocol-aware motor error/status semantics.

ENCOS/MotorA feedback exposes an error-information field where 0 means
no-error. DaMiao/MotorB feedback exposes an enable/fault status where 0 means
disabled and 1 means enabled. These raw values must never be interpreted with
one shared lookup table.
"""

import numpy as np
import pytest

from a1z.motor_drivers.motor_a_driver import MotorA
from a1z.motor_drivers.motor_b_driver import MixedMotorChain, MotorB


class _FakeBus:
    def send(self, _msg, timeout=None):
        del timeout

    def recv(self, timeout=0.0):
        del timeout
        return None


def _make_chain():
    bus = _FakeBus()
    return MixedMotorChain(
        motor_a_list=[MotorA(motor_id=i, bus=bus) for i in (1, 2, 3)],
        motor_b_list=[MotorB(motor_id=i, bus=bus) for i in (4, 5, 6)],
        motor_a_joint_indices=[0, 1, 2],
        motor_b_joint_indices=[3, 4, 5],
    )


def test_motor_a_zero_means_no_error():
    assert MotorA.error_is_fault(0x0) is False
    assert "no error" in MotorA.describe_error(0x0)


@pytest.mark.parametrize("code", range(1, 8))
def test_motor_a_nonzero_error_information_is_fault(code):
    assert MotorA.error_is_fault(code) is True


def test_motor_b_one_is_enabled_and_zero_is_disabled():
    assert MotorB.error_is_fault(0x1) is False
    assert MotorB.error_is_fault(0x0) is True
    assert "disabled" in MotorB.describe_error(0x0)


def test_mixed_chain_classifies_each_joint_by_protocol():
    chain = _make_chain()

    healthy = np.array([0, 0, 0, 1, 1, 1])
    assert not chain.classify_error_codes(healthy).any()

    motor_a_over_current = healthy.copy()
    motor_a_over_current[0] = 0x2
    assert chain.classify_error_codes(motor_a_over_current).tolist() == [
        True,
        False,
        False,
        False,
        False,
        False,
    ]
    assert "over current" in chain.describe_error_code(0, 0x2)

    motor_b_disabled = healthy.copy()
    motor_b_disabled[3] = 0x0
    assert chain.classify_error_codes(motor_b_disabled).tolist() == [
        False,
        False,
        False,
        True,
        False,
        False,
    ]
    assert "disabled" in chain.describe_error_code(3, 0x0)


def test_mixed_chain_rejects_wrong_error_vector_shape():
    with pytest.raises(ValueError, match="error code shape"):
        _make_chain().classify_error_codes(np.zeros(5, dtype=int))
