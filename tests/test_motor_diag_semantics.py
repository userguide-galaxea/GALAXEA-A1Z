"""Regression coverage for protocol-aware diagnostic output."""

from tools.motor_diag import (
    describe_motor_error,
    make_disable_message,
    make_enable_message,
    motor_error_is_fault,
)


def test_diag_classifies_motor_a_and_motor_b_zero_differently():
    assert motor_error_is_fault("MOTOR_A", 0x0) is False
    assert motor_error_is_fault("MotorB4310", 0x0) is True
    assert "no error" in describe_motor_error("MOTOR_A", 0x0)
    assert "disabled" in describe_motor_error("MotorB4310", 0x0)


def test_diag_does_not_hide_motor_a_over_temperature():
    assert motor_error_is_fault("MOTOR_A", 0x1) is True
    assert "over temperature" in describe_motor_error("MOTOR_A", 0x1)


def test_diag_uses_protocol_specific_enable_disable_frames():
    motor_a_enable = make_enable_message("MOTOR_A", 0x01)
    assert motor_a_enable.arbitration_id == 0x7FF
    assert bytes(motor_a_enable.data) == bytes([0x00, 0x01, 0x00, 0x01])

    motor_a_disable = make_disable_message("MOTOR_A", 0x01)
    assert motor_a_disable.arbitration_id == 0x7FF
    assert bytes(motor_a_disable.data) == bytes([0x00, 0x01, 0x00, 0x02])

    motor_b_enable = make_enable_message("MotorB4310", 0x04)
    assert motor_b_enable.arbitration_id == 0x04
    assert bytes(motor_b_enable.data) == bytes([0xFF] * 7 + [0xFC])

    motor_b_disable = make_disable_message("MotorB4310", 0x04)
    assert motor_b_disable.arbitration_id == 0x04
    assert bytes(motor_b_disable.data) == bytes([0xFF] * 7 + [0xFD])
