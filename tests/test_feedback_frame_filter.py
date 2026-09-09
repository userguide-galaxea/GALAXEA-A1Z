"""CAN feedback frame filtering: command echoes / non-position frames must not
corrupt positions or error codes.

Background (2026-08 on-hardware capture): MotorA MIT command frames share the
CAN ID with feedback, so a looped-back echo (is_rx=False) was decoded by
``_dispatch_feedback`` as feedback — positions landed near the range endpoints
and the feedback flipped between 0 and ±12.5 rad frame by frame. On the MotorB
side, frames whose byte0 low nibble is not the motor's own ID (e.g. 0x7F) were
likewise mis-parsed and triggered fabricated error codes.

Fix contract:
- echo frames (is_rx=False) are dropped outright;
- MotorA only updates position from report-type-1 frames (ENCOS manual
  V1.12 §10); other report types contribute only their error code
  (``last_reported_error``);
- MotorB requires the byte0 low nibble to equal the motor's own ID (DaMiao
  layout ``ID | ERR << 4``).
"""

import numpy as np

import can
from unittest.mock import patch

import a1z.motor_drivers.motor_b_driver as motor_b_driver
from a1z.motor_drivers.motor_a_driver import MotorA
from a1z.motor_drivers.motor_b_driver import MixedMotorChain, MotorB


def _motor_a_frame(report_type, error, pos_raw,
                   vel_raw=2048, curr_raw=2048, t_motor=100, t_mos=100):
    frame = ((report_type & 0x7) << 61) | ((error & 0x1F) << 56) \
        | ((pos_raw & 0xFFFF) << 40) | ((vel_raw & 0xFFF) << 28) \
        | ((curr_raw & 0xFFF) << 16) | ((t_motor & 0xFF) << 8) | (t_mos & 0xFF)
    return frame.to_bytes(8, "big")


def _motor_b_frame(motor_id, err, pos_raw,
                   vel_raw=2048, tor_raw=2048, t_mos=40, t_rotor=40):
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


def _msg(mid, data, is_rx=True):
    return can.Message(arbitration_id=mid, data=data,
                       is_extended_id=False, is_rx=is_rx)


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


def test_motor_a_type1_frame_decodes_position():
    motor = MotorA(motor_id=1, bus=_FakeBus())
    fb = motor.parse_feedback(_msg(1, _motor_a_frame(0x1, 0x0, 32768)))
    assert fb is not None and fb.valid_position
    assert abs(fb.position) < 1e-3
    assert fb.error == 0


def test_motor_a_non_type1_frame_keeps_error_only():
    motor = MotorA(motor_id=3, bus=_FakeBus())
    # Report type 0 + encoder error (0x5); the payload is garbage (decoded as
    # a position it would read +5.27 rad).
    fb = motor.parse_feedback(_msg(3, _motor_a_frame(0x0, 0x5, 46591)))
    assert fb is not None
    assert fb.valid_position is False
    assert fb.error == 0x5


def test_motor_b_rejects_id_nibble_mismatch():
    motor = MotorB(motor_id=4, bus=_FakeBus())
    # 0xF ID nibble does not match motor 4 (anomalous/echo frame seen on wire).
    assert motor.parse_feedback(_msg(4, _motor_b_frame(0xF, 0x7, 65407))) is None


def test_motor_b_accepts_matching_id():
    motor = MotorB(motor_id=4, bus=_FakeBus())
    fb = motor.parse_feedback(_msg(4, _motor_b_frame(0x4, 0x1, 32768)))
    assert fb is not None
    assert abs(fb.position) < 1e-3
    assert fb.error == 0x1


def test_dispatch_drops_echo_frames():
    chain = _make_chain()
    # Echoed MotorA MIT command frame: byte0 top 3 bits are mode=0 (report
    # type 0).
    echo = _msg(1, _motor_a_frame(0x0, 0x0, 32768), is_rx=False)
    chain._dispatch_feedback(echo)
    assert chain._motor_a_list[0].last_feedback is None


def test_error_report_does_not_corrupt_position():
    chain = _make_chain()
    # A normal report-type-1 frame first (position ≈ 0).
    chain._dispatch_feedback(_msg(1, _motor_a_frame(0x1, 0x0, 32768)))
    # Then an error report (garbage payload, decodes as +5.27 rad).
    chain._dispatch_feedback(_msg(1, _motor_a_frame(0x0, 0x5, 46591)))
    fb = chain._motor_a_list[0].last_feedback
    assert fb is not None and abs(fb.position) < 1e-3
    # The error code is still reported.
    assert chain.get_error_codes()[0] == 0x5


def test_echo_then_feedback_keeps_position_clean():
    chain = _make_chain()
    # MotorB command echo (byte0 low nibble 0xF) mixed into the feedback.
    chain._dispatch_feedback(_msg(4, _motor_b_frame(0xF, 0x7, 65407), is_rx=True))
    chain._dispatch_feedback(_msg(4, _motor_b_frame(0x4, 0x1, 32768)))
    fb = chain._motor_b_list[0].last_feedback
    assert fb is not None and abs(fb.position) < 1e-3
    assert chain.get_error_codes()[3] == 0x1


def test_dispatch_preserves_kernel_receive_time_and_queue_delay():
    chain = _make_chain()
    msg = _msg(4, _motor_b_frame(0x4, 0x1, 32768))
    msg.timestamp = 99.750

    with (
        patch.object(motor_b_driver.time, "time", return_value=100.0),
        patch.object(
            motor_b_driver.time,
            "monotonic",
            return_value=200.0,
        ),
    ):
        chain._dispatch_feedback(msg)

    source, dispatch, queue_delay = chain.get_feedback_timing()
    assert np.isclose(source[3], 199.750)
    assert np.isclose(dispatch[3], 200.0)
    assert np.isclose(queue_delay[3], 0.250)
    assert np.all(np.isnan(source[:3]))
