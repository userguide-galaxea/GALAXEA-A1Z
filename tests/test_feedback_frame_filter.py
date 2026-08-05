"""CAN 反馈帧过滤：命令回显 / 非位置帧不得污染位置与错误码。

背景（2026-08 实机抓包定位）：MotorA 的 MIT 命令帧与反馈同 CAN ID，
SocketCAN 回显（is_rx=False）被 _dispatch_feedback 当反馈解，位置被解到
量程端点附近，ROS 层反馈在 0 与 ±12.5 rad 之间逐帧翻转；MotorB 侧
byte0 低半字节非本电机 ID 的帧（如 0x7F）同样被误收并触发伪错误码。

修复约定：
- is_rx=False 的回显帧直接丢弃；
- MotorA 只有报文类型 1 的帧才更新位置（ENCOS 手册 V1.12 §10），
  其余类型只保留错误码（last_reported_error）；
- MotorB byte0 低半字节必须等于本电机 ID（达妙布局 ID | ERR<<4）。
"""

import numpy as np

import can

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
    # 类型 0 + 编码器错误(0x5),payload 是垃圾（按位置解会得到 +5.27 rad）
    fb = motor.parse_feedback(_msg(3, _motor_a_frame(0x0, 0x5, 46591)))
    assert fb is not None
    assert fb.valid_position is False
    assert fb.error == 0x5


def test_motor_b_rejects_id_nibble_mismatch():
    motor = MotorB(motor_id=4, bus=_FakeBus())
    # 0x7F：ID 半字节 0xF 与电机 4 不匹配（实测中的异常/回显帧）
    assert motor.parse_feedback(_msg(4, _motor_b_frame(0xF, 0x7, 65407))) is None


def test_motor_b_accepts_matching_id():
    motor = MotorB(motor_id=4, bus=_FakeBus())
    fb = motor.parse_feedback(_msg(4, _motor_b_frame(0x4, 0x1, 32768)))
    assert fb is not None
    assert abs(fb.position) < 1e-3
    assert fb.error == 0x1


def test_dispatch_drops_echo_frames():
    chain = _make_chain()
    # 回显的 MotorA MIT 命令帧：byte0 高 3 位是 mode=0（报文类型 0）
    echo = _msg(1, _motor_a_frame(0x0, 0x0, 32768), is_rx=False)
    chain._dispatch_feedback(echo)
    assert chain._motor_a_list[0].last_feedback is None


def test_error_report_does_not_corrupt_position():
    chain = _make_chain()
    # 先来一帧正常类型 1 反馈（位置 ≈ 0）
    chain._dispatch_feedback(_msg(1, _motor_a_frame(0x1, 0x0, 32768)))
    # 再来一帧错误上报（payload 垃圾,按位置解是 +5.27 rad）
    chain._dispatch_feedback(_msg(1, _motor_a_frame(0x0, 0x5, 46591)))
    fb = chain._motor_a_list[0].last_feedback
    assert fb is not None and abs(fb.position) < 1e-3
    # 错误码仍然上报
    assert chain.get_error_codes()[0] == 0x5


def test_echo_then_feedback_keeps_position_clean():
    chain = _make_chain()
    # MotorB 命令回显（byte0 低半字节 0xF）混在反馈里
    chain._dispatch_feedback(_msg(4, _motor_b_frame(0xF, 0x7, 65407), is_rx=True))
    chain._dispatch_feedback(_msg(4, _motor_b_frame(0x4, 0x1, 32768)))
    fb = chain._motor_b_list[0].last_feedback
    assert fb is not None and abs(fb.position) < 1e-3
    assert chain.get_error_codes()[3] == 0x1
