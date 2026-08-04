import time
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import can
import numpy as np

from a1z.motor_drivers.motor_b_driver import MixedMotorChain
from a1z.robots.arm_robot import ArmRobot


class _FakeMotor:
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.last_feedback = None

    def parse_feedback(self, msg):
        if len(msg.data) != 8:
            return None
        return SimpleNamespace(
            position=0.1,
            velocity=0.2,
            current=0.3,
            torque=0.4,
            error=1,
            temperature=25.0,
            temperature_mos=26.0,
            temperature_rotor=27.0,
        )


class FeedbackSafetyTests(unittest.TestCase):
    def test_chain_only_counts_successfully_parsed_feedback(self):
        motor = _FakeMotor(1)
        chain = MixedMotorChain([motor], [], [0], [], motor_a_kt=2.8)

        self.assertFalse(
            chain._dispatch_feedback(
                can.Message(arbitration_id=99, data=[0] * 8, is_extended_id=False)
            )
        )
        self.assertTrue(np.isinf(chain.get_feedback_ages()[0]))

        self.assertFalse(
            chain._dispatch_feedback(
                can.Message(arbitration_id=1, data=[0], is_extended_id=False)
            )
        )
        self.assertTrue(np.isinf(chain.get_feedback_ages()[0]))

        self.assertTrue(
            chain._dispatch_feedback(
                can.Message(arbitration_id=1, data=[0] * 8, is_extended_id=False)
            )
        )
        self.assertLess(chain.get_feedback_ages()[0], 0.05)

    def test_single_stale_joint_trips_safety(self):
        robot = ArmRobot.__new__(ArmRobot)
        robot._num_joints = 2
        robot._stale_warn_s = np.array([0.05, 0.25])
        robot._stale_estop_s = np.array([0.2, 0.5])
        robot._last_stale_warn_t = 0.0
        robot._last_feedback_t = time.monotonic()
        robot._running = False
        robot._motor_chain = SimpleNamespace(
            get_feedback_ages=lambda: np.array([0.25, 0.25])
        )

        with self.assertRaisesRegex(RuntimeError, "joint1=250ms/200ms"):
            robot._check_feedback_stale()

    def test_startup_failure_disables_and_flushes_before_reraising(self):
        robot = ArmRobot.__new__(ArmRobot)
        robot._stop_event = threading.Event()
        robot._running = False
        robot._motor_chain = SimpleNamespace(disable_all=Mock())
        robot._bus = SimpleNamespace(flush_tx=Mock(return_value=True))
        robot._start_impl = Mock(side_effect=RuntimeError("startup failed"))

        with self.assertRaisesRegex(RuntimeError, "startup failed"):
            robot.start()

        self.assertTrue(robot._stop_event.is_set())
        robot._motor_chain.disable_all.assert_called_once_with()
        robot._bus.flush_tx.assert_called_once_with(timeout=0.5)

    def test_disable_failure_is_reported_even_when_flush_succeeds(self):
        robot = ArmRobot.__new__(ArmRobot)
        robot._motor_chain = SimpleNamespace(disable_all=Mock(return_value=False))
        robot._bus = SimpleNamespace(flush_tx=Mock(return_value=True))

        self.assertFalse(robot._disable_motors_and_flush())
        robot._bus.flush_tx.assert_called_once_with(timeout=0.5)

    def test_flush_timeout_makes_disable_unconfirmed(self):
        robot = ArmRobot.__new__(ArmRobot)
        robot._motor_chain = SimpleNamespace(disable_all=Mock(return_value=True))
        robot._bus = SimpleNamespace(flush_tx=Mock(return_value=False))

        self.assertFalse(robot._disable_motors_and_flush())

    def test_chain_attempts_every_disable_and_reports_failure(self):
        good = Mock(motor_id=1)
        bad = Mock(motor_id=2)
        bad.disable.side_effect = RuntimeError("send failed")
        chain = MixedMotorChain([bad], [good], [0], [1])

        with self.assertLogs("a1z.motor_drivers.motor_b_driver", level="ERROR"):
            result = chain.disable_all()

        self.assertFalse(result)
        self.assertEqual(bad.disable.call_count, 2)
        self.assertEqual(good.disable.call_count, 2)


if __name__ == "__main__":
    unittest.main()
