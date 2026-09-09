"""Feedback-safety and disable-confirmation tests (P4 rewrite).

P4 replaces the emergency-disable fault policy with the safety control state
machine: stale feedback is a RecoverableControlFault (recoverable MIT hold),
and the old ``_disable_motors_and_flush`` helper is gone in favor of
``_disable_outputs()`` error-list semantics. The two chain-level tests that
do not depend on the arm-side state machine are kept verbatim.
"""

import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import can
import numpy as np

from a1z.motor_drivers.motor_b_driver import MixedMotorChain
from a1z.robots.arm_robot import (
    ArmRobot,
    ControlState,
    RecoverableControlFault,
)


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
        # P4: stale feedback is a recoverable communication fault, not a
        # plain RuntimeError — the control loop answers it with a cached MIT
        # hold instead of an emergency disable (V6).
        robot = ArmRobot.__new__(ArmRobot)
        robot._num_joints = 2
        robot._stale_warn_s = np.array([0.05, 0.25])
        robot._stale_estop_s = np.array([0.2, 0.5])
        robot._last_stale_warn_t = 0.0
        robot._last_feedback_t = time.monotonic()
        robot._control_started_t = time.monotonic()
        robot._motor_chain = SimpleNamespace(
            get_feedback_health=lambda now=None: (
                np.array([True, True]),
                np.array([0.25, 0.25]),
            )
        )

        with self.assertRaisesRegex(
            RecoverableControlFault, "joint1=250ms/200ms"
        ) as ctx:
            robot._check_feedback_stale()
        self.assertEqual(ctx.exception.fault_code, "CAN_FEEDBACK_STALE")

    def test_startup_failure_disables_outputs_and_lands_stopped(self):
        # Replaces the old flush-based startup test: the failure path is
        # expressed via _disable_outputs() plus a STOPPED state landing.
        chain = SimpleNamespace(
            enable_all=Mock(side_effect=RuntimeError("startup failed")),
            disable_all=Mock(return_value=True),
            reset_feedback_health=Mock(),
        )
        robot = ArmRobot(
            motor_chain=chain,
            bus=SimpleNamespace(),
            gravity_model=Mock(),
        )

        with self.assertRaisesRegex(RuntimeError, "startup failed"):
            robot.start()

        chain.disable_all.assert_called_once_with()
        self.assertTrue(robot._stop_event.is_set())
        self.assertFalse(robot.is_running)
        # Commands stay blocked after a failed start, but a clean startup
        # failure (no stuck thread, confirmed disable) remains restartable.
        self.assertTrue(robot.is_estopped)
        status = robot.get_fault_status()
        self.assertEqual(status["state"], "STOPPED")
        self.assertTrue(status["restart_allowed"])

    def test_disable_outputs_collects_arm_and_gripper_failures(self):
        # Replaces the old flush-based disable test: failures surface as
        # _disable_outputs() error-list entries and never abort the remaining
        # outputs.
        chain = SimpleNamespace(
            disable_all=Mock(side_effect=RuntimeError("arm bus down"))
        )
        gripper = SimpleNamespace(
            disable=Mock(side_effect=RuntimeError("gripper bus down"))
        )
        robot = ArmRobot(
            motor_chain=chain,
            bus=SimpleNamespace(),
            gravity_model=Mock(),
            gripper=gripper,
        )

        errors = robot._disable_outputs()

        self.assertEqual(
            errors, ["arm: arm bus down", "gripper: gripper bus down"]
        )

    def test_disable_outputs_reports_bool_false_from_chain(self):
        # MixedMotorChain keeps its GitHub bool contract (P1 deviation): a
        # False return without an exception still means unconfirmed disable.
        chain = SimpleNamespace(disable_all=Mock(return_value=False))
        robot = ArmRobot(
            motor_chain=chain,
            bus=SimpleNamespace(),
            gravity_model=Mock(),
        )

        errors = robot._disable_outputs()

        self.assertEqual(len(errors), 1)
        self.assertTrue(errors[0].startswith("arm:"))

    def test_stop_with_unconfirmed_disable_blocks_restart(self):
        # Replaces the old flush-timeout test: an unconfirmed disable during
        # stop() lands HARD_DISABLE_UNCONFIRMED and forbids restart (V2).
        chain = SimpleNamespace(
            disable_all=Mock(return_value=False),
            reset_feedback_health=Mock(),
        )
        robot = ArmRobot(
            motor_chain=chain,
            bus=SimpleNamespace(),
            gravity_model=Mock(),
        )
        robot._running = True  # control thread already gone; safety-net path

        robot.stop()

        status = robot.get_fault_status()
        self.assertEqual(status["state"], "HARD_DISABLE_UNCONFIRMED")
        self.assertEqual(status["code"], "STOP_DISABLE_UNCONFIRMED")
        self.assertFalse(status["restart_allowed"])
        with self.assertRaisesRegex(RuntimeError, "requires STOPPED"):
            robot.start()

    def test_stop_disables_outputs_and_lands_stopped(self):
        # C3: stop() disables every output through the main-thread safety net
        # even when no control thread exists, then lands STOPPED.
        chain = SimpleNamespace(
            disable_all=Mock(return_value=True),
            reset_feedback_health=Mock(),
        )
        robot = ArmRobot(
            motor_chain=chain,
            bus=SimpleNamespace(),
            gravity_model=Mock(),
        )
        robot._running = True

        robot.stop()

        chain.disable_all.assert_called_once_with()
        status = robot.get_fault_status()
        self.assertEqual(status["state"], "STOPPED")
        self.assertTrue(status["restart_allowed"])

    def test_stop_with_flush_timeout_lands_unconfirmed(self):
        # stop() flushes the bus TX queue after the safety-net disable; a
        # flush timeout is an unconfirmed disable (V2), matching the old
        # test_flush_timeout_makes_disable_unconfirmed coverage intent.
        chain = SimpleNamespace(
            disable_all=Mock(return_value=True),
            reset_feedback_health=Mock(),
        )
        bus = SimpleNamespace(flush_tx=Mock(return_value=False))
        robot = ArmRobot(
            motor_chain=chain,
            bus=bus,
            gravity_model=Mock(),
        )
        robot._running = True  # control thread already gone; safety-net path

        robot.stop()

        bus.flush_tx.assert_called_once_with(timeout=0.5)
        status = robot.get_fault_status()
        self.assertEqual(status["state"], "HARD_DISABLE_UNCONFIRMED")
        self.assertEqual(status["code"], "STOP_DISABLE_UNCONFIRMED")
        self.assertFalse(status["restart_allowed"])
        with self.assertRaisesRegex(RuntimeError, "requires STOPPED"):
            robot.start()

    def test_stop_with_successful_flush_lands_stopped(self):
        # A confirmed disable plus a drained TX queue lands STOPPED and keeps
        # the object restartable.
        chain = SimpleNamespace(
            disable_all=Mock(return_value=True),
            reset_feedback_health=Mock(),
        )
        bus = SimpleNamespace(flush_tx=Mock(return_value=True))
        robot = ArmRobot(
            motor_chain=chain,
            bus=bus,
            gravity_model=Mock(),
        )
        robot._running = True

        robot.stop()

        chain.disable_all.assert_called_once_with()
        bus.flush_tx.assert_called_once_with(timeout=0.5)
        status = robot.get_fault_status()
        self.assertEqual(status["state"], "STOPPED")
        self.assertTrue(status["restart_allowed"])

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
