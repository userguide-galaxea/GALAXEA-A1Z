import threading
import unittest
from unittest.mock import patch

import numpy as np

from a1z.robots.arm_robot import ArmRobot, JointCommand


class SafeReturnTests(unittest.TestCase):
    def test_gain_ramp_tracks_measured_pose_before_position_control(self):
        robot = ArmRobot.__new__(ArmRobot)
        robot._num_joints = 6
        robot._control_period_s = 0.25
        robot._default_kp = np.array([30.0, 30.0, 30.0, 20.0, 5.0, 5.0])
        robot._default_kd = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
        robot._command_lock = threading.Lock()
        robot._estop_latch = threading.Event()
        robot._command = JointCommand(
            pos=np.zeros(6),
            kp=np.zeros(6),
            kd=robot._default_kd * 0.5,
        )

        measured = np.array([0.4, 0.3, -0.2, 0.1, -0.1, 0.05])
        robot.get_joint_pos = lambda: measured.copy()
        robot._validate_joint_pos = lambda pos: np.asarray(pos, dtype=np.float64)

        ramp_commands = []

        def capture_command(_duration):
            with robot._command_lock:
                ramp_commands.append(
                    (robot._command.pos.copy(), robot._command.kp.copy())
                )

        with patch("a1z.robots.arm_robot.time.sleep", side_effect=capture_command):
            robot.move_joints(
                measured,
                speed=0.3,
                sync_to_measured=True,
                gain_ramp_s=0.5,
            )

        self.assertEqual(len(ramp_commands), 2)
        for pos, _kp in ramp_commands:
            np.testing.assert_allclose(pos, measured)
        self.assertGreater(np.min(ramp_commands[0][1]), 0.0)
        np.testing.assert_allclose(robot._command.pos, measured)
        np.testing.assert_allclose(robot._command.kp, robot._default_kp)


if __name__ == "__main__":
    unittest.main()
