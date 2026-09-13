import sys
from pathlib import Path
from unittest.mock import MagicMock
import unittest
from moveit_msgs.msg import PlanningScene
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'core'))
from grasp_recovery import checked_base_motion


class BaseRecoveryTest(unittest.TestCase):
    def test_collision_anywhere_in_sweep_prevents_execution(self):
        scene = PlanningScene()
        js = scene.robot_state.joint_state
        js.name = ['odom_x', 'odom_y', 'odom_t', 'hand_motor_joint']
        js.position = [0., 0., 0., 1.1]
        pick = MagicMock()
        pick.scene.return_value = scene
        good = MagicMock(valid=True)
        bad = MagicMock(valid=False, contacts=[])
        pick.call.side_effect = [good]*100 + [bad]
        with self.assertRaisesRegex(RuntimeError, 'Base recovery collision'):
            checked_base_motion(pick, [-.15, .10, 0.])
        pick.execute.assert_not_called()

    def test_excessive_displacement_prevents_execution(self):
        pick = MagicMock()
        with self.assertRaisesRegex(RuntimeError, 'limits'):
            checked_base_motion(pick, [.3, 0., 0.])
        pick.execute.assert_not_called()
        pick.scene.assert_not_called()
