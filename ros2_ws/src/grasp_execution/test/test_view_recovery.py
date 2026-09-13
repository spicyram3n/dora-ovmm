import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
import numpy as np
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'core'))
from finish_pick import FinishPick


class ViewRecoveryTest(unittest.TestCase):
    def test_invalid_view_and_head_collision_never_execute(self):
        pick = FinishPick(None, {}, Path('/tmp'))
        state = RobotState()
        state.joint_state.name = ['hand_motor_joint', 'head_pan_joint', 'head_tilt_joint']
        state.joint_state.position = [1.1, 0., -.7]
        pick.scene = lambda: SimpleNamespace(robot_state=state)
        pick.call = lambda *args: GetStateValidity.Response(valid=False)
        pick.execute = MagicMock()
        with patch('finish_pick.grab_rgbd', return_value=(None,)*4), \
                patch('finish_pick.sam3_client.detect', return_value=(None, .98)), \
                patch('finish_pick.pointcloud.object_depth_mask'), \
                patch('finish_pick.pointcloud.deproject'), \
                patch('finish_pick.pointcloud.transform_points', return_value=np.zeros((100,3))), \
                self.assertRaisesRegex(RuntimeError, 'view recovery path is in collision'):
            pick.observe_can()
        pick.execute.assert_not_called()

    def test_good_initial_view_needs_no_head_motion(self):
        pick = FinishPick(None, {}, Path('/tmp'))
        pick.execute = MagicMock()
        expected = (np.array([.5,0]), .033, .0001)
        with patch('finish_pick.grab_rgbd', return_value=(None,)*4), \
                patch('finish_pick.sam3_client.detect', return_value=(None, .98)), \
                patch('finish_pick.pointcloud.object_depth_mask'), \
                patch('finish_pick.pointcloud.deproject'), \
                patch('finish_pick.pointcloud.transform_points'), \
                patch('finish_pick.fit_upright_can', return_value=expected):
            self.assertIs(pick.observe_can(), expected)
        pick.execute.assert_not_called()
