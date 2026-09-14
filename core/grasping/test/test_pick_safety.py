import unittest
from contextlib import ExitStack
from unittest.mock import patch, MagicMock
import numpy as np
import core.grasping.pick as pickup


class ClosureGuards(unittest.TestCase):
    def test_grasp_mode_never_attaches_or_lifts(self):
        with ExitStack() as stack:
            for name in ['rclpy','rclcpp','model_target','mtc_node','depth_relay']:
                stack.enter_context(patch.object(pickup,name))
            planner=stack.enter_context(patch.object(pickup,'Pick')).return_value
            execute=stack.enter_context(patch.object(pickup,'execute'))
            stack.enter_context(patch('core.grasping.pick.close'))
            stack.enter_context(patch.object(pickup,'RobotTransforms')).return_value.transform.return_value=np.eye(4)
            stack.enter_context(patch.object(pickup,'perceive',return_value=(
                np.zeros((100,3)),np.eye(4)[None],np.array([.05]),{'geometry_kind':'cube top'})))
            stack.enter_context(patch.object(pickup,'joint_positions',return_value={'arm_lift_joint':.3}))
            stack.enter_context(patch.object(pickup,'fingertip_gap',return_value=.05))
            self.assertEqual(pickup.pick('cube',mode='grasp'),'grasped')
            planner.lift.assert_not_called()
            self.assertEqual(execute.call_count,1)

    def run_pick(self,approach_error=None,short_approach=False):
        with ExitStack() as stack:
            for name in ['rclpy','rclcpp','model_target','mtc_node','depth_relay','Pick']:
                stack.enter_context(patch.object(pickup,name))
            execute=stack.enter_context(patch.object(pickup,'execute'))
            execute.side_effect=approach_error
            close=stack.enter_context(patch('core.grasping.pick.close'))
            feedback=stack.enter_context(patch.object(pickup,'RobotTransforms')).return_value
            servo=stack.enter_context(patch('core.grasping.pick.VisualServo'))
            pre=np.eye(4);goal=np.eye(4);goal[2,3]=pickup.APPROACH
            end=goal.copy();end[2,3]-=.01 if short_approach else 0
            feedback.transform.return_value=end
            stack.enter_context(patch.object(pickup,'perceive',return_value=(
                np.zeros((100,3)),goal[None],np.array([.05]),{'feature':np.zeros(3),'area':100})))
            stack.enter_context(patch.object(pickup,'joint_positions',return_value={'arm_lift_joint':.3}))
            result=pickup.pick('arbitrary object label')
            servo.assert_not_called()
            return result,close.call_count

    def test_no_closure_if_planned_approach_fails(self):
        result,count=self.run_pick(RuntimeError('no collision-free approach'))
        self.assertFalse(result);self.assertEqual(count,0)

    def test_no_closure_after_short_approach(self):
        result,count=self.run_pick(short_approach=True)
        self.assertFalse(result);self.assertEqual(count,0)

if __name__=='__main__':unittest.main()
