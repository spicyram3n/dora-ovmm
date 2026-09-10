import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'core'))
from finish_pick import FinishPick


class ClosureSweepTest(unittest.TestCase):
    def make_pick(self):
        pick = FinishPick(None, {}, Path('/tmp'))
        state = RobotState()
        state.joint_state.name = ['hand_motor_joint', 'hand_l_distal_joint', 'hand_l_spring_proximal_joint']
        state.joint_state.position = [1.1, -1.187, .03]
        pick.scene = lambda: SimpleNamespace(robot_state=state)
        pick.mimics = [('hand_l_distal_joint', dict(joint='hand_motor_joint', multiplier='-1', offset='-.087'))]
        pick.execute = MagicMock()
        return pick

    def test_mid_sweep_collision_prevents_execution(self):
        pick = self.make_pick()
        pick.call = lambda kind, name, request: GetStateValidity.Response(
            valid=request.robot_state.joint_state.position[0] > 1.0)
        with self.assertRaisesRegex(RuntimeError, 'Closure sweep collision'):
            pick.close_step(.7)
        pick.execute.assert_not_called()

    def test_mimics_checked_but_only_motor_commanded(self):
        pick = self.make_pick()
        def check(kind, name, request):
            motor, distal, spring = request.robot_state.joint_state.position
            self.assertAlmostEqual(distal, -motor - .087)
            self.assertAlmostEqual(spring, .03)
            return GetStateValidity.Response(valid=True)
        pick.call = check
        pick.close_step(.7)
        trajectory = pick.execute.call_args.args[0].joint_trajectory
        self.assertEqual(trajectory.joint_names, ['hand_motor_joint'])
        self.assertAlmostEqual(trajectory.points[-1].positions[0], .7)
        self.assertTrue(all(abs(b.positions[0] - a.positions[0]) <= .005
                            for a, b in zip(trajectory.points, trajectory.points[1:])))


if __name__ == '__main__':
    unittest.main()
