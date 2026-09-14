import unittest
import numpy as np
from core.grasping.pick import side_waypoint


class SideWaypointTests(unittest.TestCase):
    def test_side_stays_above_and_retains_grasp_orientation(self):
        pose = np.eye(4); pose[:3, :3] = np.diag([1., -1., -1.])
        pose[:3, 3] = [.6, .2, .8]
        side = side_waypoint(pose, [0., 0.])
        self.assertAlmostEqual(np.linalg.norm(side[:2, 3]-pose[:2, 3]), .12)
        self.assertAlmostEqual(side[2, 3], .8)
        np.testing.assert_array_equal(side[:3, :3], pose[:3, :3])
        self.assertLess(np.linalg.norm(side[:2, 3]), np.linalg.norm(pose[:2, 3]))

    def test_front_grasps_have_no_side_waypoint(self):
        pose = np.eye(4); pose[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
        self.assertIsNone(side_waypoint(pose, [0., 0.]))

    def test_near_base_waypoint_does_not_cross_base(self):
        pose = np.diag([1., -1., -1., 1.]); pose[0, 3] = .06
        side = side_waypoint(pose, [0., 0.])
        self.assertAlmostEqual(side[0, 3], .03)
        pose[0, 3] = 0.
        self.assertIsNone(side_waypoint(pose, [0., 0.]))
