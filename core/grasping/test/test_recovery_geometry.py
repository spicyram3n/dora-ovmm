import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from grasping.recovery_geometry import planar_tracking_delta


class TrackingGeometryTest(unittest.TestCase):
    def test_planar_error_recovers_base_without_arm_change(self):
        base = np.array([-.09, .03, .04])
        actual = np.eye(4)
        actual[:3, 3] = [.48, .032, .51]
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('z', -.04).as_matrix()
        transform[:3, 3] = [.002, -.004, 0]
        goal = transform @ actual
        delta = planar_tracking_delta(actual, goal, base)
        desired = transform[:3, :3] @ np.r_[base[:2], 0] + transform[:3, 3]
        np.testing.assert_allclose(base[:2]+delta[:2], desired[:2])
        self.assertAlmostEqual(delta[2], -.04)

    def test_large_or_nonplanar_errors_rejected(self):
        for translation, rotation in [([.02, 0, 0], [0, 0, 0]),
                                       ([0, 0, .01], [0, 0, 0]),
                                       ([0, 0, 0], [.01, 0, 0]),
                                       ([0, 0, 0], [0, 0, .1])]:
            goal = np.eye(4)
            goal[:3,3] = translation
            goal[:3,:3] = Rotation.from_rotvec(rotation).as_matrix()
            with self.subTest(translation=translation, rotation=rotation), self.assertRaises(ValueError):
                planar_tracking_delta(np.eye(4), goal, np.zeros(3))
