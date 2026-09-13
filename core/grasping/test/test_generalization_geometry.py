"""Regression checks for the geometry carried from the table to the shelf."""
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from grasping.can_alignment import align_side_grasp
from grasping.target_extent import complete_cylinder_extent


class GeneralizationGeometryTest(unittest.TestCase):
    def test_leveling_preserves_pad_height_and_centers_on_new_axis(self):
        pose = np.eye(4)
        side_grasp = np.column_stack(([0., 1., 0.], [0., 0., 1.], [1., 0., 0.]))
        pose[:3, :3] = Rotation.from_euler('xyz', [3, 4, 55], degrees=True).as_matrix() @ side_grasp
        pose[:3, 3] = [.3, -.2, .7]
        pad = np.array([0., 0., .075])
        initial_contact = pose[:3, 3] + pose[:3, :3] @ pad
        center = initial_contact[:2] + [.01, -.005]
        profile = [{'gap': [g], 'center': pad.tolist()} for g in [.02, .12]]
        corrected, _, actual_pad = align_side_grasp(pose, center, .033, profile, level=True)
        contact = corrected[:3, 3] + corrected[:3, :3] @ actual_pad
        np.testing.assert_allclose(contact, [*center, initial_contact[2]], atol=1e-10)
        np.testing.assert_allclose(corrected[2, [0, 2]], [0., 0.], atol=1e-10)

    def test_top_completion_does_not_bridge_gap_to_shelf(self):
        geometry = dict(center_x=.65, center_y=0., radius=.033, min_z=.66, max_z=.865)
        heights = np.r_[np.arange(.855, .886, .001), np.arange(.92, .93, .001)]
        points = np.array([[.65, 0., z] for z in heights for _ in range(10)])
        completed = complete_cylinder_extent(geometry, points)
        self.assertAlmostEqual(completed['min_z'], .66)
        self.assertGreater(completed['max_z'], .885)
        self.assertLess(completed['max_z'], .89)


if __name__ == '__main__':
    unittest.main()
