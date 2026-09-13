import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from grasping.can_alignment import fit_upright_can, align_side_grasp


class CanAlignmentTest(unittest.TestCase):
    def test_partial_cylinder_recovers_unobserved_center(self):
        rng = np.random.default_rng(7)
        angle = rng.uniform(.6 * np.pi, 1.4 * np.pi, 3000)
        points = np.column_stack((.51 + .033 * np.cos(angle),
                                  .02 + .033 * np.sin(angle), rng.uniform(1.06, 1.25, 3000)))
        points += rng.normal(0, .0001, points.shape)
        center, radius, residual = fit_upright_can(points)
        np.testing.assert_allclose(center, [.51, .02], atol=.0002)
        self.assertAlmostEqual(radius, .033, places=3)
        self.assertLess(residual, .0002)

    def test_flat_surface_rejected(self):
        y, z = np.meshgrid(np.linspace(-.03, .03, 30), np.linspace(1.05, 1.25, 30))
        with self.assertRaises(ValueError):
            fit_upright_can(np.column_stack((np.full(y.size, .48), y.ravel(), z.ravel())))

    def test_top_heavy_view_excludes_lid_from_side_fit(self):
        rng = np.random.default_rng(19)
        angle = rng.uniform(.6*np.pi, 1.4*np.pi, 3000)
        side = np.column_stack((.51+.033*np.cos(angle),
                                .02+.033*np.sin(angle), rng.uniform(.40, .626, 3000)))
        angle = rng.uniform(0, 2*np.pi, 4000)
        radius = .033*np.sqrt(rng.uniform(0, 1, 4000))
        lid = np.column_stack((.51+radius*np.cos(angle),
                               .02+radius*np.sin(angle), np.full(4000, .626)))
        points = np.vstack((side, lid))
        points += rng.normal(0, .0001, points.shape)
        center, radius, residual = fit_upright_can(points)
        np.testing.assert_allclose(center, [.51, .02], atol=.0002)
        self.assertAlmostEqual(radius, .033, places=3)
        self.assertLess(residual, .0002)

    def test_pad_center_corrected_without_changing_height_or_rotation(self):
        pose = np.array([[0., 0., 1., .395], [-1., 0., 0., 0.],
                         [0., -1., 0., 1.18], [0., 0., 0., 1.]])
        profile = [{'gap': [.06], 'center': [0., .013, .08]},
                   {'gap': [.12], 'center': [0., .013, .05]}]
        corrected, shift, pad = align_side_grasp(pose, [.51, 0.], .033, profile)
        np.testing.assert_allclose((corrected[:3, 3] + corrected[:3, :3] @ pad)[:2], [.51, 0.])
        np.testing.assert_array_equal(corrected[:3, :3], pose[:3, :3])
        self.assertEqual(corrected[2, 3], pose[2, 3])
        self.assertGreater(shift[0], .03)
        with self.assertRaises(ValueError):
            align_side_grasp(pose, [.7, 0.], .033, profile)


if __name__ == '__main__':
    unittest.main()
