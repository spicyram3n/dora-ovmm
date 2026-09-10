import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from perception.pointcloud import object_depth_mask


class ObjectDepthMaskTest(unittest.TestCase):
    def test_discards_background_across_depth_edge(self):
        depth = np.ones((20, 20), dtype=np.float32)
        depth[3:17, 5:15] = .5
        depth[9, 9] = np.nan
        mask = np.zeros((20,20), dtype=bool)
        mask[2:18, 4:16] = True
        result = object_depth_mask(depth, mask)
        self.assertEqual(int(result.sum()), 139)
        self.assertTrue(np.all(depth[result] == .5))

    def test_preserves_gradual_surface(self):
        depth = np.tile(np.linspace(.4,.55,20), (20,1))
        self.assertTrue(object_depth_mask(depth, np.ones_like(depth,dtype=bool)).all())

    def test_rejects_empty_depth(self):
        with self.assertRaises(RuntimeError):
            object_depth_mask(np.zeros((3,3)), np.ones((3,3),dtype=bool))

if __name__ == '__main__':
    unittest.main()
