import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("grasp_frames", PACKAGE / "launch/grasp_frames.py")
frames = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(frames)
ROOT = PACKAGE.parents[2]


class GraspFramesTest(unittest.TestCase):
    def test_registration_uses_forward_pad_volume_and_free_springs(self):
        config = json.loads((ROOT / "docker/graspgenx/x_grippers/hsrc_hand/config.json").read_text())
        self.assertGreater(config["sweep_volume"]["offset"][2], 0)
        self.assertGreater(config["sweep_volume"]["offset2"][2], 0)
        for state in ("open", "close"):
            for side in ("l", "r"):
                self.assertEqual(config[state][f"hand_{side}_spring_proximal_joint"], 0.0)
            self.assertGreaterEqual(-config[state]["hand_motor_joint"] - 0.087, -1.24)

    def test_registered_hand_axes_at_nontrivial_world_pose(self):
        config = json.loads((ROOT / "docker/graspgenx/x_grippers/hsrc_hand/config.json").read_text())
        for angles in ([0, 0, 0], [0.7, -0.4, 1.2]):
            rotation = Rotation.from_euler("xyz", angles)
            grasp = {"score": 0.9, "position": dict(x=0.5, y=-0.2, z=1.1),
                     "orientation": dict(zip("xyzw", rotation.as_quat()))}
            result = frames.hand_pose(grasp, config["base_rotation"])
            actual = Rotation.from_quat([result["orientation"][a] for a in "xyzw"]).as_matrix()
            expected = rotation.as_matrix()
            # URDF +Y closes along canonical +X; approach +Z is unchanged.
            np.testing.assert_allclose(actual[:, 1], expected[:, 0], atol=1e-12)
            np.testing.assert_allclose(actual[:, 2], expected[:, 2], atol=1e-12)
            np.testing.assert_allclose(actual[:, 0], -expected[:, 1], atol=1e-12)
            self.assertEqual(result["position"], grasp["position"])
            self.assertEqual(result["score"], grasp["score"])


if __name__ == "__main__":
    unittest.main()
