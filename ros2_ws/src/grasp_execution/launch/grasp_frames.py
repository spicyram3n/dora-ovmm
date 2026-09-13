"""Convert saved canonical GraspGenX poses into URDF hand-palm poses."""

import numpy as np
from scipy.spatial.transform import Rotation


def hand_pose(grasp, base_rotation):
    """T_world_hand = T_world_grasp @ T_grasp_hand.

    The registered base_rotation maps URDF mesh coordinates into the
    canonical frame. Right multiplication preserves the world approach
    direction; left multiplication would rotate the grasp in the world.
    """
    world_grasp = np.eye(4)
    world_grasp[:3, :3] = Rotation.from_quat(
        [grasp["orientation"][axis] for axis in "xyzw"]).as_matrix()
    world_grasp[:3, 3] = [grasp["position"][axis] for axis in "xyz"]
    world_hand = world_grasp @ np.asarray(base_rotation, dtype=float)
    quaternion = Rotation.from_matrix(world_hand[:3, :3]).as_quat()
    return {
        **grasp,
        "position": dict(zip("xyz", map(float, world_hand[:3, 3]))),
        "orientation": dict(zip("xyzw", map(float, quaternion))),
    }
