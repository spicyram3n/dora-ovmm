"""What the policy's grasp branch plugs into: GraspGenX for grasps and Toyota's
IK service for ETH's reachability check.

active_grasp runs VGN on the TSDF and trac_ik on each grasp. Here GraspGenX
reads the fused surface points, and the HSR's whole-body IK service
(launch/ik_solver.launch.py, core/navigation/base_placement.py) answers
whether some base placement and arm configuration reach the palm pose,
collision-checked against the costmap.
"""

import numpy as np

from core.grasping import graspgenx_client

GRIPPER = "hsrc_hand"
MAX_CLOUD_POINTS = 8192  # as core/grasping/pick.py


def graspgenx_grasps(points):
    """grasp_fn: GraspGenX on a centred sample of the surface, palm poses back in the map."""
    sample = points[np.random.default_rng(0).choice(len(points), min(len(points), MAX_CLOUD_POINTS), replace=False)]
    centre = sample.mean(axis=0)
    poses, scores = graspgenx_client.generate(sample - centre, GRIPPER)
    poses = poses.copy()
    poses[:, :3, 3] += centre
    return poses, scores


def ik_reachable(navigator, obstacles="--costmap", top_k=5, timeout=30.0):
    """reachable_fn: which map-frame palm poses the IK service finds a base
    placement and arm solution for. Grasps come best first; only the first
    `top_k` are asked, since the service takes one call per pose."""
    from core.navigation import base_placement

    def reachable(poses):
        poses = np.asarray(poses, dtype=float)
        odom_from_map = navigator.frame_transform("odom", "map")
        asked = poses[:top_k]
        results = base_placement.solve(odom_from_map @ asked, obstacles=obstacles,
                                       goal_frame="odom", timeout=timeout)
        mask = np.zeros(len(poses), dtype=bool)
        mask[:len(asked)] = [len(bases) > 0 for bases, _ in results]
        return mask

    return reachable
