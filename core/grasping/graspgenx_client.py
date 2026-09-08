"""Client for the GraspGenX server. See docker/graspgenx/app.py for the query/reply protocol."""

import numpy as np
from grasping.grasp_io import canonical_from_palm
from utils.zenoh_rpc import query


def generate(points, gripper, num_grasps=200, timeout=30):
    """Request palm poses and scores for an object-centred cloud."""
    selector = f"graspgenx/generate?num_grasps={num_grasps};gripper_name={gripper}"
    meta, body = query(selector, points.astype(np.float32).tobytes(), timeout)
    count = meta["num_grasps"]
    if count == 0:
        raise RuntimeError("GraspGenX returned no candidate grasps")
    values_per_pose = 4 * 4
    bytes_per_value = 4
    poses_end = count * values_per_pose * bytes_per_value
    poses = np.frombuffer(body[:poses_end], dtype=np.float32).reshape(count, 4, 4)
    # The model predicts its own gripper frame; consumers expect the palm.
    poses = poses.astype(np.float64) @ canonical_from_palm(gripper)
    scores = np.frombuffer(body[poses_end:], dtype=np.float32)
    return poses, scores
