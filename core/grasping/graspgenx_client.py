"""Client for the GraspGenX server. See docker/graspgenx/app.py for the
query/reply protocol."""

import numpy as np

from grasping.gripper_frame import canonical_from_palm
from utils.zenoh_rpc import query


def generate(points, gripper, num_grasps=200, timeout=30):
    """Ask for grasps on an object-centered (N, 3) cloud -- GraspGenX is
    trained on single objects, not whole scenes.

    Returns (M, 4, 4) poses, in the frame `points` came in, and (M,) scores.
    The poses place hand_palm_link, not GraspGenX's canonical grasp frame:
    this is the boundary where that conversion happens.
    """
    # zenoh selectors separate parameters with ';', not '&' like HTTP.
    selector = f"graspgenx/generate?num_grasps={num_grasps};gripper_name={gripper}"
    meta, body = query(selector, points.astype(np.float32).tobytes(), timeout)

    count = meta["num_grasps"]
    if count == 0:
        raise RuntimeError("GraspGenX returned no candidate grasps")

    poses_end = count * 4 * 4 * 4
    poses = np.frombuffer(body[:poses_end], dtype=np.float32).reshape(count, 4, 4)

    # GraspGenX places its canonical grasp frame; MoveIt and the IK solver want
    # hand_palm_link placed instead.
    poses = poses.astype(np.float64) @ canonical_from_palm(gripper)
    return poses, np.frombuffer(body[poses_end:], dtype=np.float32)
