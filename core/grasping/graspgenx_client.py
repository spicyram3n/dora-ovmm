"""Client for the GraspGenX server. See docker/graspgenx/app.py for the
query/reply protocol."""

import json
from pathlib import Path

import numpy as np

from utils.zenoh_rpc import query

GRIPPERS_DIR = Path(__file__).resolve().parents[2] / "docker" / "graspgenx" / "x_grippers"


def base_rotation(gripper):
    """Maps the gripper's own URDF frame into GraspGenX's canonical grasp
    frame (+Z approach, +X closing). On hsrc that is a 90 degree turn about
    the approach axis, since the real fingers close along the palm's Y."""
    config = json.loads((GRIPPERS_DIR / gripper / "config.json").read_text())
    return np.asarray(config["base_rotation"], dtype=np.float32)


def generate(points, gripper, num_grasps=200, timeout=30):
    """Ask for grasps on an object-centered (N, 3) cloud -- GraspGenX is
    trained on single objects, not whole scenes.

    Returns (M, 4, 4) poses for the gripper's own URDF frame (hand_palm_link
    on hsrc), in the frame `points` came in, and (M,) scores.
    """
    # zenoh selectors separate parameters with ';', not '&' like HTTP.
    selector = f"graspgenx/generate?num_grasps={num_grasps};gripper_name={gripper}"
    meta, body = query(selector, points.astype(np.float32).tobytes(), timeout)

    count = meta["num_grasps"]
    if count == 0:
        raise RuntimeError("GraspGenX returned no candidate grasps")

    poses_end = count * 4 * 4 * 4
    poses = np.frombuffer(body[:poses_end], dtype=np.float32).reshape(count, 4, 4)
    return poses @ base_rotation(gripper), np.frombuffer(body[poses_end:], dtype=np.float32)
