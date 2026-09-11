"""Request GraspGenX candidates and convert them to palm-frame poses."""

import json
from pathlib import Path

import numpy as np
from core.utils.zenoh_rpc import query, parameter

GRIPPERS_DIR = (
    Path(__file__).resolve().parents[2] / "docker" / "graspgenx" / "x_grippers"
)


def canonical_from_palm(gripper):
    """(4, 4) mapping hand_palm_link coordinates into GraspGenX's gripper frame."""
    config = json.loads((GRIPPERS_DIR / gripper / "config.json").read_text())
    return np.asarray(config["base_rotation"], dtype=np.float64)


def generate(
    points, gripper, num_grasps=200, timeout=30, *, metadata=None, return_metadata=False
):
    """Return palm poses and scores; optionally include echoed observation metadata."""
    points = np.asarray(points, dtype=np.float32)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise ValueError("Expected nonempty finite Nx3 object points")
    if (
        isinstance(num_grasps, bool)
        or not isinstance(num_grasps, int)
        or num_grasps < 1
    ):
        raise ValueError("num_grasps must be a positive integer")
    selector = (
        f"graspgenx/generate?num_grasps={num_grasps};gripper_name={parameter(gripper)}"
    )
    meta, body = query(
        selector, points.astype(np.float32).tobytes(), timeout, metadata=metadata
    )
    count = meta.get("num_grasps")
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise RuntimeError("Malformed GraspGenX candidate count")
    if count == 0:
        raise RuntimeError("GraspGenX returned no candidate grasps")
    poses_end = count * 16 * np.dtype(np.float32).itemsize
    if len(body) != poses_end + count * 4:
        raise RuntimeError("Malformed GraspGenX reply")
    poses = np.frombuffer(body[:poses_end], dtype=np.float32).reshape(count, 4, 4)
    # The model predicts its own gripper frame; consumers expect the palm.
    poses = poses.astype(np.float64) @ canonical_from_palm(gripper)
    scores = np.frombuffer(body[poses_end:], dtype=np.float32)
    if not np.isfinite(poses).all() or not np.isfinite(scores).all():
        raise RuntimeError("GraspGenX returned nonfinite candidates")
    return (poses, scores, meta) if return_metadata else (poses, scores)
