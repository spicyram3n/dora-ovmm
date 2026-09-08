"""Read and write candidate grasps as plain YAML, best-scoring first, so a consumer (e.g."""

import json
from functools import lru_cache
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

TOP_K = 10


def _xyz(vector):
    coordinates = {}
    for axis, value in zip("xyz", vector):
        coordinates[axis] = float(value)
    return coordinates


def _grasp(pose, score):
    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
    orientation = {}
    for axis, value in zip("xyzw", quaternion):
        orientation[axis] = float(value)
    return {
        "score": float(score),
        "position": _xyz(pose[:3, 3]),
        "orientation": orientation,
    }


def save_grasps(path, poses, scores, points, frame_id, gripper, object_id):
    """Save the ten highest-scoring grasps and the object's bounding box."""
    best_first = scores.argsort()[::-1][:TOP_K]
    grasps = []
    for index in best_first:
        grasps.append(_grasp(poses[index], scores[index]))
    data = {
        "frame_id": frame_id,
        "gripper": gripper,
        "object_id": object_id,
        "bounding_box": {
            "min": _xyz(points.min(axis=0)),
            "max": _xyz(points.max(axis=0)),
        },
        "grasps": grasps,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return len(grasps)


def load_grasps(path):
    """Return pose matrices, scores, and the complete YAML record."""
    data = yaml.safe_load(Path(path).read_text())
    grasps = data["grasps"]
    poses = np.tile(np.eye(4), (len(grasps), 1, 1))
    scores = []
    for index, grasp in enumerate(grasps):
        quaternion = []
        for axis in "xyzw":
            quaternion.append(grasp["orientation"][axis])
        position = []
        for axis in "xyz":
            position.append(grasp["position"][axis])
        poses[index, :3, :3] = Rotation.from_quat(quaternion).as_matrix()
        poses[index, :3, 3] = position
        scores.append(grasp["score"])
    return poses, np.array(scores), data


GRIPPERS_DIR = (
    Path(__file__).resolve().parents[2] / "docker" / "graspgenx" / "x_grippers"
)


@lru_cache(maxsize=None)
def canonical_from_palm(gripper):
    """(4, 4) mapping hand_palm_link coordinates into GraspGenX's frame."""
    config = json.loads((GRIPPERS_DIR / gripper / "config.json").read_text())
    return np.asarray(config["base_rotation"], dtype=np.float64)


def palm_from_canonical(gripper):
    """Map canonical gripper coordinates back into the palm frame."""
    return np.linalg.inv(canonical_from_palm(gripper))
