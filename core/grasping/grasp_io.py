"""Read and write candidate grasps as plain YAML, best-scoring first, so a
consumer (e.g. MoveIt) can work down the list until one is reachable -- the top
score alone isn't always achievable by a given arm's kinematics.

Both directions live here on purpose. The schema is defined by `save_grasps`,
so anything that opens grasps.yaml elsewhere is a second definition free to
drift from it -- which is exactly how move_to_grasp.launch.py ended up reading
`grasp["header"]["frame_id"]` for a writer that emits `frame_id`. Consumers
call `load_grasps` and never see the dictionary.
"""

from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

TOP_K = 10


def _xyz(vector):
    return dict(zip("xyz", (float(v) for v in vector)))


def _grasp(pose, score):
    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
    return {
        "score": float(score),
        "position": _xyz(pose[:3, 3]),
        "orientation": dict(zip("xyzw", (float(q) for q in quaternion))),
    }


def save_grasps(path, poses, scores, points, frame_id, gripper, object_id):
    """Write the TOP_K best of (N, 4, 4) `poses` / (N,) `scores` to `path`.
    `points` contributes only its bounding box, so a consumer can register the
    object as a collision object precisely."""
    best_first = scores.argsort()[::-1][:TOP_K]
    data = {
        "frame_id": frame_id,
        "gripper": gripper,
        "object_id": object_id,
        "bounding_box": {"min": _xyz(points.min(axis=0)), "max": _xyz(points.max(axis=0))},
        "grasps": [_grasp(poses[i], scores[i]) for i in best_first],
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return len(data["grasps"])


def load_grasps(path):
    """The inverse of `save_grasps`.

    Returns (poses, scores, meta):
        poses:  (N, 4, 4) hand_palm_link poses in meta["frame_id"]
        scores: (N,)
        meta:   the parsed file, for frame_id / gripper / object_id / bounding_box
    """
    data = yaml.safe_load(Path(path).read_text())
    grasps = data["grasps"]

    poses = np.tile(np.eye(4), (len(grasps), 1, 1))
    for i, grasp in enumerate(grasps):
        poses[i, :3, :3] = Rotation.from_quat(
            [grasp["orientation"][k] for k in "xyzw"]).as_matrix()
        poses[i, :3, 3] = [grasp["position"][a] for a in "xyz"]

    return poses, np.array([g["score"] for g in grasps]), data
