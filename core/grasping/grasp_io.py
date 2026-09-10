"""Save candidate grasps as plain YAML, best-scoring first, so a consumer
(e.g. MoveIt) can work down the list until one is reachable -- the top score
alone isn't always achievable by a given arm's kinematics."""

import yaml
from scipy.spatial.transform import Rotation

from . import can_alignment

# Retain the server's full usual 100-candidate batch: the shelf test showed
# that score truncation can discard the only reachable orientations.
TOP_K = 100


def _xyz(vector):
    return dict(zip("xyz", (float(v) for v in vector)))


def _grasp(pose, score):
    quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
    return {
        "score": float(score),
        "position": _xyz(pose[:3, 3]),
        "orientation": dict(zip("xyzw", (float(q) for q in quaternion))),
    }


def _target_geometry(points):
    """Fitted upright cylinder for `points`, or None if they aren't one.

    A single RGB-D view only sees the near half of a round object, so its
    bounding box is roughly half a diameter short along the viewing axis and
    its centre sits on the visible surface rather than the axis. The executor
    needs the real axis to tell an intended fingertip contact on the target
    from a collision with the furniture, so fit the section instead.

    `fit_upright_can` raises for anything that is not a can-sized upright
    circular section, which is what keeps this off arbitrary open-vocabulary
    targets: they simply get no `target_geometry` key, exactly as before.
    """
    try:
        center, radius, _ = can_alignment.fit_upright_can(points)
    except ValueError as error:
        print(f"  upright-cylinder fit rejected: {error}")
        return None
    return {
        "center_x": float(center[0]),
        "center_y": float(center[1]),
        "min_z": float(points[:, 2].min()),
        "max_z": float(points[:, 2].max()),
        "radius": float(radius),
    }


def save_grasps(path, poses, scores, points, frame_id, gripper, object_id, scene_points=None):
    """Write the TOP_K best of (N, 4, 4) `poses` / (N,) `scores` to `path`.
    `points` contributes only its bounding box, so a consumer can register the
    object as a collision object precisely."""
    best_first = scores.argsort()[::-1][:TOP_K]
    data = {
        "frame_id": frame_id,
        "gripper": gripper,
        "object_id": object_id,
        "bounding_box": {"min": _xyz(points.min(axis=0)), "max": _xyz(points.max(axis=0))},
    }
    geometry = _target_geometry(points)
    if geometry is not None:
        if scene_points is not None:
            from .target_extent import complete_cylinder_extent
            geometry = complete_cylinder_extent(geometry, scene_points)
        data["target_geometry"] = geometry
    data["grasps"] = [_grasp(poses[i], scores[i]) for i in best_first]
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return len(data["grasps"])
