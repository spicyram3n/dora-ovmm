"""Save candidate grasps to a plain YAML file, ranked best-first, so a
consumer (e.g. MoveIt) can try each in turn until one is reachable. The
top score alone isn't always achievable by a specific arm's kinematics."""

import yaml
from scipy.spatial.transform import Rotation


def save_grasps(poses, scores, frame_id, gripper_name, out_path, top_k=10):
    """poses: (N, 4, 4) homogeneous transforms. scores: (N,).
    Writes the top_k highest-scoring poses, best first."""
    order = scores.argsort()[::-1][:top_k]

    grasps = []
    for i in order:
        x, y, z = poses[i][:3, 3].tolist()
        qx, qy, qz, qw = Rotation.from_matrix(poses[i][:3, :3]).as_quat().tolist()
        grasps.append({
            "score": float(scores[i]),
            "pose": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        })

    data = {"header": {"frame_id": frame_id}, "gripper": gripper_name, "grasps": grasps}
    with open(out_path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
