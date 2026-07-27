"""
Zenoh client for the GraspGenX server. See docker/graspgenx/app.py's module
docstring for the exact query/reply protocol this follows.
"""

import json

import numpy as np
import zenoh


def generate(points, gripper_name=None, num_grasps=200, timeout=30):
    """Query GraspGenX for grasps on an object point cloud.

    points: (N, 3) float32, object-centered (see pointcloud.center()).
            GraspGenX is trained on object-centric clouds, not full scenes.

    Returns (grasps, scores):
        grasps: (M, 4, 4) float32 homogeneous poses, in the same frame `points` was in.
        scores: (M,) float32 confidence in [0, 1].
    """
    # zenoh selectors separate parameters with ';', not '&' like HTTP query strings.
    params = f"num_grasps={num_grasps}"
    if gripper_name:
        params += f";gripper_name={gripper_name}"
    payload = points.astype(np.float32).tobytes()

    cfg = zenoh.Config()
    cfg.insert_json5("transport/shared_memory/enabled", "false")

    with zenoh.open(cfg) as session:
        for reply in session.get(f"graspgenx/generate?{params}", payload=payload, timeout=timeout):
            if not reply.ok:
                raise RuntimeError(f"graspgenx/generate failed: {reply.err.payload.to_bytes().decode()}")

            meta = json.loads(reply.ok.attachment.to_bytes())
            m = meta["num_grasps"]
            body = reply.ok.payload.to_bytes()

            grasp_bytes = m * 4 * 4 * 4
            grasps = np.frombuffer(body[:grasp_bytes], dtype=np.float32).reshape(m, 4, 4)
            scores = np.frombuffer(body[grasp_bytes:], dtype=np.float32)
            return grasps, scores

    raise RuntimeError("graspgenx/generate: no reply")
