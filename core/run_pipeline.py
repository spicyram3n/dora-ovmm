"""
End-to-end grasp pipeline: prompt -> SAM3 mask -> object point cloud ->
GraspGenX candidate grasps -> plot.

Usage: python3 run_pipeline.py "<prompt>" [gripper_name]

Needs, already running:
    docker/sam3/run_sam3.sh
    docker/graspgenx/run_graspgenx.sh
and a ROS 2 environment sourced (the only ROS2-dependent step is
grasping/camera_ros2.py).
"""

import json
import re
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from grasping import graspgenx_client, grasp_io, pointcloud, sam3_client, visualize
from grasping.camera_ros2 import BASE_FRAME, grab_rgbd

GRASPGENX_DIR = Path(__file__).resolve().parent.parent / "docker" / "graspgenx"
GRASPS_DIR = Path(__file__).resolve().parent.parent / "config" / "grasps"


def _run_name(prompt):
    """e.g. "big pringles can" -> "big_pringles_can_20260727_190533", so
    grasps from different objects (or different runs) never overwrite each other."""
    slug = re.sub(r"[^a-z0-9]+", "_", prompt.lower()).strip("_")
    return f"{slug}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def main(prompt, gripper_name="hsrc_hand"):
    print("[pipeline] grabbing a frame from the head camera...")
    rgb, depth_m, k, base_from_camera = grab_rgbd()

    print(f"[pipeline] running SAM3 detection for '{prompt}'...")
    masks, boxes, scores = sam3_client.detect(rgb, prompt)
    if len(scores) == 0:
        raise RuntimeError(f"SAM3 found no instance of '{prompt}'")
    mask, sam3_score = sam3_client.best_instance(masks, scores)
    print(f"[pipeline] using top detection, score={sam3_score:.2f}")

    # SAM3 may process the image at a different resolution than the camera
    # frame, so resize its mask back to match the depth image before using it.
    mask = cv2.resize(mask.astype(np.uint8), (depth_m.shape[1], depth_m.shape[0]),
                       interpolation=cv2.INTER_NEAREST).astype(bool)

    points = pointcloud.deproject(depth_m, k, mask)
    if len(points) == 0:
        raise RuntimeError("no valid depth points inside the mask")
    print(f"[pipeline] object point cloud: {len(points)} points")

    centered_points, centroid = pointcloud.center(points)
    print(f"[pipeline] querying GraspGenX ('{gripper_name}')...")
    grasps, grasp_scores = graspgenx_client.generate(centered_points, gripper_name=gripper_name)
    if len(grasp_scores) == 0:
        raise RuntimeError("GraspGenX returned no candidate grasps")
    print(f"[pipeline] got {len(grasp_scores)} grasps, best score={grasp_scores.max():.2f}")

    # Grasps came back in the centered frame; shift back to match `points`.
    grasps = grasps.copy()
    grasps[:, :3, 3] += centroid

    # Move everything from the camera frame into the robot's base frame, so
    # the saved grasp is usable as a MoveIt pose target even as the head moves.
    points = pointcloud.transform_points(base_from_camera, points)
    grasps = pointcloud.transform_poses(base_from_camera, grasps)

    GRASPS_DIR.mkdir(parents=True, exist_ok=True)
    run_name = _run_name(prompt)
    grasp_path = GRASPS_DIR / f"{run_name}.yaml"
    plot_path = GRASPS_DIR / f"{run_name}.png"

    grasp_io.save_grasps(grasps, grasp_scores, BASE_FRAME, gripper_name, grasp_path)
    print(f"[pipeline] saved top {min(10, len(grasp_scores))} grasps to {grasp_path}")

    gripper_dir = GRASPGENX_DIR / "x_grippers" / gripper_name
    base_rotation = json.loads((gripper_dir / "config.json").read_text())["base_rotation"]
    visualize.plot_grasps(points, grasps, grasp_scores, plot_path,
                           gripper_mesh_path=gripper_dir / "vis_mesh.obj",
                           gripper_base_rotation=base_rotation)
    print(f"[pipeline] saved plot to {plot_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit('usage: python3 run_pipeline.py "<prompt>" [gripper_name]')
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "hsrc_hand")
