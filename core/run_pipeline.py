"""
Prompt -> SAM3 mask -> object point cloud -> GraspGenX grasps, saved under
config/targets/<slug>/ as grasps.yaml, cloud.ply and plot.png.

Usage: python3 run_pipeline.py "<prompt>" ["<prompt>" ...] [--gripper hsrc_hand]

Several prompts share a single camera frame, so every target found in one run
is registered against the same instant. A target that isn't found is reported
and skipped, leaving the rest of the run intact.

Needs docker/sam3/run_sam3.sh and docker/graspgenx/run_graspgenx.sh already
running, plus a sourced ROS 2 environment (only camera_ros2.py uses it).
"""

import argparse
import re
from pathlib import Path

from grasping import grasp_io, graspgenx_client
from grasping.visualization import visualize
from perception import pointcloud, sam3_client
from perception.camera_ros2 import BASE_FRAME, grab_rgbd

TARGETS_DIR = Path(__file__).resolve().parent.parent / "config" / "targets"


def slug(prompt):
    """"big pringles can" -> "big_pringles_can". Names the target's directory
    and its MoveIt object_id, so re-querying a prompt refreshes that target
    rather than piling up beside it."""
    return re.sub(r"[^a-z0-9]+", "_", prompt.lower()).strip("_")


def find_object(prompt, rgb, depth_m, k):
    """Segment `prompt` and back-project it into camera-frame points."""
    mask, score = sam3_client.detect(rgb, prompt)
    points = pointcloud.deproject(depth_m, k, mask)
    if len(points) == 0:
        raise RuntimeError("no valid depth inside the mask")
    print(f"  detected (score {score:.2f}), {len(points)} points")
    return points


def grasps_for(points, gripper):
    """Ask GraspGenX for grasps, centering the cloud on the way in and
    undoing that on the way out, so the poses come back in `points`' frame."""
    centroid = points.mean(axis=0)
    poses, scores = graspgenx_client.generate(points - centroid, gripper)
    poses = poses.copy()
    poses[:, :3, 3] += centroid
    print(f"  {len(scores)} grasps, best score {scores.max():.2f}")
    return poses, scores


def save_target(name, points, poses, scores, gripper):
    """Write grasps.yaml, cloud.ply and plot.png into the target's directory."""
    directory = TARGETS_DIR / name
    directory.mkdir(parents=True, exist_ok=True)
    saved = grasp_io.save_grasps(directory / "grasps.yaml", poses, scores, points,
                                 BASE_FRAME, gripper, name)
    pointcloud.save_ply(points, directory / "cloud.ply")
    visualize.plot_grasps(points, poses, scores, directory / "plot.png", gripper)
    print(f"  saved top {saved} grasps to {directory}")


def process(prompt, gripper, rgb, depth_m, k, base_from_camera):
    points = find_object(prompt, rgb, depth_m, k)
    poses, scores = grasps_for(points, gripper)
    # Into the robot's base frame, so a saved grasp stays a valid MoveIt pose
    # target even once the head has moved.
    points = pointcloud.transform_points(base_from_camera, points)
    poses = pointcloud.transform_poses(base_from_camera, poses)
    save_target(slug(prompt), points, poses, scores, gripper)


def main(prompts, gripper):
    print("grabbing a frame from the head camera...")
    rgb, depth_m, k, base_from_camera = grab_rgbd()

    for prompt in prompts:
        print(f"\n{prompt}:")
        try:
            process(prompt, gripper, rgb, depth_m, k, base_from_camera)
        except RuntimeError as error:
            print(f"  skipped: {error}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find graspable targets and save their grasps.")
    parser.add_argument("prompts", nargs="+", help="what to look for, e.g. \"pringles can\"")
    parser.add_argument("--gripper", default="hsrc_hand")
    arguments = parser.parse_args()
    main(arguments.prompts, arguments.gripper)
