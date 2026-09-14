"""Generate grasp previews, or execute the consolidated MTC pickup.

    python3 core/run_pipeline.py "spray bottle" --execute --mode grasp
    python3 -m core.run_pipeline "pringles can" --execute --mode pickup

Without --execute, save perception previews for one or more prompts.
Execution delegates perception, planning, closure and verification to pick.py.
Requires a sourced ROS environment, SAM3 and GraspGenX; execution also requires
MoveIt and paused navigation, as documented by core.grasping.pick.
"""

import argparse
import re
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.grasping import grasp_io, graspgenx_client, visualize
from core.perception import pointcloud, sam3_client
from core.perception.camera_ros2 import BASE_FRAME, grab_rgbd

TARGETS_DIR = Path(__file__).resolve().parent.parent / "config" / "targets"


def slug(prompt):
    """"big pringles can" -> "big_pringles_can". Names the target's directory
    and its MoveIt object_id, so re-querying a prompt refreshes that target
    rather than piling up beside it."""
    return re.sub(r"[^a-z0-9]+", "_", prompt.lower()).strip("_")


def find_object(prompt, rgb, depth_m, k):
    """Segment `prompt` and back-project it into camera-frame points."""
    started = time.monotonic()
    mask, score = sam3_client.detect(rgb, prompt)
    print(f"  SAM3: {time.monotonic() - started:.2f}s")
    mask = pointcloud.object_depth_mask(depth_m, mask)
    points = pointcloud.deproject(depth_m, k, mask)
    if len(points) == 0:
        raise RuntimeError("no valid depth inside the mask")
    print(f"  detected (score {score:.2f}), {len(points)} points")
    return points


def grasps_for(points, gripper):
    """Ask GraspGenX for grasps, centering the cloud on the way in and
    undoing that on the way out, so the poses come back in `points`' frame."""
    # GraspGenX's memory use grows with cloud size, and 20k+ points from a
    # close-range scan OOM an 8GB GPU shared with SAM3. ~8k points are
    # plenty for grasp generation.
    if len(points) > 8192:
        points = points[np.random.choice(len(points), 8192, replace=False)]
    centroid = points.mean(axis=0)
    started = time.monotonic()
    poses, scores = graspgenx_client.generate(points - centroid, gripper)
    print(f"  GraspGenX: {time.monotonic() - started:.2f}s")
    poses = poses.copy()
    poses[:, :3, 3] += centroid
    print(f"  {len(scores)} grasps, best score {scores.max():.2f}")
    return poses, scores


def save_target(name, points, poses, scores, gripper, scene_points=None):
    """Write grasps.yaml, cloud.ply and plot.png into the target's directory."""
    directory = TARGETS_DIR / name
    directory.mkdir(parents=True, exist_ok=True)
    saved = grasp_io.save_grasps(directory / "grasps.yaml", poses, scores, points,
                                 BASE_FRAME, gripper, name, scene_points=scene_points)
    pointcloud.save_ply(points, directory / "cloud.ply")
    visualize.plot_grasps(points, poses, scores, directory / "plot.png", gripper)
    print(f"  saved top {saved} grasps to {directory}")
    return directory / "grasps.yaml"


def process(prompt, gripper, rgb, depth_m, k, base_from_camera):
    points = find_object(prompt, rgb, depth_m, k)
    poses, scores = grasps_for(points, gripper)
    # Into the robot's base frame, so a saved grasp stays a valid MoveIt pose
    # target even once the head has moved.
    points = pointcloud.transform_points(base_from_camera, points)
    poses = pointcloud.transform_poses(base_from_camera, poses)
    scene_points = pointcloud.transform_points(base_from_camera,
        pointcloud.deproject(depth_m, k, np.isfinite(depth_m) & (depth_m > 0)))
    return save_target(slug(prompt), points, poses, scores, gripper, scene_points=scene_points)


def main(prompts, gripper="hsrc_hand", execute=False, mode="auto"):
    """Return pick.py's exit code: 0 picked, 3 contact-only, 1 failed."""
    if execute:
        if len(prompts) != 1 or gripper != "hsrc_hand":
            raise ValueError("Execution requires one target and the calibrated hsrc_hand gripper")
        if mode not in ("auto", "pickup", "grasp"):
            raise ValueError("Unknown grasp mode")
        environment = os.environ.copy()
        environment["HSR_GRASP_DIAGNOSTICS"] = str(TARGETS_DIR.resolve() / slug(prompts[0]))
        result = subprocess.run(
            [sys.executable, "-m", "core.grasping.pick", prompts[0], "--mode", mode],
            cwd=ROOT, env=environment, check=False)
        # Do not retry after motion or turn a contact-only hold into pickup success.
        return result.returncode if result.returncode in (0, 3) else 1

    started = time.monotonic()
    print("grabbing a frame from the head camera...")
    rgb, depth_m, k, base_from_camera = grab_rgbd()
    succeeded = True
    for prompt in prompts:
        print(f"\n{prompt}:")
        try:
            process(prompt, gripper, rgb, depth_m, k, base_from_camera)
        except (RuntimeError, ValueError) as error:
            succeeded = False
            print(f"  failed: {error}")
    print(f"run completed in {time.monotonic() - started:.2f}s; success={succeeded}")
    return 0 if succeeded else 1


def cli():
    global TARGETS_DIR
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prompts", nargs="+", help="target description")
    parser.add_argument("--gripper", default="hsrc_hand")
    parser.add_argument("--output-dir", type=Path, default=TARGETS_DIR,
                        help="Root directory for previews or execution diagnostics")
    parser.add_argument("--execute", action="store_true", help="Execute through core.grasping.pick")
    parser.add_argument("--mode", choices=["auto", "pickup", "grasp"], default="auto",
                        help="Auto lifts cylinders; grasp stops at contact; pickup verifies a lift")
    arguments = parser.parse_args()
    if arguments.execute and (len(arguments.prompts) != 1 or arguments.gripper != "hsrc_hand"):
        parser.error("Execution requires one target and the calibrated hsrc_hand gripper")
    TARGETS_DIR = arguments.output_dir.resolve()
    return main(arguments.prompts, arguments.gripper, arguments.execute, arguments.mode)


if __name__ == "__main__":
    sys.exit(cli())
