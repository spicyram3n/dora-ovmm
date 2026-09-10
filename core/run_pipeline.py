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
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from grasping import grasp_io, graspgenx_client, visualize
from perception import pointcloud, sam3_client
from perception.camera_ros2 import BASE_FRAME, grab_rgbd

TARGETS_DIR = Path(__file__).resolve().parent.parent / "config" / "targets"


class NoReachableApproach(RuntimeError):
    pass


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


def main(prompts, gripper, execute=False, lift=.02, retreat=0., recover_base=False):
    started = time.monotonic()
    print("grabbing a frame from the head camera...")
    rgb, depth_m, k, base_from_camera = grab_rgbd()

    succeeded = True
    for prompt in prompts:
        print(f"\n{prompt}:")
        try:
            grasp_file = process(prompt, gripper, rgb, depth_m, k, base_from_camera)
            if execute:
                if not yaml.safe_load(grasp_file.read_text()).get('target_geometry'):
                    raise RuntimeError('Feedback pickup currently requires a depth-fitted upright can')
                # Only this call's newly generated file is eligible for motion.
                # The executor rejects an existing target/attachment, requires
                # an occupied scene, and validates the complete approach.
                reached = False
                grasp_index = 0
                no_candidate = False
                separated_target = False
                with subprocess.Popen(
                        ['ros2', 'launch', 'grasp_execution', 'move_to_grasp.launch.py',
                         f'grasp_file:={grasp_file}', 'stop_before_close:=true'],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as approach:
                    for line in approach.stdout:
                        print(line, end='', flush=True)
                        reached |= 'stop_before_close set, parking with the hand open' in line
                        match = re.search(r'reached grasp (\d+)/', line)
                        if match:
                            grasp_index = int(match.group(1))
                        no_candidate |= 'candidate grasps were reachable' in line and 'none of the' in line
                        separated_target |= 'separating only the target' in line
                    if approach.wait() != 0 or not reached:
                        if no_candidate and not separated_target:
                            raise NoReachableApproach('No complete approach in this fresh batch')
                        raise RuntimeError('Approach did not confirm reaching the open-hand contact pose')
                subprocess.run([sys.executable, str(Path(__file__).with_name('finish_pick.py')),
                                str(grasp_file), '--lift', str(lift), '--retreat', str(retreat),
                                '--prompt', prompt, '--grasp-index', str(grasp_index), '--execute'], check=True)
        except NoReachableApproach as error:
            print(error)
            if recover_base:
                try:
                    subprocess.run([sys.executable, str(Path(__file__).with_name('grasp_recovery.py')),
                                    str(grasp_file)], check=True)
                except subprocess.CalledProcessError:
                    print('  failed: bounded base recovery could not complete')
                    return False
            return None  # CLI may capture a new frame and generate a new batch.
        except (RuntimeError, ValueError, subprocess.CalledProcessError) as error:
            succeeded = False
            print(f"  failed: {error}")
    print(f"run completed in {time.monotonic() - started:.2f}s; success={succeeded}")
    return succeeded


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find graspable targets and save their grasps.")
    parser.add_argument("prompts", nargs="+", help="what to look for, e.g. \"pringles can\"")
    parser.add_argument("--gripper", default="hsrc_hand")
    parser.add_argument("--output-dir", type=Path, default=TARGETS_DIR,
                        help="Root directory for this run's target artifacts")
    parser.add_argument("--execute", action="store_true",
                        help="Approach, close with feedback, lift, and verify one upright can")
    parser.add_argument("--lift", type=float, default=.02, help="Checked lift distance in metres (default 2 cm)")
    parser.add_argument("--retreat", type=float, default=0., help="Checked shelf withdrawal along odom -X (0 to .25 m)")
    parser.add_argument("--attempts", type=int, default=3,
                        help="Maximum fresh batches when no approach is reachable (execution only)")
    arguments = parser.parse_args()
    if arguments.execute and (len(arguments.prompts) != 1 or not .01 <= arguments.lift <= .05):
        parser.error("Execution requires one prompt and a lift between .01 and .05 metres")
    if not 0 <= arguments.retreat <= .25:
        parser.error("Retreat must be between 0 and .25 metres")
    if not 1 <= arguments.attempts <= 5:
        parser.error("Attempts must be between 1 and 5")
    root = arguments.output_dir.resolve()
    for attempt in range(arguments.attempts if arguments.execute else 1):
        TARGETS_DIR = root if attempt == 0 else root / f'attempt_{attempt+1}'
        result = main(arguments.prompts, arguments.gripper, arguments.execute,
                      arguments.lift, arguments.retreat,
                      recover_base=arguments.execute and attempt == 0 and arguments.attempts > 1)
        if result is not None:
            break
    sys.exit(0 if result is True else 1)
