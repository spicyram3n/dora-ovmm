"""Drive the HSR to next best views around a queried object, then report the
best reachable grasp: active_grasp's GraspController.search_grasp for our stack.

    python3 -m core.active_perception.explore "pringles can"

Needs Nav2 localised in map, SAM3 and GraspGenX, and the IK solver
(launch/ik_solver.launch.py) unless --no-ik. In simulation:

    ros2 launch /home/ws/launch/search.launch.py web:=true      # sim, Nav2, IK, MoveIt, no mission
    python3 -m core.active_perception.explore "pringles can"    # in a second terminal

Each step fuses the current depth image, scores the views the head can take,
drives the base under the chosen one, lifts the torso to its height, aims the
head at the target and captures again, until the gain or grasp criterion stops it.
"""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

from core.active_perception.bbox import AABBox
from core.active_perception.grasps import graspgenx_grasps, ik_reachable
from core.active_perception.nbv import NextBestView
from core.active_perception.record import Recording
from core.active_perception.views import CAMERA_HEIGHT, ViewHalfSphere
from core.navigation import base_placement
from core.navigation.nav2_client import Navigator
from core.scene_graph import graph as sg
from core.perception import sam3_client
from core.perception.camera_ros2 import grab_rgbd
from core.perception.pointcloud import deproject, largest_cluster, shrink, transform_points

# arm_lift_joint raises the torso, and with it the head, by half its travel.
ARM_LIFT_MAX = 0.69
HOME = [0.0, -1.57, -1.57, 0.0]  # arm_flex, arm_roll, wrist_flex, wrist_roll: actions.home_goal
OUTPUTS = Path("outputs/active_perception")
GRAPH = Path(__file__).resolve().parents[2] / "config/scene_graph/kitchen_objects.json"


def furniture_footprints(scene):
    """Every piece of furniture as (centre, dimensions, yaw): the base keeps out of them."""
    return [sg.footprint(data) for data in sg.furniture(scene).values()]


def detect_box(prompt, rgb, depth, k, map_from_camera):
    mask, score = sam3_client.detect(rgb, prompt)
    points = largest_cluster(deproject(depth, k, shrink(mask)))
    if points is None:
        raise RuntimeError(f"'{prompt}' has too little depth to box")
    return AABBox.from_points(transform_points(map_from_camera, points)), score


def grow_box(policy, prompt, rgb, depth, k, map_from_camera):
    """Widen the target box with what the new view shows of the object.

    ETH's box is the object's true extent from the start; ours comes from one
    view, so a can seen head-on is a 2 cm deep sliver and almost no voxel
    counts as inside it. The cube stays where the first box put it."""
    try:
        seen, _ = detect_box(prompt, rgb, depth, k, map_from_camera)
    except (sam3_client.ObjectNotFound, RuntimeError):
        return False
    grown = AABBox(np.minimum(policy.bbox.min, seen.min), np.maximum(policy.bbox.max, seen.max))
    if np.allclose(grown.size, policy.bbox.size):
        return False
    policy.bbox = grown
    return True


def set_torso(navigator, rise):
    """Lift the head by `rise` metres with the arm otherwise at its home pose."""
    lift = float(np.clip(2 * rise, 0.0, ARM_LIFT_MAX))
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
                                   "wrist_flex_joint", "wrist_roll_joint"]
    goal.trajectory.points = [JointTrajectoryPoint(positions=[lift, *HOME], time_from_start=Duration(sec=4))]
    client = ActionClient(navigator, FollowJointTrajectory, "/arm_trajectory_controller/follow_joint_trajectory")
    try:
        outcome = navigator._run(client, goal, 30)
        return outcome is not None and outcome.status == GoalStatus.STATUS_SUCCEEDED
    finally:
        client.destroy()


def go_to_view(navigator, view, target):
    """Put the camera where `view` (map_from_camera) wants it: base under the
    eye, facing along the optical axis, torso at its height, head on the target."""
    eye, forward = (view[:3, 3], view[:3, 2])
    yaw = math.atan2(forward[1], forward[0])
    lifted = set_torso(navigator, eye[2] - CAMERA_HEIGHT)
    reached = navigator.drive_to(float(eye[0]), float(eye[1]), yaw)
    aimed = navigator.look_at(target)
    return dict(lifted=lifted, reached=reached, aimed=aimed)


def explore(navigator, prompt, length=0.3, min_z_dist=0.6, max_steps=8, qual_thresh=0.9,
            grasps=True, ik=True, sim=True, out=None, rerun=False, blockers=()):
    """The NBV loop on a live robot; returns the log dict, with `best_grasp`.

    `navigator` is a running core.navigation.nav2_client.Navigator. The robot
    must already see the object: this is where the mission's Find step ends."""
    out = out or OUTPUTS / time.strftime("explore_%Y%m%d_%H%M%S")
    out.mkdir(parents=True)
    rgb, depth, k, map_from_camera = grab_rgbd(target_frame="map", use_sim_time=sim)
    bbox, score = detect_box(prompt, rgb, depth, k, map_from_camera)
    print(f"[NBV] '{prompt}' ({score:.2f}) in a {np.round(bbox.size, 2)} m box at {np.round(bbox.center, 2)}", flush=True)
    sphere = ViewHalfSphere(bbox, min_z_dist, grid=base_placement.costmap_grid(navigator), blockers=blockers)
    policy = NextBestView(
        k, length=length, qual_thresh=qual_thresh,
        grasp_fn=graspgenx_grasps if grasps else None,
        reachable_fn=ik_reachable(navigator) if (grasps and ik) else None,
    )
    policy.activate(bbox, sphere)
    # Always recorded to the run folder; `rerun` also opens the viewer live.
    recording = Recording(k, depth.shape, path=out / "explore.rrd", spawn=rerun)
    recording.scene(policy)
    log = dict(prompt=prompt, bbox=bbox.to_dict(), radii=sphere.radii.tolist(),
               feasible_views=len(sphere.candidates()), steps=[], best_grasp=None)
    print(f"[NBV] {log['feasible_views']} views the robot can take on radii {np.round(sphere.radii, 2)} m", flush=True)
    for step in range(1, max_steps + 1):
        policy.update(depth, map_from_camera)
        recording.step(step, policy, depth, map_from_camera, image=rgb)
        record = dict(step=step, camera=map_from_camera[:3, 3].round(3).tolist(), done=policy.done,
                      grasp_quality=None if policy.best_grasp is None else round(policy.best_grasp[1], 3))
        if policy.info:
            record.update(candidates=len(policy.info["views"]),
                          gain=int(policy.info["gains"][policy.info["best"]]),
                          next_eye=policy.x_d[:3, 3].round(3).tolist())
        else:
            record["stopped"] = ("no view the robot can take" if not sphere.candidates()
                                 else "no view gains anything" if not policy.done else "policy done")
        log["steps"].append(record)
        print("[NBV] " + json.dumps(record), flush=True)
        if policy.done or not policy.info:
            break
        record["motion"] = go_to_view(navigator, policy.x_d, bbox.center)
        if not record["motion"]["reached"]:
            sphere.rejected.append(policy.x_d[:3, 3].copy())  # ETH's arm never fails to move; a base can
        rgb, depth, k, map_from_camera = grab_rgbd(target_frame="map", use_sim_time=sim)
        if grow_box(policy, prompt, rgb, depth, k, map_from_camera):
            print(f"[NBV] box grown to {np.round(policy.bbox.size, 3)} m", flush=True)
    origin = policy.base_from_task[:3, 3]
    o3d.io.write_point_cloud(str(out / "scene_cloud.ply"), policy.tsdf.get_scene_cloud().translate(origin))
    np.save(out / "grid.npy", policy.tsdf.get_grid())
    if policy.best_grasp is not None:
        log["best_grasp"] = dict(quality=policy.best_grasp[1], palm_pose=policy.best_grasp[0].tolist())
    log["views_fused"] = len(policy.views)
    log["final_bbox"] = policy.bbox.to_dict()
    (out / "explore.json").write_text(json.dumps(log, indent=2))
    print(f"[NBV] {len(policy.views)} views fused; outputs in {out}", flush=True)
    return log


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prompt")
    parser.add_argument("--length", type=float, default=0.3, help="TSDF cube side, m (VGN: 0.3)")
    parser.add_argument("--min-z-dist", type=float, default=0.6, help="closest camera range, m")
    parser.add_argument("--max-steps", type=int, default=8)
    parser.add_argument("--qual-thresh", type=float, default=0.9, help="grasp quality kept (ETH: 0.9)")
    parser.add_argument("--no-ik", action="store_true", help="skip the IK reachability filter on grasps")
    parser.add_argument("--no-grasps", action="store_true")
    parser.add_argument("--real", action="store_true", help="wall clock instead of sim time")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--rerun", action="store_true", help="open the Rerun viewer and stream the loop live")
    parser.add_argument("--graph", type=Path, default=GRAPH, help="scene graph whose furniture the base keeps clear of")
    args = parser.parse_args()

    rclpy.init()
    navigator = Navigator(use_sim_time=not args.real)
    try:
        log = explore(navigator, args.prompt, length=args.length, min_z_dist=args.min_z_dist,
                      max_steps=args.max_steps, qual_thresh=args.qual_thresh, grasps=not args.no_grasps,
                      ik=not args.no_ik, sim=not args.real, out=args.output, rerun=args.rerun,
                      blockers=furniture_footprints(sg.load(args.graph)))
        if log["best_grasp"] is None:
            print("no reachable grasp on the target")
        else:
            pose = np.array(log["best_grasp"]["palm_pose"])
            print(f"best grasp quality {log['best_grasp']['quality']:.3f} at {pose[:3, 3].round(3)}")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
