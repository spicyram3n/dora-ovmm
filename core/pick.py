"""Plan the arm to a saved grasp, and optionally execute it.

Usage, from core/:
    python3 pick.py pringles_can                 # plan only, move nothing
    python3 pick.py pringles_can --execute       # plan, then run the winner
    python3 pick.py pringles_can --group arm     # override the planning group

Plans to the *pregrasp*, not the grasp. A GraspGenX pose is a contact pose with
the gripper already closed on the object, and no collision-aware planner will
ever plan into that; the last few centimetres are a separate approach segment
that this does not do yet.

Why `whole_body_weighted` and not `arm`: kinematics.yaml gives the `arm` group
the KDL solver, and `arm` is a five-joint chain. A pose in space needs six, so
KDL is being asked for something that generally does not exist and fails almost
every time. `whole_body_weighted` uses HSR's own analytic solver and adds the
three base joints, but weights x and y at 1e9 -- base translation is
effectively forbidden while yaw stays free, which is exactly the missing sixth
degree of freedom. Since the weights only bias the search rather than
constrain the result, the plan is checked for base translation before it runs
and rejected if it drifts (see BASE_DRIFT_LIMIT).

Nav2 must be idle. Both this and Nav2 command the same base controller, and
nothing arbitrates between them.
"""

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
import trimesh
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, OrientationConstraint,
                             PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from grasping import grasp_io, select

TARGETS = Path(__file__).resolve().parent.parent / "config" / "targets"

GROUP = "whole_body_weighted"
EEF_LINK = "hand_palm_link"

# How far the base may translate in a returned plan before it is thrown away.
# The 1e9 joint weights make base translation expensive, not impossible.
BASE_DRIFT_LIMIT = 0.05

POSITION_TOLERANCE = 0.01     # m
ORIENTATION_TOLERANCE = 0.05  # rad


def _goal_constraints(pose, frame_id):
    """A pose target, as the position + orientation constraint pair MoveGroup
    actually wants. Built by hand since there are no Python bindings here."""
    position = PositionConstraint()
    position.header.frame_id = frame_id
    position.link_name = EEF_LINK
    position.weight = 1.0
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [POSITION_TOLERANCE]
    volume = BoundingVolume()
    volume.primitives = [sphere]
    volume.primitive_poses = [pose.pose]
    position.constraint_region = volume

    orientation = OrientationConstraint()
    orientation.header.frame_id = frame_id
    orientation.link_name = EEF_LINK
    orientation.orientation = pose.pose.orientation
    orientation.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
    orientation.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
    orientation.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
    orientation.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [position]
    constraints.orientation_constraints = [orientation]
    return constraints


def _pose_stamped(matrix, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = matrix[:3, 3]
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = q
    return pose


def _base_drift(trajectory):
    """Largest base translation anywhere in a planned trajectory, in metres."""
    joints = trajectory.joint_trajectory.joint_names
    if "odom_x" not in joints or "odom_y" not in joints:
        return 0.0
    ix, iy = joints.index("odom_x"), joints.index("odom_y")
    points = trajectory.joint_trajectory.points
    if not points:
        return 0.0
    start = (points[0].positions[ix], points[0].positions[iy])
    return max(math.hypot(p.positions[ix] - start[0], p.positions[iy] - start[1])
               for p in points)


class Picker(Node):
    def __init__(self, group):
        super().__init__("pick")
        self.group = group
        self.mover = ActionClient(self, MoveGroup, "move_action")
        self.executor_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

    def _run(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"'{client._action_name}' not up -- is move_group running?")
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if not handle.accepted:
            return None
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done, timeout_sec=timeout)
        return done.result() if done.done() else None

    def base_xy(self, frame, timeout=10.0):
        """The robot's (x, y) in the frame the grasps are saved in."""
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while not self.tf_buffer.can_transform(frame, "base_footprint", Time()):
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError(f"no tf {frame} <- base_footprint")
            rclpy.spin_once(self, timeout_sec=0.1)
        t = self.tf_buffer.lookup_transform(frame, "base_footprint", Time()).transform.translation
        return t.x, t.y

    def plan(self, pose_matrix, frame_id):
        """Plan to a pose without moving. Returns the trajectory, or None."""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group
        goal.request.goal_constraints = [
            _goal_constraints(_pose_stamped(pose_matrix, frame_id), frame_id)]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.planning_options.plan_only = True

        outcome = self._run(self.mover, goal, timeout=30.0)
        if outcome is None or outcome.result.error_code.val != 1:
            code = "timed out" if outcome is None else outcome.result.error_code.val
            return None, code
        return outcome.result.planned_trajectory, 1

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        outcome = self._run(self.executor_client, goal, timeout=120.0)
        return outcome is not None and outcome.status == GoalStatus.STATUS_SUCCEEDED


def main(target, execute, group, standoff):
    directory = TARGETS / target
    poses, scores, data = grasp_io.load_grasps(directory / "grasps.yaml")
    points = np.asarray(trimesh.load(directory / "cloud.ply").vertices)
    frame = data["frame_id"]
    print(f"{target}: {len(scores)} saved grasps in '{frame}', "
          f"best score {scores.max():.2f}")

    rclpy.init()
    node = Picker(group)
    try:
        base = node.base_xy(frame)
        print(f"base at ({base[0]:.2f}, {base[1]:.2f}) in {frame}")

        counts = select.report(poses, scores, points, base)
        print(f"filter: {counts}")
        order = select.feasible(poses, scores, points, base)
        if not order:
            short = select.shortfall(poses, scores, points, base)
            if not short:
                print("nothing is graspable from any base position -- the object is "
                      "outside the arm's envelope, not just out of reach from here")
                return
            i, gap, where = min(short, key=lambda row: row[1])
            print(f"nothing reachable from here. Closest is candidate {i} "
                  f"(score {scores[i]:.2f}), {gap:.2f} m out of its band.")
            print(f"  stand at ({where[0]:.2f}, {where[1]:.2f}) in {frame} and retry, "
                  f"or use navigation.reach.move_into_reach")
            return

        for rank, (i, standoff) in enumerate(order, 1):
            pregrasp = select.pregrasp(poses[i], standoff)
            print(f"\ncandidate {rank}/{len(order)} (score {scores[i]:.2f}): "
                  f"planning '{group}' to a {standoff:.2f} m pregrasp...")
            trajectory, code = node.plan(pregrasp, frame)
            if trajectory is None:
                print(f"  no plan (MoveItErrorCodes {code})")
                continue

            drift = _base_drift(trajectory)
            if drift > BASE_DRIFT_LIMIT:
                print(f"  rejected: plan drives the base {drift:.2f} m "
                      f"(limit {BASE_DRIFT_LIMIT})")
                continue
            print(f"  planned: {len(trajectory.joint_trajectory.points)} points, "
                  f"base drift {drift:.3f} m")

            if not execute:
                print("  --execute not given, stopping here")
                return
            print("  executing...")
            if node.execute(trajectory):
                print(f"reached the pregrasp for candidate {rank}. "
                      "The approach segment is not implemented yet.")
                return
            print("  execution failed, trying the next candidate")

        print("\nevery feasible candidate failed to plan")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plan the arm to a saved grasp.")
    parser.add_argument("target", help="a directory under config/targets/")
    parser.add_argument("--execute", action="store_true", help="actually move the arm")
    parser.add_argument("--group", default=GROUP)
    parser.add_argument("--standoff", type=float, default=select.STANDOFF)
    arguments = parser.parse_args()
    main(arguments.target, arguments.execute, arguments.group, arguments.standoff)
