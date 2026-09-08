"""Map-frame Nav2 goals with path checks and cancellation before retrying."""

import math
import time
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

FRAME = "map"
BASE = "base_footprint"


def _pose(x, y, yaw):
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("Navigation goals must be finite")
    pose = PoseStamped()
    pose.header.frame_id = FRAME
    pose.pose.position.x, pose.pose.position.y = (float(x), float(y))
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


class Navigator(Node):

    def __init__(self, use_sim_time=True):
        super().__init__(
            "scene_graph_navigator",
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
        )
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.driver = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

    def _run(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError("Nav2 action server is unavailable")
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10)
        if not sent.done():

            def cancel_late(future):
                handle = future.result()
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()

            # A delayed acceptance must not leave an untracked goal running.
            sent.add_done_callback(cancel_late)
            rclpy.spin_until_future_complete(self, sent, timeout_sec=10)
            raise RuntimeError("Goal acceptance timed out; stop Nav2 before retrying")
        handle = sent.result()
        if handle is None:
            raise RuntimeError("Nav2 returned no goal acknowledgement")
        if not handle.accepted:
            return None
        finished = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, finished, timeout_sec=timeout)
        except KeyboardInterrupt:
            self._cancel(handle, finished)
            raise
        if not finished.done():
            self._cancel(handle, finished)
            return None
        return finished.result()

    def _cancel(self, handle, finished):
        cancelled = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancelled, timeout_sec=5)
        rclpy.spin_until_future_complete(self, finished, timeout_sec=5)
        # Do not send another goal until the previous motion has ended.
        if not finished.done() or finished.cancelled() or finished.exception() is not None:
            raise RuntimeError(
                "Navigation cancellation unconfirmed; refusing another goal"
            )

    def robot_pose(self, timeout=10, max_age=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.tf_buffer.can_transform(FRAME, BASE, Time()):
                continue
            try:
                stamped = self.tf_buffer.lookup_transform(FRAME, BASE, Time())
            except TransformException:
                continue
            age = (self.get_clock().now() - Time.from_msg(stamped.header.stamp)).nanoseconds / 1e9
            # A cached pose after a connection loss is not an arrival measurement.
            if 0 <= age <= max_age:
                transform = stamped.transform
                break
        else:
            raise RuntimeError("No fresh map-to-base TF; check localization and clocks")
        translation, quaternion = (transform.translation, transform.rotation)
        yaw = math.atan2(
            2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        return (translation.x, translation.y, yaw)

    def robot_xy(self):
        return self.robot_pose()[:2]

    def reachable(self, x, y, yaw):
        goal = ComputePathToPose.Goal()
        goal.goal = _pose(x, y, yaw)
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.use_start = False
        outcome = self._run(self.planner, goal, 15)
        if outcome is None or outcome.status != GoalStatus.STATUS_SUCCEEDED:
            return False
        path = outcome.result.path
        if not path.poses or path.header.frame_id != FRAME:
            return False
        end = path.poses[-1].pose.position
        return math.hypot(end.x - x, end.y - y) <= 0.1

    def drive_to(self, x, y, yaw, timeout=180):
        goal = NavigateToPose.Goal()
        goal.pose = _pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        outcome = self._run(self.driver, goal, timeout)
        if outcome is None or outcome.status != GoalStatus.STATUS_SUCCEEDED:
            return False
        current_x, current_y, heading = self.robot_pose()
        error = abs(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        return math.hypot(current_x - x, current_y - y) <= 0.3 and error <= 0.3

    def look_at(self, point):
        """Aim the HSRC head approximately at a map-frame point."""
        x, y, yaw = self.robot_pose()
        if not self.tf_buffer.can_transform(FRAME, "head_pan_link", Time()):
            raise RuntimeError("No head TF available")
        pivot = self.tf_buffer.lookup_transform(FRAME, "head_pan_link", Time()).transform.translation
        bearing = math.atan2(point[1] - pivot.y, point[0] - pivot.x) - yaw
        pan = math.atan2(math.sin(bearing), math.cos(bearing))
        # HSRC RGB-D camera sits roughly 0.25 m above the head pivot.
        distance = math.hypot(point[0] - pivot.x, point[1] - pivot.y)
        tilt = math.atan2(point[2] - pivot.z - 0.248, distance)
        if not (-3.84 <= pan <= 1.75 and -1.57 <= tilt <= 0.52):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[pan, tilt], time_from_start=Duration(sec=5))]
        client = ActionClient(self, FollowJointTrajectory,
                              "/head_trajectory_controller/follow_joint_trajectory")
        try:
            result = self._run(client, goal, 15)
            return (result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
                    and result.result.error_code == 0)
        finally:
            client.destroy()

    def go_to_first_reachable(self, poses):
        for index, pose in enumerate(poses, 1):
            print(f"  candidate {index}/{len(poses)}: {pose}")
            if self.reachable(*pose) and self.drive_to(*pose):
                return pose
        return None
