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
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from core.utils import events
from core.utils.transforms import matrix_from_transform

FRAME = "map"
BASE = "base_footprint"
# config/nav2/nav2_params.yaml keeps one goal checker on purpose: a second makes every
# FollowPath abort, because the stock behaviour tree sends an empty checker id.
# Both its tolerances are dynamic, so a precise leg tightens them and puts them back.
CONTROLLER = "/controller_server"
GOAL_CHECKER = "general_goal_checker"


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

    def _lifecycle_call(self, service_type, service, request, timeout=15):
        client = self.create_client(service_type, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"{service} unavailable; motion handoff stopped")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done():
                client.remove_pending_request(future)
                raise RuntimeError(f"{service} timed out; motion handoff stopped")
            return future.result()
        finally:
            self.destroy_client(client)

    def navigation_state(self, name):
        from lifecycle_msgs.srv import GetState
        return self._lifecycle_call(
            GetState, f"/{name}/get_state", GetState.Request()).current_state.id

    def pause_navigation(self):
        """Deactivate Nav2 motion nodes before MoveIt takes control of the base."""
        from lifecycle_msgs.msg import State
        from nav2_msgs.srv import ManageLifecycleNodes
        response = self._lifecycle_call(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.PAUSE), timeout=30)
        if not response.success:
            raise RuntimeError("Nav2 pause rejected; refusing MoveIt handoff")
        for name in ("controller_server", "behavior_server", "bt_navigator", "waypoint_follower"):
            if self.navigation_state(name) != State.PRIMARY_STATE_INACTIVE:
                raise RuntimeError(f"{name} is not inactive; refusing MoveIt handoff")
        print("[HANDOFF] Nav2 motion nodes inactive; MoveIt owns base motion", flush=True)

    def resume_navigation_if_paused(self):
        """Resume a previously paused navigation stack for a new search."""
        from lifecycle_msgs.msg import State
        from nav2_msgs.srv import ManageLifecycleNodes
        if self.navigation_state("controller_server") != State.PRIMARY_STATE_INACTIVE:
            return
        response = self._lifecycle_call(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.RESUME), timeout=30)
        if not response.success:
            raise RuntimeError("Nav2 resume rejected")
        print("[READY] Nav2 resumed for search", flush=True)

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
        except BaseException:
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
        # Do not send another goal until the previous motion has ended. A cancelled
        # goal still completes its result future, carrying STATUS_CANCELED.
        if not finished.done() or finished.exception() is not None:
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
            # A stamp slightly ahead of our clock is fresh, not stale: sim time and
            # the TF publisher tick separately, so allow the same slack either way.
            if abs(age) <= max_age:
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

    def frame_transform(self, target_frame, source_frame, timeout=10):
        """(4, 4) target_from_source, waited for rather than assumed published."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.tf_buffer.can_transform(target_frame, source_frame, Time()):
                continue
            try:
                stamped = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time()
                )
            except TransformException:
                continue
            return matrix_from_transform(stamped.transform)
        raise RuntimeError(
            f"No {target_frame} <- {source_frame} TF; check localization and clocks"
        )

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

    def drive_to(self, x, y, yaw, timeout=300, tolerance=0.3, yaw_tolerance=0.3):
        """Drive and verify. Tolerances must stay at or above the goal checker's."""
        goal = NavigateToPose.Goal()
        goal.pose = _pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        events.emit("drive", pose=[x, y, yaw], reached=None)
        outcome = self._run(self.driver, goal, timeout)
        reached = False
        if outcome is not None and outcome.status == GoalStatus.STATUS_SUCCEEDED:
            offset, error = self.residual(x, y, yaw)
            reached = offset <= tolerance and error <= yaw_tolerance
        events.emit("drive", pose=[x, y, yaw], reached=reached)
        return reached

    def residual(self, x, y, yaw):
        """How far the base actually is from a goal, as (metres, radians)."""
        current_x, current_y, heading = self.robot_pose()
        error = abs(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        return (math.hypot(current_x - x, current_y - y), error)

    def set_goal_tolerance(self, xy, yaw, timeout=10):
        """Retune the running goal checker; both tolerances are dynamic parameters."""
        client = self.create_client(SetParameters, f"{CONTROLLER}/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"{CONTROLLER} parameter service is unavailable")
            request = SetParameters.Request()
            for name, value in (("xy_goal_tolerance", xy), ("yaw_goal_tolerance", yaw)):
                parameter = ParameterMsg(name=f"{GOAL_CHECKER}.{name}")
                parameter.value.type = ParameterType.PARAMETER_DOUBLE
                parameter.value.double_value = float(value)
                request.parameters.append(parameter)
            # A holonomic base needs a small backward correction if it passes
            # a precise parking goal. Normal travel retains forward-only vx.
            reverse = ParameterMsg(name='FollowPath.vx_min')
            reverse.value.type = ParameterType.PARAMETER_DOUBLE
            reverse.value.double_value = -.08 if xy < .1 else 0.
            request.parameters.append(reverse)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done():
                raise RuntimeError("Setting the goal tolerance timed out")
            for result in future.result().results:
                # Driving on with an unknown tolerance would make arrival meaningless.
                if not result.successful:
                    raise RuntimeError(f"Goal tolerance rejected: {result.reason}")
        finally:
            self.destroy_client(client)

    def look_at(self, point):
        """Aim the HSRC head approximately at a map-frame point."""
        x, y, yaw = self.robot_pose()
        if not self.tf_buffer.can_transform(FRAME, "head_pan_link", Time()):
            raise RuntimeError("No head TF available")
        pivot = self.tf_buffer.lookup_transform(FRAME, "head_pan_link", Time()).transform.translation
        bearing = math.atan2(point[1] - pivot.y, point[0] - pivot.x) - yaw
        pan = math.atan2(math.sin(bearing), math.cos(bearing))
        # The asymmetric joint range extends past -pi. A positive bearing
        # outside the upper limit can still be reachable as pan - 2*pi.
        if pan > 1.75 and pan-2*math.pi >= -3.84:
            pan -= 2*math.pi
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
            result = self._run(client, goal, 45)
            return (result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
                    and result.result.error_code == 0)
        finally:
            client.destroy()
