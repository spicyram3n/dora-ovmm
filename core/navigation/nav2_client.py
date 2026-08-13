"""Drive the base with Nav2, and ask it what is reachable before driving.

Goals go in the map frame, because bt_navigator's global_frame is map
(nav2_params.yaml). amcl starts with its initial pose at the origin and odom
starts wherever the robot spawned, so map and odom coincide at startup --
which is what lets an odom-frame scene graph be used as a goal directly.

Nav2 owns the base at all times. Nothing here plans arm motion, for the reason
docs/navigation-and-grasping.md gives: MoveIt's whole_body groups drive the
same base controller and the two fight.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

FRAME = "map"
BASE = "base_footprint"

# How far from the commanded pose still counts as having arrived. Must stay
# looser than nav2_params.yaml's own xy_goal_tolerance (0.10), since Nav2
# legitimately declares success anywhere inside that -- a guard set equal to it
# rejects honest arrivals, every candidate "fails", and a robot that parked
# perfectly well reports that it cannot stand anywhere. This is here to catch
# the robot wedged against a sofa a metre out, nothing finer.
GOAL_TOLERANCE = 0.25


def _pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = FRAME
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


class Navigator(Node):
    def __init__(self):
        super().__init__("scene_graph_navigator")
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.driver = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

    def _run(self, client, goal, timeout):
        """Send one action goal and block until it finishes. Returns the
        result wrapper, or None if it was rejected or ran out of time."""
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"nav2 action server '{client._action_name}' not up")
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if not handle.accepted:
            return None
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(self, finished, timeout_sec=timeout)
        return finished.result() if finished.done() else None

    def robot_xy(self, timeout=10.0):
        """The robot's (x, y) in the map frame."""
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while not self.tf_buffer.can_transform(FRAME, BASE, Time()):
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError(f"no tf {FRAME} <- {BASE}; is localisation up?")
            rclpy.spin_once(self, timeout_sec=0.1)
        translation = self.tf_buffer.lookup_transform(FRAME, BASE, Time()).transform.translation
        return translation.x, translation.y

    def robot_pose(self, timeout=10.0):
        """(x, y, yaw) in the map frame. The head needs the yaw as well, since
        pan is measured from wherever the base happens to be facing."""
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        while not self.tf_buffer.can_transform(FRAME, BASE, Time()):
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError(f"no tf {FRAME} <- {BASE}; is localisation up?")
            rclpy.spin_once(self, timeout_sec=0.1)
        transform = self.tf_buffer.lookup_transform(FRAME, BASE, Time()).transform
        t, q = transform.translation, transform.rotation
        return t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def reachable(self, x, y, yaw):
        """Does the global planner find a path to this pose? Far cheaper than
        driving there to find out, and it is what rules out the side of a
        shelf that faces a wall."""
        goal = ComputePathToPose.Goal()
        goal.goal = _pose(x, y, yaw)
        goal.use_start = False          # plan from wherever the robot is now
        outcome = self._run(self.planner, goal, timeout=15.0)
        return (outcome is not None
                and outcome.status == GoalStatus.STATUS_SUCCEEDED
                and len(outcome.result.path.poses) > 0)

    def drive_to(self, x, y, yaw, timeout=180.0):
        goal = NavigateToPose.Goal()
        goal.pose = _pose(x, y, yaw)
        outcome = self._run(self.driver, goal, timeout)
        if outcome is None or outcome.status != GoalStatus.STATUS_SUCCEEDED:
            return False
        here = self.robot_xy()
        off = math.hypot(here[0] - x, here[1] - y)
        if off > GOAL_TOLERANCE:
            print(f"    nav2 reported success but stopped {off:.2f} m short")
            return False
        return True

    def go_to_first_reachable(self, poses):
        """Drive to the first of `poses` the planner accepts. Returns the pose
        reached, or None if every candidate failed."""
        for index, (x, y, yaw) in enumerate(poses, 1):
            if not self.reachable(x, y, yaw):
                print(f"  candidate {index}/{len(poses)} ({x:.2f}, {y:.2f}): no path")
                continue
            print(f"  candidate {index}/{len(poses)} ({x:.2f}, {y:.2f}): driving")
            if self.drive_to(x, y, yaw):
                return x, y, yaw
            print("    navigation failed, trying the next one")
        return None
