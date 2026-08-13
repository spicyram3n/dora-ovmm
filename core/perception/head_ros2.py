"""Point the head. The geometry is in head.py; this is the ROS half.

Separate from head.py so the maths stays testable without a robot, and so the
only thing that needs a running controller is the one call that moves.
"""

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from perception import head

ACTION = "head_trajectory_controller/follow_joint_trajectory"
JOINTS = ["head_pan_joint", "head_tilt_joint"]
TRAVEL_TIME = 2.0       # both joints do 1.0 rad/s, and the sweep is under a turn


class Head(Node):
    def __init__(self):
        super().__init__("head_aim")
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def point(self, pan, tilt, seconds=TRAVEL_TIME):
        """Move the head and wait for it to get there. False if the controller
        is not up or the move was rejected."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"'{ACTION}' not up; leaving the head alone")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(pan), float(tilt)]
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points = [point]

        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if not handle.accepted:
            return False
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done, timeout_sec=seconds + 5.0)
        return done.done() and done.result().status == GoalStatus.STATUS_SUCCEEDED

    def look_at(self, target, base_pose):
        """Aim at an (x, y, z) point, given the base at (x, y, yaw) in the same
        frame. Returns False if the joint limits clamped the aim so far that
        the target is out of frame -- worth knowing before believing a
        detector that then finds nothing."""
        base_xy, yaw = base_pose[:2], base_pose[2]
        pan, tilt = head.look_at(target, base_xy, yaw)
        self.point(pan, tilt)
        return head.covers(target, base_xy, yaw)
