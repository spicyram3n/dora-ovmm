#!/usr/bin/env python3
"""Print where the robot thinks it is, in the map frame.

    ros2 run nbv where_am_i.py

A nav2 goal is absolute in `map`, not relative to the robot, so this is what
you check before picking coordinates for go_to_pose.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

TOPIC = "/amcl_pose"


def main():
    rclpy.init()
    node = Node("where_am_i")
    latest = {}

    # amcl latches its last estimate, so a transient-local subscription gets it
    # straight away instead of waiting for the robot to move enough to publish.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped, TOPIC,
                             lambda msg: latest.setdefault("pose", msg), qos)

    print(f"waiting for {TOPIC}, is nav2 up and the initial pose set?")
    while "pose" not in latest:
        rclpy.spin_once(node)

    pose = latest["pose"].pose.pose
    # Planar, so yaw is the only angle and reads straight off the quaternion.
    yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
    print(f"x {pose.position.x:+.3f}   y {pose.position.y:+.3f}   "
          f"yaw {yaw:+.3f} rad ({math.degrees(yaw):+.1f} deg)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
