#!/usr/bin/env python3
"""Save one /nbv/occupied message to a .ply, for rerun or meshlab.

    ros2 run nbv save_occupied.py --ros-args -p path:=/home/ws/outputs/octomap.ply
"""

import numpy as np
import rclpy
import trimesh
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy

TOPIC = "/nbv/occupied"


def main():
    rclpy.init()
    node = Node("save_occupied")
    path = node.declare_parameter("path", "/home/ws/outputs/octomap.ply").value
    received, message = wait_for_message(PointCloud2, node, TOPIC, time_to_wait=30.0)
    if not received:
        raise SystemExit(f"nothing on {TOPIC}; is build_octomap.py running?")
    points = read_points_numpy(message, ["x", "y", "z"])
    trimesh.PointCloud(points).export(path)
    print(f"{len(points)} cells -> {path}")


if __name__ == "__main__":
    main()
