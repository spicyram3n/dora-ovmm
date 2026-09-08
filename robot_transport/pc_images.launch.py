"""Decode bridged images into the raw topics consumed by camera_ros2.py."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="image_transport",
            executable="republish",
            name="rgb_decoder",
            arguments=["compressed", "raw"],
            remappings=[
                ("in/compressed", "/zenoh/rgbd/rgb/compressed"),
                ("out", "/head_rgbd_sensor/rgb/image_rect_color"),
            ],
            output="screen",
        ),
        Node(
            package="image_transport",
            executable="republish",
            name="depth_decoder",
            arguments=["compressedDepth", "raw"],
            remappings=[
                ("in/compressedDepth", "/zenoh/rgbd/depth/compressedDepth"),
                ("out", "/head_rgbd_sensor/depth_registered/image_rect_raw"),
            ],
            output="screen",
        ),
    ])
