"""Compress rectified RGB and registered depth using standard ROS plugins."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="image_transport",
            executable="republish",
            name="rgb_encoder",
            arguments=["raw", "compressed"],
            remappings=[
                ("in", "/head_rgbd_sensor/rgb/image_rect_color"),
                ("out/compressed", "/zenoh/rgbd/rgb/compressed"),
            ],
            parameters=[{"out.jpeg_quality": 85}],
            output="screen",
        ),
        Node(
            package="image_transport",
            executable="republish",
            name="depth_encoder",
            arguments=["raw", "compressedDepth"],
            remappings=[
                ("in", "/head_rgbd_sensor/depth_registered/image_rect_raw"),
                ("out/compressedDepth", "/zenoh/rgbd/depth/compressedDepth"),
            ],
            # Humble's depth plugin uses PNG level 3 and a 10 m cutoff.
            output="screen",
        ),
    ])
