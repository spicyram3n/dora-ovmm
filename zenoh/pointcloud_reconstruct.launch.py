"""Rebuild the head RGBD pointcloud from the compressed images bridged over
zenoh. A raw PointCloud2 is ~9.8 MB per message, too big for the link, so we
ship compressed depth + rgb and project here instead.

Publishes /head_rgbd_sensor/depth_registered/rectified_points, same name the
robot uses.

    export ROS_DOMAIN_ID=1
    ros2 launch /home/ws/zenoh/pointcloud_reconstruct.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

RGB = "/head_rgbd_sensor/rgb"
DEPTH = "/head_rgbd_sensor/depth_registered"


def generate_launch_description():
    rgb_republish = Node(
        package="image_transport",
        executable="republish",
        name="rgb_republish",
        arguments=["compressed", "raw"],
        remappings=[
            ("in/compressed", f"{RGB}/image_rect_color/compressed"),
            ("out", f"{RGB}/image_rect_color"),
        ],
        output="screen",
    )

    depth_republish = Node(
        package="image_transport",
        executable="republish",
        name="depth_republish",
        arguments=["compressedDepth", "raw"],
        remappings=[
            ("in/compressedDepth", f"{DEPTH}/image_rect_raw/compressedDepth"),
            ("out", f"{DEPTH}/image_rect_raw"),
        ],
        output="screen",
    )

    pointcloud = ComposableNodeContainer(
        name="pointcloud_reconstruct_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="points_xyzrgb",
                remappings=[
                    ("rgb/image_rect_color", f"{RGB}/image_rect_color"),
                    ("rgb/camera_info", f"{RGB}/camera_info"),
                    ("depth_registered/image_rect", f"{DEPTH}/image_rect_raw"),
                    ("points", f"{DEPTH}/rectified_points"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([rgb_republish, depth_republish, pointcloud])
