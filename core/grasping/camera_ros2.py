"""
The one ROS 2-dependent piece of this pipeline: grabs a single RGB frame,
depth frame, and camera intrinsics from the HSRC head camera, plus the TF
transform from the camera into the robot's base frame. Everything else in
core/grasping/ only talks zenoh + numpy, so swapping this module out is all
that's needed to run the pipeline on different middleware.
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

RGB_TOPIC = "/head_rgbd_sensor/rgb/image_rect_color"
DEPTH_TOPIC = "/head_rgbd_sensor/depth_registered/image_rect_raw"
# Depth is registered to the rgb frame, so the rgb intrinsics apply to it directly.
CAMERA_INFO_TOPIC = "/head_rgbd_sensor/rgb/camera_info"
# "odom" is the root robot_description.py attaches the virtual base joints
# to, and what MoveIt's "whole_body" group plans in.
BASE_FRAME = "odom"


def _matrix_from_transform(transform):
    t, q = transform.translation, transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def grab_rgbd():
    """Wait for one message on each topic, then look up the camera's pose in
    the robot's base frame.

    Returns (rgb, depth_m, k, base_from_camera):
        rgb:              (H, W, 3) uint8 bgr8
        depth_m:          (H, W) float32 meters (0 = no return)
        k:                (3, 3) float64 camera intrinsic matrix
        base_from_camera: (4, 4) float64, maps points in the camera frame
                          into BASE_FRAME
    """
    rclpy.init()
    node = Node("grasp_pipeline_camera")
    bridge = CvBridge()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    frame = {}

    def on_rgb(msg):
        frame["rgb"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_depth(msg):
        raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # HSR's real driver publishes 16UC1 depth in millimeters; the Gazebo
        # sim publishes 32FC1 in meters already.
        is_mm = msg.encoding == "16UC1"
        frame["depth"] = raw.astype(np.float32) / 1000.0 if is_mm else raw.astype(np.float32)

    def on_info(msg):
        frame["k"] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        frame["camera_frame"] = msg.header.frame_id

    node.create_subscription(Image, RGB_TOPIC, on_rgb, 1)
    node.create_subscription(Image, DEPTH_TOPIC, on_depth, 1)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, on_info, 1)

    print("[camera_ros2] waiting for rgb + depth + camera_info...")
    while not {"rgb", "depth", "k"} <= frame.keys():
        rclpy.spin_once(node)

    print(f"[camera_ros2] waiting for tf {BASE_FRAME} <- {frame['camera_frame']}...")
    while not tf_buffer.can_transform(BASE_FRAME, frame["camera_frame"], Time()):
        rclpy.spin_once(node, timeout_sec=0.1)
    transform = tf_buffer.lookup_transform(BASE_FRAME, frame["camera_frame"], Time())

    node.destroy_node()
    rclpy.shutdown()
    return frame["rgb"], frame["depth"], frame["k"], _matrix_from_transform(transform.transform)
