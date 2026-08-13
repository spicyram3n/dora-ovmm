"""The one ROS 2-dependent piece of the pipeline: grabs a single RGB frame,
depth frame and intrinsics from the HSRC head camera, plus the transform from
the camera into the robot's base frame. Everything else in core/ speaks only
zenoh and numpy, so this module is all there is to swap to move middleware."""

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
# Depth is registered to the rgb frame, so the rgb intrinsics apply to it too.
CAMERA_INFO_TOPIC = "/head_rgbd_sensor/rgb/camera_info"
# What robot_description.py attaches the virtual base joints to, and what
# MoveIt's "whole_body" group plans in.
BASE_FRAME = "odom"


def _matrix(transform):
    translation, rotation = transform.translation, transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def grab_rgbd(target_frame=BASE_FRAME):
    """Wait for one message on each camera topic, then look up where the
    camera is in the robot's base frame.

    `target_frame` is what the camera is resolved into: BASE_FRAME for grasps,
    which MoveIt plans in, but "map" for anything handed to Nav2. The two only
    coincide at startup and drift apart as amcl corrects.

    Returns (rgb, depth_m, k, base_from_camera):
        rgb:              (H, W, 3) uint8 bgr8
        depth_m:          (H, W) float32 meters, 0 where there was no return
        k:                (3, 3) intrinsic matrix
        base_from_camera: (4, 4) mapping camera-frame points into target_frame
    """
    # A caller that already runs ROS (search_object holds a Navigator) keeps
    # ownership of the context; tearing it down here would kill their node.
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = Node("grasp_pipeline_camera")
    bridge = CvBridge()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    frame = {}

    def on_rgb(msg):
        frame["rgb"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_depth(msg):
        raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
        # The real HSR driver publishes 16UC1 millimeters; Gazebo publishes
        # 32FC1 meters already.
        frame["depth"] = raw / 1000.0 if msg.encoding == "16UC1" else raw

    def on_info(msg):
        frame["k"] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        frame["camera_frame"] = msg.header.frame_id

    node.create_subscription(Image, RGB_TOPIC, on_rgb, 1)
    node.create_subscription(Image, DEPTH_TOPIC, on_depth, 1)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, on_info, 1)

    print("waiting for rgb + depth + camera_info...")
    while not {"rgb", "depth", "k"} <= frame.keys():
        rclpy.spin_once(node)

    print(f"waiting for tf {target_frame} <- {frame['camera_frame']}...")
    while not tf_buffer.can_transform(target_frame, frame["camera_frame"], Time()):
        rclpy.spin_once(node, timeout_sec=0.1)
    transform = tf_buffer.lookup_transform(target_frame, frame["camera_frame"], Time())

    node.destroy_node()
    if owns_context:
        rclpy.shutdown()
    return frame["rgb"], frame["depth"], frame["k"], _matrix(transform.transform)
