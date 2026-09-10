"""The one ROS 2-dependent piece of the pipeline: grabs a single RGB frame,
depth frame and intrinsics from the HSRC head camera, plus the transform from
the camera into the robot's base frame. Everything else in core/ speaks only
zenoh and numpy, so this module is all there is to swap to move middleware."""

import os
import time
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

RGB_TOPIC = os.environ.get("RGB_TOPIC", "/head_rgbd_sensor/rgb/image_rect_color")
# The HSRC ignition sim bridges depth as ".../depth_registered/image" instead
# of the real robot's ".../image_rect_raw"; override via DEPTH_TOPIC there.
DEPTH_TOPIC = os.environ.get("DEPTH_TOPIC", "/head_rgbd_sensor/depth_registered/image_rect_raw")
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


def grab_rgbd():
    """Wait for one message on each camera topic, then look up where the
    camera is in the robot's base frame.

    Returns (rgb, depth_m, k, base_from_camera):
        rgb:              (H, W, 3) uint8 bgr8
        depth_m:          (H, W) float32 meters, 0 where there was no return
        k:                (3, 3) intrinsic matrix
        base_from_camera: (4, 4) mapping camera-frame points into BASE_FRAME
    """
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = Node("grasp_pipeline_camera")
    bridge = CvBridge()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    frame = {}

    rgb_messages, depth_messages = deque(maxlen=10), deque(maxlen=10)

    def match():
        if "pair" in frame or not rgb_messages or not depth_messages:
            return
        stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pairs = [(abs(stamp(r) - stamp(d)), r, d)
                 for r in rgb_messages for d in depth_messages]
        delta, rgb, depth = min(pairs, key=lambda item: (item[0], -stamp(item[2])))
        if delta <= 0.05:
            frame["pair"] = (rgb, depth)
            print(f"rgb/depth timestamp difference: {delta:.4f}s")

    def on_rgb(msg):
        rgb_messages.append(msg)
        match()

    def on_depth(msg):
        depth_messages.append(msg)
        match()

    def on_info(msg):
        frame["info"] = msg

    node.create_subscription(Image, RGB_TOPIC, on_rgb, 10)
    node.create_subscription(Image, DEPTH_TOPIC, on_depth, 10)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, on_info, 1)

    print("waiting for synchronized rgb + depth + camera_info...")
    deadline = time.monotonic() + 60
    try:
        while not {"pair", "info"} <= frame.keys():
            if time.monotonic() > deadline:
                raise RuntimeError("timed out waiting for synchronized RGB-D")
            rclpy.spin_once(node, timeout_sec=0.1)
        rgb_msg, depth_msg = frame["pair"]
        info = frame["info"]
        if ((rgb_msg.height, rgb_msg.width) != (depth_msg.height, depth_msg.width) or
                (info.height, info.width) != (depth_msg.height, depth_msg.width) or
                info.header.frame_id != depth_msg.header.frame_id):
            raise RuntimeError("RGB-D registration/calibration dimensions or frames differ")
        camera_frame = depth_msg.header.frame_id
        stamp = Time.from_msg(depth_msg.header.stamp)
        print(f"waiting for tf {BASE_FRAME} <- {camera_frame} at depth timestamp...")
        while not tf_buffer.can_transform(BASE_FRAME, camera_frame, stamp):
            if time.monotonic() > deadline:
                raise RuntimeError("camera TF unavailable at captured depth timestamp")
            rclpy.spin_once(node, timeout_sec=0.1)
            # The first images can predate this listener's first dynamic TF.
            # Refresh the pair instead of waiting for an unavailable past TF.
            frame.pop("pair", None)
            match()
            if "pair" in frame:
                rgb_msg, depth_msg = frame["pair"]
                stamp = Time.from_msg(depth_msg.header.stamp)
        transform = tf_buffer.lookup_transform(BASE_FRAME, camera_frame, stamp)
        rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        raw = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float32)
        if depth_msg.encoding not in ("16UC1", "32FC1"):
            raise RuntimeError(f"unsupported depth encoding: {depth_msg.encoding}")
        depth = raw / 1000.0 if depth_msg.encoding == "16UC1" else raw
        k = np.array(info.k, dtype=np.float64).reshape(3, 3)
        return rgb, depth, k, _matrix(transform.transform)
    finally:
        node.destroy_node()
        if owns_context:
            rclpy.shutdown()
