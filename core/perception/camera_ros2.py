"""Capture synchronized RGB-D and transform it at the depth image timestamp."""

import time
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from utils.transforms import matrix_from_transform
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

RGB_TOPIC = "/head_rgbd_sensor/rgb/image_rect_color"
DEPTH_TOPIC = "/head_rgbd_sensor/depth_registered/image_rect_raw"
CAMERA_INFO_TOPIC = "/head_rgbd_sensor/rgb/camera_info"
BASE_FRAME = "odom"


def grab_rgbd(target_frame=BASE_FRAME, timeout=15, use_sim_time=True):
    """Return BGR, depth in metres, intrinsics, and T_target_camera."""
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = Node(
        "grasp_pipeline_camera",
        parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
    )
    try:
        bridge = CvBridge()
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        frames = {}
        capture_start = node.get_clock().now().nanoseconds
        rgb_sub = message_filters.Subscriber(
            node, Image, RGB_TOPIC, qos_profile=qos_profile_sensor_data
        )
        depth_sub = message_filters.Subscriber(
            node, Image, DEPTH_TOPIC, qos_profile=qos_profile_sensor_data
        )
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], 10, 0.05
        )

        def receive_images(rgb, depth):
            frames["rgb"] = rgb
            frames["depth"] = depth

        def receive_info(message):
            frames["info"] = message

        sync.registerCallback(receive_images)
        info_sub = node.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, receive_info, qos_profile_sensor_data
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not {"rgb", "depth", "info"} <= frames.keys():
                continue
            rgb, depth, info = (frames["rgb"], frames["depth"], frames["info"])
            # Reject queued images from before this capture began.
            rgb_time = Time.from_msg(rgb.header.stamp).nanoseconds
            depth_time = Time.from_msg(depth.header.stamp).nanoseconds
            if min(rgb_time, depth_time) <= capture_start:
                continue
            # Use the camera pose when the image was captured, not its pose now.
            stamp = Time.from_msg(depth.header.stamp)
            frame = depth.header.frame_id
            if not buffer.can_transform(target_frame, frame, stamp):
                continue
            if rgb.header.frame_id != frame or info.header.frame_id != frame:
                raise ValueError("Expected depth registered into the RGB optical frame")
            image = bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
            depth_values = bridge.imgmsg_to_cv2(
                depth, desired_encoding="passthrough"
            ).astype(np.float32)
            if depth.encoding not in ("16UC1", "32FC1"):
                raise ValueError(f"Unsupported depth encoding: {depth.encoding}")
            if depth_values.shape != image.shape[:2]:
                raise ValueError("RGB and registered depth dimensions differ")
            # Integer depth is millimetres; floating-point depth is metres.
            if depth.encoding == "16UC1":
                depth_meters = depth_values / 1000
            else:
                depth_meters = depth_values
            intrinsics = np.array(info.k).reshape(3, 3)
            transform = buffer.lookup_transform(target_frame, frame, stamp).transform
            return (image, depth_meters, intrinsics, matrix_from_transform(transform))
        raise RuntimeError(
            f"No synchronized RGB-D with TF to {target_frame} within {timeout}s"
        )
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()
