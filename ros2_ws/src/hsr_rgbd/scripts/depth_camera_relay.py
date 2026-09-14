#!/usr/bin/env python3
"""Publish smaller depth/CameraInfo pairs for the simulation octomap updater.

The original depth stream is also kept for full-resolution grasp perception.
No point-cloud topic is consumed or produced.
"""
from copy import deepcopy
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


class DepthCameraRelay(Node):
    def __init__(self):
        super().__init__('octomap_depth_camera')
        self.info = None
        self.pending_image = None
        self.published_first_frame = False
        self.bridge = CvBridge()
        # The simulator bridges depth under a different name than the robot.
        depth = self.declare_parameter(
            'depth_topic', '/head_rgbd_sensor/depth_registered/image').value
        rate = float(self.declare_parameter('publish_rate', 1.0).value)
        # core/grasping/pick.py sets this false to freeze the octomap while the arm moves.
        self.declare_parameter('enabled', True)
        self.max_depth = float(self.declare_parameter('max_depth', 2.5).value)
        if not math.isfinite(self.max_depth) or self.max_depth <= 0:
            raise ValueError('max_depth must be finite and positive')
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate must be finite and positive')
        self.create_timer(1.0 / rate, self.publish_latest)
        self.image_pub = self.create_publisher(Image, '/octomap_camera/image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/octomap_camera/camera_info', 1)
        # Reliable input avoids losing fragmented 1.2 MB simulator images.
        self.create_subscription(CameraInfo, '/head_rgbd_sensor/depth_registered/camera_info', self.on_info, 1)
        self.depth_topic = depth
        self.last_input = time.monotonic()
        self.image_sub = self.create_subscription(Image, depth, self.on_image, 1)
        self.watchdog = self.create_timer(2., self.check_input,
                                         clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.last_input = time.monotonic()
        self.pending_image = msg

    def check_input(self):
        if self.get_parameter('enabled').value and time.monotonic()-self.last_input > 10.:
            self.get_logger().warning('Depth input stalled; renewing image subscription')
            self.destroy_subscription(self.image_sub)
            self.pending_image = None
            self.image_sub = self.create_subscription(Image, self.depth_topic, self.on_image, 1)
            self.last_input = time.monotonic()

    def publish_latest(self):
        """Publish the newest depth frame once; never replay a queued backlog."""
        if self.info is None or self.pending_image is None:
            return
        if not self.get_parameter('enabled').value:
            return
        msg = self.pending_image
        self.pending_image = None
        if (msg.width, msg.height, msg.header.frame_id) != (self.info.width, self.info.height, self.info.header.frame_id):
            self.get_logger().error('Depth/calibration dimensions or frames differ')
            return
        # Decimation selects actual samples, without averaging depth edges.
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        sampled = np.ascontiguousarray(raw[::2, ::2]).copy()
        # Keep the manipulation map local. Distant apartment surfaces at 1 cm
        # resolution previously grew beyond the DDS planning-scene limit.
        limit = self.max_depth * (1000 if msg.encoding == '16UC1' else 1)
        sampled[sampled > limit] = 0
        encoding = msg.encoding
        if encoding == '32FC1':
            # Millimetres retain sub-voxel precision for the 1 cm map and halve
            # the fragmented payload received by MoveIt's best-effort reader.
            sampled = np.rint(np.clip(np.nan_to_num(sampled, nan=0., posinf=0., neginf=0.)*1000,
                                      0., 65535.)).astype(np.uint16)
            encoding = '16UC1'
        image = self.bridge.cv2_to_imgmsg(sampled, encoding=encoding)
        image.header = msg.header
        info = deepcopy(self.info)
        info.header = msg.header
        info.width, info.height = image.width, image.height
        for index in (0, 1, 2, 3, 4, 5):
            info.k[index] /= 2
        for index in range(8):
            info.p[index] /= 2
        self.image_pub.publish(image)
        self.info_pub.publish(info)
        if not self.published_first_frame:
            self.get_logger().info('Publishing latest depth/calibration pairs for the octomap')
            self.published_first_frame = True


def main():
    rclpy.init()
    node = DepthCameraRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
