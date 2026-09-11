#!/usr/bin/env python3
"""Publish smaller depth/CameraInfo pairs for the simulation octomap updater.

The original depth stream is also kept for full-resolution grasp perception.
No point-cloud topic is consumed or produced.
"""
from copy import deepcopy
import math
import numpy as np
import rclpy
from rclpy.node import Node
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
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate must be finite and positive')
        self.create_timer(1.0 / rate, self.publish_latest)
        self.image_pub = self.create_publisher(Image, '/octomap_camera/image', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/octomap_camera/camera_info', 1)
        # Reliable input avoids losing fragmented 1.2 MB simulator images.
        self.create_subscription(CameraInfo, '/head_rgbd_sensor/depth_registered/camera_info', self.on_info, 1)
        self.create_subscription(Image, depth, self.on_image, 1)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.pending_image = msg

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
        image = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(raw[::2, ::2]), encoding=msg.encoding)
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
