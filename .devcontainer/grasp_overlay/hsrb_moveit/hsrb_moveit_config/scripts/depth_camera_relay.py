#!/usr/bin/env python3
"""Publish smaller depth/CameraInfo pairs for the simulation octomap updater.

The original depth stream is also kept for full-resolution grasp perception.
No point-cloud topic is consumed or produced.
"""
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


class DepthCameraRelay(Node):
    def __init__(self):
        super().__init__('octomap_depth_camera')
        self.info = None
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/octomap_camera/image', 5)
        self.info_pub = self.create_publisher(CameraInfo, '/octomap_camera/camera_info', 5)
        # Reliable input avoids losing fragmented 1.2 MB simulator images.
        self.create_subscription(CameraInfo, '/head_rgbd_sensor/depth_registered/camera_info', self.on_info, 5)
        self.create_subscription(Image, '/head_rgbd_sensor/depth_registered/image', self.on_image, 5)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        if self.info is None:
            return
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
