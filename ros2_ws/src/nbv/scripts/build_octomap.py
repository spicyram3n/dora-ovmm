#!/usr/bin/env python3
"""Map the workspace from the head RGBD stream with OctoMap, and report the
sphere next best view cares about.

    ros2 run nbv build_octomap.py --ros-args \
        -p sphere_centre:="[0.51, 0.0, 1.16]" -p sphere_radius:=0.5

Every ray goes into the map, not only those ending in the sphere: a ray that
crosses the sphere to a wall behind it is what makes that stretch free.
Publishes the occupied cells inside the sphere on /nbv/occupied as a
PointCloud2 and the whole map on /nbv/octomap, to view in RViz, and answers
/nbv/unknown_fraction for any sphere. Needs octomap-python
(pip install --user octomap-python).
"""

import array
import sys
from pathlib import Path

import numpy as np
import octomap
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from nbv.srv import NextView, UnknownFraction
from octomap_msgs.msg import Octomap
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parent))
import next_view  # noqa: E402

# The depth image, not the cloud topic: a third of the bytes for the same
# points, and the stride below then skips pixels before they are ever
# deprojected. hsr_rgbd's relay names the simulator's topic, which the robot
# itself publishes under a different name.
DEPTH = "/head_rgbd_sensor/depth_registered/image"
INFO = "/head_rgbd_sensor/depth_registered/camera_info"
RESOLUTION = 0.05
# Keep every 4th pixel each way. At 5 cm cells the camera is under 2 m away,
# so a full frame is many points per cell regardless.
STRIDE = 4
MAX_RANGE = 2.5
# Rays with no return end at MAX_RANGE. OctoMap truncates rays past this, so
# those come out free along their length with nothing occupied at the end.
FREE_RANGE = MAX_RANGE - 0.01
# What tree.getLabels reports for each cell it is asked about.
UNKNOWN, FREE, OCCUPIED = -1, 0, 1


def deproject(rows, columns, depth, info, stride=STRIDE):
    """Pixel indices and their depth to (n, 3) points in the camera frame."""
    # Intrinsics index the full image, so undo the stride.
    x = (columns * stride - info.k[2]) * depth / info.k[0]
    y = (rows * stride - info.k[5]) * depth / info.k[4]
    return np.stack([x, y, depth], axis=1)


def sphere_cells(tree, centre, radius):
    """Centres of the map's own cells inside a sphere, one per cell."""
    resolution = tree.getResolution()
    centre = np.asarray(centre, dtype=np.float64)
    low = np.asarray(tree.keyToCoord(tree.coordToKey(centre - radius)))
    steps = np.arange(-resolution, 2 * radius + resolution, resolution)
    axes = [low[axis] + steps for axis in range(3)]
    cells = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return cells[np.linalg.norm(cells - centre, axis=1) <= radius]


def unknown_in_sphere(tree, centre, radius):
    """(cells never seen, all cells) of the map inside a sphere."""
    cells = sphere_cells(tree, centre, radius)
    if len(cells) == 0:
        return (0, 0)
    return (int(np.sum(tree.getLabels(cells) == UNKNOWN)), len(cells))


class OctomapBuilder(Node):
    def __init__(self):
        super().__init__("nbv_octomap")
        # map once nav2 is up; odom or base_footprint when it is not.
        self.frame = self.declare_parameter("frame", "map").value
        centre = self.declare_parameter("sphere_centre", [0.0, 0.0, 0.8]).value
        radius = self.declare_parameter("sphere_radius", 0.6).value
        self.centre = np.array(centre, dtype=np.float64)
        self.radius = float(radius)
        self.tree = octomap.OcTree(RESOLUTION)
        # Fixed for the node's life, and every tick asks about all of them.
        self.cells = sphere_cells(self.tree, self.centre, self.radius)
        self.bridge = CvBridge()
        self.info = None

        self.buffer = tf2_ros.Buffer()
        # Own thread, so the lookup below can wait for tf without blocking
        # the very callbacks that deliver it.
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=True)
        self.publisher = self.create_publisher(PointCloud2, "/nbv/occupied", 1)
        self.sphere_publisher = self.create_publisher(Marker, "/nbv/sphere", 1)
        self.octomap_publisher = self.create_publisher(Octomap, "/nbv/octomap", 1)
        # Callbacks run one at a time, so a query never sees a half-inserted frame.
        self.create_service(UnknownFraction, "/nbv/unknown_fraction", self.on_unknown_fraction)
        self.create_service(NextView, "/nbv/next_view", self.on_next_view)
        # The camera publishes best effort, and a reliable subscription would
        # match nothing at all.
        self.create_subscription(CameraInfo, INFO, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH, self.on_depth, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f"sphere r={radius} at {centre} in {self.frame}, "
            f"{len(self.cells)} cells")

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        if self.info is None:
            return
        try:
            # The image's own stamp, not the latest: the head moves while it
            # scans, and a stale transform smears the surfaces. The timeout is
            # because tf for that stamp lands a few ms after the image does.
            transform = self.buffer.lookup_transform(
                self.frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.1)).transform
        except tf2_ros.TransformException as error:
            self.get_logger().warn(f"no transform: {error}", throttle_duration_sec=5.0)
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = depth[::STRIDE, ::STRIDE].astype(np.float32)
        # Integer depth is millimetres; floating point is metres.
        if msg.encoding == "16UC1":
            depth /= 1000
        # Missing depth is 0, NaN or inf, and all three fail this, as does
        # anything past the range we would keep. One test, before any
        # arithmetic.
        seen = (depth > 0) & (depth < MAX_RANGE)
        rows, columns = np.nonzero(seen)
        points = deproject(rows, columns, depth[rows, columns], self.info)
        # A pixel that came back with nothing on it saw empty space, not
        # unknown space: carve its ray out to the range we trust. Skipping
        # these leaves the air in front of the camera unknown for ever.
        # Every other one of those: free space is bulk, and even at this
        # density the rays stay under a cell apart across the sphere.
        rows, columns = np.nonzero(~seen[::2, ::2])
        empty = deproject(rows, columns, np.full(len(rows), MAX_RANGE),
                          self.info, 2 * STRIDE)

        sensor = np.array([getattr(transform.translation, axis) for axis in "xyz"])
        rotation = Rotation.from_quat(
            [getattr(transform.rotation, axis) for axis in "xyzw"]).as_matrix().T
        cloud = np.vstack([points, empty]) @ rotation + sensor
        self.tree.insertPointCloud(cloud, sensor, maxrange=FREE_RANGE)

    def publish(self):
        # One label per cell of the sphere, which is both what to draw and how
        # much is still unseen. Asking the map only about these cells keeps the
        # tick independent of how far the robot has driven.
        labels = self.tree.getLabels(self.cells)
        occupied = self.cells[labels == OCCUPIED]
        # Carry each cell's probability as intensity, so a cell can be read off
        # in RViz (colour by intensity, or click it) instead of only seen.
        probability = [self.tree.search(cell).getOccupancy() for cell in occupied]
        points = np.column_stack([occupied, probability])
        header = Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())
        fields = [PointField(name=name, offset=4 * index,
                             datatype=PointField.FLOAT32, count=1)
                  for index, name in enumerate(("x", "y", "z", "intensity"))]
        self.publisher.publish(create_cloud(header, fields, points))
        self.sphere_publisher.publish(self.sphere(header))
        # Serialising the whole map costs tens of milliseconds and grows with
        # it, so only pay when an RViz is attached to see the result.
        if self.octomap_publisher.get_subscription_count():
            self.octomap_publisher.publish(self.octomap(header))
        self.get_logger().info(
            f"occupied {len(occupied)}  unknown {int(np.sum(labels == UNKNOWN))}",
            throttle_duration_sec=5.0)

    def on_unknown_fraction(self, request, response):
        centre = [request.centre.x, request.centre.y, request.centre.z]
        response.unknown, response.total = unknown_in_sphere(self.tree, centre, request.radius)
        if response.total:
            response.fraction = response.unknown / response.total
        return response

    def standing_at(self):
        """Where the base is now, as (x, y, yaw) in the map's frame."""
        transform = self.buffer.lookup_transform(self.frame, "base_footprint",
                                                 Time()).transform
        rotation = Rotation.from_quat(
            [getattr(transform.rotation, axis) for axis in "xyzw"])
        return (transform.translation.x, transform.translation.y,
                rotation.as_euler("xyz")[2])

    def on_next_view(self, request, response):
        if self.info is None:
            self.get_logger().warn("no camera_info yet, so no view can be scored")
            return response
        try:
            standing = self.standing_at()
        except tf2_ros.TransformException as error:
            self.get_logger().warn(f"no base transform: {error}")
            return response
        centre = np.array([request.centre.x, request.centre.y, request.centre.z])
        cells = sphere_cells(self.tree, centre, request.radius)
        options = next_view.candidates(self.tree, centre, standing)
        best, response.bits = next_view.best_view(self.tree, cells, self.info,
                                                  options, standing)
        if best is None:
            self.get_logger().warn(f"nowhere to stand that sees {centre}")
            return response
        response.found = True
        response.base_x, response.base_y, response.base_yaw = best.base
        response.lift, response.pan, response.tilt = best.lift, best.pan, best.tilt
        return response

    def sphere(self, header):
        """The region being mapped, as a see-through ball for RViz."""
        marker = Marker(header=header, type=Marker.SPHERE, action=Marker.ADD)
        marker.ns = "nbv"
        (marker.pose.position.x, marker.pose.position.y,
         marker.pose.position.z) = self.centre.tolist()
        marker.pose.orientation.w = 1.0
        diameter = 2 * self.radius
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        marker.color.g = marker.color.b = 1.0
        marker.color.a = 0.15
        return marker

    def octomap(self, header):
        """The whole map as octomap_msgs, for RViz's OccupancyMap display."""
        # The message carries OctoMap's binary data without the file header in front.
        _, data = self.tree.writeBinary().split(b"data\n", 1)
        message = Octomap(header=header, binary=True, id="OcTree",
                          resolution=self.tree.getResolution())
        message.data = array.array("b", data)
        return message


def main():
    rclpy.init()
    try:
        rclpy.spin(OctomapBuilder())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
