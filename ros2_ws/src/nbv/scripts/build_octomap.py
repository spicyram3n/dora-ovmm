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
import numpy as np
import octomap
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from nbv.srv import UnknownFraction
from octomap_msgs.msg import Octomap
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

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


def deproject(rows, columns, depth, info):
    """Pixel indices and their depth to (n, 3) points in the camera frame."""
    # Intrinsics index the full image, so undo the stride.
    x = (columns * STRIDE - info.k[2]) * depth / info.k[0]
    y = (rows * STRIDE - info.k[5]) * depth / info.k[4]
    return np.stack([x, y, depth], axis=1)


def sphere_cells(tree, centre, radius):
    """Centres of the map's own cells inside a sphere, one per cell."""
    resolution = tree.getResolution()
    centre = np.asarray(centre, dtype=np.float64)
    low = np.asarray(tree.keyToCoord(tree.coordToKey(centre - radius)))
    steps = np.arange(-resolution, 2 * radius + resolution, resolution)
    axes = []
    for axis in range(3):
        axes.append(low[axis] + steps)
    cells = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return cells[np.linalg.norm(cells - centre, axis=1) <= radius]


def unknown_in_sphere(tree, centre, radius):
    """(cells never seen, all cells) of the map inside a sphere."""
    cells = sphere_cells(tree, centre, radius)
    if len(cells) == 0:
        return (0, 0)
    return (int(np.sum(tree.getLabels(cells) == -1)), len(cells))


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
        # The camera publishes best effort, and a reliable subscription would
        # match nothing at all.
        self.create_subscription(CameraInfo, INFO, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH, self.on_depth, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f"sphere r={radius} at {centre} in {self.frame}, "
            f"{len(sphere_cells(self.tree, self.centre, self.radius))} cells")

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
        empty = deproject(rows * 2, columns * 2,
                          np.full(len(rows), MAX_RANGE, np.float32), self.info)

        sensor = np.float32([getattr(transform.translation, axis) for axis in "xyz"])
        rotation = Rotation.from_quat(
            [getattr(transform.rotation, axis) for axis in "xyzw"]).as_matrix()
        rotation = rotation.T.astype(np.float32)
        cloud = np.vstack([points @ rotation + sensor, empty @ rotation + sensor])
        self.tree.insertPointCloud(cloud.astype(np.float64), sensor.astype(np.float64),
                                   maxrange=FREE_RANGE)

    def publish(self):
        occupied, _ = self.tree.extractPointCloud()
        occupied = occupied[np.linalg.norm(occupied - self.centre, axis=1) <= self.radius]
        # Carry each cell's probability as intensity, so a cell can be read off
        # in RViz (colour by intensity, or click it) instead of only seen.
        probability = []
        for point in occupied:
            probability.append(self.tree.search(point).getOccupancy())
        points = np.column_stack([occupied, probability]).reshape(-1, 4)
        header = Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())
        fields = [PointField(name=name, offset=4 * index,
                             datatype=PointField.FLOAT32, count=1)
                  for index, name in enumerate(("x", "y", "z", "intensity"))]
        self.publisher.publish(create_cloud(header, fields, points))
        self.sphere_publisher.publish(self.sphere(header))
        self.octomap_publisher.publish(self.octomap(header))
        unknown, _ = unknown_in_sphere(self.tree, self.centre, self.radius)
        self.get_logger().info(f"occupied {len(occupied)}  unknown {unknown}",
                               throttle_duration_sec=5.0)

    def on_unknown_fraction(self, request, response):
        centre = [request.centre.x, request.centre.y, request.centre.z]
        response.unknown, response.total = unknown_in_sphere(self.tree, centre, request.radius)
        if response.total:
            response.fraction = response.unknown / response.total
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
