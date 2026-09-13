"""Request base-placement candidates from Toyota's IK service (launch/ik_solver.launch.py)."""

import copy
import math
import time
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneWorld
from nav_msgs.msg import OccupancyGrid
from shape_msgs.msg import SolidPrimitive
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tmc_manipulation_msgs.srv import SolveIkWithCollision
from core.utils.transforms import (
    matrix_from_transform,
    pose_from_matrix,

    yaw_from_quaternion,
)

SERVICE = "/ik_solver_node/solve_ik_with_collision"
ARM_JOINTS = [
    "arm_lift_joint",
    "arm_flex_joint",
    "arm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
]
NEUTRAL = [0.0, 0.0, 0.0, -1.57, 0.0]
MAP_YAML = (
    Path(__file__).resolve().parents[2] / "config" / "map" / "apartment_world_map.yaml"
)
COSTMAP_TOPIC = "/global_costmap/costmap"
FREE, BLOCKED = (0, 100)
INSCRIBED, UNKNOWN = (99, -1)


@contextmanager
def ros_node(name):
    """A node that is always torn down, including when the body raises."""
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    node = rclpy.create_node(name)
    try:
        yield node
    finally:
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


def spin_until(node, ready, complaint, timeout):
    """Spin `node` until `ready()` holds, or raise RuntimeError(complaint)."""
    deadline = time.monotonic() + timeout
    while not ready():
        if time.monotonic() > deadline:
            raise RuntimeError(complaint)
        rclpy.spin_once(node, timeout_sec=0.1)


def latest_message(node, message_type, topic, complaint, timeout):
    """Keep the first received latched-topic message."""
    received = {}
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def receive_message(message):
        received.setdefault("message", message)

    def message_arrived():
        return "message" in received

    subscription = node.create_subscription(message_type, topic, receive_message, qos)
    try:
        spin_until(node, message_arrived, complaint, timeout)
    finally:
        node.destroy_subscription(subscription)
    return received["message"]


def load_occupancy_grid(yaml_path=MAP_YAML, frame_id="map"):
    """The saved nav map as an OccupancyGrid, for masking out base positions inside walls."""
    import cv2

    meta = yaml.safe_load(Path(yaml_path).read_text())
    image = cv2.imread(
        str(Path(yaml_path).parent / meta["image"]), cv2.IMREAD_GRAYSCALE
    )
    if image is None:
        raise ValueError(f"Cannot read map image from {yaml_path}")
    if meta.get("mode", "trinary") != "trinary":
        raise ValueError("Offline loading supports trinary maps; use --costmap for other modes")
    if not 0 <= meta["free_thresh"] < meta["occupied_thresh"] <= 1:
        raise ValueError("Invalid map occupancy thresholds")
    if not np.isfinite(meta["resolution"]) or meta["resolution"] <= 0:
        raise ValueError("Map resolution must be positive")
    brightness = np.flipud(image).astype(np.float32) / 255.0
    if meta.get("negate", 0):
        occupancy = brightness
    else:
        occupancy = 1.0 - brightness
    # Only known free pixels are safe; grey/unknown pixels remain blocked.
    free = occupancy < meta["free_thresh"]
    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.resolution = float(meta["resolution"])
    grid.info.height, grid.info.width = occupancy.shape
    grid.info.origin.position.x = float(meta["origin"][0])
    grid.info.origin.position.y = float(meta["origin"][1])
    yaw = float(meta["origin"][2])
    grid.info.origin.orientation.z = math.sin(yaw / 2)
    grid.info.origin.orientation.w = math.cos(yaw / 2)
    grid.data = np.where(free, FREE, BLOCKED).astype(np.int8).ravel().tolist()
    return grid


def costmap_grid(node, topic=COSTMAP_TOPIC, blocked_at=INSCRIBED, timeout=10.0):
    """Nav2's global costmap, rebinned into the free/blocked values the solver expects."""
    grid = latest_message(
        node,
        OccupancyGrid,
        topic,
        f"no costmap on {topic} within {timeout:.0f}s -- is nav2 up?",
        timeout,
    )
    cost = np.asarray(grid.data, dtype=np.int16)
    # Unknown cells must not become candidate base positions.
    blocked = (cost >= blocked_at) | (cost == UNKNOWN)
    grid.data = np.where(blocked, BLOCKED, FREE).astype(np.int8).tolist()
    return grid


def _grid_in_frame(node, grid, goal_frame, timeout=5.0):
    """`grid` with its origin re-expressed in `goal_frame`."""
    if grid.header.frame_id == goal_frame:
        return grid
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def transform_available():
        return buffer.can_transform(goal_frame, grid.header.frame_id, Time())

    spin_until(
        node,
        transform_available,
        f"no {goal_frame} <- {grid.header.frame_id} transform; is navigation up? (hsrb_rosnav_config navigation_launch.py brings up amcl)",
        timeout,
    )
    goal_from_map = matrix_from_transform(
        buffer.lookup_transform(goal_frame, grid.header.frame_id, Time()).transform
    )
    # The grid origin can include yaw as well as translation.
    from geometry_msgs.msg import Transform
    origin = Transform()
    origin.translation.x = grid.info.origin.position.x
    origin.translation.y = grid.info.origin.position.y
    origin.translation.z = grid.info.origin.position.z
    origin.rotation = grid.info.origin.orientation
    map_from_grid = matrix_from_transform(origin)
    shifted = OccupancyGrid()
    shifted.header.frame_id = goal_frame
    shifted.info = copy.deepcopy(grid.info)
    shifted.info.origin = pose_from_matrix(goal_from_map @ map_from_grid)
    shifted.data = grid.data
    return shifted


def _obstacle_map(node, obstacles, goal_frame):
    """Whatever `solve`'s caller asked to mask with, as a grid in `goal_frame`."""
    if obstacles == "--costmap":
        obstacles = costmap_grid(node)
    elif obstacles == "--map":
        obstacles = load_occupancy_grid()
    if obstacles is None:
        return None
    return _grid_in_frame(node, obstacles, goal_frame)


def collision_world(boxes, frame_from_map):
    """Furniture boxes (centre, dimensions, yaw) in map, as solver obstacles in the hand
    goal's frame. The solver checks the whole robot against them, arm included, and
    does not transform them itself."""
    world = PlanningSceneWorld()
    for index, (centre, dimensions, yaw) in enumerate(boxes):
        placed = np.eye(4)
        placed[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        placed[:3, 3] = centre
        box = CollisionObject(id=f"furniture_{index}", operation=CollisionObject.ADD)
        box.pose = pose_from_matrix(frame_from_map @ placed)
        box.primitives.append(SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=[float(size) for size in dimensions]))
        box.primitive_poses.append(Pose())
        world.collision_objects.append(box)
    return world


def _ik_request(hand_goal, obstacle_map, environment):
    request = SolveIkWithCollision.Request()
    request.origin_to_hand_goal = pose_from_matrix(hand_goal)
    if obstacle_map is None:
        request.obstacle_map = OccupancyGrid()
    else:
        request.obstacle_map = obstacle_map
    if environment is None:
        request.environment = PlanningSceneWorld()
    else:
        request.environment = environment
    request.initial_joint_state = JointState(name=ARM_JOINTS, position=NEUTRAL)
    request.initial_origin_to_base.orientation.w = 1.0
    return request


def _base_xy_yaw(result):
    """One ik result's base position as (x, y, yaw)."""
    position = result.origin_to_base.position
    yaw = yaw_from_quaternion(result.origin_to_base.orientation)
    return (position.x, position.y, yaw)


def _solve_one(node, client, hand_goal, obstacle_map, environment, timeout):
    """Every (bases, joints) the solver found for a single hand pose."""
    future = client.call_async(_ik_request(hand_goal, obstacle_map, environment))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        future.cancel()
        raise RuntimeError("IK service timed out; reachability is unknown")
    response = future.result()
    if response is None:
        raise RuntimeError("IK service returned no response")
    found = response.ik_results
    base_poses = []
    joint_positions = []
    for result in found:
        base_poses.append(_base_xy_yaw(result))
        joint_positions.append(result.joint_positions)
    bases = np.array(base_poses).reshape(-1, 3)
    joints = np.array(joint_positions).reshape(-1, len(ARM_JOINTS))
    return (bases, joints)


def solve(poses, obstacles=None, environment=None, timeout=30.0, goal_frame="odom"):
    """For each (4, 4) hand_palm_link pose, return (bases, joints):"""
    if goal_frame != "odom":
        raise ValueError("This IK integration expects odom-frame hand poses")
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4) or not np.isfinite(poses).all():
        raise ValueError("Expected finite hand poses with shape (N, 4, 4)")
    with ros_node("base_placement") as node:
        client = node.create_client(SolveIkWithCollision, SERVICE)
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f"{SERVICE} not available -- is ik_solver.launch.py running?"
            )
        obstacle_map = _obstacle_map(node, obstacles, goal_frame)
        results = []
        for pose in poses:
            results.append(
                _solve_one(node, client, pose, obstacle_map, environment, timeout)
            )
        return results
