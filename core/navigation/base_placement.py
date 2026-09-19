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
    """Create a temporary ROS node and always release it on exit."""
    # Start ROS only if the caller has not already started it.
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
    """Process ROS messages until ready() succeeds or the timeout expires."""
    # Use a wall-clock deadline while spinning ROS to receive the needed data.
    deadline = time.monotonic() + timeout
    while not ready():
        if time.monotonic() > deadline:
            raise RuntimeError(complaint)
        rclpy.spin_once(node, timeout_sec=0.1)


def latest_message(node, message_type, topic, complaint, timeout):
    """Keep the first received latched-topic message."""
    received = {}
    # Ask for the saved topic sample, even if it was published before we subscribed.
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
    # Flip image rows so the map origin is at the bottom, then scale pixels to 0–1.
    brightness = np.flipud(image).astype(np.float32) / 255.0
    # Interpret image brightness using the map's configured occupied/free convention.
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
    # Copy the saved map position and heading into the grid origin.
    grid.info.origin.position.x = float(meta["origin"][0])
    grid.info.origin.position.y = float(meta["origin"][1])
    yaw = float(meta["origin"][2])
    grid.info.origin.orientation.z = math.sin(yaw / 2)
    grid.info.origin.orientation.w = math.cos(yaw / 2)
    # Flatten the grid into the row order expected by the ROS message.
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
    # Unknown stays UNKNOWN rather than folding into BLOCKED: it must not become
    # a candidate base position either way (standoff.free accepts only FREE), but
    # standoff.unobstructed needs to tell an unseen cell from a wall.
    grid.data = np.where(cost == UNKNOWN, UNKNOWN,
                         np.where(cost >= blocked_at, BLOCKED, FREE)
                         ).astype(np.int8).tolist()
    return grid


def sight_grid(node, topic=COSTMAP_TOPIC, timeout=10.0):
    """Nav2's costmap with only lethal cells blocked, for standoff.unobstructed.

    costmap_grid blocks at INSCRIBED, the collar one robot radius wide that keeps
    the base off a wall. Nothing in that collar is opaque, so a view test run
    against it throws away poses with a clear line to the piece."""
    return costmap_grid(node, topic, blocked_at=BLOCKED, timeout=timeout)


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
    # Combine the grid origin with the transform into the requested frame.
    map_from_grid = matrix_from_transform(origin)
    shifted = OccupancyGrid()
    shifted.header.frame_id = goal_frame
    shifted.info = copy.deepcopy(grid.info)
    shifted.info.origin = pose_from_matrix(goal_from_map @ map_from_grid)
    shifted.data = grid.data
    return shifted


def _obstacle_map(node, obstacles, goal_frame):
    """Load the requested obstacle grid and express its origin in the goal frame."""
    # Choose a live costmap, saved map, supplied grid, or no grid.
    if obstacles == "--costmap":
        obstacles = costmap_grid(node)
    elif obstacles == "--map":
        obstacles = load_occupancy_grid()
    if obstacles is None:
        return None
    return _grid_in_frame(node, obstacles, goal_frame)


def collision_world(boxes, frame_from_map):
    """Turn map-frame furniture boxes into whole-robot obstacles in the hand goal frame."""
    world = PlanningSceneWorld()
    for index, (centre, dimensions, yaw) in enumerate(boxes):
        # Place each furniture box in map coordinates, then transform it for the solver.
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
    # Use empty scene messages when the caller supplies no obstacles.
    if obstacle_map is None:
        request.obstacle_map = OccupancyGrid()
    else:
        request.obstacle_map = obstacle_map
    if environment is None:
        request.environment = PlanningSceneWorld()
    else:
        request.environment = environment
    # Seed the solver with the neutral arm pose and an unrotated base.
    request.initial_joint_state = JointState(name=ARM_JOINTS, position=NEUTRAL)
    request.initial_origin_to_base.orientation.w = 1.0
    return request


def _base_xy_yaw(result):
    """One ik result's base position as (x, y, yaw)."""
    position = result.origin_to_base.position
    yaw = yaw_from_quaternion(result.origin_to_base.orientation)
    return (position.x, position.y, yaw)


def _solve_one(node, client, hand_goal, obstacle_map, environment, timeout):
    """Return the base poses and arm joints found for one hand pose."""
    future = client.call_async(_ik_request(hand_goal, obstacle_map, environment))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    # Cancel a timed-out request because missing results cannot establish reachability.
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
    # Return matching rows of base poses and arm-joint positions.
    bases = np.array(base_poses).reshape(-1, 3)
    joints = np.array(joint_positions).reshape(-1, len(ARM_JOINTS))
    return (bases, joints)


def solve(poses, obstacles=None, environment=None, timeout=30.0, goal_frame="odom"):
    """For each (4, 4) hand_palm_link pose, return (bases, joints):"""
    if goal_frame != "odom":
        raise ValueError("This IK integration expects odom-frame hand poses")
    poses = np.asarray(poses, dtype=float)
    # Require one finite 4x4 matrix for every candidate hand pose.
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
        # Ask for base and arm solutions separately for each candidate hand pose.
        for pose in poses:
            results.append(
                _solve_one(node, client, pose, obstacle_map, environment, timeout)
            )
        return results
