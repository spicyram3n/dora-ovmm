"""Where should the base stand to reach a grasp?

Wraps Toyota's ik_solver_node (see grasp_execution/launch/ik_solver.launch.py).
Given a hand_palm_link pose it returns every base position from which the arm
can reach it collision-free, already filtered to the most robust ones -- the
node scores each candidate by how many neighbouring base cells admit a similar
arm configuration, so what comes back tolerates the base parking a little off.

An empty result means unreachable, and costs well under a millisecond: the
solver rejects a pose from hsrb_analytic_ik's closed-form base-position range
before it searches anything. That makes this a cheap filter to run before
handing a grasp to MoveIt, which takes seconds to reach the same conclusion.

Usage: python3 core/placement/base_placement.py <target> [--costmap|--map]
"""

import sys
import time
from contextlib import contextmanager
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
import yaml
from moveit_msgs.msg import PlanningSceneWorld
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tmc_manipulation_msgs.srv import SolveIkWithCollision

from grasping.grasp_io import load_grasps
from utils.transforms import (matrix_from_transform, pose_from_matrix,
                              translation_matrix, yaw_from_quaternion)

SERVICE = "/ik_solver_node/solve_ik_with_collision"
ARM_JOINTS = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
              "wrist_flex_joint", "wrist_roll_joint"]
# Roughly the "go" stow pose. Only seeds the collision check's starting state;
# the solver searches base positions regardless.
NEUTRAL = [0.0, 0.0, 0.0, -1.57, 0.0]

TARGETS_DIR = Path(__file__).resolve().parents[2] / "config" / "targets"
MAP_YAML = Path(__file__).resolve().parents[2] / "config" / "map" / "apartment_world_map.yaml"

COSTMAP_TOPIC = "/global_costmap/costmap"

# The only two cell values the solver understands: it treats any nonzero cell as
# blocked, so every grid we hand it must already be binned into these.
FREE, BLOCKED = 0, 100
# What nav2 puts in those cells instead. Its costmap_2d_publisher maps lethal to
# 100 and inscribed to 99, with a 1..98 gradient below, and -1 for unknown.
INSCRIBED, UNKNOWN = 99, -1


@contextmanager
def ros_node(name):
    """A node that is always torn down, including when the body raises."""
    rclpy.init()
    node = rclpy.create_node(name)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def spin_until(node, ready, complaint, timeout):
    """Spin `node` until `ready()` holds, or raise RuntimeError(complaint)."""
    deadline = time.monotonic() + timeout
    while not ready():
        if time.monotonic() > deadline:
            raise RuntimeError(complaint)
        rclpy.spin_once(node, timeout_sec=0.1)


def latest_message(node, message_type, topic, complaint, timeout):
    """The most recent message on a latched topic.

    Subscribes transient-local, so a latched publisher replays what it last sent
    immediately rather than making us wait out its publish period.
    """
    received = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        message_type, topic, lambda message: received.setdefault("message", message), qos)
    try:
        spin_until(node, lambda: "message" in received, complaint, timeout)
    finally:
        node.destroy_subscription(subscription)
    return received["message"]


def load_occupancy_grid(yaml_path=MAP_YAML, frame_id="map"):
    """The saved nav map as an OccupancyGrid, for masking out base positions
    that sit inside walls.

    Comes back in `map`, the frame its yaml `origin` is written in. It must be
    moved into the grasps' frame before use -- `solve` does that. See
    `_grid_in_frame` for why that is not optional.
    """
    import cv2

    meta = yaml.safe_load(Path(yaml_path).read_text())
    image = cv2.imread(str(Path(yaml_path).parent / meta["image"]), cv2.IMREAD_GRAYSCALE)

    # PGM rows run top-to-bottom, OccupancyGrid rows bottom-to-top. Dark means
    # occupied unless the map says otherwise.
    brightness = np.flipud(image).astype(np.float32) / 255.0
    occupancy = brightness if meta.get("negate", 0) else 1.0 - brightness

    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.resolution = float(meta["resolution"])
    grid.info.height, grid.info.width = occupancy.shape
    grid.info.origin.position.x, grid.info.origin.position.y = meta["origin"][:2]
    grid.info.origin.orientation.w = 1.0
    grid.data = np.where(occupancy > meta["occupied_thresh"], BLOCKED, FREE) \
                  .astype(np.int8).ravel().tolist()
    return grid


def costmap_grid(node, topic=COSTMAP_TOPIC, blocked_at=INSCRIBED, timeout=10.0):
    """Nav2's global costmap, rebinned into what the solver expects.

    Better than the static pgm on two counts: the laser's obstacle layer means
    it sees things the map never had, and the inflation layer has already grown
    obstacles by robot_radius, so a cell marked inscribed is one the base
    genuinely cannot occupy.

    The rebinning is not optional. The solver treats *any* nonzero cell as
    blocked, while nav2 publishes a 0-100 gradient: with inflation_radius 0.5 m
    and cost_scaling 3.0 the soft halo around furniture reaches far past the
    arm's own 0.49 m of reach, so passing it through raw rejects essentially
    every base pose next to the thing you are trying to grasp. Only inscribed
    and lethal are real; the gradient below that is nav2's steering preference,
    not geometry.

    Unknown counts as blocked: with track_unknown_space on, that is the inside
    of furniture and the far side of walls.
    """
    grid = latest_message(node, OccupancyGrid, topic,
                          f"no costmap on {topic} within {timeout:.0f}s -- is nav2 up?",
                          timeout)
    cost = np.asarray(grid.data, dtype=np.int16)
    blocked = (cost >= blocked_at) | (cost == UNKNOWN)
    grid.data = np.where(blocked, BLOCKED, FREE).astype(np.int8).tolist()
    return grid


def _grid_in_frame(node, grid, goal_frame, timeout=5.0):
    """`grid` with its origin re-expressed in `goal_frame`.

    The solver never looks at header.frame_id. It does
    `info.origin.inverse() * origin_to_base` with origin_to_base in whatever
    frame the hand goal was given, so info.origin is *assumed* to already be in
    that frame. Our grid's origin comes from the map yaml, i.e. `map`, while
    grasps are saved in `odom`. amcl starts those coincident and then corrects,
    so an unshifted grid is right at boot and quietly drifts wrong afterwards --
    walls appearing where there are none, and vice versa.
    """
    if grid.header.frame_id == goal_frame:
        return grid

    buffer = Buffer()
    TransformListener(buffer, node)
    spin_until(
        node,
        lambda: buffer.can_transform(goal_frame, grid.header.frame_id, Time()),
        f"no {goal_frame} <- {grid.header.frame_id} transform; is navigation up? "
        f"(hsrb_rosnav_config navigation_launch.py brings up amcl)",
        timeout)

    goal_from_map = matrix_from_transform(
        buffer.lookup_transform(goal_frame, grid.header.frame_id, Time()).transform)
    map_from_grid = translation_matrix(grid.info.origin.position.x,
                                       grid.info.origin.position.y)

    shifted = OccupancyGrid()
    shifted.header.frame_id = goal_frame
    shifted.info = grid.info
    shifted.info.origin = pose_from_matrix(goal_from_map @ map_from_grid)
    shifted.data = grid.data
    return shifted


def _obstacle_map(node, obstacles, goal_frame):
    """Whatever `solve`'s caller asked to mask with, as a grid in `goal_frame`."""
    if obstacles == "--costmap":
        obstacles = costmap_grid(node)
    elif obstacles == "--map":
        obstacles = load_occupancy_grid()
    return None if obstacles is None else _grid_in_frame(node, obstacles, goal_frame)


def _ik_request(hand_goal, obstacle_map, environment):
    request = SolveIkWithCollision.Request()
    request.origin_to_hand_goal = pose_from_matrix(hand_goal)
    request.obstacle_map = obstacle_map if obstacle_map is not None else OccupancyGrid()
    request.environment = environment if environment is not None else PlanningSceneWorld()
    request.initial_joint_state = JointState(name=ARM_JOINTS, position=NEUTRAL)
    request.initial_origin_to_base.orientation.w = 1.0
    return request


def _base_xy_yaw(result):
    """One ik result's base position as (x, y, yaw)."""
    position = result.origin_to_base.position
    return position.x, position.y, yaw_from_quaternion(result.origin_to_base.orientation)


def _solve_one(node, client, hand_goal, obstacle_map, environment, timeout):
    """Every (bases, joints) the solver found for a single hand pose."""
    future = client.call_async(_ik_request(hand_goal, obstacle_map, environment))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    found = future.result().ik_results if future.result() else []

    bases = np.array([_base_xy_yaw(r) for r in found]).reshape(-1, 3)
    joints = np.array([r.joint_positions for r in found]).reshape(-1, len(ARM_JOINTS))
    return bases, joints


def solve(poses, obstacles=None, environment=None, timeout=30.0, goal_frame="odom"):
    """For each (4, 4) hand_palm_link pose, return (bases, joints):

        bases:  (M, 3) base poses as (x, y, yaw), empty if unreachable
        joints: (M, 5) arm angles, ordered as ARM_JOINTS

    `obstacles` masks out base positions: "--costmap" for nav2's live global
    costmap, "--map" for the saved pgm, an OccupancyGrid to pass one directly,
    or None for no mask. Prefer the costmap -- it sees what the laser sees and
    is already inflated by robot_radius.

    `environment` must exclude the target itself -- a grasp pose puts the
    gripper on the object, so including it rejects every solution.
    """
    with ros_node("base_placement") as node:
        client = node.create_client(SolveIkWithCollision, SERVICE)
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"{SERVICE} not available -- is ik_solver.launch.py running?")

        obstacle_map = _obstacle_map(node, obstacles, goal_frame)
        return [_solve_one(node, client, pose, obstacle_map, environment, timeout)
                for pose in poses]


def _describe(index, score, bases):
    if len(bases) == 0:
        return f"  grasp {index} (score {score:.2f}): unreachable"
    centre_x, centre_y = bases[:, :2].mean(axis=0)
    x, y, yaw = bases[0]
    return (f"  grasp {index} (score {score:.2f}): {len(bases):3d} base poses, "
            f"centred ({centre_x:+.2f}, {centre_y:+.2f}), "
            f"first ({x:+.2f}, {y:+.2f}, yaw {yaw:+.2f})")


def main(argv):
    source = next((a for a in argv if a in ("--map", "--costmap")), None)
    argv = [a for a in argv if a not in ("--map", "--costmap")]
    if not argv:
        print(__doc__)
        return 1

    poses, scores, data = load_grasps(TARGETS_DIR / argv[0] / "grasps.yaml")
    results = solve(poses, obstacles=source)

    print(f"\n{data['object_id']}: {len(poses)} grasps, frame {data['frame_id']}"
          f"{f', masked by {source}' if source else ''}")
    for index, (bases, _) in enumerate(results):
        print(_describe(index, scores[index], bases))

    reachable = sum(len(bases) > 0 for bases, _ in results)
    print(f"\n  {reachable}/{len(poses)} grasps reachable")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
