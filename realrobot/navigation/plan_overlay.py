"""Ask Nav2 for a path to a queried scene-graph target and draw it on the lab map.

    python3 realrobot/navigation/plan_overlay.py --target laptop
    python3 realrobot/navigation/plan_overlay.py --furniture bookshelf --seed 3

No robot and no bag. `planner.launch.py` runs map_server and planner_server on the
saved map, the start is drawn at random from its free cells, and the path is Nav2's
own Theta* answer rather than a redrawing of one.

The goal is not the target itself: `core.pipeline.actions.plan` turns the query into
observation poses around the furniture the target sits on, and the first one it would
drive to becomes the goal, so the picture shows the leg the mission would ask for.

With --blockers (the default) the scene graph's furniture and floor-standing objects
are stamped into a copy of the map before the planner reads it. The laser mapped the
lab at 19 cm, so a desk is four thin legs there and its middle counts as open floor;
footprint_check.py measures how far that goes.

Needs ROS: source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash first.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402
from matplotlib.patheffects import withStroke  # noqa: E402

DEFAULT_GRAPH = ROOT / "config/realrobot/scene_graph/lab_20260811.json"
DEFAULT_MAP = ROOT / "config/realrobot/map/lab_20260811.yaml"
LAUNCH = Path(__file__).resolve().parent / "planner.launch.py"
# Kept off the devcontainer's ROS_DOMAIN_ID=5 so a running sim or mission cannot
# answer these goals, and so this stack's /map cannot reach theirs.
DOMAIN = 91
FREE, UNKNOWN, OCCUPIED = 254, 205, 0  # nav2 trinary PGM values
# An object's box underside this close to z = 0 means it rests on the floor.
# In this graph nothing lands near the line: the three floor objects sit at
# -0.02, -0.02 and 0.02 m, and the next object up starts at 0.50 m.
FLOOR = 0.15
# Shorter than this and the base rides over it: a cable, a mat, a threshold.
MIN_HEIGHT = 0.05
# Occupied cells this close to the target's footprint are the laser's return off
# the target itself and do not block the view of it. Wider than this and a pose
# starts seeing through the thin wall a piece of furniture stands against.
SKIN = 0.15
# A path that stops short of the goal is a refusal, not a route: the same check
# core.navigation.nav2_client.reachable makes before the mission drives.
REACHED = 0.1
# realrobot/visualize.py's palette, so the figures read as one set.
INK, MUTED, PATH, CAM = "#1f2933", "#7b8794", "#d14d1f", "#2464b0"
FURNITURE, OBJECT, TARGET = "#2464b0", "#7b8794", "#1f9d55"


def read_map(yaml_path):
    """The PGM, its resolution and its map-frame origin, rows running north."""
    meta = yaml.safe_load(yaml_path.read_text())
    if meta.get("mode", "trinary") != "trinary" or meta.get("negate", 0):
        raise ValueError(f"{yaml_path} is not a plain trinary map")
    image = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"cannot read the map image named in {yaml_path}")
    return np.flipud(image), float(meta["resolution"]), np.array(meta["origin"][:2], float)


def to_cells(points, resolution, origin):
    """Map-frame xy to (column, row) in a north-up image."""
    return (np.asarray(points, float) - origin) / resolution


def standing_on_floor(data):
    """Whether an object's box rests on the floor instead of on a piece of furniture.

    The map frame's z = 0 is base_footprint, which is the floor the base drives on,
    so the test is the height of the box's underside and nothing else. The graph's
    own `on`/`in` edge will not do: computer tower (36) is edged `in desk (4)`
    because it is inside the desk's box, and it is standing under it on the floor.
    Boxer gives axis-aligned bounds for objects (only furniture gets a fitted yaw),
    so the blocker is that box seen from above."""
    lower, upper = np.asarray(data["bounds"], float)
    return lower[2] <= FLOOR and upper[2] - lower[2] >= MIN_HEIGHT


def blockers(scene):
    """Every scene-graph shape the base can run into: furniture by its fitted
    footprint, plus objects standing on the floor by their bounds."""
    from core.scene_graph import graph as sg

    shapes = []
    for data in sg.furniture(scene).values():
        centre, dimensions, yaw = sg.footprint(data)
        shapes.append((centre[:2], dimensions[:2], yaw, data))
    for data in sg.objects(scene).values():
        if standing_on_floor(data):
            lower, upper = np.asarray(data["bounds"], float)
            shapes.append(((lower[:2] + upper[:2]) / 2, (upper - lower)[:2], 0.0, data))
    return shapes


def stamped_map(image, resolution, origin, scene, destination):
    """`image` with every blocker filled in as occupied, saved as a map Nav2 can serve."""
    from core.utils import geometry

    stamped = image.copy()
    for centre, size, yaw, _ in blockers(scene):
        corners = to_cells(geometry.footprint_corners(centre, size, yaw), resolution, origin)
        cv2.fillPoly(stamped, [np.round(corners).astype(np.int32)], OCCUPIED)
    destination.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(destination.with_suffix(".pgm")), np.flipud(stamped))
    destination.with_suffix(".yaml").write_text(yaml.safe_dump({
        "image": destination.with_suffix(".pgm").name,
        "mode": "trinary",
        "resolution": resolution,
        "origin": [float(origin[0]), float(origin[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }, sort_keys=False))
    return destination.with_suffix(".yaml"), stamped


def navigable(image, resolution, clearance):
    """Known-free cells at least `clearance` from anything else. Unknown counts as
    blocked, the same way global_costmap's track_unknown_space treats it."""
    return distance_transform_edt(image == FREE) * resolution >= clearance


def occupancy_grid(mask, resolution, origin):
    """`mask` as the OccupancyGrid standoff.free reads, so poses inside walls or
    against them are dropped before Nav2 is asked about them."""
    from nav_msgs.msg import OccupancyGrid

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = resolution
    grid.info.height, grid.info.width = mask.shape
    grid.info.origin.position.x, grid.info.origin.position.y = map(float, origin)
    grid.info.origin.orientation.w = 1.0
    grid.data = np.where(mask, 0, 100).astype(np.int8).ravel().tolist()
    return grid


def start_poses(mask, resolution, origin, away_from, minimum, rng, count):
    """Free starting points at least `minimum` from the target, so a random run
    shows a drive across the room rather than a step sideways."""
    rows, columns = np.nonzero(mask)
    points = np.column_stack([origin[0] + columns * resolution, origin[1] + rows * resolution])
    far = points[np.hypot(*(points - np.asarray(away_from)).T) >= minimum]
    if not len(far):
        raise SystemExit(f"no free cell is {minimum} m or more from the target")
    return far[rng.choice(len(far), min(count, len(far)), replace=False)]


class Planner:
    """map_server and planner_server on `map_yaml`, torn down on the way out."""

    def __init__(self, map_yaml, domain, timeout=60.0):
        self.map_yaml, self.domain, self.timeout = map_yaml, domain, timeout

    def __enter__(self):
        import rclpy
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient

        os.environ["ROS_DOMAIN_ID"] = str(self.domain)
        environment = dict(os.environ)
        self.stack = subprocess.Popen(
            ["ros2", "launch", str(LAUNCH), f"map:={self.map_yaml}"],
            env=environment, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        rclpy.init()
        self.node = rclpy.create_node("plan_overlay")
        self.client = ActionClient(self.node, ComputePathToPose, "compute_path_to_pose")
        print(f"[planner] starting Nav2 on {self.map_yaml.name}, ROS_DOMAIN_ID={self.domain}")
        if not self.client.wait_for_server(timeout_sec=self.timeout):
            self.__exit__(None, None, None)
            raise SystemExit("planner_server never appeared; is another Nav2 on this domain?")
        # wait_for_server returns as soon as the action exists, which is before the
        # lifecycle manager has activated the costmap and the map has been served.
        time.sleep(3.0)
        return self

    def path(self, start, goal, timeout=25.0):
        """Nav2's path from `start` to the (x, y, yaw) `goal`, or None if it refuses."""
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputePathToPose

        def stamped(x, y, yaw):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            return pose

        request = ComputePathToPose.Goal()
        request.use_start = True  # there is no robot and no localisation to start from
        request.start = stamped(start[0], start[1], 0.0)
        request.goal = stamped(*goal)
        sent = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=timeout)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None
        outcome = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, outcome, timeout_sec=timeout)
        result = outcome.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        poses = result.result.path.poses
        if not poses or result.result.path.header.frame_id != "map":
            return None
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in poses])
        if math.dist(path[-1], goal[:2]) > REACHED:
            return None
        return path

    def __exit__(self, *_):
        import rclpy

        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        group = os.getpgid(self.stack.pid)
        os.killpg(group, signal.SIGINT)
        try:
            self.stack.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(group, signal.SIGKILL)
        return False


def draw(figure_path, image, resolution, origin, scene, location, start, goal, path,
         stamped, full_map):
    """The map, the scene graph on top of it, and the planned leg from start to goal."""
    from core.scene_graph import graph as sg
    from core.utils import geometry

    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.titlecolor": INK})
    height, width = image.shape
    figure, axes = plt.subplots(figsize=(10, 10 * height / width))
    extent = [origin[0], origin[0] + width * resolution,
              origin[1], origin[1] + height * resolution]
    axes.imshow(image, cmap="gray", vmin=0, vmax=255, extent=extent,
                interpolation="nearest", origin="lower")

    def outline(centre, size, yaw, colour, alpha, width):
        axes.add_patch(Polygon(geometry.footprint_corners(centre, size, yaw), closed=True,
                               facecolor=colour, edgecolor=colour, alpha=alpha,
                               linewidth=width, zorder=3))

    axes.plot(path[:, 0], path[:, 1], color=PATH, linewidth=2.0, zorder=5,
              label=f"Nav2 Theta* path, {path_length(path):.1f} m")
    axes.plot(*start, "o", color=INK, markersize=8, zorder=6, label="start")
    axes.quiver(goal[0], goal[1], 0.5 * math.cos(goal[2]), 0.5 * math.sin(goal[2]),
                color=CAM, angles="xy", scale_units="xy", scale=1, width=0.004, zorder=6,
                label="goal: first observation pose")

    for node, data in sg.furniture(scene).items():
        chosen = node == location.furniture_id
        centre, dimensions, yaw = sg.footprint(data)
        outline(centre[:2], dimensions[:2], yaw, TARGET if chosen else FURNITURE,
                0.5 if chosen else 0.3, 1.6 if chosen else 0.8)
    axes.plot([], [], "s", color=FURNITURE, alpha=0.5,
              label=f"furniture footprints ({len(sg.furniture(scene))})")
    floor_objects = 0
    for centre, size, yaw, data in blockers(scene):
        if data["movable"]:
            outline(centre, size, yaw, OBJECT, 0.55, 0.8)
            floor_objects += 1
    if floor_objects:
        axes.plot([], [], "s", color=OBJECT, alpha=0.6,
                  label=f"objects standing on the floor ({floor_objects})")
    if location.object_id is not None:
        data = scene.nodes[location.object_id]
        axes.plot(*data["centroid"][:2], "*", color=TARGET, markersize=16, zorder=7,
                  markeredgecolor=INK, markeredgewidth=0.5,
                  label=f"target: {data['name']} ({location.object_id})")
    # Labelled last and over everything, so the dense desk row stays readable.
    for node, data in sg.furniture(scene).items():
        chosen = node == location.furniture_id
        axes.annotate(f"{data['name'] or data['label']} ({node})",
                      sg.footprint(data)[0][:2], color=TARGET if chosen else INK,
                      fontsize=6.5, ha="center", va="center", zorder=8,
                      path_effects=[withStroke(linewidth=2.2, foreground="white")])

    if not full_map:
        interest = np.vstack([path, [start], [goal[:2]],
                              [centre for centre, _, _, _ in blockers(scene)]])
        axes.set_xlim(interest[:, 0].min() - 1.5, interest[:, 0].max() + 1.5)
        axes.set_ylim(interest[:, 1].min() - 1.5, interest[:, 1].max() + 1.5)
    axes.set_aspect("equal")
    axes.set_xlabel("map x [m]", color=MUTED)
    axes.set_ylabel("map y [m]", color=MUTED)
    axes.tick_params(colors=MUTED)
    for spine in axes.spines.values():
        spine.set_color("#d9dde2")
    axes.legend(loc="upper left", fontsize=8, facecolor="white", framealpha=0.9,
                edgecolor="#d9dde2")
    if location.object_id is None:
        query = f"furniture {location.label} ({location.furniture_id})"
    elif location.furniture_id is None:
        query = f"{location.label} ({location.object_id}), on nothing the graph knows"
    else:
        query = (f"{location.label} ({location.object_id}) {location.relation} "
                 f"{scene.nodes[location.furniture_id]['label']} ({location.furniture_id})")
    axes.set_title(f"{query}   ·   planned on the "
                   f"{'map with the scene graph stamped in' if stamped else 'laser map alone'}")
    figure.tight_layout()
    figure_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(figure_path, dpi=150)
    plt.close(figure)


def sight_line(image, resolution, origin, pose, aim, footprint):
    """Occupied cells between a pose and what it is meant to look at.

    standoff.candidates rings the furniture geometrically and filters only against
    other furniture boxes, so a pose on the far side of a wall survives: it stands
    on a free cell and no furniture box covers it. On the lab map desk (8) sits
    against a wall, its fitted box overhangs it, and the ring puts four poses out
    in the corridor behind it. Cells within SKIN of the target's own footprint are
    the laser's return off the target and do not count.
    """
    from matplotlib.path import Path as Outline
    from core.utils import geometry

    centre, dimensions, yaw = footprint
    own = Outline(geometry.footprint_corners(np.asarray(centre)[:2],
                                             np.asarray(dimensions)[:2] + 2 * SKIN, yaw))
    pose, aim = np.asarray(pose[:2], float), np.asarray(aim[:2], float)
    steps = max(2, int(np.hypot(*(aim - pose)) / (resolution / 2)))
    hits = 0
    for point in pose + np.linspace(0, 1, steps)[1:-1, None] * (aim - pose):
        column, row = np.floor(to_cells(point, resolution, origin)).astype(int)
        if not (0 <= column < image.shape[1] and 0 <= row < image.shape[0]):
            continue
        if image[row, column] == OCCUPIED and not own.contains_point(point):
            hits += 1
    return hits


def in_sight(scene, location, poses, image, resolution, origin):
    """`poses` that can actually see the place they were generated for."""
    from core.navigation import standoff
    from core.scene_graph import graph as sg

    if location.furniture_id is None:
        return poses, []
    shape = sg.footprint(sg.furniture(scene)[location.furniture_id])
    kept, dropped = [], []
    for pose in poses:
        if location.object_id is not None:
            aim = location.centroid[:2]  # _views re-aims furniture poses at the object
        else:
            aim = standoff.aim_point(pose[:2], *shape)
        (kept if not sight_line(image, resolution, origin, pose, aim, shape)
         else dropped).append(pose)
    return kept, dropped


def why_blocked(scene, location, image, resolution, origin, clearance, sight=True):
    """Why standoff's ring around `location` left nothing to drive to.

    The ring itself is the same call actions.plan filters, so this re-runs it
    unfiltered and reads each pose off the map instead of guessing."""
    from core.navigation import standoff
    from core.pipeline.actions import is_storage
    from core.scene_graph import graph as sg

    if location.furniture_id is None:
        ring = standoff.candidates(location.centroid, [0, 0, 0])
    else:
        data = sg.furniture(scene)[location.furniture_id]
        ring = standoff.candidates(*sg.footprint(data), storage=is_storage(data))
    clear = distance_transform_edt(image == FREE) * resolution
    reasons = {}
    for pose in ring:
        x, y, _ = pose
        column, row = np.floor(to_cells((x, y), resolution, origin)).astype(int)
        if not (0 <= column < image.shape[1] and 0 <= row < image.shape[0]):
            reason = "outside the map"
        elif image[row, column] == UNKNOWN:
            reason = "on a cell the laser never mapped"
        elif image[row, column] == OCCUPIED:
            reason = "inside a wall or a stamped blocker"
        elif clear[row, column] < clearance:
            reason = f"under {clearance} m of clearance for the base"
        elif sight and not in_sight(scene, location, [pose], image, resolution, origin)[0]:
            reason = "a wall between it and the target"
        else:
            reason = "usable here, refused by Nav2 itself"
        reasons[reason] = reasons.get(reason, 0) + 1
    print(f"[blocked] {len(ring)} observation poses around "
          f"{location.label} ({location.furniture_id}), none usable:")
    for reason, count in sorted(reasons.items(), key=lambda item: -item[1]):
        print(f"          {count:>3}  {reason}")


def path_length(path):
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    query = parser.add_mutually_exclusive_group(required=True)
    query.add_argument("--target", help="object to look for, by label or instance name")
    query.add_argument("--furniture", help="furniture to drive to, by name, label or node ID")
    parser.add_argument("--graph", type=Path, default=DEFAULT_GRAPH)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--output", type=Path, help="PNG to write "
                        "(default outputs/realrobot/<map>/nav/<query>.png)")
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"),
                        help="start in map metres; random free cell without it")
    parser.add_argument("--seed", type=int, default=0, help="which random start")
    parser.add_argument("--min-distance", type=float, default=4.0,
                        help="metres the random start must keep from the target")
    parser.add_argument("--attempts", type=int, default=6,
                        help="random starts to try before giving up")
    parser.add_argument("--clearance", type=float, default=0.3,
                        help="metres of free space a start or goal needs; the "
                             "global_costmap robot_radius")
    parser.add_argument("--blockers", action=argparse.BooleanOptionalAction, default=True,
                        help="stamp the scene graph into the map Nav2 plans on")
    parser.add_argument("--sight-line", action=argparse.BooleanOptionalAction, default=True,
                        help="drop observation poses with a wall between them and the "
                             "target; standoff.candidates does not check this")
    parser.add_argument("--full-map", action="store_true", help="draw all 50 x 28 m")
    parser.add_argument("--domain-id", type=int, default=DOMAIN)
    args = parser.parse_args()

    from core.pipeline import actions
    from core.scene_graph import graph as sg

    scene = sg.load(args.graph)
    image, resolution, origin = read_map(args.map)
    name = args.furniture or args.target
    scratch = ROOT / "outputs" / "realrobot" / args.map.stem / "nav"
    output = args.output or scratch / f"{name.replace(' ', '_')}.png"

    served, drawn = args.map, image
    if args.blockers:
        served, drawn = stamped_map(image, resolution, origin, scene,
                                    scratch / f"{args.map.stem}_blockers")
        print(f"[map] {len(blockers(scene))} scene-graph shapes stamped into {served.name}")
    mask = navigable(drawn, resolution, args.clearance)
    grid = occupancy_grid(mask, resolution, origin)

    rng = np.random.default_rng(args.seed)
    with Planner(served, args.domain_id) as planner:
        for attempt, start in enumerate(
            [args.start] if args.start else
            start_poses(mask, resolution, origin, centroid_of(scene, actions, args),
                        args.min_distance, rng, args.attempts), 1
        ):
            location = next(actions.targets(scene, args.target, args.furniture,
                                            robot_xy=tuple(start)))
            poses, views = actions.plan(scene, location, robot_xy=tuple(start), grid=grid)
            if args.sight_line:
                poses, behind = in_sight(scene, location, poses, image, resolution, origin)
                if behind:
                    print(f"[sight] {len(behind)} of {len(poses) + len(behind)} poses "
                          f"look at the target through a wall; dropped")
            if not poses:
                why_blocked(scene, location, drawn, resolution, origin,
                            args.clearance, args.sight_line)
                raise SystemExit("nothing to plan to; try another target, a smaller "
                                 "--clearance, or --no-blockers")
            for goal in poses:
                path = planner.path(start, goal)
                if path is not None:
                    print(f"[nav2] start ({start[0]:.2f}, {start[1]:.2f}) -> goal "
                          f"({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}): "
                          f"{len(path)} poses, {path_length(path):.2f} m")
                    draw(output, image, resolution, origin, scene, location,
                         start, goal, path, args.blockers, args.full_map)
                    print(f"[figure] {output}")
                    return
            print(f"[nav2] attempt {attempt}: no path from "
                  f"({start[0]:.2f}, {start[1]:.2f}) to any of {len(poses)} poses")
    raise SystemExit(f"Nav2 refused every start/goal pair after {args.attempts} attempts")


def centroid_of(scene, actions, args):
    """Where the target is, so the random start can be kept away from it."""
    location = next(actions.targets(scene, args.target, args.furniture))
    return location.centroid[:2]


if __name__ == "__main__":
    main()
