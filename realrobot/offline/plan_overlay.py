"""Ask Nav2 for a path to a queried scene-graph target and draw it on the lab map.

    python3 realrobot/offline/plan_overlay.py --target laptop
    python3 realrobot/offline/plan_overlay.py --furniture bookshelf --seed 3

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

from core.utils.recording import Paths, name_from_map  # noqa: E402

RECORDING = Paths()
DEFAULT_GRAPH, DEFAULT_MAP = (RECORDING.graph, RECORDING.map)
LAUNCH = Path(__file__).resolve().parent / "planner.launch.py"
# Kept off the devcontainer's ROS_DOMAIN_ID=5 so a running sim or mission cannot
# answer these goals, and so this stack's /map cannot reach theirs.
DOMAIN = 91
FREE, UNKNOWN, OCCUPIED = 254, 205, 0  # nav2 trinary PGM values
# Occupied cells this close to the target's footprint are the laser's return off
# the target itself and do not block the view of it. Wider than this and a pose
# starts seeing through the thin wall a piece of furniture stands against.
SKIN = 0.15
# A path that stops short of the goal is a refusal, not a route: the same check
# core.navigation.nav2_client.reachable makes before the mission drives.
REACHED = 0.1
# realrobot/dataprep/visualize.py's palette, so the figures read as one set.
INK, MUTED, PATH, CAM = "#1f2933", "#7b8794", "#d14d1f", "#2464b0"
FURNITURE, OBJECT, TARGET = "#2464b0", "#7b8794", "#1f9d55"
# Rooms land anywhere on the map, so any two can end up side by side: that is the
# all-pairs case, where only the first three categorical slots clear the
# colour-vision separation floors. Past three the hue is a hint and the name
# written across each region is what identifies it. Assigned in this order by
# sorted room name and never cycled, so a room keeps its colour between runs.
ROOM_COLOURS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100",
                "#e87ba4", "#008300", "#4a3aa7", "#e34948")


from core.utils.occupancy import read_map, to_cells  # noqa: E402,F401


def rooms_of(scene):
    """{room name: colour}, or {} for a graph step 3 built without --rooms."""
    names = sorted({data["room"] for _, data in scene.nodes(data=True) if data["room"]})
    return {name: ROOM_COLOURS[index % len(ROOM_COLOURS)]
            for index, name in enumerate(names)}


def room_anchor(scene, room):
    """Where to write a room's name: the median of its furniture centres.

    No region is drawn around a room. These rooms are not convex -- the office
    wraps the corridor -- so a hull over its furniture would claim the floor in
    between and swallow the dining area whole, inventing a nesting that is not
    in the graph. The colour of each footprint is what says which room it is in.
    The median, not the mean, so one far-flung piece cannot drag the name off
    the cluster it belongs to."""
    from core.scene_graph import graph as sg

    centres = [sg.footprint(data)[0][:2] for data in sg.furniture(scene).values()
               if data["room"] == room]
    return np.median(np.asarray(centres, float), axis=0) if centres else None


def blockers(scene, exclude=()):
    """The map-frame outlines core.scene_graph.graph.blockers keeps the base out
    of, each with its node attributes so the figures can label and count them."""
    from core.scene_graph import graph as sg

    shapes = []
    for node_id, (centre, dimensions, yaw) in sg.blockers(scene, exclude=exclude).items():
        shapes.append((centre[:2], dimensions[:2], yaw, scene.nodes[node_id]))
    return shapes


def stamp(image, resolution, origin, scene, exclude=()):
    """`image` with every blocker filled in as occupied."""
    from core.utils import geometry

    stamped = image.copy()
    for centre, size, yaw, _ in blockers(scene, exclude):
        corners = to_cells(geometry.footprint_corners(centre, size, yaw), resolution, origin)
        cv2.fillPoly(stamped, [np.round(corners).astype(np.int32)], OCCUPIED)
    return stamped


def pose_grid(image, resolution, origin, scene, clearance, exclude=()):
    """The costmap core.pipeline.actions.plan filters observation poses against.

    Pass the target in `exclude`, the same way sg.blockers wants it. Left in, its
    own box grown by `clearance` lies across every ray from the ring to the piece,
    and standoff.unobstructed forgives only cells within SKIN (0.15 m) of the
    footprint; at the 0.3 m default that collar drops every pose the piece has."""
    return occupancy_grid(np.where(navigable(stamp(image, resolution, origin, scene,
                                                    exclude), resolution, clearance),
                                   0, 100), resolution, origin)


def sight_grid(stamped, resolution, origin):
    """The costmap standoff.unobstructed reads: walls only, no clearance collar.

    pose_grid grows every blocker by --clearance so the base does not park against
    one, and counts unknown as blocked the way track_unknown_space does. Neither
    is opaque -- a pose sees straight across floor it could not stand on, and an
    unknown cell is floor the laser never reached rather than something standing
    in it -- so a view test against pose_grid drops poses with a clear line to the
    piece. Matches base_placement.sight_grid, which is what the robot uses."""
    return occupancy_grid(np.where(stamped == FREE, 0,
                                   np.where(stamped == UNKNOWN, -1, 100)),
                          resolution, origin)


def stamped_map(image, resolution, origin, scene, destination):
    """`image` with every blocker filled in as occupied, saved as a map Nav2 can serve."""
    stamped = stamp(image, resolution, origin, scene)
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


def occupancy_grid(values, resolution, origin):
    """`values` as the OccupancyGrid standoff.free and .unobstructed read, so poses
    inside walls or against them are dropped before Nav2 is asked about them."""
    from nav_msgs.msg import OccupancyGrid

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = resolution
    grid.info.height, grid.info.width = np.shape(values)
    grid.info.origin.position.x, grid.info.origin.position.y = map(float, origin)
    grid.info.origin.orientation.w = 1.0
    grid.data = np.asarray(values).astype(np.int8).ravel().tolist()
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
    """The map, the scene graph on top of it, and the planned leg from start to goal.

    location/start/goal/path are None in --survey mode, which draws the graph
    alone: no target is highlighted and there is no leg to show."""
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

    # Only the survey answers "are these the right rooms"; a planned figure needs
    # its colours for the chosen piece and the path.
    rooms = rooms_of(scene) if location is None else {}
    if path is not None:
        axes.plot(path[:, 0], path[:, 1], color=PATH, linewidth=2.0, zorder=5,
                  label=f"Nav2 Theta* path, {path_length(path):.1f} m")
        axes.plot(*start, "o", color=INK, markersize=8, zorder=6, label="start")
        axes.quiver(goal[0], goal[1], 0.5 * math.cos(goal[2]), 0.5 * math.sin(goal[2]),
                    color=CAM, angles="xy", scale_units="xy", scale=1, width=0.004,
                    zorder=6, label="goal: first observation pose")

    chosen_id = location.furniture_id if location else None
    for node, data in sg.furniture(scene).items():
        chosen = node == chosen_id
        centre, dimensions, yaw = sg.footprint(data)
        colour = rooms.get(data["room"], FURNITURE)
        outline(centre[:2], dimensions[:2], yaw, TARGET if chosen else colour,
                0.5 if chosen else 0.45, 1.6 if chosen else 0.8)
    if rooms:
        for room, colour in rooms.items():
            count = sum(data["room"] == room for data in sg.furniture(scene).values())
            axes.plot([], [], "s", color=colour, alpha=0.6,
                      label=f"{room} ({count} furniture)")
    else:
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
    if location is not None and location.object_id is not None:
        data = scene.nodes[location.object_id]
        axes.plot(*data["centroid"][:2], "*", color=TARGET, markersize=16, zorder=7,
                  markeredgecolor=INK, markeredgewidth=0.5,
                  label=f"target: {data['name']} ({location.object_id})")
    # Over the furniture labels, in a box, because the desk row is dense enough
    # that a stroked name alone disappears into it.
    for room, colour in rooms.items():
        anchor = room_anchor(scene, room)
        if anchor is None:
            continue
        axes.annotate(room.upper(), anchor, color=colour, fontsize=11,
                      fontweight="bold", ha="center", va="center", zorder=9,
                      bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
                                edgecolor=colour, alpha=0.9, linewidth=1.2))
    # Labelled last and over everything, so the dense desk row stays readable.
    for node, data in sg.furniture(scene).items():
        chosen = node == chosen_id
        axes.annotate(f"{data['name'] or data['label']} ({node})",
                      sg.footprint(data)[0][:2], color=TARGET if chosen else INK,
                      fontsize=6.5, ha="center", va="center", zorder=8,
                      path_effects=[withStroke(linewidth=2.2, foreground="white")])

    if not full_map:
        interest = np.vstack([[centre for centre, _, _, _ in blockers(scene)]]
                             + ([path, [start], [goal[:2]]] if path is not None else []))
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
    if location is None:
        query = (f"{len(sg.furniture(scene))} furniture, "
                 f"{sum(data['movable'] for _, _, _, data in blockers(scene))} "
                 f"floor objects"
                 + (f", {len(rooms)} rooms" if rooms else ", no rooms assigned"))
    elif location.object_id is None:
        query = f"furniture {location.label} ({location.furniture_id})"
    elif location.furniture_id is None:
        query = f"{location.label} ({location.object_id}), on nothing the graph knows"
    else:
        query = (f"{location.label} ({location.object_id}) {location.relation} "
                 f"{scene.nodes[location.furniture_id]['label']} ({location.furniture_id})")
    if location is None:
        axes.set_title(f"{query}   ·   {figure_path.stem} on the laser map")
    else:
        axes.set_title(f"{query}   ·   planned on the "
                       f"{'map with the scene graph stamped in' if stamped else 'laser map alone'}")
    figure.tight_layout()
    figure_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(figure_path, dpi=200 if location is None else 150)
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
    query = parser.add_mutually_exclusive_group()
    query.add_argument("--target", help="object to look for, by label or instance name")
    query.add_argument("--furniture", help="furniture to drive to, by name, label or node ID")
    parser.add_argument("--survey", action="store_true",
                        help="draw the whole scene graph on the map and stop: no query, "
                             "no planning, no ROS")
    parser.add_argument("--graph", type=Path, default=DEFAULT_GRAPH)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--output", type=Path, help="PNG to write "
                        "(default outputs/realrobot/<recording>/nav/<query>.png)")
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
    if not (args.survey or args.target or args.furniture):
        parser.error("one of --target, --furniture or --survey is required")
    # --graph and --map default independently, so overriding one and not the
    # other plans a new graph on an old map: every pose then lands on floor that
    # map never saw, and the run reports the target as unreachable rather than
    # as a mismatch. Recordings keep one name across both paths, so compare it.
    if name_from_map(args.map) != args.graph.stem:
        parser.error(
            f"--map is {name_from_map(args.map)} but --graph is {args.graph.stem}; "
            "export RECORDING=<name> to move both, or pass --map and --graph together")

    from core.pipeline import actions
    from core.scene_graph import graph as sg

    scene = sg.load(args.graph)
    image, resolution, origin = read_map(args.map)
    name = args.furniture or args.target or "survey"
    recording = name_from_map(args.map)
    scratch = ROOT / "outputs" / "realrobot" / recording / "nav"
    output = args.output or scratch / f"{name.replace(' ', '_')}.png"

    if args.survey:
        # Nothing is planned, so the stamped map would only be written and
        # thrown away; the picture is drawn on the laser map either way.
        draw(output, image, resolution, origin, scene, None, None, None, None,
             False, args.full_map)
        print(f"[figure] {output}")
        return

    served, drawn = args.map, image
    if args.blockers:
        served, drawn = stamped_map(image, resolution, origin, scene,
                                    scratch / f"{recording}_blockers")
        print(f"[map] {len(blockers(scene))} scene-graph shapes stamped into {served.name}")
    mask = navigable(drawn, resolution, args.clearance)

    rng = np.random.default_rng(args.seed)
    with Planner(served, args.domain_id) as planner:
        for attempt, start in enumerate(
            [args.start] if args.start else
            start_poses(mask, resolution, origin, centroid_of(scene, actions, args),
                        args.min_distance, rng, args.attempts), 1
        ):
            location = next(actions.targets(scene, args.target, args.furniture,
                                            robot_xy=tuple(start)))
            # Rebuilt per target: --target can resolve to different furniture from
            # different starts, and the piece being looked at must be left out.
            spare = () if location.furniture_id is None else (location.furniture_id,)
            unstamped = stamp(image, resolution, origin, scene, spare) if args.blockers \
                else image
            grid = occupancy_grid(
                np.where(navigable(unstamped, resolution, args.clearance), 0, 100),
                resolution, origin)
            poses, views = actions.plan(scene, location, robot_xy=tuple(start), grid=grid,
                                        sight_grid=sight_grid(unstamped, resolution, origin))
            if args.sight_line:
                poses, behind = in_sight(scene, location, poses, image, resolution, origin)
                if behind:
                    print(f"[sight] {len(behind)} of {len(poses) + len(behind)} poses "
                          f"look at the target through a wall; dropped")
            if not poses:
                why_blocked(scene, location, unstamped, resolution, origin,
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
