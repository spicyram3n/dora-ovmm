"""Observation poses around furniture, filtered against surrounding furniture.

A footprint is (centre, dimensions, yaw): dimensions run along the piece's own
axes and yaw turns those axes in the map. Gazebo-built graphs carry no yaw;
their axis-aligned boxes are exact there because every apartment piece stands
at a multiple of 90 degrees."""

import math
import numpy as np
from core.utils.geometry import to_local, to_map

WORKING_DISTANCE = 0.8
ROBOT_RADIUS = 0.3
# Spacing of candidate poses along the standoff line, so bigger furniture gets more.
POSE_SPACING = 0.5
# Head camera field of view, narrowed from roughly 58 x 45 degrees so objects at
# the image edge are not counted as seen. Check against the HSR-C camera_info.
HALF_FOV = 0.45
HALF_FOV_VERTICAL = 0.35
# Beyond this a small object is few pixels, and looking down a long table at a
# grazing angle hides objects behind each other, so it does not count as seen.
MAX_RANGE = 1.5
# Surface samples used to count how much of the furniture a view takes in.
SURFACE_SPACING = 0.1
# A view must show this share of new surface to be worth a drive; gains within
# the same step count as equal and the shorter drive wins.
GAIN_STEP = 0.05
MIN_VIEWS = 2
MAX_VIEWS = 6
# Storage is looked into only through faces at least this share of its longest
# side: a bookcase opens on its long face, not its end panels.
FACE_SHARE = 0.75


def blocks(pose, blockers):
    """Check the base centre against boxes expanded by the robot radius."""
    # Expand each obstacle so the whole base has clearance.
    for centre, dimensions, yaw in blockers:
        x_distance, y_distance = np.abs(to_local(pose[:2], centre, yaw))
        x_limit = dimensions[0] / 2 + ROBOT_RADIUS
        y_limit = dimensions[1] / 2 + ROBOT_RADIUS
        if x_distance <= x_limit and y_distance <= y_limit:
            return True
    return False


def free(grid, pose):
    """Whether `pose` stands on a free cell of `grid`, a map-frame costmap as
    base_placement.costmap_grid returns it; walls and off-map poses are not free."""
    info = grid.info
    column = math.floor((pose[0] - info.origin.position.x) / info.resolution)
    row = math.floor((pose[1] - info.origin.position.y) / info.resolution)
    if not (0 <= column < info.width and 0 <= row < info.height):
        return False
    return grid.data[row * info.width + column] == 0


def aim_point(xy, centre, dimensions, yaw):
    """Where a view from `xy` should centre: level with it along the piece's long
    axis and halfway through its depth, so one view takes in the whole depth."""
    local = to_local(xy, centre, yaw)
    if dimensions[0] >= dimensions[1]:
        aim = [np.clip(local[0], -dimensions[0] / 2, dimensions[0] / 2), 0.0]
    else:
        aim = [0.0, np.clip(local[1], -dimensions[1] / 2, dimensions[1] / 2)]
    return to_map(aim, centre, yaw)


def _standoff_line(dimensions):
    """Box-frame points WORKING_DISTANCE out from the footprint, about POSE_SPACING
    apart, the corners rounded by quarter circles."""
    half_x, half_y = (dimensions[0] / 2, dimensions[1] / 2)
    # Counter-clockwise from the +x face: each face runs between two corners,
    # then a quarter circle turns around the second corner to the next face.
    corners = [(half_x, -half_y), (half_x, half_y), (-half_x, half_y), (-half_x, -half_y)]
    lengths = [2 * half_y, 2 * half_x, 2 * half_y, 2 * half_x]
    arc = WORKING_DISTANCE * math.pi / 2
    perimeter = sum(lengths) + 4 * arc
    count = max(4, round(perimeter / POSE_SPACING))
    samples = []
    for index in range(count):
        along = perimeter * index / count
        for face in range(4):
            normal = face * math.pi / 2
            start = np.array(corners[face])
            end = np.array(corners[(face + 1) % 4])
            if along < lengths[face]:
                point = start + (end - start) * along / lengths[face]
                offset = [math.cos(normal), math.sin(normal)]
                samples.append(point + WORKING_DISTANCE * np.array(offset))
                break
            along -= lengths[face]
            if along < arc:
                angle = normal + along / WORKING_DISTANCE
                offset = [math.cos(angle), math.sin(angle)]
                samples.append(end + WORKING_DISTANCE * np.array(offset))
                break
            along -= arc
    return samples


def _long_faces(dimensions):
    """Box-frame points WORKING_DISTANCE off each long face, end to end and at most
    POSE_SPACING apart, so even a narrow bookcase gets two poses per face."""
    points = []
    for axis in (0, 1):
        # The face whose outward normal runs along `axis` spans the other one.
        length = dimensions[1 - axis]
        if length < FACE_SHARE * max(dimensions[0], dimensions[1]):
            continue
        count = math.ceil(length / POSE_SPACING) + 1
        for side in (-1, 1):
            for along in np.linspace(-length / 2, length / 2, count):
                point = [0.0, 0.0]
                point[axis] = side * (dimensions[axis] / 2 + WORKING_DISTANCE)
                point[1 - axis] = along
                points.append(point)
    return points


def candidates(centre, dimensions, yaw=0.0, blockers=(), storage=False):
    """Poses around the furniture, each facing its aim point, clear of `blockers`.

    Storage gets poses only off its long faces, since the ends and corners
    look at side panels rather than into the shelves."""
    if storage:
        samples = _long_faces(dimensions)
    else:
        samples = _standoff_line(dimensions)
    poses = []
    for local in samples:
        x, y = to_map(local, centre, yaw)
        aim = aim_point((x, y), centre, dimensions, yaw)
        pose = (float(x), float(y), float(math.atan2(aim[1] - y, aim[0] - x)))
        if not blocks(pose, blockers):
            poses.append(pose)
    return poses


def surface_points(centre, dimensions, yaw):
    """Map XY samples covering the footprint: what the views together must take in."""
    axes = []
    for size in dimensions[:2]:
        count = int(math.ceil(size / SURFACE_SPACING)) + 1
        axes.append(np.linspace(-size / 2, size / 2, count))
    grid = np.stack(np.meshgrid(axes[0], axes[1], indexing="ij"), axis=-1)
    return to_map(grid.reshape(-1, 2), centre, yaw)


def sees(pose, points):
    """Which points lie inside the camera's horizontal view from `pose`."""
    offset = np.asarray(points, dtype=float) - np.asarray(pose[:2])
    bearing = np.arctan2(offset[:, 1], offset[:, 0]) - pose[2]
    bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
    distance = np.hypot(offset[:, 0], offset[:, 1])
    return (distance <= MAX_RANGE) & (np.abs(bearing) <= HALF_FOV)


def order(poses, points, robot_xy=None):
    """Poses in visiting order, and how many views the location needs.

    The first view is the nearest one showing within GAIN_STEP of what the best
    single view shows. Each next view shows the most still-unseen surface, the
    shorter drive winning near-ties. Once no view adds a GAIN_STEP of new surface, the view furthest
    from those already chosen among those still seeing some of it comes next,
    so a retry looks from another side."""
    visible = []
    best_first = 0.0
    for pose in poses:
        visible.append(sees(pose, points))
        best_first = max(best_first, float(np.mean(visible[-1])))
    seen = np.zeros(len(points), dtype=bool)
    remaining = list(range(len(poses)))
    ordered = []
    needed = None
    here = robot_xy
    while remaining:

        def gain(index):
            return np.count_nonzero(visible[index] & ~seen) / len(points)

        def rank(index):
            x, y, _ = poses[index]
            step = math.floor(gain(index) / GAIN_STEP)
            separation = 0.0
            # Slivers left at the image edges are not worth a drive next door.
            if step == 0:
                separation = math.inf
                for chosen in ordered:
                    separation = min(separation, math.hypot(x - chosen[0], y - chosen[1]))
            travel = 0.0
            if here is not None:
                travel = math.hypot(x - here[0], y - here[1])
            blind = not np.any(visible[index])
            if not ordered:
                # Nearest first, but not an end view that shows a fifth of a long table.
                return (gain(index) < best_first - GAIN_STEP, travel)
            return (-step, blind, -separation, travel)

        best = min(remaining, key=rank)
        if needed is None and gain(best) < GAIN_STEP:
            needed = len(ordered)
        remaining.remove(best)
        seen |= visible[best]
        ordered.append(poses[best])
        here = poses[best][:2]
    if needed is None:
        needed = len(ordered)
    return (ordered, min(MAX_VIEWS, max(MIN_VIEWS, needed)))


def shelf_heights(lower, upper, distance):
    """Aim heights whose views, stacked, cover [lower, upper] from `distance` away."""
    band = 2 * distance * math.tan(HALF_FOV_VERTICAL)
    count = max(1, math.ceil((upper - lower) / band))
    step = (upper - lower) / count
    heights = []
    for index in range(count):
        heights.append(lower + step * (index + 0.5))
    return heights
