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
# Narrow the usable camera view to leave a margin at image edges.
HALF_FOV = 0.45
HALF_FOV_VERTICAL = 0.35
# Ignore distant surface samples where small objects are hard to see.
MAX_RANGE = 1.5
# Surface samples used to count how much of the furniture a view takes in.
SURFACE_SPACING = 0.1
# Group similar coverage gains; prefer shorter drives within each group.
GAIN_STEP = 0.05
MIN_VIEWS = 2
MAX_VIEWS = 6
# Search storage from its long faces so the camera looks into the shelves.
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
    """Check a map costmap cell; treat blocked and off-map positions as unavailable."""
    info = grid.info
    # Convert the base position to a grid cell; reject cells outside the map.
    column = math.floor((pose[0] - info.origin.position.x) / info.resolution)
    row = math.floor((pose[1] - info.origin.position.y) / info.resolution)
    if not (0 <= column < info.width and 0 <= row < info.height):
        return False
    return grid.data[row * info.width + column] == 0


def aim_point(xy, centre, dimensions, yaw):
    """Aim halfway through the furniture depth, aligned with the viewer along its long side."""
    # Work in furniture-aligned coordinates so the same aiming rule handles rotated boxes.
    local = to_local(xy, centre, yaw)
    if dimensions[0] >= dimensions[1]:
        aim = [np.clip(local[0], -dimensions[0] / 2, dimensions[0] / 2), 0.0]
    else:
        aim = [0.0, np.clip(local[1], -dimensions[1] / 2, dimensions[1] / 2)]
    return to_map(aim, centre, yaw)


def _standoff_line(dimensions):
    """Sample a path WORKING_DISTANCE outside the box, with rounded corners."""
    half_x, half_y = (dimensions[0] / 2, dimensions[1] / 2)
    # Walk around each face, joining adjacent faces with a rounded corner.
    corners = [(half_x, -half_y), (half_x, half_y), (-half_x, half_y), (-half_x, -half_y)]
    lengths = [2 * half_y, 2 * half_x, 2 * half_y, 2 * half_x]
    arc = WORKING_DISTANCE * math.pi / 2
    perimeter = sum(lengths) + 4 * arc
    count = max(4, round(perimeter / POSE_SPACING))
    samples = []
    for index in range(count):
        # Space sample positions evenly around the full offset perimeter.
        along = perimeter * index / count
        for face in range(4):
            normal = face * math.pi / 2
            start = np.array(corners[face])
            end = np.array(corners[(face + 1) % 4])
            # Place samples on this straight face until its length has been used.
            if along < lengths[face]:
                point = start + (end - start) * along / lengths[face]
                offset = [math.cos(normal), math.sin(normal)]
                samples.append(point + WORKING_DISTANCE * np.array(offset))
                break
            along -= lengths[face]
            # Place the next samples on the quarter-circle around this corner.
            if along < arc:
                angle = normal + along / WORKING_DISTANCE
                offset = [math.cos(angle), math.sin(angle)]
                samples.append(end + WORKING_DISTANCE * np.array(offset))
                break
            along -= arc
    return samples


def _long_faces(dimensions):
    """Sample positions outside the long faces, at most POSE_SPACING apart."""
    points = []
    for axis in (0, 1):
        # The face whose outward normal runs along `axis` spans the other one.
        length = dimensions[1 - axis]
        # Skip short end panels when generating views into storage furniture.
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
    """Return clear viewing poses; use only the long faces when searching storage."""
    # Use long-face views for shelves and a full surrounding path for other furniture.
    if storage:
        samples = _long_faces(dimensions)
    else:
        samples = _standoff_line(dimensions)
    poses = []
    for local in samples:
        x, y = to_map(local, centre, yaw)
        aim = aim_point((x, y), centre, dimensions, yaw)
        # Turn each base pose toward its selected point on the furniture.
        pose = (float(x), float(y), float(math.atan2(aim[1] - y, aim[0] - x)))
        if not blocks(pose, blockers):
            poses.append(pose)
    return poses


def surface_points(centre, dimensions, yaw):
    """Map XY samples covering the footprint: what the views together must take in."""
    axes = []
    # Generate regularly spaced X and Y samples across the furniture surface.
    for size in dimensions[:2]:
        count = int(math.ceil(size / SURFACE_SPACING)) + 1
        axes.append(np.linspace(-size / 2, size / 2, count))
    # Combine every X and Y sample to cover the furniture footprint.
    grid = np.stack(np.meshgrid(axes[0], axes[1], indexing="ij"), axis=-1)
    return to_map(grid.reshape(-1, 2), centre, yaw)


def sees(pose, points):
    """Which points lie inside the camera's horizontal view from `pose`."""
    offset = np.asarray(points, dtype=float) - np.asarray(pose[:2])
    bearing = np.arctan2(offset[:, 1], offset[:, 0]) - pose[2]
    # Wrap the angle to [-pi, pi] before checking the camera limits.
    bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
    distance = np.hypot(offset[:, 0], offset[:, 1])
    return (distance <= MAX_RANGE) & (np.abs(bearing) <= HALF_FOV)


def order(poses, points, robot_xy=None):
    """Order views by new surface coverage and travel distance; return the required view count.

    Start near the robot with good coverage. Once gains are small, prefer a
    different side for retries. Clamp the required count to MIN_VIEWS/MAX_VIEWS."""
    # Precompute the surface points visible from each candidate pose.
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

        # Measure only surface points that earlier chosen views have not covered.
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

        # Select the best remaining view and remember when extra coverage becomes small.
        best = min(remaining, key=rank)
        if needed is None and gain(best) < GAIN_STEP:
            needed = len(ordered)
        remaining.remove(best)
        # Mark this view's surface as seen before choosing the next view.
        seen |= visible[best]
        ordered.append(poses[best])
        here = poses[best][:2]
    if needed is None:
        needed = len(ordered)
    return (ordered, min(MAX_VIEWS, max(MIN_VIEWS, needed)))


def shelf_heights(lower, upper, distance):
    """Aim heights whose views, stacked, cover [lower, upper] from `distance` away."""
    # Find the height covered by one view, then stack enough views for the shelf.
    band = 2 * distance * math.tan(HALF_FOV_VERTICAL)
    count = max(1, math.ceil((upper - lower) / band))
    step = (upper - lower) / count
    heights = []
    for index in range(count):
        heights.append(lower + step * (index + 0.5))
    return heights
