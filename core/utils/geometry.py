"""Planar box geometry shared by the scene graph and navigation. No ROS needed.

A turned box is (centre, size, yaw): size runs along the box's own axes and
yaw turns those axes about the vertical from map +x."""

import math
import numpy as np
from scipy.spatial import ConvexHull, QhullError


def to_local(xy, centre, yaw):
    """Map XY points expressed along the box's own axes."""
    # Subtract the box centre before undoing its rotation.
    offset = np.asarray(xy, dtype=float)[..., :2] - np.asarray(centre, dtype=float)[:2]
    c, s = (math.cos(yaw), math.sin(yaw))
    x = c * offset[..., 0] + s * offset[..., 1]
    y = -s * offset[..., 0] + c * offset[..., 1]
    return np.stack([x, y], axis=-1)


def to_map(local, centre, yaw):
    """Box-frame XY points back in map."""
    local = np.asarray(local, dtype=float)
    c, s = (math.cos(yaw), math.sin(yaw))
    # Rotate local points and add the map position of the box centre.
    x = centre[0] + c * local[..., 0] - s * local[..., 1]
    y = centre[1] + s * local[..., 0] + c * local[..., 1]
    return np.stack([x, y], axis=-1)


def footprint_corners(centre, size, yaw=0.0):
    """The four floor corners of a turned box, counter-clockwise."""
    half_x, half_y = (size[0] / 2, size[1] / 2)
    local = [(-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y)]
    return to_map(local, centre, yaw)


def box_corners(centre, size, yaw=0.0):
    """The eight corners of a box turned by `yaw` about its vertical centre line."""
    corners = []
    # Pair each floor corner with the bottom and top heights.
    for x, y in footprint_corners(centre, size, yaw):
        for z in (centre[2] - size[2] / 2, centre[2] + size[2] / 2):
            corners.append([x, y, z])
    return np.array(corners)


def fit_footprint(points):
    """Fit the smallest-area floor rectangle; return (centre, size, yaw) with yaw within 45 degrees of map X."""
    xy = np.asarray(points, dtype=float)[:, :2]
    try:
        hull = xy[ConvexHull(xy).vertices]
    except QhullError:
        # Too few or collinear points have no hull; their own edges still work.
        hull = xy
    best = None
    # Try a rectangle aligned with each boundary edge and keep the smallest area.
    for index in range(len(hull)):
        edge = hull[(index + 1) % len(hull)] - hull[index]
        # A rectangle turned by a quarter turn is the same rectangle.
        yaw = math.atan2(edge[1], edge[0]) % (math.pi / 2)
        if yaw >= math.pi / 4:
            yaw -= math.pi / 2
        local = to_local(hull, (0.0, 0.0), yaw)
        # Find the bounds in this trial orientation and calculate their rectangle area.
        lower, upper = (local.min(axis=0), local.max(axis=0))
        area = float(np.prod(upper - lower))
        if best is None or area < best[0] - 1e-9:
            best = (area, yaw, lower, upper)
    _, yaw, lower, upper = best
    # Move the winning rectangle's centre back into map coordinates.
    centre = to_map((lower + upper) / 2, (0.0, 0.0), yaw)
    return (centre, upper - lower, yaw)
