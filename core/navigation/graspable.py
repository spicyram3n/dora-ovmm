"""Reposition the base so the arm can reach a detected object.

The probe poses below stand in for grasps: reaching one says the hand can be
placed at the object from that direction, not that closing on it would hold.
GraspGenX poses replace them once grasp generation is connected. Reachability
comes from Toyota's collision-aware IK service through `base_placement.solve`.
"""

import numpy as np
from core.navigation import standoff

# hand_palm_link: +z leaves the palm along the approach, +y is the finger axis.
FINGER_LENGTH = 0.094
APPROACH_DEPTH = 0.07
# config/nav2/nav2_params.yaml xy_goal_tolerance; the solver certifies a much tighter cluster.
NAV_TOLERANCE = 0.25
NAV_YAW_TOLERANCE = 0.25
# The solver's robustness radius is 1.5 cells x 0.05 m, so parking inside 0.08 m
# keeps the certificate it issued. Yaw is looser: the arm can roll to compensate.
PRECISE_TOLERANCE = 0.08
PRECISE_YAW_TOLERANCE = 0.15
APPROACH_TIMEOUT = 180


def _palm_pose(point, approach, closing):
    """(4, 4) hand pose whose +z reaches `point` and whose +y closes the fingers."""
    z = np.asarray(approach, dtype=float)
    z = z / np.linalg.norm(z)
    y = np.asarray(closing, dtype=float)
    # Keep only the part of the finger axis that is perpendicular to the approach.
    y = y - (y @ z) * z
    length = np.linalg.norm(y)
    if length < 1e-6:
        raise ValueError("Finger axis must not be parallel to the approach")
    y = y / length
    pose = np.eye(4)
    pose[:3, 0] = np.cross(y, z)
    pose[:3, 1] = y
    pose[:3, 2] = z
    # The palm sits back from the grasp point, not on it.
    pose[:3, 3] = np.asarray(point, dtype=float) - APPROACH_DEPTH * z
    return pose


def probes(centre, dimensions, bearings=6, top_down=True):
    """Hand poses that reach the object box from around it and from above."""
    if bearings < 1:
        raise ValueError("bearings must be positive")
    centre = np.asarray(centre, dtype=float)
    dimensions = np.asarray(dimensions, dtype=float)
    if centre.shape != (3,) or dimensions.shape != (3,):
        raise ValueError("Expected a 3D centre and 3D dimensions")
    up = np.array([0.0, 0.0, 1.0])
    poses = []
    for angle in np.linspace(0, 2 * np.pi, bearings, endpoint=False):
        # The approach points at the object; the fingers stay level either side.
        approach = np.array([-np.cos(angle), -np.sin(angle), 0.0])
        poses.append(_palm_pose(centre, approach, np.cross(up, approach)))
    if top_down:
        # Grip near the top of a tall object; the fingers cannot reach deeper.
        top = centre[2] + dimensions[2] / 2
        point = [centre[0], centre[1], max(centre[2], top - FINGER_LENGTH)]
        # Close across the narrower horizontal extent.
        if dimensions[0] <= dimensions[1]:
            closing = [1.0, 0.0, 0.0]
        else:
            closing = [0.0, 1.0, 0.0]
        poses.append(_palm_pose(point, -up, closing))
    return np.array(poses)


def _planar(matrix, pose):
    """(x, y, yaw) re-expressed by a planar (4, 4); map and odom differ only in the plane."""
    x, y, yaw = pose
    point = matrix @ np.array([x, y, 0.0, 1.0])
    turned = yaw + np.arctan2(matrix[1, 0], matrix[0, 0])
    return (
        float(point[0]),
        float(point[1]),
        float(np.arctan2(np.sin(turned), np.cos(turned))),
    )


def _cluster(bases):
    """A returned pose near the middle of the tied-for-best cluster, and its radius."""
    centre = bases[:, :2].mean(axis=0)
    offsets = np.linalg.norm(bases[:, :2] - centre, axis=1)
    # Never drive to an averaged pose the solver did not actually certify.
    return (bases[int(np.argmin(offsets))], float(offsets.max()))


def rank(results, robot_xy, map_from_odom):
    """Every returned base pose in map, least travel first.

    Do not collapse a probe's results to its centre before collision/path checks:
    that representative may be blocked while other returned poses are usable.
    """
    options = []
    for index, (bases, _) in enumerate(results):
        if len(bases) == 0:
            continue
        _, radius = _cluster(bases)
        for base in bases:
            x, y, yaw = _planar(map_from_odom, base)
            travel = float(np.hypot(x - robot_xy[0], y - robot_xy[1]))
            options.append((travel, index, (x, y, yaw), radius, len(bases)))
    options.sort(key=lambda option: option[0])
    return options


def _approach(navigator, pose):
    """Drive the last leg with the goal checker tightened, then measure what we got.

    The stock 0.25 m tolerance is three times the solver's robustness radius, so
    arriving under it proves nothing about reach. Restore it whatever happens:
    a permanently precise checker would make ordinary navigation fail."""
    # Stop inside the acceptance band, leaving margin for final localization
    # updates instead of repeatedly failing just beyond the same boundary.
    navigator.set_goal_tolerance(PRECISE_TOLERANCE*.5, PRECISE_YAW_TOLERANCE*.5)
    try:
        navigator.drive_to(
            *pose,
            timeout=APPROACH_TIMEOUT,
            tolerance=PRECISE_TOLERANCE,
            yaw_tolerance=PRECISE_YAW_TOLERANCE,
        )
    finally:
        navigator.set_goal_tolerance(NAV_TOLERANCE, NAV_YAW_TOLERANCE)
    # Where the base ended up decides this, not whether the action reported success.
    return navigator.residual(*pose)


def reposition(navigator, centre, dimensions, obstacles="--costmap", bearings=6,
               timeout=30.0, blockers=(), hand_poses=None):
    """Park at a base pose the IK solver certified.

    Returns (pose, offset) only when the final measured position and yaw meet
    the approach tolerances; otherwise returns None. Reach probes still do not
    prove grasp feasibility at the measured joint/base configuration."""
    from core.navigation import base_placement

    poses = probes(centre, dimensions, bearings) if hand_poses is None else np.asarray(hand_poses)
    # The IK service takes an unstamped hand goal: transform to odom ourselves.
    odom_from_map = navigator.frame_transform("odom", "map")
    results = base_placement.solve(
        odom_from_map @ poses, obstacles=obstacles, goal_frame="odom", timeout=timeout
    )
    map_from_odom = navigator.frame_transform("map", "odom")
    options = rank(results, navigator.robot_xy(), map_from_odom)
    # The costmap masks the IK search, but a table is four thin legs to a laser:
    # under its top reads as free space. Apply the same box filter the
    # observation poses get, or the base parks beneath the furniture.
    clear = []
    for option in options:
        if not standoff.blocks(option[2], blockers):
            clear.append(option)
    print(
        f"Base placement: {sum(len(bases) > 0 for bases, _ in results)}/{len(poses)}"
        f" probe directions reachable; {len(options)} returned base poses,"
        f" {len(clear)} clear of furniture"
    )
    for travel, index, pose, radius, count in clear:
        print(
            f"  probe {index}: {count} base poses, cluster radius {radius:.2f} m,"
            f" {travel:.2f} m away -> ({pose[0]:+.2f}, {pose[1]:+.2f}, yaw {pose[2]:+.2f})"
        )
        if not navigator.reachable(*pose):
            continue
        offset, error = _approach(navigator, pose)
        print(f"    parked {offset:.3f} m and {error:.3f} rad from it")
        if offset <= PRECISE_TOLERANCE and error <= PRECISE_YAW_TOLERANCE:
            return (pose, offset)
    # Earlier misses do not describe the final robot pose, and a small XY
    # residual alone must not hide a failed yaw check.
    return None
