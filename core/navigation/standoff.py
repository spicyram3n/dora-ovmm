"""Observation poses around furniture, filtered against surrounding furniture."""

import numpy as np

WORKING_DISTANCE = 0.8
MIN_STANDOFF = 0.8
ROBOT_RADIUS = 0.3


def ring_radius(dimensions, angle):
    """Add observation distance to the box extent on this bearing."""
    half_x, half_y = (dimensions[0] / 2, dimensions[1] / 2)
    # Measure from the furniture edge, not just its centre.
    reach = half_x * abs(np.cos(angle)) + half_y * abs(np.sin(angle))
    return float(max(reach + WORKING_DISTANCE, MIN_STANDOFF))


def blocks(pose, blockers):
    """Check the base centre against boxes expanded by the robot radius."""
    # Expand each obstacle so the whole base has clearance.
    for centre, dimensions in blockers:
        x_distance = abs(pose[0] - centre[0])
        y_distance = abs(pose[1] - centre[1])
        x_limit = dimensions[0] / 2 + ROBOT_RADIUS
        y_limit = dimensions[1] / 2 + ROBOT_RADIUS
        if x_distance <= x_limit and y_distance <= y_limit:
            return True
    return False


def candidates(centroid, dimensions, count=12, robot_xy=None, blockers=()):
    """Poses around the furniture, each turned to face it, best first."""
    poses = []
    for angle in np.linspace(0, 2 * np.pi, count, endpoint=False):
        radius = ring_radius(dimensions, angle)
        poses.append(
            (
                float(centroid[0] + radius * np.cos(angle)),
                float(centroid[1] + radius * np.sin(angle)),
                float(angle + np.pi),
                radius,
            )
        )

    def rank(pose):
        if robot_xy is not None:
            travel = np.hypot(pose[0] - robot_xy[0], pose[1] - robot_xy[1])
        else:
            travel = 0.0
        return (round(pose[3] / 0.1), travel)

    poses.sort(key=rank)
    results = []
    for x, y, yaw, _ in poses:
        if not blocks((x, y), blockers):
            results.append((x, y, yaw))
    # Interleave opposite sides so the next view is not an adjacent sample.
    if len(results) > 1:
        first = results[0]
        def separation(pose):
            return np.hypot(pose[0] - first[0], pose[1] - first[1])

        opposite = max(results[1:], key=separation)
        results.remove(opposite)
        results.insert(1, opposite)
    return results
