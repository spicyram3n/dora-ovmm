"""Where to stand to look at a piece of furniture.

Driving to a furniture's own centroid drives into it. What is wanted is a pose
just clear of its footprint, facing it, close enough that the head camera can
resolve a small object on it.

The ring is offset from the *footprint*, not drawn at a fixed radius from the
centroid, which matters as soon as a piece is not square: a 0.5 x 2.3 m table
needs 1.05 m of clearance approached from its long side and 1.95 m from its
end, and a single radius has to take the larger of those everywhere.

stretch-compose instead sizes the standoff so the whole piece fits the camera
frame, then finds a viewpoint by Poisson-meshing a prescanned point cloud and
raycasting for occlusion (source/utils/point_clouds.py, ~110 lines). Fitting
the whole piece in frame puts the robot 2 m from a long table, where a can on
it is a handful of pixels; the head pans and tilts, so covering a wide piece
is a sweep, not a reason to back away. And which viewpoints are reachable is
something Nav2's costmap and planner already know.
"""

import numpy as np

# Clearance between the robot's footprint circle and the furniture's. Nav2
# uses robot_radius 0.3 and inflation_radius 0.5 (nav2_params.yaml), so much
# less than this puts the goal in inflated, high-cost space where the
# controller struggles even when the planner accepts it.
WORKING_DISTANCE = 0.8

# Never closer than this to the centroid, however small the piece.
MIN_STANDOFF = 0.8

# nav2_params.yaml's robot_radius: how much clearance a pose needs from
# another piece's footprint before the base actually fits there.
ROBOT_RADIUS = 0.3


def ring_radius(dimensions, angle):
    """Distance from the centroid to stand on this bearing: clear of the
    footprint, plus a working distance.

    The first term is the box's support function, i.e. how far the footprint
    itself reaches out along `angle`.
    """
    half_x, half_y = dimensions[0] / 2, dimensions[1] / 2
    reach = half_x * abs(np.cos(angle)) + half_y * abs(np.sin(angle))
    return float(max(reach + WORKING_DISTANCE, MIN_STANDOFF))


def blocks(pose, blockers):
    """Is (x, y) inside another piece's footprint, plus room for the base?"""
    return any(abs(pose[0] - c[0]) <= d[0] / 2 + ROBOT_RADIUS
               and abs(pose[1] - c[1]) <= d[1] / 2 + ROBOT_RADIUS
               for c, d in blockers)


def candidates(centroid, dimensions, count=12, robot_xy=None, blockers=()):
    """Poses around the furniture, each turned to face it, best first.

    Returns [(x, y, yaw)]. Ordered by how close to the piece that bearing lets
    the robot stand, in 10 cm bands, and then by how far the robot has to
    drive. Approaching a long table from its side beats approaching it from
    the end, because everything on it is then within arm's reach rather than
    two metres away; among comparable bearings there is no reason to cross the
    room.

    That preference is for the *broadside*, which is exactly where a coffee
    table's sofas sit, so `blockers` -- (centroid, dimensions) for the other
    furniture -- drops poses landing inside one of them. Nav2 does not catch
    these on its own: it finds a path to such a pose, then its 0.25 m goal
    tolerance lets the controller stop against the sofa and report success.
    Whether the rest are reachable is still the planner's job.
    """
    poses = []
    for angle in np.linspace(0, 2 * np.pi, count, endpoint=False):
        radius = ring_radius(dimensions, angle)
        poses.append((float(centroid[0] + radius * np.cos(angle)),
                      float(centroid[1] + radius * np.sin(angle)),
                      float(angle + np.pi),      # face back towards the centroid
                      radius))

    def rank(pose):
        travel = np.hypot(pose[0] - robot_xy[0], pose[1] - robot_xy[1]) if robot_xy else 0.0
        return round(pose[3] / 0.1), travel

    poses.sort(key=rank)
    return [(x, y, yaw) for x, y, yaw, _ in poses if not blocks((x, y), blockers)]
