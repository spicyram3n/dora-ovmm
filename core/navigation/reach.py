"""Where the base must stand for the arm to reach a given hand pose.

`standoff.py` parks the robot close enough to *see* a piece of furniture.
This answers the other question -- close enough to *reach* something on it --
and the two numbers are not the same. Reach tops out around half a metre,
while standoff deliberately keeps 0.8 m of clearance to stay out of Nav2's
inflated space, so a standoff pose is always too far to grasp from. The move
in is a mandatory second stage, not a refinement.

docs/navigation-and-grasping.md Part 3 planned to get this by sampling a
dense grid of hand poses, running IK on each, and inverting the result into a
map of base offsets (the Reuleaux technique). That is unnecessary here: HSR's
inverse reachability is available in closed form, and Toyota already derived
it. `hsrb_analytic_ik`'s `GetHsrcBasePositionRange` takes a hand pose and
returns the annulus of base positions it can be reached from. This module is
that function ported to numpy, so the Python half of the stack can call it
without a pybind wrapper or a ROS service.

Why it collapses to an annulus, from ik_solver_base.cpp:43-124: back the palm
offset off the hand pose and you get the wrist centre, where the arm_roll,
wrist_flex and wrist_roll axes meet. Its height is absorbed entirely by the
prismatic lift, so height alone decides reachable-at-all; what is left is a
circle the base can sit anywhere on. The source comment puts it as "this
point ... is the rotation center of the cart position".

Sampled maps have a resolution and a build step and a file to keep in sync.
This has none of those, is exact, and costs microseconds per grasp -- cheap
enough to run over every candidate GraspGenX returns.

`move_into_reach` at the bottom is the one thing here that drives. It takes a
navigator rather than importing one, so this module stays pure numpy and the
tests need no ROS.
"""

import math
from dataclasses import dataclass

import numpy as np

from navigation import standoff

# Link lengths and joint limits, copied from hsrb_analytic_ik so this stays a
# pure-numpy module. hsrc differs from hsrb in more than the name -- a taller
# lift mount and a longer palm offset -- and using the wrong set shifts every
# answer by centimetres without failing, so the robot is named, not defaulted
# silently. Sources:
#   hsrc: src/hsrb_analytic_ik/hsrc_ik_solver.cpp:41-57
#   hsrb: src/hsrb_analytic_ik/hsrb_ik_solver.cpp:41-57
# Only the seven lengths and three limits BasePositionRange actually reads are
# kept; the wrist limits do not enter this calculation.
PARAMETERS = {
    "hsrc": dict(L3=0.350, L41=0.141, L42=0.0785, L51=0.005, L52=0.345,
                 L81=0.0, L82=0.155, t3_min=0.0, t3_max=0.69, t4_min=-2.62),
    "hsrb": dict(L3=0.340, L41=0.141, L42=0.078, L51=0.005, L52=0.345,
                 L81=0.012, L82=0.1405, t3_min=0.0, t3_max=0.69, t4_min=-2.62),
}

DEFAULT_ROBOT = "hsrc"

# How far inside the annulus edges to keep a chosen base pose. Both edges are
# singular postures -- the outer one is the arm fully extended, the inner one
# is it folded against the body -- where manipulability goes to nothing and
# Nav2's 0.25 m goal tolerance is enough to push the target out of reach
# entirely. This is the cheap stand-in for the manipulability score a sampled
# map would have stored.
MARGIN = 0.05

# Hand z pointing down: the usual pose for picking something off a surface.
TOP_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
# Hand z pointing along +x: reaching into a shelf rather than down onto it.
SIDE_ON = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])


def hand_pose(centroid, rotation=TOP_DOWN):
    """A 4x4 hand_palm_link pose at `centroid`, approaching along rotation's z."""
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = centroid
    return pose


@dataclass
class Annulus:
    """Base positions that can reach a hand pose: a ring centred on the wrist
    centre's (x, y), in whatever frame the hand pose was given in."""

    center: tuple
    radius_min: float
    radius_max: float

    @property
    def usable(self):
        """The ring less a `MARGIN` band at each edge, or None if the ring is
        too thin to have an interior."""
        low, high = self.radius_min + MARGIN, self.radius_max - MARGIN
        return (low, high) if low < high else None


def _wrist_center(pose, p):
    """The (x, y, z) where arm_roll, wrist_flex and wrist_roll meet, backed
    out of a hand pose by the palm offset."""
    rotation, position = pose[:3, :3], pose[:3, 3]
    return (rotation[0, 0] * p["L81"] - rotation[0, 2] * p["L82"] + position[0],
            rotation[1, 0] * p["L81"] - rotation[1, 2] * p["L82"] + position[1],
            rotation[2, 0] * p["L81"] - rotation[2, 2] * p["L82"] + position[2])


def _radius(t4, p):
    """Horizontal distance from the base origin to the wrist centre with the
    arm flexed to `t4`."""
    return math.hypot(p["L52"] * math.sin(t4) - p["L51"] * math.cos(t4) - p["L41"], p["L42"])


def base_annulus(pose, robot=DEFAULT_ROBOT):
    """The ring of base positions from which `pose` is reachable, or None if
    no base position reaches it.

    `pose` is a 4x4 homogeneous matrix for **hand_palm_link**, in a frame
    whose z is height above the floor -- map or odom, the same frame
    GraspGenX's poses are saved in. A None means the pose is simply too high
    or too low for the lift, and no amount of driving fixes it.

    hand_palm_link, not GraspGenX's canonical grasp frame. graspgenx_client
    already converts between them via the gripper's `base_rotation`, so a
    saved grasp is a hand_palm_link pose and goes straight in.
    """
    p = PARAMETERS[robot]
    center_x, center_y, zwo = _wrist_center(pose, p)

    # Height is settled by the lift alone, so it is a straight yes/no before
    # any geometry in the plane is worth computing.
    if zwo > p["t3_max"] + p["L3"] + p["L52"]:
        return None
    floor = (p["t3_min"] + p["L3"] + p["L52"] * math.cos(p["t4_min"])
             + p["L51"] * math.sin(p["t4_min"]))
    if zwo < floor:
        return None

    reach = math.hypot(p["L51"], p["L52"])
    alpha = math.atan2(p["L52"], p["L51"])

    # Outer edge: the most the arm can stretch out while still holding this
    # height. In the middle band the lift does the work and the arm extends
    # straight; outside it the arm has to angle down or up to make up the
    # difference, which costs horizontal reach.
    if zwo < p["t3_min"] + p["L3"]:
        radius_max = _radius(math.asin((zwo - p["t3_min"] - p["L3"]) / reach) - alpha, p)
    elif zwo > p["t3_max"] + p["L3"]:
        radius_max = _radius(math.asin((zwo - p["t3_max"] - p["L3"]) / reach) - alpha, p)
    else:
        radius_max = _radius(-alpha, p)

    # Inner edge: how close the arm can be tucked and still hold this height.
    folded = -p["t4_min"] - alpha
    if zwo >= p["t3_min"] + p["L3"] + p["L52"]:
        radius_min = math.hypot(-p["L51"] - p["L41"], p["L42"])
    elif zwo > (p["t3_min"] + p["L3"] + p["L52"] * math.cos(folded)
                + p["L51"] * math.sin(folded)):
        radius_min = _radius(math.asin((zwo - p["t3_min"] - p["L3"]) / reach) - alpha, p)
    else:
        radius_min = _radius(p["t4_min"], p)

    return Annulus((center_x, center_y), radius_min, radius_max)


def _yaw(annulus, x, y, radius, p):
    """Which way the base must face, standing at (x, y) on the ring.

    The arm only flexes in its own plane and the lift only moves along z, so
    the wrist centre is always exactly `L42` to the left of the base's x axis
    -- the arm is mounted off-centre. That pins the bearing to the wrist
    centre in the base frame at asin(L42 / radius), and the base yaw is
    whatever makes the world bearing come out to that.
    """
    bearing = math.atan2(annulus.center[1] - y, annulus.center[0] - x)
    return bearing - math.asin(p["L42"] / radius)


def base_poses(annulus, robot_xy=None, count=16, robot=DEFAULT_ROBOT):
    """Base poses on the ring, each turned so the arm lines up, best first.

    Returns [(x, y, yaw)], ordered by how little the robot has to move, since
    every one of them reaches the grasp equally well by construction --
    reachability is already decided by being on the ring at all. Which of them
    is *collision free* is not decided here; that is what Nav2's costmap and
    tmc_ik_solver_node's SolveIkWithCollision are for.

    Every pose sits at the *middle* of the usable band, and `robot_xy` only
    chooses which bearing to approach from. Standing at the nearest point of
    the band instead would save a little driving, but Nav2 stops anywhere
    within `xy_goal_tolerance` (0.25 m) of what it was asked for, and the band
    is often narrower than that. Aiming mid-band spends the whole half-width
    absorbing that error; aiming at the near edge spends none of it and misses
    by centimetres every time.

    Returns [] if the ring is too thin to keep `MARGIN` clear of both edges.
    """
    band = annulus.usable
    if band is None:
        return []
    p = PARAMETERS[robot]
    low, high = band
    center_x, center_y = annulus.center

    radius = (low + high) / 2
    poses = []
    for angle in np.linspace(0, 2 * np.pi, count, endpoint=False):
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        poses.append((x, y, _yaw(annulus, x, y, radius, p)))

    if robot_xy is not None:
        poses.sort(key=lambda pose: np.hypot(pose[0] - robot_xy[0], pose[1] - robot_xy[1]))
    return poses


def reachable_from(pose, base_xy, robot=DEFAULT_ROBOT, margin=MARGIN):
    """Can the arm reach `pose` without the base moving at all?

    Worth asking before planning a reposition: Nav2's goal tolerance is 0.25 m
    and driving is slow, so if the robot already stands in the band there is
    nothing to gain by shuffling.
    """
    annulus = base_annulus(pose, robot)
    if annulus is None:
        return False
    band = annulus.usable if margin == MARGIN else (annulus.radius_min + margin,
                                                    annulus.radius_max - margin)
    if band is None or band[0] >= band[1]:
        return False
    distance = math.hypot(base_xy[0] - annulus.center[0], base_xy[1] - annulus.center[1])
    return band[0] <= distance <= band[1]


def move_into_reach(navigator, pose, blockers=(), robot=DEFAULT_ROBOT, attempts=3):
    """Drive until the arm can reach `pose`. True once it can; False if the
    pose is outside the arm's envelope at every base position, none of them
    clear the furniture, or Nav2 could not converge.

    Drives repeatedly, not once. The usable band is about 0.2 m wide while
    Nav2's own xy_goal_tolerance is 0.25 m, so a single goal cannot be
    expected to land inside it -- the first drive gets close, and each further
    one starts from a smaller error. Checking `reachable_from` after every
    drive is what turns "Nav2 said it arrived" into "the arm can actually
    reach", which are not the same claim and were being confused.

    `blockers` is (centroid, dimensions) for the furniture, and is not
    optional in practice: this ring reaches to 0.49 m of a target that usually
    sits *on* a piece of furniture, while Nav2 needs `robot_radius` 0.3 m of
    clearance from it. A kinematically perfect pose is routinely one the
    costmap counts as a collision, and Nav2 will drive in and then be unable
    to get out, burning its recovery behaviours.
    """
    for attempt in range(attempts):
        robot_xy = navigator.robot_xy()
        if reachable_from(pose, robot_xy, robot):
            return True
        annulus = base_annulus(pose, robot)
        if annulus is None:
            return False
        poses = [p for p in base_poses(annulus, robot_xy, robot=robot)
                 if not standoff.blocks(p, blockers)]
        if not poses:
            return False
        if navigator.go_to_first_reachable(poses) is None:
            return False
        landed = navigator.robot_xy()
        gap = math.hypot(landed[0] - poses[0][0], landed[1] - poses[0][1])
        print(f"    attempt {attempt + 1}/{attempts}: landed {gap:.2f} m from the goal")
    return reachable_from(pose, navigator.robot_xy(), robot)
