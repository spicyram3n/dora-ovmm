"""Which of GraspGenX's candidates are worth trying, and in what order.

GraspGenX returns a couple of hundred contact poses ranked only by its own
confidence. Two things it cannot know get applied here: whether the arm can
reach a pose from where the robot actually ended up, and whether the object is
narrow enough at that pose for the fingers to close on it.

That filtering is what replaces arm redundancy. The arm has five joints and a
grasp pose has six degrees of freedom, so the base yaw is what makes a pose
reachable at all -- and Nav2 stops within 0.25 rad of the yaw it was asked for.
Nothing absorbs that error. What saves the grasp is having two hundred
candidates and taking one that happens to suit where the robot is standing,
rather than insisting on the one that was best before it drove.

Pure numpy: no ROS, no MoveIt, no planning scene. Deciding which candidates are
worth planning is separate from planning them.
"""

import numpy as np

from navigation import reach

# Fingertip separation of the real HSRC hand, from forward kinematics on
# hsrc_hand.urdf with the mimic linkage honoured: 0.1293 m open, 0.0015 closed.
# x_grippers/hsrc_hand/config.json claims 0.196 m, which is wrong -- its bbox
# was measured from a vis_mesh exported with the fingers folded backwards.
JAW_OPEN = 0.1293

# How much narrower than the jaw the object must be, so the fingers close on it
# rather than onto their own stops.
JAW_CLEARANCE = 0.010

# Window along the grasp's approach axis, measured from hand_palm_link, in
# which the fingers can actually enclose something. The tips sweep from
# z = 0.045 (open) to z = 0.092 (closed).
FINGER_WINDOW = (0.030, 0.100)

# How far to the side of the grasp a point can be and still lie between the
# fingers: half the open jaw across the closing axis, and the fingers' own
# breadth across the third axis. Without this bound a grasp a metre away still
# "sees" the object, since being at the right height says nothing about being
# in the right place.
JAW_REACH = JAW_OPEN / 2
FINGER_BREADTH = 0.040

# Pregrasp standoffs, longest first. 0.12 m clears sensors_xtion.yaml's
# padding_offset of 0.1 m, so it is the one to want; the shorter ones are
# fallbacks, and a plan against a padded octomap may well refuse them.
#
# Several are needed because backing off along a *downward* approach raises
# the pregrasp, and an object near the top-down reach ceiling can have a
# perfectly reachable grasp whose pregrasp is above the lift's range. Rejecting
# such a candidate outright throws away a grasp the arm can actually make.
STANDOFFS = (0.12, 0.10, 0.08, 0.06)
STANDOFF = STANDOFFS[0]


def pregrasp(pose, standoff=STANDOFF):
    """`pose` backed off along its own approach axis, which for a
    hand_palm_link pose is local +Z. This is what a planner can reach; the
    grasp itself is a contact pose, with the gripper already on the object."""
    back = np.eye(4)
    back[2, 3] = -standoff
    return pose @ back


def in_grasp_frame(points, pose):
    """`points` (N, 3) expressed in the grasp's own frame, so the axes mean
    something: x is across the fingers, y is the closing direction, z is
    approach."""
    return (points - pose[:3, 3]) @ pose[:3, :3]


def jaw_span(points, pose):
    """How wide the object is along the closing direction, counting only what
    lies between the fingers. Infinite if nothing does -- a grasp closing on
    empty space is not a grasp."""
    local = in_grasp_frame(points, pose)
    between = local[
        (local[:, 2] > FINGER_WINDOW[0])
        & (local[:, 2] < FINGER_WINDOW[1])
        & (np.abs(local[:, 1]) < JAW_REACH)
        & (np.abs(local[:, 0]) < FINGER_BREADTH)
    ]
    if len(between) == 0:
        return np.inf
    return float(between[:, 1].max() - between[:, 1].min())


def fits(points, pose):
    return jaw_span(points, pose) <= JAW_OPEN - JAW_CLEARANCE


def usable_standoff(pose, base_xy, standoffs=STANDOFFS, robot=reach.DEFAULT_ROBOT):
    """The longest standoff whose pregrasp the arm can still reach, or None.

    Longest first: a short standoff buys reachability at the cost of planning
    into the octomap's 0.1 m padding.
    """
    for standoff in standoffs:
        if reach.reachable_from(pregrasp(pose, standoff), base_xy, robot):
            return standoff
    return None


def feasible(poses, scores, points, base_xy, standoffs=STANDOFFS, robot=reach.DEFAULT_ROBOT):
    """(index, standoff) for the candidates worth planning, best score first.

    A candidate survives if the object fits the jaw, the arm can reach the
    contact pose, and *some* standoff leaves the pregrasp reachable too.

    Insisting on one fixed standoff threw away grasps the arm could plainly
    make: backing off along a downward approach *raises* the pregrasp, so for
    an object near the top-down ceiling the grasp sits inside the envelope
    while its pregrasp sits outside it.
    """
    keep = []
    for i in np.argsort(scores)[::-1]:
        if not fits(points, poses[i]):
            continue
        if not reach.reachable_from(poses[i], base_xy, robot):
            continue
        standoff = usable_standoff(poses[i], base_xy, standoffs, robot)
        if standoff is not None:
            keep.append((int(i), standoff))
    return keep


def report(poses, scores, points, base_xy, standoffs=STANDOFFS, robot=reach.DEFAULT_ROBOT):
    """Why candidates were dropped, as counts. These are four different
    problems with four different fixes -- pick another object, drive closer,
    shorten the approach, or nothing."""
    counts = {"total": len(scores), "too_wide": 0, "out_of_reach": 0,
              "no_standoff": 0, "kept": 0}
    for i in range(len(scores)):
        if not fits(points, poses[i]):
            counts["too_wide"] += 1
        elif not reach.reachable_from(poses[i], base_xy, robot):
            counts["out_of_reach"] += 1
        elif usable_standoff(poses[i], base_xy, standoffs, robot) is None:
            counts["no_standoff"] += 1
        else:
            counts["kept"] += 1
    return counts


def shortfall(poses, scores, points, base_xy, robot=reach.DEFAULT_ROBOT):
    """For candidates that fit the jaw but are out of reach, how far the base
    is from the nearest point of the *grasp's* usable band, best first.

    Measured on the grasp, matching what `feasible` gates on. Measuring the
    pregrasp instead made this contradict the filter -- it would report a
    candidate as 0.00 m out of band, and name the robot's current position as
    the fix, for a grasp `feasible` had just rejected.

    Returns [(index, metres, where)]; `where` is the nearest base position that
    would work.
    """
    out = []
    for i in np.argsort(scores)[::-1]:
        if not fits(points, poses[i]):
            continue
        annulus = reach.base_annulus(poses[i], robot)
        if annulus is None or annulus.usable is None:
            continue
        low, high = annulus.usable
        centre = np.asarray(annulus.center)
        offset = np.asarray(base_xy) - centre
        distance = float(np.linalg.norm(offset))
        if low <= distance <= high:
            continue
        target = float(np.clip(distance, low, high))
        direction = offset / distance if distance > 1e-9 else np.array([1.0, 0.0])
        out.append((int(i), abs(distance - target), tuple(centre + direction * target)))
    return out
