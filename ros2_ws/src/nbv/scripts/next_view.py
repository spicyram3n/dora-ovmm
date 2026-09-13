#!/usr/bin/env python3
"""Where to stand and where to look, to learn the most about the target.

A view the HSR can take is six numbers: where the base is (x, y, yaw), how far
arm_lift_joint is up, and the head's pan and tilt. This offers a grid of them --
where the robot stands now, and a ring of places it could drive to -- and scores
each with view_gain.

Pan and tilt are not searched: for a base pose and a lift there is exactly one
pair that aims the camera at the target, and every other pair is worth less.
That turns a six-dimensional search into a three-dimensional one.

No ROS here, so a plan can be checked against a tree built in memory:

    python3 -m pytest -p no:anyio ros2_ws/src/nbv/test/test_next_view.py
"""

import sys
from collections import namedtuple
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from view_gain import MAX_RANGE, OCCUPIED, view_gain  # noqa: E402

# The camera's chain, read off hsrc1s.urdf. torso_lift_joint mimics
# arm_lift_joint at half its rate, so the head rises 0.345 m over the lift's
# full 0.69 m of travel.
BASE_TO_TORSO = 0.762
TORSO_PER_LIFT = 0.5
PAN_TO_TILT = np.array([0.02, 0.0, 0.0])
TILT_TO_CAMERA = np.array([-0.076822, 0.022, 0.248])
# head_rgbd_sensor_joint's own rpy, which turns the head's x-forward into the
# optical frame view_gain expects: z forward, x right, y down.
OPTICAL = Rotation.from_euler("xyz", [-np.pi / 2, 0.0, -np.pi / 2]).as_matrix()
LIFT_LIMITS = (0.0, 0.69)
PAN_LIMITS = (-3.84, 1.75)
TILT_LIMITS = (-1.57, 0.52)

# Nav2's robot_radius, from hsrb_rosnav_config/config/nav2_params.yaml.
BASE_RADIUS = 0.3
# Knee height to the torso: the band a base pose would actually collide in.
KNEE_HEIGHT = 0.1
# How far the robot will drive for a view worth twice what it can see from
# where it already stands.
DISTANCE_SCALE = 1.0

Candidate = namedtuple("Candidate", "base lift pan tilt")


def camera_pose(base, lift, pan, tilt):
    """(where the camera sits, its axes as columns) in the frame `base` is in.

    `base` is (x, y, yaw) on the floor and `lift` is arm_lift_joint.
    """
    turn = Rotation.from_euler("z", base[2]).as_matrix()
    panned = Rotation.from_euler("z", pan).as_matrix()
    # head_tilt_joint turns about -y, so a positive tilt looks up.
    tilted = Rotation.from_euler("y", -tilt).as_matrix()
    head = np.array([0.0, 0.0, BASE_TO_TORSO + TORSO_PER_LIFT * lift])
    return (np.array([base[0], base[1], 0.0])
            + turn @ (head + panned @ (PAN_TO_TILT + tilted @ TILT_TO_CAMERA)),
            turn @ panned @ tilted @ OPTICAL)


def head_towards(base, lift, target, passes=3):
    """The (pan, tilt) that aims the camera at `target`, or None if the head
    cannot turn that far.

    The camera hangs off both axes, so where it ends up depends on the angles it
    is being asked for; each pass divides the error by about twenty, and three
    leave the aim within 1.5 mm of the target -- a thirtieth of a cell. Its
    forward axis is exactly (cos tilt cos pan, cos tilt sin pan, sin tilt) in the
    base's frame, which is what makes each pass a plain pair of angles.
    """
    pan = tilt = 0.0
    for _ in range(passes):
        origin, _ = camera_pose(base, lift, pan, tilt)
        towards = np.asarray(target, dtype=np.float64) - origin
        pan = np.arctan2(towards[1], towards[0]) - base[2]
        tilt = np.arctan2(towards[2], np.hypot(towards[0], towards[1]))
    if not TILT_LIMITS[0] <= tilt <= TILT_LIMITS[1]:
        return None
    # The pan range is wider than half a turn on one side, so an angle out of
    # reach as measured may be in reach a turn away.
    for turn in (0.0, -2 * np.pi, 2 * np.pi):
        if PAN_LIMITS[0] <= pan + turn <= PAN_LIMITS[1]:
            return pan + turn, tilt
    return None


def standable(tree, x, y, radius=BASE_RADIUS):
    """Whether the base fits here, as far as the map has seen.

    Only occupied cells block. Unknown space is passable, which is the same
    optimism view_gain takes towards seeing through it, and the reason this is
    not a substitute for Nav2's costmap once the robot drives any distance.
    """
    resolution = tree.getResolution()
    # On the map's own grid, as sphere_cells does. A point left on a cell
    # boundary lands in either neighbour depending on the last bit of the
    # arithmetic, and a column of cells can be missed entirely.
    corner = np.asarray(tree.keyToCoord(tree.coordToKey(
        np.array([x - radius, y - radius, KNEE_HEIGHT]))))
    across = np.arange(0, 2 * radius + resolution, resolution)
    up = np.arange(0, BASE_TO_TORSO - KNEE_HEIGHT, resolution)
    column = np.stack(np.meshgrid(corner[0] + across, corner[1] + across,
                                  corner[2] + up, indexing="ij"), axis=-1).reshape(-1, 3)
    inside = np.linalg.norm(column[:, :2] - [x, y], axis=1) <= radius
    return not np.any(tree.getLabels(column[inside]) == OCCUPIED)


def candidates(tree, target, standing_at, radii=(0.6, 0.9, 1.2), headings=8,
               lifts=(0.0, 0.35, 0.69)):
    """Every view worth scoring: from where the robot stands, and from a ring of
    places around the target it could drive to and still fit."""
    places = [tuple(standing_at)]
    for radius in radii:
        for heading in np.linspace(0, 2 * np.pi, headings, endpoint=False):
            places.append((target[0] + radius * np.cos(heading),
                           target[1] + radius * np.sin(heading),
                           heading + np.pi))
    found = []
    for place in places:
        if not standable(tree, place[0], place[1]):
            continue
        for lift in lifts:
            head = head_towards(place, lift, target)
            if head is not None:
                found.append(Candidate(place, lift, *head))
    return found


def best_view(tree, cells, info, options, standing_at, max_range=MAX_RANGE):
    """(the candidate worth the most, the bits it would uncover), or (None, 0).

    Bits are traded against driving, so a distant view has to be worth the trip:
    see DISTANCE_SCALE.
    """
    best, best_bits, best_worth = None, 0, -1.0
    for option in options:
        origin, rotation = camera_pose(*option)
        bits = view_gain(tree, origin, rotation, cells, info, max_range)
        distance = np.hypot(option.base[0] - standing_at[0],
                            option.base[1] - standing_at[1])
        worth = bits / (1 + distance / DISTANCE_SCALE)
        if worth > best_worth:
            best, best_bits, best_worth = option, bits, worth
    return best, best_bits
