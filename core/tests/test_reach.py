"""Pins the numbers reach.py produces against hsrb_analytic_ik's own geometry.

reach.py is a hand port of C++ that is not compiled into this stack, so
nothing else would notice if a constant were mistyped -- every answer would
just be quietly wrong by a few centimetres. These pin the values that fall out
of the port, so a bad edit fails here instead of in front of a shelf.

From core/:
    python3 -m pytest tests/test_reach.py -p no:anyio
    python3 -m tests.test_reach
"""

import math

import numpy as np

from navigation import reach, standoff

# Hand z pointing down: the usual pose for picking something off a surface.
TOP_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
# Hand z pointing along +x: reaching into a shelf rather than down onto it.
SIDE_ON = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])


def pose(rotation, x, y, z):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (x, y, z)
    return matrix


def test_reach_tops_out_just_under_half_a_metre():
    """The single number the whole standoff/reposition split turns on."""
    annulus = reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 0.75))
    assert math.isclose(annulus.radius_max, 0.4923, abs_tol=1e-4)
    assert math.isclose(annulus.radius_min, 0.1658, abs_tol=1e-4)


def test_a_standoff_pose_can_never_reach_what_it_is_looking_at():
    """Not a bug in either module, but the reason Part 3 exists at all: the
    ring standoff.py drives to is outside the ring reach.py requires, always.
    If these two ever do overlap the reposition stage becomes optional, which
    is a big enough change that it should break a test first."""
    annulus = reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 0.75))
    assert standoff.WORKING_DISTANCE > annulus.radius_max
    assert standoff.MIN_STANDOFF > annulus.radius_max


def test_top_down_grasps_have_a_ceiling():
    """Lift travel plus palm offset. Anything above this needs a side grasp,
    which is worth knowing before driving to a tall shelf."""
    assert reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 1.229)) is not None
    assert reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 1.2301)) is None


def test_a_side_grasp_reaches_higher_than_a_top_down_one():
    """The palm offset works with the arm instead of against it, so the same
    shelf that is out of reach from above is reachable from the front."""
    assert reach.base_annulus(pose(SIDE_ON, 2.0, 0.0, 1.2301)) is not None


def test_the_floor_is_reachable_from_above_but_not_from_the_side():
    """The palm offset is what makes floor picking work: reaching straight
    down puts the wrist centre 0.155 m above the hand, well inside the lift's
    range. Coming at the same height horizontally puts the wrist centre at the
    hand's own height, which is below anything the lift can hold."""
    assert reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 0.0)) is not None
    assert reach.base_annulus(pose(SIDE_ON, 2.0, 0.0, 0.03)) is None


def test_the_annulus_narrows_as_the_target_gets_high():
    """Near the ceiling the base has to be almost on top of the target, which
    is exactly where the costmap will refuse to put it. Better to find out
    from the ring than from a failed drive."""
    low = reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 0.75))
    high = reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 1.2))
    assert high.radius_max < low.radius_max - 0.15


def test_every_returned_pose_puts_the_wrist_centre_where_the_arm_can_be():
    """The yaw check, and the real content of the port.

    The arm flexes only in its own plane and mounts L42 off centre, so in the
    base frame the wrist centre is always at exactly (sqrt(r^2 - L42^2), L42).
    Any yaw error shows up here immediately.
    """
    parameters = reach.PARAMETERS[reach.DEFAULT_ROBOT]
    lateral = parameters["L42"]
    annulus = reach.base_annulus(pose(TOP_DOWN, 2.0, 0.5, 0.75))
    poses = reach.base_poses(annulus, robot_xy=(0.0, 0.0), count=24)
    assert len(poses) == 24

    for x, y, yaw in poses:
        dx, dy = annulus.center[0] - x, annulus.center[1] - y
        # the wrist centre, expressed in the base frame this pose defines
        in_base = (math.cos(-yaw) * dx - math.sin(-yaw) * dy,
                   math.sin(-yaw) * dx + math.cos(-yaw) * dy)
        radius = math.hypot(dx, dy)
        assert math.isclose(in_base[1], lateral, abs_tol=1e-9)
        assert math.isclose(in_base[0], math.sqrt(radius ** 2 - lateral ** 2), abs_tol=1e-9)


def test_poses_come_back_nearest_first():
    annulus = reach.base_annulus(pose(TOP_DOWN, 2.0, 0.0, 0.75))
    poses = reach.base_poses(annulus, robot_xy=(0.0, 0.0))
    travel = [np.hypot(x, y) for x, y, _ in poses]
    assert travel == sorted(travel)


def test_standing_in_the_band_needs_no_reposition():
    target = pose(TOP_DOWN, 2.0, 0.0, 0.75)
    assert reach.reachable_from(target, (2.0 - 0.33, 0.0))
    assert not reach.reachable_from(target, (2.0 - 0.8, 0.0))     # a standoff pose
    assert not reach.reachable_from(target, (2.0 - 0.05, 0.0))    # inside the ring


def test_hsrb_and_hsrc_are_not_interchangeable():
    """They differ by ~5 mm of lift mount and 15 mm of palm. Small enough to
    look like noise, big enough to miss a grasp -- so if someone defaults the
    wrong way this should say so."""
    target = pose(TOP_DOWN, 2.0, 0.0, 1.232)
    assert reach.base_annulus(target, "hsrb") is not None
    assert reach.base_annulus(target, "hsrc") is None
    assert reach.DEFAULT_ROBOT == "hsrc"


class FakeNavigator:
    """Nav2 stand-in. Drives to the first pose offered, but lands `error`
    metres short of it -- which is the whole reason move_into_reach iterates,
    since the real controller's tolerance is wider than the usable band."""

    def __init__(self, xy, error=0.0):
        self.xy, self.driven, self.error, self.drives = xy, None, error, 0

    def robot_xy(self):
        return self.xy

    def go_to_first_reachable(self, poses):
        """Lands `error` metres from the goal, in the worst direction -- Nav2
        guarantees only a radius, not a bearing."""
        if not poses:
            return None
        self.driven = poses[0]
        self.drives += 1
        dx, dy = self.driven[0] - self.xy[0], self.driven[1] - self.xy[1]
        span = math.hypot(dx, dy)
        away = (dx / span, dy / span) if span > 1e-9 else (1.0, 0.0)
        self.xy = (self.driven[0] - away[0] * self.error,
                   self.driven[1] - away[1] * self.error)
        return self.driven


def test_standing_at_a_standoff_triggers_a_second_drive():
    """Arriving at the furniture leaves the object out of reach, so this must
    drive rather than report success."""
    target = reach.hand_pose((2.0, 0.0, 0.75))
    navigator = FakeNavigator((2.0 - standoff.WORKING_DISTANCE, 0.0))
    assert reach.move_into_reach(navigator, target)
    assert navigator.driven is not None
    assert reach.reachable_from(target, navigator.robot_xy())


def test_the_controller_tolerance_must_be_tighter_than_the_band():
    """The constraint the whole reposition rests on, asserted rather than
    assumed. Aiming mid-band leaves half the band as error budget, so a
    controller that stops anywhere within `tol` of its goal reaches the band
    only when tol is under that half-width. nav2_params.yaml's stock 0.25 m is
    not, which is why it must be tightened for this stage."""
    annulus = reach.base_annulus(reach.hand_pose((2.0, 0.0, 0.75)))
    low, high = annulus.usable
    assert (high - low) / 2 < 0.25, "if this ever passes, stock nav2 tolerance is fine"

    target = reach.hand_pose((2.0, 0.0, 0.75))
    within = FakeNavigator((2.0 - standoff.WORKING_DISTANCE, 0.0), error=0.08)
    assert reach.move_into_reach(within, target)
    assert reach.reachable_from(target, within.robot_xy())


def test_giving_up_is_reported_rather_than_pretended():
    """A navigator whose error exceeds the band must come back False. Saying
    True here is how the robot ends up parked politely out of range while the
    grasp filter rejects all hundred candidates."""
    target = reach.hand_pose((2.0, 0.0, 0.75))
    navigator = FakeNavigator((2.0 - standoff.WORKING_DISTANCE, 0.0), error=0.9)
    assert not reach.move_into_reach(navigator, target)


def test_already_in_reach_does_not_drive():
    target = reach.hand_pose((2.0, 0.0, 0.75))
    navigator = FakeNavigator((2.0 - 0.33, 0.0))
    assert reach.move_into_reach(navigator, target)
    assert navigator.driven is None


def test_an_object_too_high_to_reach_is_not_driven_at():
    """No base position works, so this is a failure to report, not a drive to
    attempt -- the caller should try another grasp or give up."""
    navigator = FakeNavigator((0.0, 0.0))
    assert not reach.move_into_reach(navigator, reach.hand_pose((2.0, 0.0, 1.5)))
    assert navigator.driven is None


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"{name} ok")
