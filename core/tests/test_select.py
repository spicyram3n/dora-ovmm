"""What survives the filter between GraspGenX and a planner.

From core/:
    python3 -m pytest tests/test_select.py -p no:anyio
    python3 -m tests.test_select
"""

import numpy as np

from grasping import select
from navigation import reach

CENTRE = np.array([0.5, 0.0, 0.80])


def cylinder(radius=0.0375, half_height=0.10, n=3000, centre=CENTRE, seed=0):
    """A can standing upright, as a point cloud."""
    rng = np.random.default_rng(seed)
    angle = rng.uniform(0, 2 * np.pi, n)
    return centre + np.column_stack([radius * np.cos(angle), radius * np.sin(angle),
                                     rng.uniform(-half_height, half_height, n)])


def slab(half_width, half_depth=0.02, n=3000, centre=CENTRE, seed=1):
    rng = np.random.default_rng(seed)
    return centre + np.column_stack([rng.uniform(-half_width, half_width, n),
                                     rng.uniform(-half_depth, half_depth, n),
                                     rng.uniform(-0.10, 0.10, n)])


def top_down(point, yaw, depth=0.065):
    """Approach straight down, fingers closing along `yaw`."""
    approach = np.array([0.0, 0.0, -1.0])
    close = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = np.cross(close, approach), close, approach
    pose[:3, 3] = point + np.array([0.0, 0.0, depth])
    return pose


def test_pregrasp_backs_off_along_the_grasps_own_axis():
    """Not along world z. A side grasp must back off horizontally, or the
    standoff drives the gripper into the shelf above instead of away from it."""
    side = np.eye(4)
    side[:3, :3] = reach.SIDE_ON
    side[:3, 3] = (1.0, 0.0, 0.9)
    backed = select.pregrasp(side, 0.12)
    assert np.allclose(backed[:3, 3], (1.0 - 0.12, 0.0, 0.9))
    assert np.allclose(backed[:3, :3], side[:3, :3])   # orientation untouched


def test_a_can_fits_the_jaw_and_a_wide_box_does_not():
    can = cylinder()
    assert select.jaw_span(can, top_down(CENTRE, 0.0)) < select.JAW_OPEN
    assert select.fits(can, top_down(CENTRE, 0.0))

    box = slab(half_width=0.125)          # 25 cm across, wider than the 12.9 cm jaw
    assert not select.fits(box, top_down(CENTRE, 0.0))
    # ... but the same box is graspable across its thin axis
    assert select.fits(box, top_down(CENTRE, np.pi / 2))


def test_a_grasp_closing_on_empty_space_is_rejected():
    """Nothing between the fingers means an infinite span, not a zero one."""
    can = cylinder()
    away = top_down(CENTRE + np.array([1.0, 0.0, 0.0]), 0.0)
    assert select.jaw_span(can, away) == np.inf
    assert not select.fits(can, away)


def test_reachability_depends_on_where_the_robot_actually_stands():
    """The whole point of the filter: the same candidates, different base."""
    can = cylinder()
    poses = np.stack([top_down(CENTRE, y) for y in np.linspace(0, np.pi, 12)])
    scores = np.linspace(0.5, 0.95, len(poses))

    annulus = reach.base_annulus(select.pregrasp(poses[0]))
    inside = (CENTRE[0] - 0.35, 0.0)
    assert select.feasible(poses, scores, can, inside)
    # standing on top of the target is too close, not too far
    assert not select.feasible(poses, scores, can, (CENTRE[0], 0.0))
    assert not select.feasible(poses, scores, can, (CENTRE[0] + 2.5, 0.0))
    assert annulus.usable[0] > 0.0


def test_candidates_come_back_best_score_first():
    can = cylinder()
    poses = np.stack([top_down(CENTRE, y) for y in np.linspace(0, np.pi, 12)])
    scores = np.linspace(0.5, 0.95, len(poses))
    keep = select.feasible(poses, scores, can, (CENTRE[0] - 0.35, 0.0))
    assert [scores[i] for i, _ in keep] == sorted((scores[i] for i, _ in keep), reverse=True)


def test_a_reachable_grasp_is_not_thrown_away_for_an_unreachable_pregrasp():
    """The pregrasp sits further out along the approach axis than the grasp, so
    for a target near the reach ceiling it can leave the envelope while the
    grasp stays inside it. Shortening the standoff must rescue that, not
    reject the candidate."""
    high = CENTRE.copy()
    high[2] = 1.05                       # high enough that 0.12 m fails, 0.08 does not
    can = cylinder(centre=high)
    poses = np.stack([top_down(high, y) for y in np.linspace(0, np.pi, 12)])
    scores = np.linspace(0.5, 0.95, len(poses))
    assert reach.base_annulus(select.pregrasp(poses[0], 0.12)) is None, \
        "this fixture is only meaningful while the long standoff is unreachable"
    # stand mid-band of the shortest standoff that does work
    band = reach.base_annulus(select.pregrasp(poses[0], select.STANDOFFS[-1])).usable
    base = (high[0] - (band[0] + band[1]) / 2, 0.0)
    kept = select.feasible(poses, scores, can, base)
    assert kept, "a reachable grasp must survive even if the 0.12 m pregrasp does not"
    assert all(s < 0.12 for _, s in kept)


def test_report_separates_the_two_reasons_for_failure():
    """'Nothing was reachable' and 'nothing fit the gripper' need different
    fixes, so they must not collapse into one number."""
    poses = np.stack([top_down(CENTRE, y) for y in np.linspace(0, np.pi, 6)])
    scores = np.linspace(0.5, 0.9, len(poses))

    # a 30 cm drum: too wide whichever way the fingers are turned
    wide = select.report(poses, scores, cylinder(radius=0.15), (CENTRE[0] - 0.35, 0.0))
    assert wide["too_wide"] == wide["total"] and wide["kept"] == 0

    far = select.report(poses, scores, cylinder(), (CENTRE[0] + 2.5, 0.0))
    assert far["out_of_reach"] == far["total"] and far["too_wide"] == 0


if __name__ == "__main__":
    for name, function in sorted(globals().items()):
        if name.startswith("test_"):
            function()
            print(f"{name} ok")
