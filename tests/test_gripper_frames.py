"""Frame-convention and gripper-kinematics regression tests.

These exist because two things here are invisible in a pose dump and wrong on
the robot: a quarter turn about the approach axis (the gripper closes across
the object instead of around it), and trusting config.json's stored angles for
joints the URDF drives through <mimic> (the hand appears to open twice as wide
as it can). Both were shipped bugs.

Everything runs offline -- no ROS, no GraspGenX, no GPU -- against the URDF and
gripper description already in the repo.

    python3 -m pytest
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "core"))

from grasping.gripper_frame import (  # noqa: E402
    GRIPPERS_DIR,
    canonical_from_palm,
    palm_from_canonical,
)
from grasping.visualization.animate_grasp import (  # noqa: E402
    forward_kinematics,
    load_gripper,
    resolve_mimics,
)

GRIPPER = "hsrc_hand"

# Fingertip separation from forward kinematics with the URDF's mimic linkage
# honoured. Not from config.json, whose stored values for the driven joints
# contradict the URDF and open the hand to 0.254 m -- twice its real span.
JAW_OPEN, JAW_CLOSED = 0.1278, 0.0015


@pytest.fixture(scope="module")
def config():
    return json.loads((GRIPPERS_DIR / GRIPPER / "config.json").read_text())


@pytest.fixture(scope="module")
def gripper():
    return load_gripper(GRIPPER)


def _fingertips(joints, values):
    transforms = forward_kinematics(joints, values)
    return (transforms["hand_l_finger_tip_frame"][:3, 3],
            transforms["hand_r_finger_tip_frame"][:3, 3])


# --- the palm <-> canonical bridge -----------------------------------------

def test_canonical_from_palm_is_a_rotation():
    matrix = canonical_from_palm(GRIPPER)
    rotation = matrix[:3, :3]
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(rotation), 1.0)
    # A pure rotation about the shared approach axis: no translation. If this
    # ever fails the two frames differ by an offset as well, and every
    # conversion in the pipeline is incomplete.
    np.testing.assert_allclose(matrix[:3, 3], 0.0, atol=1e-9)


def test_canonical_from_palm_maps_closing_axis():
    """The HSR closes along palm Y; GraspGenX closes along canonical X."""
    rotation = canonical_from_palm(GRIPPER)[:3, :3]
    np.testing.assert_allclose(rotation @ [0, 1, 0], [1, 0, 0], atol=1e-9)
    # The approach axis is shared, so Z must survive untouched.
    np.testing.assert_allclose(rotation @ [0, 0, 1], [0, 0, 1], atol=1e-9)


def test_inverse_round_trips():
    np.testing.assert_allclose(
        palm_from_canonical(GRIPPER) @ canonical_from_palm(GRIPPER), np.eye(4), atol=1e-9)


# --- the mimic linkage -----------------------------------------------------

def test_config_disagrees_with_the_urdf_on_driven_joints(gripper, config):
    """Pins the discovery, so nobody 'fixes' this by trusting config.json
    again. The stored angles for the driven joints are not merely redundant,
    they are wrong -- hand_l_distal_joint is stored as +0.798 where the URDF
    says -1.0 * hand_motor_joint - 0.087 = -1.327."""
    _, joints = gripper
    resolved = resolve_mimics(joints, config["open"])
    driven = {name: mimic for name, _, _, _, _, mimic in joints if mimic}
    assert driven, "gripper.urdf should declare mimic joints"

    disagreements = [name for name in driven
                     if not np.isclose(resolved[name], config["open"][name], atol=1e-6)]
    assert "hand_l_distal_joint" in disagreements
    assert "hand_l_mimic_distal_joint" in disagreements


def test_mimics_are_driven_from_their_source(gripper, config):
    _, joints = gripper
    resolved = resolve_mimics(joints, config["open"])
    for name, _, _, _, _, mimic in joints:
        if mimic:
            source, multiplier, offset = mimic
            assert resolved[name] == pytest.approx(multiplier * resolved[source] + offset)


# --- the gripper's own geometry, straight from gripper.urdf ----------------

def test_fingers_separate_along_palm_y(gripper, config):
    """Derived from the URDF joint chain alone, independent of config.json's
    base_rotation -- real corroboration of the frame convention, not a
    restatement of it."""
    _, joints = gripper
    left, right = _fingertips(joints, config["open"])

    separation = np.abs(left - right)
    assert separation[1] > 0.10, "open fingertips should be far apart along Y"
    assert separation[0] < 0.005 and separation[2] < 0.005, "and only along Y"


def test_closing_narrows_the_jaws(gripper, config):
    _, joints = gripper
    gap = {state: float(np.linalg.norm(np.subtract(*_fingertips(joints, config[state]))))
           for state in ("open", "close")}

    assert gap["open"] == pytest.approx(JAW_OPEN, abs=0.005)
    assert gap["close"] == pytest.approx(JAW_CLOSED, abs=0.005)
    # The bug this guards: ignoring <mimic> gives 0.254 m open, 0.042 closed.
    assert gap["open"] < 0.20


def test_closed_fingertips_reach_past_the_palm(gripper, config):
    """Closed tips sit ~0.092 m along the approach axis, which is where an
    object has to be for the fingers to enclose it. A grasp pose whose object
    is not near this depth closes on air."""
    _, joints = gripper
    left, right = _fingertips(joints, config["close"])
    assert left[2] == pytest.approx(0.092, abs=0.005)
    assert right[2] == pytest.approx(0.092, abs=0.005)


# --- the shipped mesh ------------------------------------------------------

def test_vis_mesh_is_in_the_canonical_frame():
    """The viewers apply palm_from_canonical to vis_mesh.obj, which is only
    correct while the mesh ships canonical: body behind the origin along -Z,
    widest across X. A re-export that changes either breaks the views
    silently."""
    import trimesh

    vertices = np.asarray(trimesh.load(GRIPPERS_DIR / GRIPPER / "vis_mesh.obj",
                                       force="mesh").vertices)
    assert vertices[:, 2].max() <= 1e-3, "gripper body should sit behind the grasp origin"
    extents = vertices.ptp(axis=0)
    assert extents[0] > extents[1], "the export's widest axis is canonical X"


# --- the reach band used to discard grasps ---------------------------------

def test_wrist_origin_sits_behind_the_palm():
    """A top-down grasp puts the wrist above the palm, never below -- the sign
    that makes the height filter a filter and not a coin flip."""
    from grasping.visualization.view_target import WRIST_OFFSET, _wrist_z

    top_down = np.eye(4)
    top_down[:3, :3] = np.diag([1.0, -1.0, -1.0])  # local +Z pointing at the floor
    top_down[:3, 3] = [0.0, 0.0, 1.0]

    assert _wrist_z(top_down[None]) == pytest.approx(1.0 + WRIST_OFFSET)
