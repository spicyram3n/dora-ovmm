"""The HSRC planning model: Toyota's URDF with its fake degrees of freedom removed.

hsrb_moveit_config's own robot_description.parse() adds the odom_x/odom_y/odom_t
virtual base joints the whole_body group plans in. This wraps it rather than
copying it, so upstream keeps owning that part. Named planning_model rather
than robot_description so it cannot shadow the module it imports.
"""

import os
import sys
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory

MOVEIT_CONFIG = get_package_share_directory("hsrb_moveit_config")
REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(REPOSITORY, "config")

sys.path.append(os.path.join(MOVEIT_CONFIG, "launch"))
import robot_description as vendor  # noqa: E402

# joint_state_publisher is launched with this path, so it has to be rewritten
# after the model changes; upstream parse() wrote the unfixed model there.
MODEL_PATH = "/tmp/robot_description.urdf"

# The finger springs are passive (SRDF), but the gripper group carries their
# links, so their bounds are still checked. The URDF allows 0 to 0.698 rad;
# the real hand closed on nothing reports 0.79 to 0.86 (2026-09-16), and the
# start state then fails bounds by more than the adapter tolerates, so every
# pick died at "open hand before pregrasp: START_STATE_INVALID".
PASSIVE_SPRING_JOINT_SUFFIX = "_spring_proximal_joint"
PASSIVE_SPRING_RANGE = (-0.2, 1.2)


def ours(name):
    """One of the settings this project owns, named relative to config/.

    Simulation files sit in config/moveit/; the real robot's are under
    config/realrobot/moveit/, so the caller passes the subdirectory too.
    """
    with open(os.path.join(CONFIG_DIR, name)) as handle:
        return yaml.safe_load(handle)


def theirs(name):
    """One of hsrb_moveit_config's own settings, read rather than imported:
    its launch/utils.py would have to join sys.path, where it collides with
    core/utils."""
    with open(os.path.join(MOVEIT_CONFIG, "config", name)) as handle:
        return handle.read()


def moveit_params():
    """Robot model, IK, joint limits and OMPL settings. move_group and the
    MTC pick node both load these, so they plan with the same model."""
    urdf = parse("hsrc_description", "hsrc1s.urdf.xacro")
    # hsrb_moveit_config keeps its OMPL pipeline under the move_group namespace.
    ompl = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": " ".join(
            [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]
        ),
        "start_state_max_bounds_error": 0.1,
        **ours("moveit/ompl_planning.yaml"),
    }
    return {
        "robot_description": urdf,
        "robot_description_semantic": semantic(urdf, theirs("hsrb.srdf")),
        "robot_description_planning": yaml.safe_load(theirs("joint_limits.yaml")),
        **ours("moveit/kinematics.yaml"),
        "move_group": ompl,
    }


def freeze_zero_travel_joints(root):
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if joint.get("type") not in ("revolute", "prismatic") or limit is None:
            continue
        lower, upper = (limit.get("lower"), limit.get("upper"))
        # Gazebo declares the wrist force sensor as a revolute joint with zero
        # travel. It is mechanically fixed and must not become a ninth planning
        # DOF: the HSR analytic solver returns exactly five arm + three base
        # values, and a ninth variable makes every IK response unusable.
        if lower is None or upper is None or float(lower) or float(upper):
            continue
        joint.set("type", "fixed")
        joint.remove(limit)
        axis = joint.find("axis")
        if axis is not None:
            joint.remove(axis)


def widen_passive_springs(root):
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if not joint.get("name", "").endswith(PASSIVE_SPRING_JOINT_SUFFIX) or limit is None:
            continue
        limit.set("lower", str(PASSIVE_SPRING_RANGE[0]))
        limit.set("upper", str(PASSIVE_SPRING_RANGE[1]))


def parse(description_package, description_file):
    """Return the planning URDF as a string, and rewrite MODEL_PATH."""
    root = ET.fromstring(vendor.parse(description_package, description_file))
    freeze_zero_travel_joints(root)
    widen_passive_springs(root)
    ET.ElementTree(root).write(MODEL_PATH)
    return ET.tostring(root, encoding="unicode", method="xml")


def semantic(urdf, srdf):
    """Match the SRDF robot name and named states to the planning URDF."""
    robot = ET.fromstring(urdf)
    name = robot.get("name")
    if not name:
        raise ValueError("Planning URDF must specify a robot name")
    movable = {
        joint.get("name")
        for joint in robot.findall("joint")
        if joint.get("type") not in (None, "fixed")
    }
    root = ET.fromstring(srdf)
    root.set("name", name)
    for state in root.findall("group_state"):
        for joint in state.findall("joint"):
            if joint.get("name") not in movable:
                state.remove(joint)
    return ET.tostring(root, encoding="unicode", method="xml")
