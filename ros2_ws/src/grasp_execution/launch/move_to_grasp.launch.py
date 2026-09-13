import glob
import importlib.util
import json
import os
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from grasp_frames import hand_pose

TARGETS_DIR = "/home/ws/config/targets"
GRIPPER_DIR = "/home/ws/docker/graspgenx/x_grippers/hsrc_hand"
# The fitted radius describes the object's surface. Octomap cells are 1 cm and
# the fit has its own residual, so an intended fingertip contact can register
# just outside the ideal cylinder; widen the "this contact is the target"
# test by the same margin the earlier aligned runs used.
TARGET_CONTACT_MARGIN = 0.008


def _load_module(name, path):
    """Load a single project module by path.

    Not sys.path: /home/ws/core has its own utils package, and putting it on
    the path shadows the hsrb_moveit_config launch utils imported below.
    """
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


align_side_grasp = _load_module(
    "can_alignment", "/home/ws/core/grasping/can_alignment.py").align_side_grasp

sys.path.append(os.path.join(get_package_share_directory("hsrb_moveit_config"), "launch"))
import robot_description  # noqa: E402
from utils import load_file, load_yaml  # noqa: E402


def _pad_aligned(grasps, geometry):
    """Shift each candidate so its pads close on the fitted object axis.

    GraspGenX places hand_palm_link at the target, but the rubber pads sit a
    few centimetres further along the approach, so an uncorrected candidate
    closes short of the axis: measured 2.3 cm short on the run that toppled
    the can. Runs on canonical GraspGenX poses, before hand_pose() maps them
    into URDF coordinates, because the closing profile's pad centres were
    measured in that same canonical frame.

    Candidates that cannot be centred on this target - top-down grasps, or
    ones needing an implausible correction - are dropped rather than executed:
    they cannot enclose the object, and driving one into it is how the can got
    knocked over before.
    """
    with open(os.path.join(GRIPPER_DIR, "closing_profile.json")) as f:
        profile = json.load(f)
    center = (geometry["center_x"], geometry["center_y"])
    aligned = []
    for grasp in grasps:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat(
            [grasp["orientation"][axis] for axis in "xyzw"]).as_matrix()
        pose[:3, 3] = [grasp["position"][axis] for axis in "xyz"]
        try:
            corrected, _, _ = align_side_grasp(pose, center, geometry["radius"], profile, level=True)
        except ValueError:
            continue
        aligned.append({**grasp,
                        "position": dict(zip("xyz", map(float, corrected[:3, 3]))),
                        "orientation": dict(zip("xyzw", map(float,
                            Rotation.from_matrix(corrected[:3, :3]).as_quat())))})
    return aligned


def _newest_grasp_file():
    files = sorted(glob.glob(os.path.join(TARGETS_DIR, "*", "grasps.yaml")), key=os.path.getmtime)
    return files[-1] if files else ""


def launch_setup(context, grasp_file, group_name, description_package, description_file):
    grasp_file_str = context.perform_substitution(grasp_file) or _newest_grasp_file()
    with open(grasp_file_str) as f:
        grasp = yaml.safe_load(f)
    grasps = grasp["grasps"]
    if grasp["gripper"] != "hsrc_hand":
        raise ValueError("This executor requires the registered hsrc_hand gripper")
    with open("/home/ws/docker/graspgenx/x_grippers/hsrc_hand/config.json") as f:
        registration = json.load(f)
    geometry = grasp.get("target_geometry", {})
    # Perception only fits a target_geometry for a shape the pad alignment
    # understands; anything else keeps every candidate exactly as generated.
    if geometry:
        generated = len(grasps)
        grasps = _pad_aligned(grasps, geometry)
        print(f"[move_to_grasp.launch] pad-aligned {len(grasps)} of {generated} "
              f"candidates onto the fitted target axis")
        if not grasps:
            raise ValueError("no candidate could be pad-aligned onto the fitted target")
    # Saved YAML and plots stay in GraspGenX's canonical frame. Convert
    # exactly once, at the boundary to MoveIt's hand_palm_link target.
    grasps = [hand_pose(g, registration["base_rotation"]) for g in grasps]

    robot_description_moveit = robot_description.parse(
        context.perform_substitution(description_package), context.perform_substitution(description_file))

    node = Node(
        package="grasp_execution",
        executable="move_to_grasp",
        output="screen",
        remappings=[("joint_states", "/whole_body_moveit/joint_states")],
        parameters=[
            {"robot_description": robot_description_moveit},
            {"robot_description_semantic": load_file("config/hsrb.srdf")},
            load_yaml("config/kinematics.yaml"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_name": LaunchConfiguration("robot_name"),
                "frame_id": grasp["frame_id"],
                "group_name": context.perform_substitution(group_name),
                "xs": [g["position"]["x"] for g in grasps],
                "ys": [g["position"]["y"] for g in grasps],
                "zs": [g["position"]["z"] for g in grasps],
                "qxs": [g["orientation"]["x"] for g in grasps],
                "qys": [g["orientation"]["y"] for g in grasps],
                "qzs": [g["orientation"]["z"] for g in grasps],
                "qws": [g["orientation"]["w"] for g in grasps],
                "scores": [g["score"] for g in grasps],
                "target_center_x": float(geometry.get("center_x", 0.0)),
                "target_center_y": float(geometry.get("center_y", 0.0)),
                "target_min_z": float(geometry.get("min_z", -10.0)),
                "target_max_z": float(geometry.get("max_z", 10.0)),
                "target_radius": (float(geometry["radius"]) + TARGET_CONTACT_MARGIN
                                  if geometry else 0.0),
                "allow_preview_failure": LaunchConfiguration("allow_preview_failure"),
                "stop_before_close": LaunchConfiguration("stop_before_close"),
                "exclude_target_after_pregrasp": LaunchConfiguration("exclude_target_after_pregrasp"),
                "target_shape_radius": float(geometry.get("radius", 0.0)),
            },
        ])

    print(f"[move_to_grasp.launch] using {grasp_file_str} ({len(grasps)} candidate grasps)")
    return [node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("grasp_file", default_value="",
                              description=f"Grasp YAML to load. Defaults to the newest grasps.yaml under {TARGETS_DIR}."),
        DeclareLaunchArgument("group_name", default_value="whole_body",
                              description="MoveIt group to plan with."),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("description_package", default_value="hsrc_description"),
        DeclareLaunchArgument("description_file", default_value="hsrc1s.urdf.xacro"),
        DeclareLaunchArgument("robot_name", default_value="hsrc", choices=["hsrb", "hsrc"]),
        DeclareLaunchArgument("allow_preview_failure", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("exclude_target_after_pregrasp", default_value="true", choices=["true", "false"],
                              description="After pregrasp, model the fitted can separately from the RGB-D octomap."),
        DeclareLaunchArgument("stop_before_close", default_value="false", choices=["true", "false"],
                              description="Park at the validated contact pose with the hand open, for measurement."),
    ]
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration("grasp_file"), LaunchConfiguration("group_name"),
                             LaunchConfiguration("description_package"), LaunchConfiguration("description_file")])])
