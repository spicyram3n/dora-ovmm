# Reads a grasp YAML saved by core/run_pipeline.py (see
# core/grasping/grasp_io.py for the format) and sends the pose to the
# move_to_grasp node as parameters.
#
# MoveGroupInterface needs robot_description / robot_description_semantic /
# kinematics parameters to build its robot model, same as any other MoveIt
# node. Rather than duplicate that parsing, this reuses hsrb_moveit_config's
# own launch helpers (read-only import, same pattern its own
# hsrc_example.launch.py uses). See robot_description.py and utils.py
# there for what these calls actually do.
import glob
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GRASPS_DIR = "/home/ws/config/grasps"

sys.path.append(os.path.join(get_package_share_directory("hsrb_moveit_config"), "launch"))
import robot_description  # noqa: E402
from utils import load_file, load_yaml  # noqa: E402


def _newest_grasp_file():
    files = sorted(glob.glob(os.path.join(GRASPS_DIR, "*.yaml")), key=os.path.getmtime)
    return files[-1] if files else ""


def launch_setup(context, grasp_file, group_name, description_package, description_file):
    grasp_file_str = context.perform_substitution(grasp_file) or _newest_grasp_file()
    with open(grasp_file_str) as f:
        grasp = yaml.safe_load(f)
    grasps = grasp["grasps"]

    robot_description_moveit = robot_description.parse(
        context.perform_substitution(description_package), context.perform_substitution(description_file))

    node = Node(
        package="grasp_execution",
        executable="move_to_grasp",
        output="screen",
        parameters=[
            {"robot_description": robot_description_moveit},
            {"robot_description_semantic": load_file("config/hsrb.srdf")},
            load_yaml("config/kinematics.yaml"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_name": LaunchConfiguration("robot_name"),
                "frame_id": grasp["header"]["frame_id"],
                "group_name": context.perform_substitution(group_name),
                "xs": [g["pose"]["position"]["x"] for g in grasps],
                "ys": [g["pose"]["position"]["y"] for g in grasps],
                "zs": [g["pose"]["position"]["z"] for g in grasps],
                "qxs": [g["pose"]["orientation"]["x"] for g in grasps],
                "qys": [g["pose"]["orientation"]["y"] for g in grasps],
                "qzs": [g["pose"]["orientation"]["z"] for g in grasps],
                "qws": [g["pose"]["orientation"]["w"] for g in grasps],
                "scores": [g["score"] for g in grasps],
            },
        ])

    print(f"[move_to_grasp.launch] using {grasp_file_str} ({len(grasps)} candidate grasps)")
    return [node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("grasp_file", default_value="",
                              description=f"Grasp YAML to load. Defaults to the newest file in {GRASPS_DIR}."),
        DeclareLaunchArgument("group_name", default_value="whole_body",
                              description="MoveIt group to plan with."),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("description_package", default_value="hsrc_description"),
        DeclareLaunchArgument("description_file", default_value="hsrc1s.urdf.xacro"),
        DeclareLaunchArgument("robot_name", default_value="hsrc", choices=["hsrb", "hsrc"]),
    ]
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration("grasp_file"), LaunchConfiguration("group_name"),
                             LaunchConfiguration("description_package"), LaunchConfiguration("description_file")])])
