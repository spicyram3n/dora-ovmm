import glob
import os
import sys

from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# One directory per target, written by core/run_pipeline.py.
TARGETS_DIR = "/home/ws/config/targets"

sys.path.append(os.path.join(get_package_share_directory("hsrb_moveit_config"), "launch"))
import robot_description  # noqa: E402
from utils import load_file, load_yaml  # noqa: E402

sys.path.append("/home/ws/core")
from grasping import grasp_io  # noqa: E402


def _resolve_grasp_file(grasp_file, target):
    """An explicit path wins, then target:=<name>, then the most recently
    written target."""
    if grasp_file:
        return grasp_file
    if target:
        return os.path.join(TARGETS_DIR, target, "grasps.yaml")
    files = sorted(glob.glob(os.path.join(TARGETS_DIR, "*", "grasps.yaml")),
                   key=os.path.getmtime)
    if not files:
        raise RuntimeError(f"no grasps.yaml under {TARGETS_DIR} -- run core/run_pipeline.py first")
    return files[-1]


def launch_setup(context, grasp_file, target, group_name, description_package, description_file):
    grasp_file_str = _resolve_grasp_file(context.perform_substitution(grasp_file),
                                         context.perform_substitution(target))
    # grasp_io owns the schema. Parsing it here as well is how this file came to
    # read grasp["header"]["frame_id"] for a writer that emits frame_id.
    poses, scores, meta = grasp_io.load_grasps(grasp_file_str)
    quaternions = Rotation.from_matrix(poses[:, :3, :3]).as_quat()

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
                "frame_id": meta["frame_id"],
                "group_name": context.perform_substitution(group_name),
                "xs": poses[:, 0, 3].tolist(),
                "ys": poses[:, 1, 3].tolist(),
                "zs": poses[:, 2, 3].tolist(),
                "qxs": quaternions[:, 0].tolist(),
                "qys": quaternions[:, 1].tolist(),
                "qzs": quaternions[:, 2].tolist(),
                "qws": quaternions[:, 3].tolist(),
                "scores": scores.tolist(),
            },
        ])

    print(f"[move_to_grasp.launch] using {grasp_file_str} ({len(poses)} candidate grasps)")
    return [node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("grasp_file", default_value="",
                              description="Explicit path to a grasps.yaml. Overrides target."),
        DeclareLaunchArgument("target", default_value="",
                              description=f"Target name under {TARGETS_DIR}, e.g. pringles_can. "
                                          "Defaults to the most recently written target."),
        DeclareLaunchArgument("group_name", default_value="whole_body",
                              description="MoveIt group to plan with."),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("description_package", default_value="hsrc_description"),
        DeclareLaunchArgument("description_file", default_value="hsrc1s.urdf.xacro"),
        DeclareLaunchArgument("robot_name", default_value="hsrc", choices=["hsrb", "hsrc"]),
    ]
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup,
                       args=[LaunchConfiguration("grasp_file"), LaunchConfiguration("target"),
                             LaunchConfiguration("group_name"),
                             LaunchConfiguration("description_package"),
                             LaunchConfiguration("description_file")])])
