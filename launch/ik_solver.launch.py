"""Bring up Toyota's collision-aware IK solver for the HSRC.

The node answers "where can the base stand so the hand reaches this pose",
using hsrb_analytic_ik's closed-form base-position range plus a collision
check per candidate. Much cheaper than asking MoveIt to plan and finding out.

hsrb_ik_solver_node ships an example.launch.py, but it hardcodes the HSR-B
plugin and cannot be overridden from the command line -- the B and C arm
geometries genuinely differ, so this launches the C one instead.

    ros2 launch /home/ws/launch/ik_solver.launch.py

Then call /ik_solver_node/solve_ik_with_collision. See core/navigation/base_placement.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tmc_launch_ros_utils.tmc_launch_ros_utils import (
    load_collision_description,
    load_robot_description,
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package", default_value="hsrc_description"),
        DeclareLaunchArgument("description_file", default_value="hsrc1s.urdf.xacro"),
        DeclareLaunchArgument(
            "collision_file", default_value="collision_pair_hsrc.xml"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"]
        ),
    ]

    ik_solver = Node(
        package="tmc_ik_solver_node",
        executable="ik_solver_node",
        output="screen",
        parameters=[
            load_robot_description(),
            load_collision_description(),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ik_plugin_type": "hsrb_ik_solver_node::HsrcIkSolverPluginRobustToBasePositionError",
                "map_convolution_type": "tmc_ik_solver_node::EuclideanDistanceMapConvolution",
                # explained under "IK solver settings" in core/navigation/README.md
                "convolution": {
                    "grid_distance_threhsold": 1.5,  # cells;
                    "ik_base_resolution": 0.05,  # metres between candidates
                },
            },
        ],
    )

    return LaunchDescription(declared_arguments + [ik_solver])
