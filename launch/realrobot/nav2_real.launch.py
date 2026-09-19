#!/usr/bin/env python3
"""Nav2 against the real HSR, using the robot's own localization.

The robot already runs Toyota's stack: pose_integrator publishes map->odom at
100 Hz (laser_2d_localizer only feeds it a pose, and stopping the localizer
leaves the transform running off the last correction), and the saved lab map is
in that same map frame.
So this launch brings up map_server and the four Nav2 servers, and no AMCL --
a second map->odom publisher would fight the robot's.

    ros2 launch /home/ws/launch/realrobot/nav2_real.launch.py

Arguments: map, params_file, cmd_vel_topic, autostart.

Localize before the first goal. The robot's localizer is seeded on
/laser_2d_correct_pose, which RViz knows nothing about, so this launch relays
/initialpose onto it: RViz's "2D Pose Estimate" then works as usual. Unseeded,
laser_2d_localizer reports the map origin and every plan aborts with "start or
goal pose is an obstacle".

Set localization:=true only after stopping the robot's own localization; with
both running the TF tree has two publishers for map->odom and the base jumps.

Never send a goal to Toyota's /move_base while this is up: its base_path_follower
publishes to /base_velocity as well, and the two would fight for the base.

/base_velocity is only the right output while Toyota's navigation.py runs, since
safety_velocity_limiter and velocity_switcher relay it to the wheels and stop
with that launch. Without them the topic has no subscriber and the base never
moves, with no error anywhere. Then:
    cmd_vel_topic:=/omni_base_controller/cmd_vel
which reaches the wheels directly, at the cost of the bumper stop, the obstacle
slowdown and teleop's override (confirmed on hardware 2026-09-19).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from core.utils.recording import Paths  # noqa: E402
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAP = str(Paths().map)
PARAMS = "/home/ws/config/realrobot/nav2/nav2_params_real.yaml"
SCAN_FILTER = "/home/ws/config/realrobot/nav2/scan_filter.yaml"

# The safety limiter's input, not the controller's output. Keeps
# safety_velocity_limiter (obstacle slowdown, bumpers) and velocity_switcher
# (teleop override) between Nav2 and the wheels -- both of which live in
# Toyota's navigation.py, so pass cmd_vel_topic:=/omni_base_controller/cmd_vel
# when that launch is down or nothing subscribes to this.
CMD_VEL = "/base_velocity"

# Toyota's laser_2d_localizer listens here, not on /initialpose.
CORRECT_POSE = "/laser_2d_correct_pose"


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    cmd_vel = LaunchConfiguration("cmd_vel_topic")
    use_amcl = IfCondition(LaunchConfiguration("localization"))
    no_amcl = UnlessCondition(LaunchConfiguration("localization"))

    args = [
        DeclareLaunchArgument("map", default_value=MAP),
        DeclareLaunchArgument("params_file", default_value=PARAMS),
        DeclareLaunchArgument("cmd_vel_topic", default_value=CMD_VEL),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="true starts AMCL. Only with the robot's own "
            "localization stopped.",
        ),
    ]

    common = {"parameters": [params, {"use_sim_time": False}], "output": "screen"}
    drive = [("cmd_vel", cmd_vel)]

    # RViz publishes "2D Pose Estimate" on /initialpose. AMCL reads that topic;
    # the robot's own localizer reads /laser_2d_correct_pose, so relay it there.
    # Same message type, so the relay is a straight copy.
    initial_pose_relay = Node(
        package="topic_tools", executable="relay", name="initialpose_relay",
        condition=no_amcl, output="screen",
        parameters=[{"input_topic": "/initialpose", "output_topic": CORRECT_POSE,
                     "use_sim_time": False}])

    # The robot boots on laser scan-matching odometry, which loses metres along a
    # corridor; see use_wheel_odom.py. Every launch, because a restart of the robot's
    # own stack silently puts it back.
    wheel_odom = ExecuteProcess(
        cmd=["python3", "/home/ws/realrobot/live/use_wheel_odom.py"], output="screen")

    # Both costmaps read /scan_filtered; see scan_filter.yaml for why.
    scan_filter = Node(
        package="laser_filters", executable="scan_to_scan_filter_chain", name="scan_filter",
        parameters=[SCAN_FILTER, {"use_sim_time": False}], output="screen",
        remappings=[("scan", "/scan"), ("scan_filtered", "/scan_filtered")])

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            params,
            {"use_sim_time": False, "yaml_filename": LaunchConfiguration("map")},
        ],
        output="screen",
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        condition=use_amcl,
        **common
    )

    servers = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=drive,
            **common
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            **common
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            remappings=drive,
            **common
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            **common
        ),
    ]

    def manager(name, nodes, condition=None):
        return Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name=name,
            output="screen",
            condition=condition,
            parameters=[
                {"use_sim_time": False, "autostart": autostart, "node_names": nodes}
            ],
        )

    return LaunchDescription(
        args
        + [
            wheel_odom,
            scan_filter,
            map_server,
            amcl,
            initial_pose_relay,
            *servers,
            GroupAction(
                [manager("lifecycle_manager_localization", ["map_server"])],
                condition=no_amcl,
            ),
            GroupAction(
                [manager("lifecycle_manager_localization", ["map_server", "amcl"])],
                condition=use_amcl,
            ),
            manager(
                "lifecycle_manager_navigation",
                [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            ),
        ]
    )
