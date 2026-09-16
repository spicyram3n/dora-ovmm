"""move_group for the HSRC, configured for grasping.

This composes the node itself instead of including hsrb_moveit_config's
move_group.py, because grasping needs settings that live in files that
package owns: a 1 cm RGB-D octomap fed by hsr_rgbd's depth relay, an IK
timeout the 5-DOF arm group can actually use, a planning model without
Gazebo's zero-travel sensor joint, and the capability that executes
MoveIt Task Constructor solutions (core/grasping/pick.py). Controllers,
joint limits and the SRDF are still loaded from that package unchanged, so
nothing here writes to the vendor checkout.

The octomap's topics live in one file under config/moveit/, chosen with
sensors_config:=. Its move_group part names the depth stream the updater
subscribes to; its relay block names what the depth relay reads. Simulation
uses sensors_xtion.yaml; the real robot, with depth arriving through vision
transport RX, uses sensors_xtion_remote.yaml (launch/grasp_real.launch.py).
"""

import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planning_model  # noqa: E402
from planning_model import RVIZ_DIR, ours, theirs  # noqa: E402


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "rviz_only",
            default_value="false",
            choices=["true", "false"],
            description="Open RViz against an existing move_group; also set use_rviz:=true.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "enable_grasp_servo", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "controllers_config",
            default_value="",
            description="Path to a MoveIt controllers YAML; empty uses hsrb_moveit_config's hsrb_controllers.yaml.",
        ),
        DeclareLaunchArgument(
            "sensors_config",
            default_value="sensors_xtion.yaml",
            description="Octomap file under config/moveit/. sensors_xtion_remote.yaml "
            "reads the real robot's depth from vision transport RX.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="",
            description="Depth stream feeding the octomap relay; empty takes the "
            "sensors file's relay.depth_topic.",
        ),
        DeclareLaunchArgument(
            "depth_info_topic",
            default_value="",
            description="CameraInfo for that stream; empty takes relay.info_topic.",
        ),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def octomap_wiring(context):
    """Split the chosen sensors file into move_group's part and the relay's."""
    sensors = ours(LaunchConfiguration("sensors_config").perform(context))
    relay = sensors.pop("relay", {})
    for key, argument in (("depth_topic", "depth_topic"), ("info_topic", "depth_info_topic")):
        override = LaunchConfiguration(argument).perform(context)
        if override:
            relay[key] = override
    # The updater subscribes to image_topic and, through image_transport, to
    # the camera_info beside it. Point both at the relay's decimated stream
    # rather than the raw one.
    image_topic = sensors[sensors["sensors"][0]]["image_topic"]
    remappings = [
        (image_topic, "/octomap_camera/image"),
        (image_topic.rsplit("/", 1)[0] + "/camera_info", "/octomap_camera/camera_info"),
    ]
    return sensors, relay, remappings


def launch_setup(context):
    sensors, relay_inputs, octomap_remappings = octomap_wiring(context)
    controllers_path = LaunchConfiguration("controllers_config").perform(context)
    if controllers_path:
        with open(controllers_path) as handle:
            controllers = yaml.safe_load(handle)
    else:
        controllers = yaml.safe_load(theirs("hsrb_controllers.yaml"))
    sim_time = LaunchConfiguration("use_sim_time")
    model = [planning_model.moveit_params()]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # The depth self-filter opens GLX contexts from worker threads. NVIDIA's
        # X11 path aborted here during a live run; use Mesa for this process.
        # Gazebo keeps its separate NVIDIA rendering environment.
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "LIBGL_DRI3_DISABLE": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "mesa",
        },
        parameters=model
        + [
            sensors,
            {
                "robot_name": "hsrc",
                "use_sim_time": sim_time,
                "capabilities": "move_group/ExecuteTaskSolutionCapability",
                "moveit_manage_controllers": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_simple_controller_manager": controllers,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
                "trajectory_execution.execution_duration_monitoring": False,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
        remappings=[("joint_states", "whole_body_moveit/joint_states")] + octomap_remappings,
    )
    relay = Node(
        package="hsr_rgbd",
        executable="depth_camera_relay.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": sim_time,
                **relay_inputs,
                # Paused until core/grasping/pick.py rebuilds the octomap for a
                # pick. Fed from startup, move_group spent every callback on
                # depth frames and answered no service, so the mission's
                # readiness check timed out before the robot moved.
                "enabled": False,
            }
        ],
    )
    stack = [
        move_group,
        # Starts idle. The pickup client explicitly starts/pauses Servo around
        # IBVS so its topic commands never compete with MTC's action goals.
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="grasp_servo",
            condition=IfCondition(LaunchConfiguration("enable_grasp_servo")),
            output="screen",
            parameters=model
            + [{"moveit_servo": ours("servo.yaml"), "use_sim_time": sim_time}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "world"],
        ),
        Node(
            package="hsrb_moveit_config",
            executable="odom_joint_states_publisher.py",
            name="odom_joint_states_publisher",
            parameters=[{"use_sim_time": sim_time}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace="whole_body_moveit",
            arguments=[planning_model.MODEL_PATH],
            parameters=[
                {
                    "source_list": ["/joint_states", "/odom_joint_states"],
                    "use_sim_time": sim_time,
                }
            ],
            remappings=[("robot_description", "/robot_description")],
        ),
        # Let robot state and TF arrive before integrating the first depth image.
        TimerAction(period=5.0, actions=[relay]),
    ]
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=model + [{"use_sim_time": sim_time}],
        arguments=["-d", os.path.join(RVIZ_DIR, "moveit.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return [
        GroupAction(
            actions=stack,
            condition=UnlessCondition(LaunchConfiguration("rviz_only")),
        ),
        rviz,
    ]
