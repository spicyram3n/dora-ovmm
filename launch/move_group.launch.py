"""move_group for the HSRC, configured for grasping.

This composes the node itself instead of including hsrb_moveit_config's
move_group.py, because grasping needs settings that live in files that
package owns: a 1 cm RGB-D octomap fed by hsr_rgbd's depth relay, an IK
timeout the 5-DOF arm group can actually use, a planning model without
Gazebo's zero-travel sensor joint, and the capability that executes
MoveIt Task Constructor solutions (core/grasping/pick.py). Controllers,
joint limits and the SRDF are still loaded from that package unchanged, so
nothing here writes to the vendor checkout.
"""

import os
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planning_model  # noqa: E402
from planning_model import RVIZ_DIR, ours, theirs  # noqa: E402


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("rviz_only", default_value="false", choices=["true", "false"],
                              description="Open RViz against an existing move_group; also set use_rviz:=true."),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_grasp_servo", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("depth_topic",
                              default_value="/head_rgbd_sensor/depth_registered/image",
                              description="Simulator depth stream feeding the octomap relay."),
    ]
    sim_time = LaunchConfiguration("use_sim_time")
    model = [planning_model.moveit_params()]
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=model + [
            ours("sensors_xtion.yaml"),
            {"robot_name": "hsrc",
             "use_sim_time": sim_time,
             "capabilities": "move_group/ExecuteTaskSolutionCapability",
             "moveit_manage_controllers": True,
             "moveit_controller_manager":
                 "moveit_simple_controller_manager/MoveItSimpleControllerManager",
             "moveit_simple_controller_manager": yaml.safe_load(theirs("hsrb_controllers.yaml")),
             "trajectory_execution.allowed_execution_duration_scaling": 1.2,
             "trajectory_execution.allowed_goal_duration_margin": 0.5,
             "trajectory_execution.allowed_start_tolerance": 0.01,
             "trajectory_execution.execution_duration_monitoring": False,
             "publish_planning_scene": True,
             "publish_geometry_updates": True,
             "publish_state_updates": True,
             "publish_transforms_updates": True}],
        # The octomap updater subscribes to the topic sensors_xtion.yaml names;
        # point it at the relay's decimated stream rather than the raw one.
        remappings=[("joint_states", "whole_body_moveit/joint_states"),
                    ("/head_rgbd_sensor/depth_registered/image_rect_raw", "/octomap_camera/image"),
                    ("/head_rgbd_sensor/depth_registered/camera_info", "/octomap_camera/camera_info")])
    relay = Node(package="hsr_rgbd", executable="depth_camera_relay.py", output="screen",
                 parameters=[{"use_sim_time": sim_time,
                              "depth_topic": LaunchConfiguration("depth_topic")}])
    stack = [
        move_group,
        # Starts idle. The pickup client explicitly starts/pauses Servo around
        # IBVS so its topic commands never compete with MTC's action goals.
        Node(package="moveit_servo", executable="servo_node_main", name="grasp_servo",
             condition=IfCondition(LaunchConfiguration('enable_grasp_servo')),
             output="screen", parameters=model + [{"moveit_servo": ours("servo.yaml"),
                                                   "use_sim_time": sim_time}]),
        Node(package="tf2_ros", executable="static_transform_publisher", output="log",
             arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "world"]),
        Node(package="hsrb_moveit_config", executable="odom_joint_states_publisher.py",
             name="odom_joint_states_publisher", parameters=[{"use_sim_time": sim_time}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             name="joint_state_publisher", namespace="whole_body_moveit",
             arguments=[planning_model.MODEL_PATH],
             parameters=[{"source_list": ["/joint_states", "/odom_joint_states"],
                          "use_sim_time": sim_time}],
             remappings=[("robot_description", "/robot_description")]),
        # Let robot state and TF arrive before integrating the first depth image.
        TimerAction(period=5.0, actions=[relay]),
    ]
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             parameters=model + [{"use_sim_time": sim_time}],
             arguments=["-d", os.path.join(RVIZ_DIR, "moveit.rviz")],
             condition=IfCondition(LaunchConfiguration("use_rviz")))
    return LaunchDescription(arguments + [
        GroupAction(actions=stack, condition=UnlessCondition(LaunchConfiguration("rviz_only"))),
        rviz,
    ])
