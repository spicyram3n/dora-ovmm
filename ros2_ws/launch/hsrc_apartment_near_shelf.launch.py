#!/usr/bin/env python3
# Apartment world with the HSRC spawned right in front of the high_shelf
# (pringles at x=4.34 y=13.00 z=0.77), instead of the stock (5.0, 6.6) pose.
# Same include chain as hsrc_apartment_world.launch.py, only the pose differs.
import os

from ament_index_python.packages import get_package_share_directory
from hsrb_launch_utils.hsrb_launch_utils import declare_launch_arguments
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# SRDF "go" pose (arm tucked for driving); sent once after the controllers
# come up so the HSRC starts every session stowed without touching the
# shared hsrb_description xacro.
STOW_GOAL = ('{trajectory: {joint_names: [arm_lift_joint, arm_flex_joint, arm_roll_joint, '
             'wrist_flex_joint, wrist_roll_joint], points: [{positions: [0.0, 0.0, -1.57, -1.57, 0.0], '
             'time_from_start: {sec: 3}}]}}')


def generate_launch_description():
    declared_arguments = declare_launch_arguments()

    hsrb_gazebo_common_path = os.path.join(
        get_package_share_directory('hsrb_gazebo_launch'),
        'launch/include/hsrb_gazebo_common.launch.py')

    hsrb_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hsrb_gazebo_common_path),
        launch_arguments={
            "map": os.path.join(
                get_package_share_directory('tmc_potential_maps'),
                'maps/apartment/map.yaml'),
            # Facing hsr_pringles_02 on the high_table (4.67, 3.89, z=1.16)
            # from the table's east edge (x=4.85): can 0.51m ahead, open
            # approach with no shelf walls.
            "robot_pos_x": LaunchConfiguration("spawn_x"),
            "robot_pos_y": LaunchConfiguration("spawn_y"),
            "robot_pos_z": "0.0",
            "robot_rpy_Y": LaunchConfiguration("spawn_yaw"),
            "world_name": os.path.join(
                get_package_share_directory('tmc_gazebo_worlds'),
                'worlds/apartment.world'),
            "robot_name": "hsrc",
            "description_package": "hsrc_description",
            "description_file": "hsrc1s.urdf.xacro",
            "collision_file": "collision_pair_hsrc.xml",
        }.items())

    stow_arm = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'for i in 1 2 3 4 5; do '
                 'ros2 action send_goal /arm_trajectory_controller/follow_joint_trajectory '
                 "control_msgs/action/FollowJointTrajectory '" + STOW_GOAL + "' "
                 '&& break; sleep 5; done'],
            name='stow_arm', output='screen')])

    return LaunchDescription(declared_arguments + [
        DeclareLaunchArgument("spawn_x", default_value="5.18"),
        DeclareLaunchArgument("spawn_y", default_value="3.893"),
        DeclareLaunchArgument("spawn_yaw", default_value="3.1416"),
        hsrb_gazebo_common, stow_arm])
