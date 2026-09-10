#!/usr/bin/env python3
# move_group against the live HSRC ignition sim: parses the hsrc URDF the
# same way move_to_grasp.launch.py does and hands it to hsrb_moveit_config's
# move_group.py (whose robot_description default is a placeholder string).
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('hsrb_moveit_config'), 'launch'))
import robot_description  # noqa: E402


def generate_launch_description():
    urdf = robot_description.parse('hsrc_description', 'hsrc1s.urdf.xacro')
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hsrb_moveit_config'), 'launch/move_group.py')),
        launch_arguments={
            'robot_description': urdf,
            'robot_name': 'hsrc',
            'use_rviz': 'false',
            'use_sim_time': 'true',
            'depth_image_topic': '/octomap_camera/image',
            'depth_camera_info_topic': '/octomap_camera/camera_info',
        }.items())
    # The depth-image updater uses OpenGL for robot self-filtering. Match
    # the working container graphics settings used by run_grasp_sim.sh.
    display_environment = [SetEnvironmentVariable(name, os.environ.get(name, '1'))
                           for name in ('QT_X11_NO_MITSHM', 'LIBGL_DRI3_DISABLE',
                                        'LIBGL_ALWAYS_SOFTWARE')]
    depth_relay = Node(package='hsrb_moveit_config', executable='depth_camera_relay.py',
                       parameters=[{'use_sim_time': True}], output='screen')
    # Let robot state / TF arrive before integrating the first depth image.
    return LaunchDescription(display_environment + [move_group, TimerAction(period=5.0, actions=[depth_relay])])
