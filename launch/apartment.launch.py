"""Kitchen objects, no room doors; HSR starts beside the high table."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from hsrb_launch_utils.hsrb_launch_utils import declare_launch_arguments
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]


def generate_launch_description():
    models = get_package_share_directory('tmc_gazebo_worlds') + '/models'
    resource_paths = os.pathsep.join([str(ROOT / 'assets/models'), models])
    common = get_package_share_directory('hsrb_gazebo_launch') + '/launch/include/hsrb_gazebo_common.launch.py'
    return LaunchDescription([
        *declare_launch_arguments(),
        DeclareLaunchArgument('world', default_value=str(ROOT / 'worlds/kitchen_objects.world')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(common), launch_arguments={
            'world_name': LaunchConfiguration('world'),
            'robot_pos_x': '5.6', 'robot_pos_y': '4.0', 'robot_pos_z': '0.0',
            'robot_rpy_Y': '3.141592653589793',
            'robot_name': 'hsrc', 'description_package': 'hsrc_description',
            'description_file': 'hsrc1s.urdf.xacro', 'use_sim_time': 'true',
            'use_navigation': 'false', 'use_joy_node': 'false', 'rviz': 'false',
        }.items()),
    ])
