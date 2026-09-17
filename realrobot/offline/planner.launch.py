"""Nav2's global planner alone, serving one saved map.

    ros2 launch realrobot/offline/planner.launch.py \
        map:=/home/ws/config/realrobot/map/lab_20260811.pgm's yaml

plan_overlay.py starts this itself; launch it by hand only to keep the planner
up across several queries, or to look at /global_costmap/costmap in RViz.

Costmap2DROS refuses to activate until it can look up map -> base_footprint, so
a static transform parks a stand-in robot at the map origin. Nothing reads that
pose: ComputePathToPose is called with use_start, and the costmap carries only
the static and inflation layers, neither of which depends on where the robot is.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from pathlib import Path

PARAMS = str(Path(__file__).resolve().parent / "lab_planner.yaml")
NODES = ["map_server", "planner_server"]


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params = RewrittenYaml(
        source_file=PARAMS,
        param_rewrites={"yaml_filename": map_yaml},
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument("map", description="occupancy map YAML to plan on"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="planner_stand_in_base", output="log",
             arguments=["0", "0", "0", "0", "0", "0", "map", "base_footprint"]),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[params]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_planner", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True, "node_names": NODES}]),
    ])
