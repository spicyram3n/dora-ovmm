"""Simulation bringup for the core.pipeline query-to-grasp mission."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
    from launch.conditions import IfCondition
    from launch.event_handlers import OnProcessExit
    from launch.substitutions import LaunchConfiguration as Arg

    arguments = [
        DeclareLaunchArgument('target', description='Object to search for, e.g. pringles'),
        DeclareLaunchArgument('graph', default_value=str(ROOT / 'outputs/scene_graph/apartment.json')),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('top_k', default_value='3'),
        DeclareLaunchArgument('bearings', default_value='6'),
        DeclareLaunchArgument('startup_timeout', default_value='180'),
        DeclareLaunchArgument('map', default_value=str(ROOT / 'config/map/apartment_world_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(ROOT / 'config/nav2/nav2_params.yaml')),
        DeclareLaunchArgument('grasp', default_value='true', choices=['true', 'false'],
                              description='Generate grasps and pick the object up once parked.'),
    ]
    processes = []
    for name, command in [
        ('simulation', ['hsrb_gazebo_launch', 'hsrb_apartment_world.launch.py',
                        'use_sim_time:=true', 'use_navigation:=false', 'robot_name:=hsrc',
                        'description_package:=hsrc_description', 'description_file:=hsrc1s.urdf.xacro']),
        ('navigation', ['hsrb_rosnav_config', 'navigation_launch.py', 'use_sim_time:=true',
                        ['map:=', Arg('map')], ['params_file:=', Arg('params_file')]]),
        ('ik', [str(ROOT / 'launch/ik_solver.launch.py'), 'use_sim_time:=true']),
        ('move_group', [str(ROOT / 'launch/move_group.launch.py'),
                        ['use_rviz:=', Arg('use_rviz')]]),
    ]:
        arguments.append(DeclareLaunchArgument('start_' + name, default_value='true',
                                              choices=['true', 'false']))
        processes.append(ExecuteProcess(
            cmd=['ros2', 'launch', *command], name=name, output='both',
            condition=IfCondition(Arg('start_' + name)),
        ))
    mission = ExecuteProcess(
        cmd=[sys.executable, '-u', '-m', 'core.pipeline',
             '--target', Arg('target'), '--graph', Arg('graph'),
             '--top-k', Arg('top_k'), '--bearings', Arg('bearings'),
             '--startup-timeout', Arg('startup_timeout'), '--grasp', Arg('grasp')],
        name='search', output='both', cwd=str(ROOT),
    )

    def finished(event, context):
        meanings = {0: 'SUCCEEDED', 1: 'NOT FOUND', 2: 'FOUND but placement failed',
                    3: 'STARTUP / SERVICE ERROR', 4: 'PARKED but pickup failed'}
        return [LogInfo(msg=f'[mission] {meanings.get(event.returncode, "FAILED")} '
                        f'(exit {event.returncode}). Simulation remains open; Ctrl+C to stop.')]

    return LaunchDescription(arguments + [
        RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=finished)),
        *processes, mission,
    ])
