"""Grasping on its own: simulation and move_group, no Nav2, no mission.

    ros2 launch /home/ws/launch/grasp.launch.py
    python3 -m core.grasping.pick "pringles can"   # another terminal, repeat freely

Nothing here drives the base, so whatever the robot spawns facing is what it
tries to pick. SAM3 and GraspGenX still have to be up (docker/compose.yaml).
Pass target:= to run one pick automatically once the stack has settled.
"""

import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration as Arg
from launch.substitutions import PythonExpression

ROOT = Path(__file__).resolve().parents[1]


def stop_gazebo(context):
    """Gazebo runs under the `ign` ruby wrapper, which does not pass Ctrl+C on."""
    subprocess.run(["pkill", "-INT", "-f", "^ign gazebo "], check=False)
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "target",
            default_value="",
            description="Pick this once the stack has settled; empty just leaves it running.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "settle",
            default_value="25",
            description="Seconds to let Gazebo and move_group come up before picking.",
        ),
    ]
    processes = [
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "hsrb_gazebo_launch", "hsrb_apartment_world.launch.py",
                "use_sim_time:=true", "use_navigation:=false", "robot_name:=hsrc",
                "description_package:=hsrc_description",
                "description_file:=hsrc1s.urdf.xacro",
            ],
            name="simulation",
            output="both",
            sigterm_timeout="30",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "launch", str(ROOT / "launch/move_group.launch.py"),
                ["use_rviz:=", Arg("use_rviz")],
            ],
            name="move_group",
            output="both",
            sigterm_timeout="30",
        ),
    ]
    # A sleep rather than a readiness wait: pick.py already waits on every
    # service it needs, this only keeps it from starting before they exist.
    pick = ExecuteProcess(
        cmd=[
            "bash", "-c",
            ["sleep ", Arg("settle"), " && exec ", sys.executable,
             " -u -m core.grasping.pick ", '"', Arg("target"), '"'],
        ],
        name="pick",
        output="both",
        cwd=str(ROOT),
        condition=IfCondition(PythonExpression(["'", Arg("target"), "' != ''"])),
    )

    def finished(event, context):
        meanings = {0: "HELD", 3: "not picked, arm stowed", 4: "NOT STOWED, clear it by hand"}
        return [
            LogInfo(
                msg=f'[grasp] {meanings.get(event.returncode, "failed")} '
                f"(exit {event.returncode}). Simulation stays up; Ctrl+C to stop."
            )
        ]

    return LaunchDescription(
        arguments
        + [
            RegisterEventHandler(OnProcessExit(target_action=pick, on_exit=finished)),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_gazebo)])),
            *processes,
            pick,
        ]
    )
