"""Simulation bringup for the behaviour-tree query-to-grasp mission (core.pipeline.mission_tree)."""

import subprocess
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as Arg

ROOT = Path(__file__).resolve().parents[1]


def stop_gazebo(context):
    """Signal Gazebo itself, not its launcher.

    hsrb_gazebo_launch starts Gazebo through the `ign` ruby wrapper, which does
    not pass Ctrl+C on; stopping only the wrapper leaves Gazebo running with no
    parent. Gazebo's own processes stop their server and GUI within seconds."""
    if context.launch_configurations["start_simulation"] == "true":
        # Anchored to the start of the command line, so it cannot match a shell.
        subprocess.run(["pkill", "-INT", "-f", "^ign gazebo "], check=False)
    return []


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "target",
            default_value="",
            description="Object to search for, e.g. pringles; typed in the page with web:=true",
        ),
        DeclareLaunchArgument(
            "graph", default_value=str(ROOT / "outputs/scene_graph/apartment.json")
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument("top_k", default_value="3"),
        DeclareLaunchArgument("bearings", default_value="6"),
        DeclareLaunchArgument("startup_timeout", default_value="180"),
        DeclareLaunchArgument(
            "map", default_value=str(ROOT / "config/map/apartment_world_map.yaml")
        ),
        DeclareLaunchArgument(
            "params_file", default_value=str(ROOT / "config/nav2/nav2_params.yaml")
        ),
        DeclareLaunchArgument(
            "grasp",
            default_value="true",
            choices=["true", "false"],
            description="Generate grasps and pick the object up once parked.",
        ),
        DeclareLaunchArgument(
            "navigate_only",
            default_value="false",
            choices=["true", "false"],
            description="Reason and drive to the target; no SAM3 or GraspGenX.",
        ),
        DeclareLaunchArgument(
            "shutdown_when_done",
            default_value="false",
            choices=["true", "false"],
            description="Shut every launched process down when the mission ends.",
        ),
        DeclareLaunchArgument(
            "web",
            default_value="false",
            choices=["true", "false"],
            description="Start the stack without a mission; queries come from the web dashboard.",
        ),
    ]
    processes = []
    for name, command in [
        (
            "simulation",
            [
                "hsrb_gazebo_launch",
                "hsrb_apartment_world.launch.py",
                "use_sim_time:=true",
                "use_navigation:=false",
                "robot_name:=hsrc",
                "description_package:=hsrc_description",
                "description_file:=hsrc1s.urdf.xacro",
            ],
        ),
        (
            "navigation",
            [
                "hsrb_rosnav_config",
                "navigation_launch.py",
                "use_sim_time:=true",
                ["map:=", Arg("map")],
                ["params_file:=", Arg("params_file")],
            ],
        ),
        ("ik", [str(ROOT / "launch/ik_solver.launch.py"), "use_sim_time:=true"]),
        (
            "move_group",
            [
                str(ROOT / "launch/move_group.launch.py"),
                ["use_rviz:=", Arg("use_rviz")],
            ],
        ),
    ]:
        arguments.append(
            DeclareLaunchArgument(
                "start_" + name, default_value="true", choices=["true", "false"]
            )
        )
        # Each is a whole launch of its own. Gazebo takes longer than launch's
        # default 5 s to stop, and killing its launch sooner leaves it running.
        processes.append(
            ExecuteProcess(
                cmd=["ros2", "launch", *command],
                name=name,
                output="both",
                condition=IfCondition(Arg("start_" + name)),
                sigterm_timeout="30",
            )
        )
    common = [
        "--target",
        Arg("target"),
        "--graph",
        Arg("graph"),
        "--top-k",
        Arg("top_k"),
        "--bearings",
        Arg("bearings"),
        "--startup-timeout",
        Arg("startup_timeout"),
        "--grasp",
        Arg("grasp"),
    ]
    # No mission with web:=true: queries come from the web dashboard instead.
    tree = ExecuteProcess(
        cmd=[
            sys.executable,
            "-u",
            "-m",
            "core.pipeline.mission_tree",
            *common,
            "--navigate-only",
            Arg("navigate_only"),
        ],
        name="mission_tree",
        output="both",
        cwd=str(ROOT),
        condition=UnlessCondition(Arg("web")),
    )

    def finished(event, context):
        meanings = {
            0: "SUCCEEDED",
            1: "NOT FOUND",
            2: "FOUND but placement failed",
            3: "STARTUP / SERVICE ERROR",
            4: "PARKED but pickup failed",
        }
        done = context.launch_configurations["shutdown_when_done"] == "true"
        after = (
            "Shutting everything down."
            if done
            else "Simulation remains open; Ctrl+C to stop."
        )
        actions = [
            LogInfo(
                msg=f'[mission] {meanings.get(event.returncode, "FAILED")} '
                f"(exit {event.returncode}). {after}"
            )
        ]
        if done:
            actions.append(EmitEvent(event=Shutdown(reason="mission finished")))
        return actions

    return LaunchDescription(
        arguments
        + [
            RegisterEventHandler(OnProcessExit(target_action=tree, on_exit=finished)),
            RegisterEventHandler(
                OnShutdown(on_shutdown=[OpaqueFunction(function=stop_gazebo)])
            ),
            *processes,
            tree,
        ]
    )
