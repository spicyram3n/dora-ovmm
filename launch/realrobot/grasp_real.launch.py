"""Grasp test on the real HSR, PC side: move_group on the robot's DDS topics,
RGB-D through vision transport RX, no simulator, no Nav2, no mission.

    ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py
    HSR_REAL_ROBOT=1 python3 -m core.grasping.pick "pringles can"   # another terminal

Pass target:= to run one pick once move_group has settled; mode:=grasp holds
without the test lift. Everything the robot already runs stays on the robot:
controllers, joint states, TF, the camera driver and a vision transport TX.
SAM3, GraspGenX and RX run on this PC in Docker. Run
realrobot/live/grasp_preflight.py before the first pick. README: "Real robot:
grasp test".
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration as Arg
from launch.substitutions import PythonExpression

ROOT = Path(__file__).resolve().parents[2]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "target",
            default_value="",
            description="Pick this once move_group has settled; empty just leaves it running.",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="auto",
            choices=["auto", "pickup", "grasp"],
            description="core.grasping.pick --mode; grasp closes without the test lift.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="realrobot/rviz/grasp_real.rviz",
            description="RViz file under config/. The default adds RX's RGB and "
            "depth to the MoveIt displays, best-effort so they match RX.",
        ),
        DeclareLaunchArgument(
            "sensors_config",
            default_value="realrobot/moveit/sensors_xtion_remote.yaml",
            description="Octomap file under config/moveit/ naming RX's depth output.",
        ),
        DeclareLaunchArgument(
            "image_prefix",
            default_value="/remote",
            description="Prefix of RX's RGB and depth output topics, for the pick's camera.",
        ),
        DeclareLaunchArgument(
            "settle",
            default_value="10",
            description="Seconds to let move_group come up before picking.",
        ),
    ]
    move_group = ExecuteProcess(
        cmd=[
            "ros2", "launch", str(ROOT / "launch/move_group.launch.py"),
            "use_sim_time:=false",
            ["sensors_config:=", Arg("sensors_config")],
            ["use_rviz:=", Arg("use_rviz")],
            ["rviz_config:=", Arg("rviz_config")],
        ],
        name="move_group",
        output="both",
        sigterm_timeout="30",
    )
    # A sleep rather than a readiness wait: pick.py already waits on every
    # service it needs, this only keeps it from starting before they exist.
    pick = ExecuteProcess(
        cmd=[
            "bash", "-c",
            ["sleep ", Arg("settle"), " && exec ", sys.executable,
             " -u -m core.grasping.pick --mode ", Arg("mode"), ' "', Arg("target"), '"'],
        ],
        name="pick",
        output="both",
        cwd=str(ROOT),
        additional_env={"HSR_REAL_ROBOT": "1", "HSR_IMAGE_PREFIX": Arg("image_prefix")},
        condition=IfCondition(PythonExpression(["'", Arg("target"), "' != ''"])),
    )

    def finished(event, context):
        meanings = {0: "HELD", 3: "not picked, arm stowed", 4: "NOT STOWED, clear it by hand"}
        return [
            LogInfo(
                msg=f'[grasp] {meanings.get(event.returncode, "failed")} '
                f"(exit {event.returncode}). move_group stays up; Ctrl+C to stop."
            )
        ]

    return LaunchDescription(
        arguments
        + [
            RegisterEventHandler(OnProcessExit(target_action=pick, on_exit=finished)),
            move_group,
            pick,
        ]
    )
