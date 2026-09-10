"""Simulation bringup and one search mission; also runs its own readiness worker."""

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
        DeclareLaunchArgument('top_k', default_value='3'),
        DeclareLaunchArgument('bearings', default_value='6'),
        DeclareLaunchArgument('startup_timeout', default_value='180'),
        DeclareLaunchArgument('map', default_value=str(ROOT / 'config/map/apartment_world_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(ROOT / 'config/nav2_params.yaml')),
    ]
    processes = []
    for name, command in [
        ('simulation', ['hsrb_gazebo_launch', 'hsrb_apartment_world.launch.py',
                        'use_sim_time:=true', 'use_navigation:=false', 'robot_name:=hsrc',
                        'description_package:=hsrc_description', 'description_file:=hsrc1s.urdf.xacro']),
        ('navigation', ['hsrb_rosnav_config', 'navigation_launch.py', 'use_sim_time:=true',
                        ['map:=', Arg('map')], ['params_file:=', Arg('params_file')]]),
        ('ik', [str(ROOT / 'ros2_ws/src/grasp_execution/launch/ik_solver.launch.py'),
                'use_sim_time:=true']),
    ]:
        arguments.append(DeclareLaunchArgument('start_' + name, default_value='true',
                                              choices=['true', 'false']))
        processes.append(ExecuteProcess(
            cmd=['ros2', 'launch', *command], name=name, output='both',
            condition=IfCondition(Arg('start_' + name)),
        ))
    mission = ExecuteProcess(
        cmd=[sys.executable, '-u', str(Path(__file__).resolve()),
             '--target', Arg('target'), '--graph', Arg('graph'),
             '--top-k', Arg('top_k'), '--bearings', Arg('bearings'),
             '--startup-timeout', Arg('startup_timeout')],
        name='search', output='both', cwd=str(ROOT),
    )

    def finished(event, context):
        meanings = {0: 'READY for grasp integration', 1: 'NOT FOUND',
                    2: 'FOUND but placement failed', 3: 'STARTUP / SERVICE ERROR'}
        return [LogInfo(msg=f'[mission] {meanings.get(event.returncode, "FAILED")} '
                        f'(exit {event.returncode}). Simulation remains open; Ctrl+C to stop.')]

    return LaunchDescription(arguments + [
        RegisterEventHandler(OnProcessExit(target_action=mission, on_exit=finished)),
        *processes, mission,
    ])


def home_arm(navigator, wait):
    """Complete the user's navigation home pose before starting the search."""
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectoryPoint

    client = ActionClient(navigator, FollowJointTrajectory,
                         '/arm_trajectory_controller/follow_joint_trajectory')
    try:
        wait('arm trajectory action', client.server_is_ready)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
            'wrist_flex_joint', 'wrist_roll_joint',
        ]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[0.0, 0.0, -1.57, -1.57, 0.0],
            time_from_start=Duration(sec=3),
        )]
        print('[HOME] moving arm to home pose', flush=True)
        # Reuse the navigator's bounded action wait and cancellation handling.
        outcome = navigator._run(client, goal, timeout=30)
        if outcome is None or outcome.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Arm homing did not succeed; search will not start')
        if outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Arm homing failed: {outcome.result.error_string}')
        print('[READY] arm home pose reached', flush=True)
    finally:
        client.destroy()


def run():
    import argparse
    import math
    import time

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', required=True)
    parser.add_argument('--graph', type=Path, required=True)
    parser.add_argument('--top-k', type=int, default=3)
    parser.add_argument('--bearings', type=int, default=6)
    parser.add_argument('--startup-timeout', type=float, default=180)
    args = parser.parse_args()
    if not args.target.strip() or args.top_k < 1 or args.bearings < 1:
        parser.error('target must be nonempty; top-k and bearings must be positive')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('startup-timeout must be finite and positive')

    sys.path.insert(0, str(ROOT / 'core'))
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from tmc_manipulation_msgs.srv import SolveIkWithCollision
    from navigation.nav2_client import Navigator
    from perception.camera_ros2 import grab_rgbd
    from perception import sam3_client
    from scene_graph import graph as sg
    from search_object import search, make_graspable, FOUND, EXIT_CODES

    # Validate the graph before waiting for any robot or model service.
    scene = sg.load(args.graph)
    sg.require_map(scene)
    rclpy.init()
    navigator = Navigator()
    deadline = time.monotonic() + args.startup_timeout

    def remaining():
        seconds = deadline - time.monotonic()
        if seconds <= 0:
            raise TimeoutError('Startup deadline exceeded; see the last WAIT message')
        return seconds

    def wait(label, check):
        print(f'[WAIT] {label}', flush=True)
        while True:
            remaining()
            if check():
                print(f'[READY] {label}', flush=True)
                return
            rclpy.spin_once(navigator, timeout_sec=min(0.5, remaining()))

    try:
        for name in ('amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator'):
            client = navigator.create_client(GetState, f'/{name}/get_state')
            try:
                def active():
                    if not client.service_is_ready():
                        return False
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(navigator, future, timeout_sec=min(1., remaining()))
                    if not future.done():
                        client.remove_pending_request(future)
                        return False
                    return future.result().current_state.id == 3  # lifecycle ACTIVE
                wait(name + ' active', active)
            finally:
                navigator.destroy_client(client)
        wait('Nav2 actions', lambda: navigator.planner.server_is_ready() and navigator.driver.server_is_ready())
        head = ActionClient(navigator, FollowJointTrajectory,
                            '/head_trajectory_controller/follow_joint_trajectory')
        try:
            wait('head trajectory action', head.server_is_ready)
        finally:
            head.destroy()
        ik = navigator.create_client(SolveIkWithCollision, '/ik_solver_node/solve_ik_with_collision')
        try:
            wait('IK service', ik.service_is_ready)
        finally:
            navigator.destroy_client(ik)

        def localized():
            try:
                navigator.robot_pose(timeout=min(1., remaining()))
                return True
            except RuntimeError:
                return False
        wait('fresh map-to-base localization', localized)

        print('[WAIT] synchronized RGB-D and camera TF', flush=True)
        while True:
            try:
                rgb, _, _, _ = grab_rgbd(target_frame='map', timeout=min(5., remaining()))
                break
            except RuntimeError as error:
                print(f'[WAIT] {error}', flush=True)
        print('[READY] RGB-D and camera TF', flush=True)
        print('[WAIT] SAM3 response (start docker/sam3/run_sam3.sh on host)', flush=True)
        while True:
            try:
                sam3_client.detect(rgb, args.target, timeout=min(30., remaining()))
                break
            except sam3_client.ObjectNotFound:
                break  # A valid negative detection still proves the server is ready.
            except RuntimeError as error:
                print(f'[WAIT] SAM3: {error}', flush=True)
                rclpy.spin_once(navigator, timeout_sec=min(1., remaining()))
        print('[READY] SAM3; startup detection discarded', flush=True)
        home_arm(navigator, wait)
        print(f'[SEARCH] target={args.target!r}; graph={args.graph}', flush=True)
        status, object_id = search(scene, args.target, navigator, top_k=args.top_k)
        if object_id is not None:
            sg.save(scene, args.graph)
        if status == FOUND:
            print('[APPROACH] finding and reaching an arm-reachable base pose', flush=True)
            status = make_graspable(scene, object_id, navigator, bearings=args.bearings)
        print(f'[RESULT] {status}', flush=True)
        return EXIT_CODES[status]
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        raise SystemExit(run())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print(f'[ERROR] {type(error).__name__}: {error}', file=sys.stderr, flush=True)
        raise SystemExit(3)
