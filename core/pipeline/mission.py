"""Run one query-to-grasp mission against an already running ROS stack."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]


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


def run(argv=None):
    import argparse
    import math
    import time

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', required=True)
    parser.add_argument('--graph', type=Path, default=ROOT / 'outputs/scene_graph/apartment.json')
    parser.add_argument('--top-k', type=int, default=3)
    parser.add_argument('--bearings', type=int, default=6)
    parser.add_argument('--startup-timeout', type=float, default=180)
    parser.add_argument('--grasp', default='true', choices=['true', 'false'])
    args = parser.parse_args(argv)
    if not args.target.strip() or args.top_k < 1 or args.bearings < 1:
        parser.error('target must be nonempty; top-k and bearings must be positive')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('startup-timeout must be finite and positive')

    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from tmc_manipulation_msgs.srv import SolveIkWithCollision
    from core.navigation.nav2_client import Navigator
    from core.perception.camera_ros2 import grab_rgbd
    from core.perception import sam3_client
    from core.scene_graph import graph as sg
    from core.pipeline.search import execute_mission, EXIT_CODES

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
        navigator.resume_navigation_if_paused()
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
        status = execute_mission(
            scene, args.target, navigator, args.graph, top_k=args.top_k,
            bearings=args.bearings, grasp=args.grasp == 'true',
        )
        return EXIT_CODES[status]
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

