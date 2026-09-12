"""Readiness checks and arm homing for core.pipeline.mission_tree and the web dashboard."""
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from tmc_manipulation_msgs.srv import SolveIkWithCollision
from trajectory_msgs.msg import JointTrajectoryPoint

from core.perception import sam3_client
from core.perception.camera_ros2 import grab_rgbd


def home_goal():
    """The user's navigation home pose, as an arm trajectory goal."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [
        'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
        'wrist_flex_joint', 'wrist_roll_joint',
    ]
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=[0.0, 0.0, -1.57, -1.57, 0.0],
        time_from_start=Duration(sec=3),
    )]
    return goal


def home_arm(navigator, wait):
    """Complete the user's navigation home pose before starting the search."""
    client = ActionClient(navigator, FollowJointTrajectory,
                         '/arm_trajectory_controller/follow_joint_trajectory')
    try:
        wait('arm trajectory action', client.server_is_ready)
        goal = home_goal()
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


def waiter(navigator, deadline):
    """(remaining, wait) for readiness checks that share one deadline."""
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

    return remaining, wait


def get_ready(navigator, target, timeout, perception=True):
    """Wait for every service the mission uses; skip SAM3 without perception.

    Returns the deadline-bound wait, for homing the arm within the same budget."""
    remaining, wait = waiter(navigator, time.monotonic() + timeout)
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
    if perception:
        print('[WAIT] SAM3 response (start docker/sam3/run_sam3.sh on host)', flush=True)
        while True:
            try:
                sam3_client.detect(rgb, target, timeout=min(30., remaining()))
                break
            except sam3_client.ObjectNotFound:
                break  # A valid negative detection still proves the server is ready.
            except RuntimeError as error:
                print(f'[WAIT] SAM3: {error}', flush=True)
                rclpy.spin_once(navigator, timeout_sec=min(1., remaining()))
        print('[READY] SAM3; startup detection discarded', flush=True)
    return wait
