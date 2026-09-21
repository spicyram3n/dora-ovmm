#!/usr/bin/env python3
"""Bring the arm back to the observation pose after a pick stopped part-way.

core.grasping.pick leaves the arm wherever the failure happened: pick() prints
the error and returns, and only a lift failure has its own recovery. After a
closure failure the hand is still around the object with the fingers part
closed, so folding the arm straight home would drag the hand through it.

This does four steps in the order that keeps the hand clear:

    1. open the hand on the hardware, releasing whatever it holds;
    2. drop the failed run's target box and depth map from the planning scene;
    3. retreat the palm straight back along its own approach axis;
    4. plan the arm to the observation pose, collision-aware through move_group.

Then the head is centred: the failed run left the camera on its last target. And
Nav2, which the mission paused for the pick, is resumed if it is running.

Step 2 matters more than it looks. Both the target box and the octomap describe
where things were before the arm disturbed them, and a run that ends with the
hand against the object leaves the arm inside that box. A start state in
collision cannot be planned from at all, so without this the fold never gets a
plan and the arm stays where it stopped (2026-09-18).

Needs move_group running (launch/realrobot/grasp_real.launch.py).

    HSR_REAL_ROBOT=1 python3 realrobot/live/recover_home.py
    HSR_REAL_ROBOT=1 python3 realrobot/live/recover_home.py --retreat 0.15
    HSR_REAL_ROBOT=1 python3 realrobot/live/recover_home.py --direct
"""
import argparse
import sys

import rclcpp
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit.task_constructor import stages
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, "/home/ws")
from core.grasping import pick  # noqa: E402
from core.grasping.pick import (  # noqa: E402
    FINGERS, HAND, HEAD_LINKS, OPEN_HAND, TARGET, Pick, aim_head, call, execute,
    joint_positions, move_hand, open_hand, run_action,
)
from core.navigation.nav2_client import Navigator  # noqa: E402
from core.perception.camera_ros2 import USE_SIM_TIME  # noqa: E402
from rclpy.parameter import Parameter as NodeParameter  # noqa: E402

# The observation pose, as Pick.clear_view() plans it.
HOME = {"arm_lift_joint": 0.0, "arm_flex_joint": 0.0, "arm_roll_joint": -1.57,
        "wrist_flex_joint": -1.57, "wrist_roll_joint": 0.0}


def forget_the_scene(node):
    """Drop the target box and the depth map the failed run left behind."""
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    # An empty id detaches whatever the hand was holding in the scene.
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(object=CollisionObject(operation=CollisionObject.REMOVE))]
    scene.world.collision_objects = [
        CollisionObject(id=TARGET, operation=CollisionObject.REMOVE)]
    call(node, ApplyPlanningScene, "/apply_planning_scene",
         ApplyPlanningScene.Request(scene=scene))
    call(node, Empty, "/clear_octomap", Empty.Request())


def fold_directly(node, seconds):
    """Drive the arm to the observation pose with no planning and no collision
    checking.

    The last resort, for when the arm is somewhere move_group will not plan out
    of. The fold rotates the arm up and away from the table, which is why it is
    usually survivable, but nothing here checks that. Hand empty, arm clear,
    and watch the robot.
    """
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(HOME)
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=[HOME[name] for name in HOME],
        time_from_start=Duration(sec=int(seconds)))]
    run_action(node, FollowJointTrajectory,
               "/arm_trajectory_controller/follow_joint_trajectory", goal, seconds + 15.)


def retreat_task(planner, distance):
    """Straight back along the palm's own z axis, out of the object."""
    task = planner.task("retreat from the object")
    allow = stages.ModifyPlanningScene("ignore the target during the retreat")
    allow.allowCollisions(TARGET, FINGERS, True)
    allow.allowCollisions("<octomap>", HEAD_LINKS, True)
    task.add(allow)
    task.add(move_hand("retreat", planner.cartesian, (0.0, 0.0, -1.0),
                       HAND, distance * 0.5, distance))
    return task


def centre_head(node):
    """The failed run left the camera on its last target. The arm is already home by
    now, so a head that will not move is said, not raised: main() would call that
    an arm that has not been stowed."""
    try:
        aim_head(node, 0.0, 0.0)
        print("[RECOVER] head centred", flush=True)
    except RuntimeError as error:
        print(f"[RECOVER] head not centred ({error}); the arm is home", flush=True)


def resume_nav():
    """The mission paused Nav2 for the pick and nothing resumes it until the next one,
    so an RViz goal does nothing meanwhile. Like the head: the arm is already home, so
    a Nav2 that is absent (the grasp stack alone) or refuses is said, not raised."""
    navigator = Navigator(use_sim_time=USE_SIM_TIME)
    try:
        if not navigator._is_running("controller_server"):
            print("[RECOVER] no Nav2 running; nothing to resume", flush=True)
            return
        navigator.resume_navigation_if_paused()
        print("[RECOVER] Nav2 active", flush=True)
    except RuntimeError as error:
        print(f"[RECOVER] Nav2 not resumed ({error}); the arm is home", flush=True)
    finally:
        navigator.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--retreat", type=float, default=0.10,
                        help="metres to back the palm off before folding the arm")
    parser.add_argument("--keep-hand", action="store_true",
                        help="do not open the hand first; use when it holds something you want kept")
    parser.add_argument("--direct", action="store_true",
                        help="skip planning and drive the arm to the observation pose. "
                             "NO COLLISION CHECKING: hand empty, arm clear, watch it")
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="duration of the --direct motion; longer is slower")
    arguments = parser.parse_args()

    rclpy.init()
    rclcpp.init()
    node = rclpy.create_node(
        "recover_home",
        parameter_overrides=[NodeParameter("use_sim_time", value=USE_SIM_TIME)])
    try:
        if arguments.direct:
            # pick's topic exists only while move_group's launch is up, and --direct
            # is for when it is not. The robot's own topic has the hand and arm joints.
            pick.JOINT_STATES = "/joint_states"
        if not arguments.keep_hand:
            if joint_positions(node)["hand_motor_joint"] < OPEN_HAND - .05:
                print("[RECOVER] opening the hand", flush=True)
                open_hand(node)
            else:
                print("[RECOVER] hand already open", flush=True)

        if arguments.direct:
            print(f"[RECOVER] driving to the observation pose over "
                  f"{arguments.seconds:.0f} s, unplanned", flush=True)
            fold_directly(node, arguments.seconds)
            print("[RECOVER] arm home", flush=True)
            centre_head(node)
            resume_nav()
            return 0

        from core.grasping.pick import mtc_node
        planner = Pick(mtc_node())

        print("[RECOVER] dropping the stale target box and depth map", flush=True)
        forget_the_scene(node)

        print(f"[RECOVER] retreating up to {arguments.retreat*100:.0f} cm "
              "along the palm axis", flush=True)
        try:
            execute(node, retreat_task(planner, arguments.retreat))
        except RuntimeError as error:
            # A retreat that will not plan is not worth forcing; the fold below
            # is collision-aware too, and the operator can see what is in the way.
            print(f"[RECOVER] retreat failed ({error}); trying the fold anyway", flush=True)

        print("[RECOVER] folding the arm to the observation pose", flush=True)
        execute(node, planner.clear_view())
        print("[RECOVER] arm home", flush=True)
        centre_head(node)
        resume_nav()
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"[RECOVER] failed: {error}")
        print("[RECOVER] the arm has NOT been stowed. Try --direct (no collision "
              "checking, hand empty, watch it), or clear it by hand.")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        rclcpp.shutdown()


if __name__ == "__main__":
    sys.exit(main())
