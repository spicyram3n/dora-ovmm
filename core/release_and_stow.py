"""Release the held simulation can, clear its planning model, and stow via MoveIt."""
import argparse
import time
from pathlib import Path
import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
from finish_pick import FinishPick


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print('Use --execute to release and stow.')
        return
    rclpy.init()
    node = rclpy.create_node('release_and_stow')
    pick = FinishPick(node, {}, Path('/tmp'))
    try:
        scene = pick.scene()
        if not any(a.object.id == 'grasp_target' for a in scene.robot_state.attached_collision_objects):
            raise RuntimeError('No held grasp_target to release')
        pick.close_step(1.10)
        print('gripper_open_release_completed', flush=True)
        remove = AttachedCollisionObject(link_name='hand_palm_link')
        remove.object.id = 'grasp_target'
        remove.object.operation = CollisionObject.REMOVE
        world_remove = CollisionObject(id='grasp_target', operation=CollisionObject.REMOVE)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [remove]
        request.scene.world.collision_objects = [world_remove]
        if not pick.call(ApplyPlanningScene, '/apply_planning_scene', request).success:
            raise RuntimeError('Failed to clear released collision model')
        publisher = node.create_publisher(CollisionObject, '/octomap_target_exclusion', 1)
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            publisher.publish(world_remove)
            rclpy.spin_once(node, timeout_sec=.1)
        scene = pick.scene()
        if any(a.object.id == 'grasp_target' for a in scene.robot_state.attached_collision_objects):
            raise RuntimeError('Released target remains attached')
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = 'arm'
        plan.start_state = scene.robot_state
        plan.allowed_planning_time = 10.
        plan.num_planning_attempts = 3
        plan.max_velocity_scaling_factor = .15
        plan.max_acceleration_scaling_factor = .15
        names = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        plan.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name=name,
            position=value, tolerance_above=.001, tolerance_below=.001, weight=1.)
            for name, value in zip(names, [0., 0., -1.57, -1.57, 0.])])]
        response = pick.call(GetMotionPlan, '/plan_kinematic_path', request).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError('Collision-checked stow rejected: ' + str(response.error_code.val))
        pick.execute(response.trajectory)
        print('release_and_stow_complete', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
