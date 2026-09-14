"""Recover a home-arm robot from an inflated costmap cell after manipulation."""
import math
import numpy as np
from nav2_msgs.srv import GetCostmap


def recover(navigator):
    request = GetCostmap.Request()
    cm = navigator._lifecycle_call(GetCostmap, '/global_costmap/get_costmap', request).map
    x, y, yaw = navigator.robot_pose()
    grid = np.asarray(cm.data).reshape(cm.metadata.size_y, cm.metadata.size_x)
    ox, oy = cm.metadata.origin.position.x, cm.metadata.origin.position.y
    resolution = cm.metadata.resolution

    def cost(px, py):
        row, col = math.floor((py-oy)/resolution), math.floor((px-ox)/resolution)
        return int(grid[row, col]) if 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] else 255

    if cost(x, y) < 253:
        return
    if cost(x, y) != 253:
        raise RuntimeError('Navigation start is lethal or unknown; cannot recover automatically')
    candidates = []
    for radius in [.05, .10, .15, .20, .25]:
        for angle in np.linspace(0, 2*np.pi, 32, endpoint=False):
            dx, dy = radius*np.cos(angle), radius*np.sin(angle)
            if cost(x+dx, y+dy) >= 200:
                continue
            if any(cost(x+dx+.10*np.cos(a), y+dy+.10*np.sin(a)) >= 253
                   for a in np.linspace(0, 2*np.pi, 16, endpoint=False)):
                continue
            if any(cost(x+t*dx, y+t*dy) >= 254 for t in np.linspace(0, 1, 12)):
                continue
            candidates.append((radius, cost(x+dx, y+dy), dx, dy))
    if not candidates:
        raise RuntimeError('No nearby free cell for manipulation-to-navigation recovery')

    from core.grasping import pick as p
    from rclpy.time import Time
    q = p.joint_positions(navigator)
    if abs(q['arm_flex_joint']) > .05 or q['arm_lift_joint'] > .05:
        raise RuntimeError('Navigation recovery requires the arm home')
    snapshot = p.call(navigator, p.GetPlanningScene, '/get_planning_scene',
                     p.GetPlanningScene.Request(components=p.PlanningSceneComponents(components=1023))).scene
    if snapshot.robot_state.attached_collision_objects or not snapshot.world.octomap.octomap.data:
        raise RuntimeError('Recovery requires an empty hand and a populated collision map')
    if not p.call(navigator, p.GetStateValidity, '/check_state_validity',
                  p.GetStateValidity.Request(robot_state=snapshot.robot_state, group_name=p.GROUP)).valid:
        raise RuntimeError('Recovery start is in collision')
    navigator.pause_navigation()
    p.depth_relay(navigator, False)
    p.rclcpp.init()
    try:
        radius, _, dx, dy = min(candidates)
        tf = navigator.tf_buffer.lookup_transform('odom', 'map', Time()).transform.rotation
        delta = p.Rotation.from_quat([tf.x, tf.y, tf.z, tf.w]).apply([dx, dy, 0])
        planner = p.Pick(p.mtc_node())
        task = planner.task('recover navigation clearance')
        step = p.stages.MoveTo('nearby free costmap cell', planner.ompl)
        step.group = p.GROUP
        names = ['odom_x', 'odom_y', 'odom_t', 'arm_lift_joint', 'arm_flex_joint',
                 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        goal = {name: q[name] for name in names}
        goal['odom_x'] += float(delta[0]); goal['odom_y'] += float(delta[1])
        step.setGoal(goal); step.timeout = 10.; task.add(step)
        p.execute(navigator, task)
        px, py, _ = navigator.robot_pose()
        if np.hypot(px-x-dx, py-y-dy) > .04 or cost(px, py) >= 253:
            raise RuntimeError('Navigation clearance recovery did not reach its goal')
        print(f'[RECOVERY] moved {radius:.2f} m out of inflated start cell', flush=True)
    finally:
        p.depth_relay(navigator, True)
        p.rclcpp.shutdown()
    navigator.resume_navigation_if_paused()
