"""Optional robot-facing staging route to an overhead pregrasp."""
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit.task_constructor import core, stages
from scipy.spatial.transform import Rotation


def side_waypoint(overhead, base_xy, distance=.12):
    """Keep overhead height/orientation, offset horizontally toward the robot."""
    if overhead[2, 2] > -.9:
        return None
    toward = np.asarray(base_xy)-overhead[:2, 3]
    norm = np.linalg.norm(toward)
    if norm < .01:
        return None
    side = overhead.copy()
    side[:2, 3] += min(distance, norm/2)*toward/norm
    return side


def pregrasp_routes(planner, overhead, parked, base_xy, group, hand, frame):
    def move(name, matrix, timeout=3.):
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = matrix[:3, 3].tolist()
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q.tolist()
        stage = stages.MoveTo(name, planner)
        stage.group = group
        ik = PoseStamped(); ik.header.frame_id = hand; ik.pose.orientation.w = 1.
        stage.ik_frame = ik
        stage.setGoal(goal)
        stage.path_constraints = parked
        stage.timeout = timeout
        return stage

    side = side_waypoint(overhead, base_xy)
    if side is None:
        return move('pregrasp', overhead)
    routes = core.Fallbacks('overhead pregrasp routes')
    staged = core.SerialContainer('via robot-facing side')
    staged.insert(move('side staging', side, timeout=10.))
    staged.insert(move('move above target', overhead, timeout=10.))
    routes.insert(staged)
    routes.insert(move('direct overhead pregrasp', overhead))
    return routes
