"""Checked local base recovery; no navigation, attachment, or contact bypass."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint
from grasping.recovery_geometry import execution_grasps, planar_tracking_delta


def candidates(data):
    root = Path(__file__).resolve().parent.parent / 'docker/graspgenx/x_grippers/hsrc_hand'
    return execution_grasps(data, json.loads((root/'config.json').read_text()),
                            json.loads((root/'closing_profile.json').read_text()))


def checked_base_motion(pick, delta, duration=15.):
    delta = np.asarray(delta, dtype=float)
    if (delta.shape != (3,) or not np.isfinite(delta).all()
            or np.linalg.norm(delta[:2]) > .25 or abs(delta[2]) > .15):
        raise RuntimeError('Local recovery displacement exceeds limits')
    state = pick.scene().robot_state
    if state.attached_collision_objects:
        raise RuntimeError('Base recovery requires an empty gripper')
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    if values['hand_motor_joint'] < .8:
        raise RuntimeError('Base recovery requires open fingers')
    names = ['odom_x', 'odom_y', 'odom_t']
    start = np.array([values[n] for n in names])
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = names
    for i in range(201):
        a = i/200.
        pos = start + delta*(10*a**3-15*a**4+6*a**5)
        checked = copy.deepcopy(state)
        current = values.copy()
        current.update(zip(names, pos))
        checked.joint_state.position = [float(current[n]) for n in state.joint_state.name]
        valid = pick.call(GetStateValidity, '/check_state_validity',
                          GetStateValidity.Request(robot_state=checked))
        if not valid.valid:
            raise RuntimeError('Base recovery collision: ' + str([
                (c.contact_body_1, c.contact_body_2) for c in valid.contacts]))
        point = JointTrajectoryPoint(positions=pos.tolist(),
            velocities=(delta/duration*30*a*a*(1-a)**2).tolist(),
            accelerations=(delta/duration**2*(60*a-180*a*a+120*a**3)).tolist())
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(a*duration*1e9), 1000000000)
        trajectory.joint_trajectory.points.append(point)
    print(json.dumps({'checked_base_recovery': delta.tolist(), 'samples': 201}), flush=True)
    pick.execute(trajectory)
    feedback = pick.scene().robot_state.joint_state
    measured = dict(zip(feedback.name, feedback.position))
    error = np.array([measured[n] for n in names]) - (start+delta)
    error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
    if np.linalg.norm(error[:2]) > .003 or abs(error[2]) > .005:
        raise RuntimeError('Base recovery did not track its endpoint')


def recover_reachability(pick, data):
    scene = pick.scene()
    if any(o.id == 'grasp_target' for o in scene.world.collision_objects):
        raise RuntimeError('No base-search recovery after target separation')
    state = scene.robot_state
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    if abs(values['arm_flex_joint']) > .05:
        raise RuntimeError('Reachability recovery requires a stowed arm')
    base = np.array([values[k] for k in ['odom_x', 'odom_y', 'odom_t']])
    options = []
    for pose, score in candidates(data):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = 'whole_body'
        ik.ik_link_name = 'hand_palm_link'
        ik.robot_state = state
        ik.avoid_collisions = True
        ik.timeout.sec = 1
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = data['frame_id']
        pos = pose[:3, 3] - .08*pose[:3, 2]
        for k, value in zip('xyz', pos):
            setattr(ik.pose_stamped.pose.position, k, float(value))
        for k, value in zip('xyzw', Rotation.from_matrix(pose[:3, :3]).as_quat()):
            setattr(ik.pose_stamped.pose.orientation, k, float(value))
        result = pick.call(GetPositionIK, '/compute_ik', request)
        if result.error_code.val != 1:
            continue
        solved = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        delta = np.array([solved[k] for k in ['odom_x', 'odom_y', 'odom_t']])-base
        delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
        distance = np.linalg.norm(delta[:2])
        if .02 <= distance <= .25 and abs(delta[2]) <= .15:
            options.append((distance+.1*abs(delta[2])+.001*(1-score), delta))
    for _, delta in sorted(options, key=lambda item: item[0])[:5]:
        # A collision rejection occurs before any motion. Execution errors stop.
        try:
            checked_base_motion(pick, delta)
        except RuntimeError as error:
            if str(error).startswith('Base recovery collision:'):
                continue
            raise
        print('BASE_RECOVERY_COMPLETE: recapture and regenerate grasps', flush=True)
        return
    raise RuntimeError('No bounded collision-free base recovery found')


def correct_tracking(pick, data, index):
    poses = candidates(data)
    if not 1 <= index <= len(poses):
        raise RuntimeError('Missing executed grasp identity')
    state = pick.scene().robot_state
    request = GetPositionFK.Request(robot_state=state, fk_link_names=['hand_palm_link'])
    request.header.frame_id = data['frame_id']
    result = pick.call(GetPositionFK, '/compute_fk', request)
    if result.error_code.val != 1:
        raise RuntimeError('Tracking FK unavailable')
    pose = result.pose_stamped[0].pose
    actual = np.eye(4)
    actual[:3, :3] = Rotation.from_quat([getattr(pose.orientation,k) for k in 'xyzw']).as_matrix()
    actual[:3, 3] = [getattr(pose.position,k) for k in 'xyz']
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    delta = planar_tracking_delta(actual, poses[index-1][0],
                                   [values[k] for k in ['odom_x','odom_y','odom_t']])
    if np.linalg.norm(delta[:2]) > .0005 or abs(delta[2]) > .001:
        checked_base_motion(pick, delta, duration=12.)
        print('TRACKING_RECOVERY_COMPLETE', flush=True)


def main():
    from finish_pick import FinishPick
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('grasp_file', type=Path)
    args = parser.parse_args()
    data = yaml.safe_load(args.grasp_file.read_text())
    if data['frame_id'] != 'odom' or not data.get('target_geometry'):
        raise RuntimeError('Recovery requires a fitted target in odom')
    rclpy.init()
    node = rclpy.create_node('local_grasp_recovery')
    try:
        recover_reachability(FinishPick(node, data['target_geometry'], args.grasp_file.parent), data)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
