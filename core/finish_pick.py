"""Feedback closure and a checked short lift for a depth-fitted upright can.

Run after move_to_grasp with stop_before_close=true. This is a bounded can
experiment, not a general object controller. No simulator object names or
poses enter closure, planning, or RGB-D verification.
"""
import argparse
import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, JointConstraint, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint

from perception.camera_ros2 import grab_rgbd
from perception import pointcloud, sam3_client
from grasping.can_alignment import fit_upright_can


def support_contact(contact, delta, geometry):
    return (set((contact.contact_body_1, contact.contact_body_2)) == {'grasp_target', '<octomap>'}
            and delta <= .005 and contact.depth <= .0015
            and geometry['min_z'] - .01 <= contact.position.z <= geometry['min_z'] + .005
            and math.hypot(contact.position.x - geometry['center_x'],
                           contact.position.y - geometry['center_y']) <= geometry['radius'] + .015)


def primitive_center(target):
    """MoveIt may normalize a primitive's world pose into CollisionObject.pose."""
    origin = target.pose
    rotation = Rotation.from_quat([getattr(origin.orientation, axis) for axis in 'xyzw'])
    local = target.primitive_poses[0].position
    return np.array([getattr(origin.position, a) for a in 'xyz']) + rotation.apply(
        [getattr(local, a) for a in 'xyz'])


class FinishPick:
    def __init__(self, node, geometry, output, prompt='can'):
        self.node, self.geometry, self.output = node, geometry, output
        self.prompt = prompt
        self.clients = {}
        self.observation_offset = np.zeros(3)

    def wait(self, future, seconds=30):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def call(self, kind, name, request):
        if name not in self.clients:
            self.clients[name] = self.node.create_client(kind, name)
        client = self.clients[name]
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(name + ' unavailable')
        return self.wait(client.call_async(request))

    def scene(self):
        request = GetPlanningScene.Request()
        request.components.components = 2 | 4 | 16 | 32 | 128
        return self.call(GetPlanningScene, '/get_planning_scene', request).scene

    def execute(self, trajectory):
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                 for p in trajectory.joint_trajectory.points]
        if len(times) < 2 or not all(b > a for a, b in zip(times, times[1:])):
            raise RuntimeError('Incomplete or untimed trajectory')
        client = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        try:
            if not client.wait_for_server(timeout_sec=5):
                raise RuntimeError('Execution server unavailable')
            handle = self.wait(client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)))
            if not handle.accepted:
                raise RuntimeError('Execution rejected')
            future = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=max(90, times[-1] * 3))
            if not future.done():
                self.wait(handle.cancel_goal_async())
                raise RuntimeError('Execution timed out; cancellation requested')
            if future.result().result.error_code.val != 1:
                raise RuntimeError('Execution failed')
        finally:
            client.destroy()

    def close_step(self, motor):
        if not .30 <= motor <= 1.10:
            raise RuntimeError('Motor command outside bounded closure range')
        if not hasattr(self, 'mimics'):
            description = self.call(GetParameters, '/move_group/get_parameters',
                                     GetParameters.Request(names=['robot_description'])).values[0].string_value
            self.mimics = [(joint.attrib['name'], joint.find('mimic').attrib)
                           for joint in ET.fromstring(description).findall('joint')
                           if joint.find('mimic') is not None]
        current = self.scene().robot_state
        js = current.joint_state
        initial = dict(zip(js.name, js.position))
        start = initial['hand_motor_joint']
        duration = max(3., abs(motor - start) / .025)
        # Quintic peak slope is 1.875. Keep motor increments below .005 rad,
        # less than 1 mm of calibrated pad travel, with all mimic joints set.
        steps = max(20, math.ceil(abs(motor-start)*1.875/.005))
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ['hand_motor_joint']
        for index in range(steps+1):
            alpha = index/steps
            position = start + (motor-start)*(10*alpha**3 - 15*alpha**4 + 6*alpha**5)
            values = initial.copy()
            values['hand_motor_joint'] = position
            for _ in range(len(self.mimics)):
                for name, mimic in self.mimics:
                    if mimic['joint'] in values:
                        values[name] = (float(mimic.get('multiplier', 1))*values[mimic['joint']]
                                        + float(mimic.get('offset', 0)))
            checked = copy.deepcopy(current)
            checked.joint_state.position = [float(values[name]) for name in js.name]
            validity = self.call(GetStateValidity, '/check_state_validity',
                                 GetStateValidity.Request(robot_state=checked))
            if not validity.valid:
                raise RuntimeError('Closure sweep collision: ' + str([
                    (c.contact_body_1, c.contact_body_2, c.depth) for c in validity.contacts]))
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.velocities = [(motor-start)/duration*30*alpha**2*(1-alpha)**2]
            point.accelerations = [(motor-start)/duration**2*(60*alpha - 180*alpha**2 + 120*alpha**3)]
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(alpha*duration*1e9), 1000000000)
            trajectory.joint_trajectory.points.append(point)
        print(json.dumps({'checked_closure_samples': steps+1, 'motor_goal': motor}), flush=True)
        self.execute(trajectory)

    def observe_can(self):
        """Try fresh checked head views when the open hand obscures the side."""
        errors = []
        for view in (None, (0., -.55), (.25, -.7), (-.25, -.7)):
            if view is not None:
                state = self.scene().robot_state
                values = dict(zip(state.joint_state.name, state.joint_state.position))
                if values['hand_motor_joint'] < .8:
                    raise RuntimeError('View recovery requires an open hand')
                names = ['head_pan_joint', 'head_tilt_joint']
                start = np.array([values[n] for n in names])
                delta = np.array(view)-start
                if np.max(np.abs(delta)) > .6:
                    continue
                trajectory = RobotTrajectory()
                trajectory.joint_trajectory.joint_names = names
                for i in range(41):
                    a = i/40.
                    position = start+delta*(10*a**3-15*a**4+6*a**5)
                    checked = copy.deepcopy(state)
                    updated = values.copy()
                    updated.update(zip(names, position))
                    checked.joint_state.position = [float(updated[n]) for n in state.joint_state.name]
                    valid = self.call(GetStateValidity, '/check_state_validity',
                                      GetStateValidity.Request(robot_state=checked))
                    if not valid.valid:
                        raise RuntimeError('Head view recovery path is in collision')
                    point = JointTrajectoryPoint(positions=position.tolist(),
                        velocities=(delta/4*30*a*a*(1-a)**2).tolist(),
                        accelerations=(delta/16*(60*a-180*a*a+120*a**3)).tolist())
                    point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(a*4e9), 1000000000)
                    trajectory.joint_trajectory.points.append(point)
                self.execute(trajectory)
                print(json.dumps({'view_recovery': view}), flush=True)
            rgb, depth, k, transform = grab_rgbd()
            mask, _ = sam3_client.detect(rgb, self.prompt)
            cloud = pointcloud.transform_points(transform, pointcloud.deproject(
                depth, k, pointcloud.object_depth_mask(depth, mask)))
            try:
                return fit_upright_can(cloud)
            except ValueError as error:
                errors.append(str(error))
                print(f'Can view rejected: {error}', flush=True)
        raise RuntimeError('No reliable contact view after bounded head recovery: ' + '; '.join(errors))

    def refine_contact(self):
        """Correct <=1 cm residual using fresh vision and the reached palm FK."""
        registration_dir = Path(__file__).resolve().parent.parent / 'docker/graspgenx/x_grippers/hsrc_hand'
        registration = json.loads((registration_dir / 'config.json').read_text())
        profile = sorted(json.loads((registration_dir / 'closing_profile.json').read_text()),
                         key=lambda sample: sample['gap'][0])
        for iteration in range(3):
            current = self.scene()
            js = current.robot_state.joint_state
            joints = dict(zip(js.name, js.position))
            if joints['hand_motor_joint'] < .8:
                raise RuntimeError('Contact refinement requires open fingers')
            center, radius, _ = self.observe_can()
            current = self.scene()
            g = self.geometry
            if (np.linalg.norm(center - [g['center_x'], g['center_y']]) > .01
                    or abs(radius - g['radius']) > .002):
                raise RuntimeError('Target moved beyond bounded contact refinement')
            request = GetPositionFK.Request(robot_state=current.robot_state,
                                            fk_link_names=['hand_palm_link'])
            request.header.frame_id = 'odom'
            fk = self.call(GetPositionFK, '/compute_fk', request)
            if fk.error_code.val != 1:
                raise RuntimeError('Palm FK unavailable')
            pose = fk.pose_stamped[0].pose
            rotation = Rotation.from_quat([getattr(pose.orientation, a) for a in 'xyzw']).as_matrix()
            palm = np.array([getattr(pose.position, a) for a in 'xyz'])
            pad = np.array([np.interp(2*radius, [s['gap'][0] for s in profile],
                                      [s['center'][a] for s in profile]) for a in range(3)])
            measured_pad = palm + rotation @ np.asarray(registration['base_rotation'])[:3, :3].T @ pad
            correction = np.r_[center - measured_pad[:2], 0.]
            print(json.dumps({'refinement': iteration, 'correction_m': correction.tolist()}), flush=True)
            if np.linalg.norm(correction) > .01:
                raise RuntimeError('Required contact correction exceeds 1 cm')
            if np.linalg.norm(correction) <= .002:
                return
            if iteration == 2:
                raise RuntimeError('Contact centering did not converge')
            names = ['odom_x', 'odom_y', 'odom_t', 'arm_lift_joint', 'arm_flex_joint',
                     'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
            trajectory = RobotTrajectory()
            trajectory.joint_trajectory.joint_names = names
            previous = copy.deepcopy(current.robot_state)
            for index in range(21):
                checked = copy.deepcopy(previous)
                if index:
                    request = GetPositionIK.Request()
                    ik = request.ik_request
                    ik.group_name = 'whole_body'
                    ik.ik_link_name = 'hand_palm_link'
                    ik.robot_state = previous
                    ik.avoid_collisions = True
                    ik.timeout.sec = 1
                    ik.pose_stamped = PoseStamped(pose=copy.deepcopy(pose))
                    ik.pose_stamped.header.frame_id = 'odom'
                    for axis, value in zip('xyz', palm + correction * index/20):
                        setattr(ik.pose_stamped.pose.position, axis, float(value))
                    ik.constraints.joint_constraints = [JointConstraint(joint_name=name,
                        position=joints[name], tolerance_above=.01, tolerance_below=.01, weight=1.)
                        for name in names[:3]]
                    response = self.call(GetPositionIK, '/compute_ik', request)
                    if response.error_code.val != 1:
                        raise RuntimeError('Collision-aware refinement IK failed')
                    values = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
                    for name in names:
                        checked.joint_state.position[js.name.index(name)] = values[name]
                    old = dict(zip(previous.joint_state.name, previous.joint_state.position))
                    if any(not np.isfinite(values[name]) or abs(values[name] - old[name]) > .04 for name in names):
                        raise RuntimeError('Refinement joint jump rejected')
                    if any(abs(values[name] - joints[name]) > .0101 for name in names[:3]):
                        raise RuntimeError('Refinement base bound exceeded')
                for fraction in (.5, 1.):
                    sample = copy.deepcopy(previous)
                    sample.joint_state.position = [float(a + fraction*(b-a)) for a, b in
                        zip(previous.joint_state.position, checked.joint_state.position)]
                    if not self.call(GetStateValidity, '/check_state_validity',
                                     GetStateValidity.Request(robot_state=sample)).valid:
                        raise RuntimeError('Refinement path collision rejected')
                point = JointTrajectoryPoint()
                point.positions = [checked.joint_state.position[js.name.index(name)] for name in names]
                point.velocities = [0.] * len(names)
                point.accelerations = [0.] * len(names)
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(index*500000000, 1000000000)
                trajectory.joint_trajectory.points.append(point)
                previous = checked
            self.execute(trajectory)

    def close(self):
        profile_path = Path(__file__).resolve().parent.parent / 'docker/graspgenx/x_grippers/hsrc_hand/closing_profile.json'
        profile = sorted(json.loads(profile_path.read_text()), key=lambda sample: sample['gap'][0])
        # Take up free space in one checked slow motion, leaving 2 cm more
        # aperture than the measured diameter before contact-sized steps.
        preclose = float(np.interp(2*self.geometry['radius'] + .02,
                                  [s['gap'][0] for s in profile], [s['motor'] for s in profile]))
        for step in range(34):
            js = self.scene().robot_state.joint_state
            joints = dict(zip(js.name, js.position))
            motor = joints['hand_motor_joint']
            springs = [joints['hand_' + side + '_spring_proximal_joint'] for side in ('l', 'r')]
            print(json.dumps({'closure_step': step, 'motor': motor, 'springs': springs}), flush=True)
            if not np.isfinite([motor, *springs]).all() or max(springs) > .20:
                raise RuntimeError('Invalid or excessive finger deflection')
            if min(springs) >= .08 and abs(springs[0] - springs[1]) <= .06:
                return
            if motor <= .301:
                raise RuntimeError('Bilateral grip not established within bounded closure')
            goal = preclose if max(springs) < .005 and motor > preclose + .025 else motor - .025
            self.close_step(max(.30, goal))
        raise RuntimeError('Closure step limit reached')

    def observe_top(self, label, rise=0.):
        rgb, depth, k, transform = grab_rgbd()
        points = pointcloud.transform_points(transform, pointcloud.deproject(
            depth, k, np.isfinite(depth) & (depth > 0)))
        g = self.geometry
        radial = np.hypot(points[:, 0] - g['center_x'] - self.observation_offset[0],
                          points[:, 1] - g['center_y'] - self.observation_offset[1])
        expected = g['max_z'] + rise
        selected = points[(radial < g['radius'] + .002)
                          & (points[:, 2] > expected - .015) & (points[:, 2] < expected + .008)]
        np.savez_compressed(self.output / (label + '.npz'), rgb=rgb, depth=depth, k=k,
                            transform=transform, top_points=selected)
        if len(selected) < 30:
            raise RuntimeError('Target upper surface is occluded; pickup unverified')
        height = float(np.quantile(selected[:, 2], .98))
        print(json.dumps({'observation': label, 'top_height': height, 'points': len(selected)}), flush=True)
        return height

    def retreat(self, distance):
        """Withdraw along odom -X, the test's initial forward viewing axis.

        This is a local, bounded shelf extraction, not a navigation planner.
        Every robot and held-object sample must be strictly collision-free.
        """
        if distance == 0:
            return
        current = self.scene()
        if not any(a.object.id == 'grasp_target' for a in current.robot_state.attached_collision_objects):
            raise RuntimeError('Retreat requires the held target collision model')
        names = ['odom_x', 'odom_y', 'odom_t']
        js = current.robot_state.joint_state
        joints = dict(zip(js.name, js.position))
        springs = [joints['hand_' + side + '_spring_proximal_joint'] for side in ('l', 'r')]
        if not np.isfinite(springs).all() or min(springs) < .05 or max(springs) > .20:
            raise RuntimeError('Grip feedback lost before withdrawal')
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = names
        for index in range(201):
            alpha = index/200.
            delta = -distance * (10*alpha**3 - 15*alpha**4 + 6*alpha**5)
            checked = copy.deepcopy(current.robot_state)
            checked.joint_state.position[js.name.index('odom_x')] += delta
            validity = self.call(GetStateValidity, '/check_state_validity',
                                 GetStateValidity.Request(robot_state=checked))
            if not validity.valid:
                raise RuntimeError('Shelf withdrawal collision: ' + str([
                    (c.contact_body_1, c.contact_body_2, c.depth) for c in validity.contacts]))
            point = JointTrajectoryPoint()
            point.positions = [joints['odom_x'] + delta, joints['odom_y'], joints['odom_t']]
            point.velocities = [-distance/12 * 30*alpha**2*(1-alpha)**2, 0., 0.]
            point.accelerations = [-distance/144 * (60*alpha - 180*alpha**2 + 120*alpha**3), 0., 0.]
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(index*60000000, 1000000000)
            trajectory.joint_trajectory.points.append(point)
        print(json.dumps({'withdrawal_m': distance, 'checked_samples': 201}), flush=True)
        self.execute(trajectory)
        self.observation_offset[0] -= distance

    def lift(self, distance):
        current = self.scene()
        if current.robot_state.attached_collision_objects:
            raise RuntimeError('An object is already attached; refusing another pickup')
        target = next((o for o in current.world.collision_objects if o.id == 'grasp_target'), None)
        if target is None or target.header.frame_id != 'odom' or len(target.primitives) != 1:
            raise RuntimeError('Expected separated target is missing')
        g = self.geometry
        center = primitive_center(target)
        expected = [g['center_x'], g['center_y'], (g['min_z'] + g['max_z']) / 2]
        if np.max(np.abs(center - expected)) > .002:
            raise RuntimeError('Planning target differs from fresh perception')
        attachment = AttachedCollisionObject(link_name='hand_palm_link', object=copy.deepcopy(target))
        attachment.object.operation = CollisionObject.ADD
        attachment.touch_links = ['hand_' + side + '_' + part + '_link'
                                  for side in ('l', 'r') for part in ('distal', 'spring_proximal')]
        update = ApplyPlanningScene.Request()
        update.scene.is_diff = True
        update.scene.robot_state.is_diff = True
        update.scene.robot_state.attached_collision_objects = [attachment]
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', update).success:
            raise RuntimeError('Planning attachment failed')
        executing = False
        try:
            current = self.scene()
            js = current.robot_state.joint_state
            joints = dict(zip(js.name, js.position))
            if joints['arm_lift_joint'] + distance >= .69:
                raise RuntimeError('Insufficient arm lift travel')
            names = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
            trajectory = RobotTrajectory()
            trajectory.joint_trajectory.joint_names = names
            allowed = 0
            for index in range(101):
                alpha = index / 100.
                delta = distance * (10*alpha**3 - 15*alpha**4 + 6*alpha**5)
                checked = copy.deepcopy(current.robot_state)
                checked.joint_state.position[js.name.index('arm_lift_joint')] += delta
                validity = self.call(GetStateValidity, '/check_state_validity',
                                     GetStateValidity.Request(robot_state=checked))
                if not validity.valid:
                    if not validity.contacts or not all(support_contact(c, delta, g) for c in validity.contacts):
                        raise RuntimeError('Lift collision at sample ' + str(index) + ': ' + str([
                            (c.contact_body_1, c.contact_body_2, c.depth) for c in validity.contacts]))
                    allowed += 1
                point = JointTrajectoryPoint()
                point.positions = [joints[n] + (delta if n == 'arm_lift_joint' else 0.) for n in names]
                point.velocities = [distance/8 * 30*alpha**2*(1-alpha)**2, 0., 0., 0., 0.]
                point.accelerations = [distance/64 * (60*alpha - 180*alpha**2 + 120*alpha**3), 0., 0., 0., 0.]
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(index*80000000, 1000000000)
                trajectory.joint_trajectory.points.append(point)
            print(json.dumps({'lift_m': distance, 'checked_samples': 101, 'support_samples': allowed}), flush=True)
            executing = True
            self.execute(trajectory)
        finally:
            if not executing:
                update = ApplyPlanningScene.Request()
                update.scene.is_diff = True
                update.scene.robot_state.is_diff = True
                remove = AttachedCollisionObject(link_name='hand_palm_link')
                remove.object.id = 'grasp_target'
                remove.object.operation = CollisionObject.REMOVE
                update.scene.robot_state.attached_collision_objects = [remove]
                update.scene.world.collision_objects = [target]
                self.call(ApplyPlanningScene, '/apply_planning_scene', update)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('grasp_file', type=Path)
    parser.add_argument('--lift', type=float, default=.02)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--prompt', default='can')
    parser.add_argument('--grasp-index', type=int, default=0,
                        help='Executed aligned candidate identity for bounded tracking recovery')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--resume-closed', action='store_true',
                        help='Retry a stopped lift while retaining the established grip')
    parser.add_argument('--retreat', type=float, default=0., help='Checked withdrawal along odom -X, up to .25 m')
    parser.add_argument('--resume-lifted', action='store_true',
                        help='Withdraw/verify an already lifted target without another lift')
    args = parser.parse_args()
    if not .01 <= args.lift <= .05:
        parser.error('Lift must be between 1 and 5 cm')
    if not 0 <= args.retreat <= .25 or (args.resume_closed and args.resume_lifted):
        parser.error('Retreat must be 0 to .25 m; choose at most one recovery mode')
    data = yaml.safe_load(args.grasp_file.read_text())
    if data['frame_id'] != 'odom' or not data.get('target_geometry'):
        parser.error('A depth-fitted upright cylinder in odom is required')
    if not args.execute:
        print('No motion requested. Use --execute after reaching the open-hand contact pose.')
        return
    output = args.output_dir or args.grasp_file.parent / 'pickup'
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('feedback_can_pickup')
    result = {'verified': False, 'lift_m': args.lift}
    try:
        pick = FinishPick(node, data['target_geometry'], output, args.prompt)
        initial_scene = pick.scene()
        if args.resume_lifted:
            if not any(o.object.id == 'grasp_target' for o in initial_scene.robot_state.attached_collision_objects):
                raise RuntimeError('No lifted planning target to resume')
            before = data['target_geometry']['max_z']
            result['baseline'] = 'saved pregrasp RGB-D target top'
        else:
            if initial_scene.robot_state.attached_collision_objects or not any(
                    o.id == 'grasp_target' for o in initial_scene.world.collision_objects):
                raise RuntimeError('Reach the separated target with an open hand first')
            if not args.resume_closed:
                try:
                    pick.refine_contact()
                except RuntimeError as error:
                    if str(error) != 'Required contact correction exceeds 1 cm' or not args.grasp_index:
                        raise
                    from grasp_recovery import correct_tracking
                    correct_tracking(pick, data, args.grasp_index)
                    pick.refine_contact()
            before = pick.observe_top('before')
            pick.close()
            pick.lift(args.lift)
        pick.retreat(args.retreat)
        heights = [pick.observe_top('hold_' + str(i), args.lift) for i in range(3)]
        result.update(before_top=before, held_tops=heights)
        result['verified'] = bool(min(heights) - before >= args.lift - .006
                                  and max(heights) - min(heights) < .005)
        if not result['verified']:
            raise RuntimeError('RGB-D did not verify the requested pickup and hold')
        print('RGBD_PICKUP_VERIFIED', json.dumps(result), flush=True)
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
