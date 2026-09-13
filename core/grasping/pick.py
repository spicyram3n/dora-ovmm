"""Pick up an object with MoveIt Task Constructor (MTC).

SAM3 segments the object and GraspGenX proposes grasps. One MTC task opens
the hand before moving to pregrasp. Calibrated pad placement sets the final grasp.
The hand closes in bounded position steps until both finger springs register
contact. Auto mode test-lifts cylinders and holds other geometry without lift.

Needs move_group (launch/move_group.launch.py), SAM3 and GraspGenX running,
and Nav2 paused so it does not fight the base. Visual servoing is disabled.
Exit code 0 requires observed object motion during a short test lift;
exit code 3 reports a contact-only hold. Other failures return 1.

    python3 -m core.grasping.pick "pringles can"
"""

import sys
import tempfile
import time
import math
import os
import argparse
from pathlib import Path

import numpy as np
import cv2
import rclcpp  # the C++ node MTC runs on (py_binding_tools), separate from rclpy
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from moveit.task_constructor import core, stages
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from rclpy.action import ActionClient
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rclpy.parameter import Parameter as NodeParameter
from rclpy.wait_for_message import wait_for_message
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from tmc_control_msgs.action import GripperApplyEffort

from core.grasping import graspgenx_client
from core.grasping.contact_geometry import contact_candidates, verify_object_lift
from core.grasping.pregrasp_routes import pregrasp_routes
from core.grasping.visual_servo import feature
from core.perception import pointcloud, sam3_client
from core.perception.camera_ros2 import BASE_FRAME, grab_rgbd, RobotTransforms

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "launch"))
import planning_model  # noqa: E402

# The 5-DOF arm alone cannot reach most grasp orientations; the base adds three.
GROUP = "whole_body"
HAND = "hand_palm_link"
FINGERS = [
    f"hand_{side}_{part}_link"
    for side in "lr"
    for part in ("proximal", "spring_proximal", "mimic_distal", "distal")
]
TARGET = "target"
GRIPPER = "hsrc_hand"
CANDIDATES = 20  # best-scored GraspGenX grasps offered to MTC
APPROACH = 0.08  # m, straight final approach along the palm's z axis
TEST_LIFT = 0.03  # verify physical motion before accepting a hold
MAX_CLOUD_POINTS = 8192  # bounds GraspGenX GPU memory
OPEN_HAND = 1.1  # hand_motor_joint; wider breaks the distal finger limit
GRASP_LIFT_MAX = 0.65  # reserve travel for approach and the verification lift
CLOSE_EFFORT = -0.3  # Nm, negative closes
# Fingertips closer than this hold nothing: 0 cm empty, 2.1 cm around a pringles
# can. The motor angle cannot tell; the springs let it close past contact.
EMPTY_GAP = 0.01
BASE_SLACK = {
    "odom_x": 0.1,
    "odom_y": 0.1,
    "odom_t": 0.2,
}  # m, m, rad around parked base
JOINT_STATES = "/whole_body_moveit/joint_states"  # includes the base's odom joints


class GeometryOccluded(RuntimeError):
    def __init__(self,message,centre):
        super().__init__(message)
        self.centre=centre


def aim_head(node,pan,tilt):
    goal=FollowJointTrajectory.Goal()
    goal.trajectory.joint_names=['head_pan_joint','head_tilt_joint']
    goal.trajectory.points=[JointTrajectoryPoint(positions=[float(pan),float(tilt)],
                                                time_from_start=Duration(sec=2))]
    result=run_action(node,FollowJointTrajectory,'/head_trajectory_controller/follow_joint_trajectory',goal,30.)
    if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code!=0:
        raise RuntimeError('Head did not reach its observation pose')


def reframe_target(node,poses,centre):
    pivot=poses.transform('head_pan_link')[:3,3]
    base=poses.transform('base_footprint')
    yaw=math.atan2(base[1,0],base[0,0])
    offset=centre-pivot
    pan=math.atan2(offset[1],offset[0])-yaw
    pan=math.atan2(math.sin(pan),math.cos(pan))
    # Same HSRC camera-height approximation used by navigation.look_at.
    tilt=math.atan2(offset[2]-.248,np.linalg.norm(offset[:2]))
    if not (-3.84<=pan<=1.75 and -1.57<=tilt<=.52):
        raise RuntimeError('Target is outside the head observation workspace')
    aim_head(node,pan,tilt)


def foreground_occlusion(depth, mask, support_mask=None):
    """Fraction of the silhouette adjacent to a distinctly nearer surface."""
    mask=np.asarray(mask,bool)
    outer=cv2.dilate(mask.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)&~mask
    # Use interior target pixels: mixed depth on the segmentation boundary
    # can otherwise make the target itself appear to be a foreground blocker.
    interior=cv2.erode(mask.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
    if not interior.any():
        interior=mask
    _, nearest=distance_transform_edt(~interior,return_indices=True)
    target_depth=depth[nearest[0],nearest[1]]
    valid=outer&np.isfinite(depth)&(depth>0)&np.isfinite(target_depth)&(target_depth>0)
    if support_mask is not None:
        valid &= ~support_mask
    if not valid.any():
        return 0.
    return float(np.mean(depth[valid] < target_depth[valid]-.03))


def support_pixels(depth,k,transform,target):
    """Identify a broad horizontal supporting surface from measured depth."""
    v,u=np.indices(depth.shape)
    x=(u-k[0,2])*depth/k[0,0];y=(v-k[1,2])*depth/k[1,1]
    xyz=np.stack([x,y,depth],axis=-1)@transform[:3,:3].T+transform[:3,3]
    sample=xyz[::4,::4].reshape(-1,3)
    centre=np.median(target,axis=0);bottom=np.quantile(target[:,2],.01)
    valid=np.isfinite(sample).all(axis=1)&(np.linalg.norm(sample[:,:2]-centre[:2],axis=1)<.35)
    valid &= (sample[:,2]>=bottom-.25)&(sample[:,2]<=bottom+.015)
    sample=sample[valid]
    if len(sample)<100:
        return np.zeros(depth.shape,bool)
    bins=np.floor(sample[:,2]/.01).astype(int)
    values=np.unique(bins)
    # From a low camera, only the broad table edge may be visible, so require
    # lateral span rather than a fully visible two-dimensional tabletop patch.
    for value in values[::-1]:
        plane=sample[bins==value]
        if len(plane)<100 or np.max(np.ptp(plane[:,:2],axis=0))<.30:
            continue
        height=np.median(plane[:,2])
        return np.abs(xyz[:,:,2]-height)<.015
    return np.zeros(depth.shape,bool)


def observe(prompt):
    rgb, depth, k, odom_from_camera = grab_rgbd()
    mask, score = sam3_client.detect(rgb, prompt)
    points = pointcloud.deproject(depth, k, pointcloud.object_depth_mask(depth, mask))
    if len(points) < 100:
        raise RuntimeError(f"only {len(points)} depth points on the {prompt}")
    _, image_point, area = feature(depth, k, mask)
    v,u=np.nonzero(mask)
    clipped=bool(u.min()<3 or v.min()<3 or u.max()>=mask.shape[1]-3 or v.max()>=mask.shape[0]-3)
    world_points=pointcloud.transform_points(odom_from_camera, points)
    support=support_pixels(depth,k,odom_from_camera,world_points)
    return dict(points=world_points,
                camera=odom_from_camera[:3, 3], score=score, area=area,
                clipped=clipped, occlusion=foreground_occlusion(depth,mask,support),
                feature=odom_from_camera[:3, :3] @ image_point + odom_from_camera[:3, 3])


def merge_target_views(views):
    """Combine overlapping static-object observations in odom, not image space."""
    merged = views[0]['points'].copy()
    for view in views[1:]:
        points = view['points']
        forward = cKDTree(merged).query(points)[0]
        reverse = cKDTree(points).query(merged)[0]
        if min(np.mean(forward < .012), np.mean(reverse < .012)) < .15:
            raise RuntimeError('Head views do not overlap on the same static target')
        merged = np.vstack([merged, points])
    # Equal spatial weighting prevents a close/dense view dominating the fit.
    _, indices = np.unique(np.floor(merged/.002).astype(np.int64), axis=0, return_index=True)
    result = dict(views[0])
    result['points'] = merged[indices]
    return result


def observe_geometry(node, prompt):
    """Recover complementary target surfaces with a bounded head-only scan."""
    joints = joint_positions(node)
    pan, tilt = joints['head_pan_joint'], joints['head_tilt_joint']
    views = [observe(prompt)]
    def aim(value):
        aim_head(node,pan,value)
    try:
        for offset in [-.20, .20]:
            angle = float(np.clip(tilt+offset, -1.57, .52))
            if abs(angle-tilt) < .01:
                continue
            aim(angle)
            try:
                view = observe(prompt)
            except sam3_client.ObjectNotFound:
                continue
            # Reject a wrong instance immediately rather than expanding the hull.
            try:
                merge_target_views(views+[view])
            except RuntimeError:
                continue
            views.append(view)
    finally:
        aim(tilt)
    if len(views)<2:
        raise RuntimeError('Could not obtain complementary target depth views')
    result = merge_target_views(views)
    if all(view['clipped'] for view in views) or any(view['occlusion']>.08 for view in views):
        raise GeometryOccluded('Head scan remains clipped or contains foreground-occluded geometry',
                               (result['points'].min(0)+result['points'].max(0))/2)
    extent = np.ptp(result['points'],axis=0)
    print(f'[GEOMETRY] {len(views)} head views, extent {np.round(extent*1000,1)} mm',flush=True)
    return result


def perceive(prompt, node=None):
    """Generate candidates, then place calibrated pads using observed geometry."""
    observation = observe_geometry(node,prompt) if node is not None else observe(prompt)
    points = observation['points']
    sample = points[np.random.choice(len(points), min(len(points), MAX_CLOUD_POINTS), replace=False)]
    centre = sample.mean(axis=0)
    poses, scores = graspgenx_client.generate(sample - centre, GRIPPER)
    poses[:, :3, 3] += centre
    best = np.argsort(scores)[::-1]
    envelope, grasps, widths, kind = contact_candidates(points, poses[best], observation['camera'])
    observation['geometry_kind'] = kind
    print(f"{prompt}: SAM3 {observation['score']:.2f}, {len(points)} points, "
          f"{kind}, {len(grasps)} aperture-compatible grasps", flush=True)
    return envelope, grasps[:CANDIDATES], widths[:CANDIDATES], observation


def wait(node, future, what, seconds=30.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise RuntimeError(f"{what} timed out")
    return future.result()


def call(node, kind, name, request):
    client = node.create_client(kind, name)
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"{name} unavailable; is move_group running?")
    return wait(node, client.call_async(request), name)


def joint_positions(node):
    received, message = wait_for_message(
        JointState, node, JOINT_STATES, time_to_wait=5.0
    )
    if not received:
        raise RuntimeError(f"no {JOINT_STATES}")
    return dict(zip(message.name, message.position))


def fingertip_gap(node):
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841, fills buffer while spun
    tips = ("hand_l_finger_tip_frame", "hand_r_finger_tip_frame", Time())
    deadline = time.monotonic() + 5.0
    while not buffer.can_transform(*tips):
        if time.monotonic() > deadline:
            raise RuntimeError("no fingertip TF")
        rclpy.spin_once(node, timeout_sec=0.1)
    gap = buffer.lookup_transform(*tips).transform.translation
    return float(np.linalg.norm([gap.x, gap.y, gap.z]))


def model_target(node, points):
    """Put the target in move_group's scene as a box, replacing the last one.

    The depth updater leaves modelled objects out of new frames, so clearing
    the octomap rebuilds it without the target's voxels but with everything
    around it, such as the table the target stands on."""
    lower, upper = points.min(axis=0), points.max(axis=0)
    box = CollisionObject(id=TARGET, operation=CollisionObject.ADD)
    box.header.frame_id = BASE_FRAME
    box.pose.position.x, box.pose.position.y, box.pose.position.z = (
        (lower + upper) / 2
    ).tolist()
    box.primitives = [
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=(upper - lower).tolist())
    ]
    box.pose.orientation.w = 1.0
    primitive_pose = Pose()
    primitive_pose.orientation.w = 1.0
    box.primitive_poses = [primitive_pose]
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    # An empty id detaches whatever a previous pick left in the hand.
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(
            object=CollisionObject(operation=CollisionObject.REMOVE)
        )
    ]
    scene.world.collision_objects = [box]
    if not call(
        node,
        ApplyPlanningScene,
        "/apply_planning_scene",
        ApplyPlanningScene.Request(scene=scene),
    ).success:
        raise RuntimeError("move_group rejected the target box")
    depth_relay(node, True)
    call(node, Empty, "/clear_octomap", Empty.Request())
    # The updater uses simulation time; three wall seconds may contain no
    # update. Require actual map data after clearing, then check the start.
    deadline = time.monotonic() + 30.0
    while True:
        snapshot = call(node, GetPlanningScene, '/get_planning_scene',
                        GetPlanningScene.Request(components=PlanningSceneComponents(
                            components=PlanningSceneComponents.ROBOT_STATE |
                            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                            PlanningSceneComponents.OCTOMAP))).scene
        if snapshot.world.octomap.octomap.data:
            break
        if time.monotonic() > deadline:
            raise RuntimeError('No fresh depth map after clearing OctoMap')
        time.sleep(.25)
    valid = call(node, GetStateValidity, '/check_state_validity',
                 GetStateValidity.Request(robot_state=snapshot.robot_state,
                                          group_name=GROUP))
    if not valid.valid:
        raise RuntimeError('Robot overlaps the rebuilt collision scene; refusing motion')
    # Freeze it: move_group stops a running plan when new voxels (a new view
    # or the moving arm itself) appear on its path.
    depth_relay(node, False)


def depth_relay(node, enabled):
    """Start or stop the depth frames feeding move_group's octomap."""
    value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)
    request = SetParameters.Request(parameters=[Parameter(name="enabled", value=value)])
    call(node, SetParameters, "/octomap_depth_camera/set_parameters", request)


def run_action(node, kind, name, goal, seconds):
    """Send goal to the action server name; return its result once finished."""
    client = ActionClient(node, kind, name)
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"{name} unavailable")
        handle = wait(node, client.send_goal_async(goal), f"{name} goal")
        if not handle.accepted:
            raise RuntimeError(f"{name} rejected the goal")
        try:
            return wait(node, handle.get_result_async(), name, seconds)
        except BaseException:
            # Includes Ctrl+C: wait for cancellation acknowledgement before
            # dropping the client, rather than leaving an action moving.
            cancellation = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancellation, timeout_sec=5.0)
            raise
    finally:
        client.destroy()


def close_hand(node):
    effort = float(os.environ.get('HSR_GRASP_EFFORT_NM', str(CLOSE_EFFORT)))
    if not np.isfinite(effort) or not -.3 <= effort <= -.012:
        raise ValueError('HSR_GRASP_EFFORT_NM must be between -0.3 and -0.012 Nm')
    print(f'[GRASP] closing effort {effort:.3f} Nm', flush=True)
    goal = GripperApplyEffort.Goal(effort=effort)
    closed = run_action(node, GripperApplyEffort, "/gripper_controller/grasp", goal, 30.0)
    if closed.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError("the hand did not close")


def mtc_node():
    """An rclcpp node carrying move_group's robot model and planner settings."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml") as params:
        yaml.safe_dump(
            {"/**": {"ros__parameters": {**planning_model.moveit_params(), "use_sim_time": True}}}, params
        )
        params.flush()
        options = rclcpp.NodeOptions()
        options.arguments = ["--ros-args", "--params-file", params.name]
        options.automatically_declare_parameters_from_overrides = True
        return rclcpp.Node("mtc_pick", options)


def stamped(frame, matrix=np.eye(4)):
    pose = PoseStamped()
    pose.header.frame_id = frame
    position, orientation = pose.pose.position, pose.pose.orientation
    position.x, position.y, position.z = matrix[:3, 3].tolist()
    orientation.x, orientation.y, orientation.z, orientation.w = (
        Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
    )
    return pose


def move_hand(name, planner, direction, frame, min_distance, max_distance):
    """A straight hand motion along direction, of at least min_distance."""
    stage = stages.MoveRelative(name, planner)
    stage.group = GROUP
    stage.ik_frame = stamped(HAND)
    stage.min_distance = min_distance
    stage.max_distance = max_distance
    vector = Vector3Stamped()
    vector.header.frame_id = frame
    vector.vector.x, vector.vector.y, vector.vector.z = direction
    stage.setDirection(vector)
    return stage


class Pick:
    def __init__(self, node):
        self.node = node
        self.ompl = None
        self.cartesian = core.CartesianPath()
        self.cartesian.step_size = 0.005  # enough steps for MoveIt's joint-jump check
        for planner in (self.cartesian,):
            planner.max_velocity_scaling_factor = 0.15
            planner.max_acceleration_scaling_factor = 0.15

    def task(self, name):
        task = core.Task()
        task.name = name
        task.loadRobotModel(self.node)
        # PipelinePlanner caches a pipeline bound to the first task's model.
        # Humble can reload that model between tasks, so do not reuse planners.
        self.ompl = core.PipelinePlanner(self.node, 'move_group')
        self.ompl.max_velocity_scaling_factor = .15
        self.ompl.max_acceleration_scaling_factor = .15
        task.add(stages.CurrentState("current state"))
        return task

    def reach(self, grasps, base):
        """Plan pregrasp and complete approach together before executing either."""
        task = self.task("reach")
        # A previous attempt may have left target contact enabled in the scene.
        # Transit and opening must check the target as well as the table.
        protect = stages.ModifyPlanningScene("check target during pregrasp")
        protect.allowCollisions(TARGET, FINGERS, False)
        task.add(protect)
        if abs(base.get("hand_motor_joint", 0.0) - OPEN_HAND) > 0.005:
            open_hand = stages.MoveTo("open hand before pregrasp", self.ompl)
            open_hand.group = "gripper"
            open_hand.setGoal({"hand_motor_joint": OPEN_HAND})
            task.add(open_hand)
        # Keep the base near where Nav2 parked it.
        parked = Constraints(
            joint_constraints=[
                JointConstraint(
                    joint_name=joint,
                    position=base[joint],
                    tolerance_above=slack,
                    tolerance_below=slack,
                    weight=1.0,
                )
                for joint, slack in BASE_SLACK.items()
            ]
        )
        parked.joint_constraints.append(JointConstraint(
            joint_name='arm_lift_joint',position=GRASP_LIFT_MAX/2,
            tolerance_above=GRASP_LIFT_MAX/2,tolerance_below=GRASP_LIFT_MAX/2,
            weight=1.))
        back_off = np.eye(4)
        back_off[2, 3] = -APPROACH
        # Best-scored first; stop at the first grasp that works.
        candidates = core.Fallbacks("grasp candidates")
        for index, grasp in enumerate(grasps):
            candidate = core.SerialContainer(f"grasp {index}")
            if grasp[2, 2] < -.9:
                clearance = stages.MoveTo('unfold arm for overhead reach', self.ompl)
                clearance.group = 'arm'
                clearance.setGoal(dict(zip(
                    ['arm_lift_joint','arm_flex_joint','arm_roll_joint',
                     'wrist_flex_joint','wrist_roll_joint'], [.35,-.5,-1.57,-1.07,0.])))
                clearance.timeout = 10.
                candidate.insert(clearance)
            pregrasp = pregrasp_routes(
                self.ompl, grasp @ back_off, parked,
                [base['odom_x'], base['odom_y']], GROUP, HAND, BASE_FRAME)
            candidate.insert(pregrasp)
            allow = stages.ModifyPlanningScene("allow finger contact")
            allow.allowCollisions(TARGET, FINGERS, True)
            candidate.insert(allow)
            candidate.insert(move_hand("complete approach", self.cartesian,
                                       (0., 0., 1.), HAND, APPROACH, APPROACH))
            candidates.insert(candidate)
        task.add(candidates)
        return task

    def approach(self):
        task = self.task("approach")
        task.add(move_hand("complete approach", self.cartesian, (0., 0., 1.),
                           HAND, APPROACH, APPROACH))
        return task

    def correct_endpoint(self, grasp):
        task = self.task('correct grasp endpoint')
        move = stages.MoveTo('bounded endpoint correction', self.cartesian)
        move.group = GROUP
        move.ik_frame = stamped(HAND)
        move.setGoal(stamped(BASE_FRAME, grasp))
        task.add(move)
        return task

    def clear_view(self):
        task=self.task('clear arm from target observation')
        home=stages.MoveTo('observation home',self.ompl)
        home.group='arm'
        home.setGoal(dict(zip(['arm_lift_joint','arm_flex_joint','arm_roll_joint',
                              'wrist_flex_joint','wrist_roll_joint'],[0.,0.,-1.57,-1.57,0.])))
        home.timeout=10.
        task.add(home)
        return task

    def lift(self):
        """Attach the closed-on object to the hand and raise it."""
        task = self.task("lift")
        attach = stages.ModifyPlanningScene("attach target")
        attach.attachObject(TARGET, HAND)
        task.add(attach)
        task.add(
            move_hand("lift", self.cartesian, (0.0, 0.0, 1.0), BASE_FRAME, TEST_LIFT, TEST_LIFT)
        )
        return task


def execute(node, task):
    """Plan task and have move_group execute the best solution.

    Sent through rclpy: task.execute() once reported an abort while
    move_group went on to execute the motion."""
    if not task.plan(max_solutions=1):
        raise RuntimeError(f"MTC found no {task.name} plan")
    solution = task.solutions[0].toMsg()
    from core.grasping.scene_payload import compact
    compact(solution)
    # Preserve the collision snapshot throughout execution. Replacing it with
    # empty maps can erase the scene used by the next manipulation stage.
    size = len(serialize_message(solution))
    print(f'[MTC] execution payload {size/1024/1024:.2f} MiB', flush=True)
    if size > 32*1024*1024:
        raise RuntimeError('Task collision snapshot exceeds the 32 MiB execution budget')
    goal = ExecuteTaskSolution.Goal(solution=solution)
    done = run_action(node, ExecuteTaskSolution, "/execute_task_solution", goal, 300.0)
    if done.result.error_code.val != 1:
        raise RuntimeError(
            f"executing {task.name} failed: MoveIt error {done.result.error_code.val}"
        )


def clear_observation_pose(node, planner):
    """Use one collision-map snapshot during the camera-clearance motion."""
    depth_relay(node, False)
    try:
        execute(node, planner.clear_view())
    finally:
        depth_relay(node, True)


def pick(prompt, mode='auto'):
    if mode not in ('auto', 'pickup', 'grasp'):
        raise ValueError('Unknown grasp mode')
    rclpy.init()
    rclcpp.init()
    node = rclpy.create_node("pick", parameter_overrides=[NodeParameter("use_sim_time", value=True)])
    try:
        poses = RobotTransforms(node)
        planner = Pick(mtc_node())
        if joint_positions(node)['arm_lift_joint'] > GRASP_LIFT_MAX:
            previous=observe(prompt)['points']
            print('[GRASP] restoring arm-lift margin before replanning',flush=True)
            clear_observation_pose(node,planner)
            reframe_target(node,poses,(previous.min(0)+previous.max(0))/2)
        try:
            points, grasps, widths, observation = perceive(prompt,node)
        except GeometryOccluded as blocked:
            print('[GEOMETRY] foreground blocks target; planning one return to observation home',flush=True)
            clear_observation_pose(node,planner)
            reframe_target(node,poses,blocked.centre)
            points, grasps, widths, observation = perceive(prompt,node)
        # Small explicit trial adjustment, in the grasp approach frame. Zero
        # remains the calibration baseline; do not bake one object's tuning in.
        is_can = observation.get('geometry_kind') == 'cylinder'
        lift_requested = mode == 'pickup' or (mode == 'auto' and is_can)
        insertion = float(os.environ.get('HSR_GRASP_INSERTION_M', '.01' if is_can else '0'))
        if not np.isfinite(insertion) or not 0 <= insertion <= .01:
            raise ValueError('HSR_GRASP_INSERTION_M must be between 0 and 0.01 m')
        if insertion:
            grasps = grasps.copy()
            grasps[:, :3, 3] += np.minimum(insertion, .2*widths)[:, None]*grasps[:, :3, 2]
            print(f'[GRASP] trial forward insertion up to {insertion*1000:.1f} mm', flush=True)
        diagnostics = os.environ.get('HSR_GRASP_DIAGNOSTICS')
        if diagnostics:
            Path(diagnostics).mkdir(parents=True, exist_ok=True)
            np.savez(Path(diagnostics)/'planned_grasps.npz', points=points, grasps=grasps, widths=widths)
        model_target(node, points)
        execute(node, planner.reach(grasps, joint_positions(node)))
        reached = poses.transform()
        distances = np.linalg.norm(grasps[:, :3, 3]-reached[:3, 3], axis=1)
        angles = Rotation.from_matrix(
            grasps[:, :3, :3].transpose(0, 2, 1) @ reached[:3, :3]).magnitude()
        index = int(np.argmin(distances + .1*angles))
        if .004 < distances[index] <= .01 and angles[index] <= .04:
            print(f'[GRASP] correcting {distances[index]*1000:.1f} mm endpoint error', flush=True)
            execute(node, planner.correct_endpoint(grasps[index]))
            reached = poses.transform()
            distances[index] = np.linalg.norm(grasps[index,:3,3]-reached[:3,3])
            angles[index] = Rotation.from_matrix(grasps[index,:3,:3].T@reached[:3,:3]).magnitude()
        if distances[index] > .004 or angles[index] > .04:
            raise RuntimeError(f'Palm did not reach the calibrated grasp pose '
                               f'({distances[index]*1000:.1f} mm, {angles[index]:.3f} rad); refusing closure')
        print(f"[GRASP] reached candidate {index}, contact width {widths[index]*1000:.1f} mm", flush=True)
        from core.grasping.contact_closure import close, contact_state, wait_for_hold
        close(node, threshold=.14 if lift_requested else .10)
        gap = fingertip_gap(node)
        if gap < EMPTY_GAP:
            raise RuntimeError("the hand closed on nothing")
        if not lift_requested:
            print('[GRASPED] bilateral contact; holding without lift', flush=True)
            return 'grasped'
        before_lift = poses.transform()
        execute(node, planner.lift())
        after_lift = poses.transform()
        measured = observe(prompt)
        if diagnostics:
            np.savez(Path(diagnostics)/'lift_observation.npz', before=observation['points'],
                     after=measured['points'], expected=after_lift[:3,3]-before_lift[:3,3])
        evidence = verify_object_lift(observation['points'], measured['points'],
                                      after_lift[:3,3]-before_lift[:3,3])
        print(f"[LIFT] {prompt}: object rose {evidence['rise_m']*1000:.1f} mm; checking hold", flush=True)
        def read_contact():
            q = joint_positions(node)
            rclpy.spin_once(node, timeout_sec=0.)  # update the /clock subscription
            return tuple(q[f'hand_{s}_spring_proximal_joint'] for s in 'lr')
        wait_for_hold(read_contact, lambda: node.get_clock().now().nanoseconds / 1e9)
        held = observe(prompt)
        if diagnostics:
            np.savez(Path(diagnostics)/'hold_observation.npz', before=observation['points'],
                     after=held['points'], expected=after_lift[:3,3]-before_lift[:3,3])
        evidence = verify_object_lift(observation['points'], held['points'],
                                      after_lift[:3,3]-before_lift[:3,3])
        if not contact_state(*read_contact(), .06):
            raise RuntimeError('Object slipped during final camera verification')
        print(f"[VERIFIED] {prompt}: sustained contact and {evidence['rise_m']*1000:.1f} mm lift", flush=True)
        return 'picked'
    except (RuntimeError, ValueError) as error:
        print(f"pick failed: {error}")
        return False
    finally:
        try:
            depth_relay(node, True)  # the arm is done; let the octomap follow the world
        except RuntimeError as error:
            print(f"octomap left frozen: {error}")
        node.destroy_node()
        rclpy.shutdown()
        rclcpp.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target')
    parser.add_argument('--mode', choices=['auto', 'pickup', 'grasp'], default='auto')
    args = parser.parse_args()
    outcome = pick(args.target, args.mode)
    # Distinguish contact-only success from a verified pickup for the mission.
    raise SystemExit(3 if outcome == 'grasped' else 0 if outcome else 1)
