"""Pick up an object with MoveIt Task Constructor (MTC).

SAM3 segments the object and GraspGenX proposes grasps. One MTC task opens the
hand and reaches the best reachable grasp; the hand then closes with the
gripper's effort action, because a position trajectory aborts when the fingers
stall on the object; a second MTC task attaches the object and lifts it.

Needs move_group (launch/move_group.launch.py), SAM3 and GraspGenX running,
and Nav2 paused so it does not fight the base. Exit code 0 means held.

    python3 -m core.grasping.pick "pringles can"
"""

import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclcpp  # the C++ node MTC runs on (py_binding_tools), separate from rclpy
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from moveit.task_constructor import core, stages
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from octomap_msgs.msg import OctomapWithPose
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.wait_for_message import wait_for_message
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tmc_control_msgs.action import GripperApplyEffort

from core.grasping import graspgenx_client
from core.perception import pointcloud, sam3_client
from core.perception.camera_ros2 import BASE_FRAME, grab_rgbd

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
# Where the pads close on a 1-8 cm object, in front of the palm (closing profile).
PAD_DEPTH = 0.075
MAX_CLOUD_POINTS = 8192  # bounds GraspGenX GPU memory
OPEN_HAND = 1.1  # hand_motor_joint; wider breaks the distal finger limit
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


def perceive(prompt):
    """Return the object's points and its best grasps (palm poses), in odom."""
    rgb, depth, k, odom_from_camera = grab_rgbd()
    mask, score = sam3_client.detect(rgb, prompt)
    points = pointcloud.deproject(depth, k, pointcloud.object_depth_mask(depth, mask))
    if len(points) < 100:
        raise RuntimeError(f"only {len(points)} depth points on the {prompt}")
    sample = points[
        np.random.choice(len(points), min(len(points), MAX_CLOUD_POINTS), replace=False)
    ]
    centre = sample.mean(axis=0)
    poses, scores = graspgenx_client.generate(sample - centre, GRIPPER)
    poses[:, :3, 3] += centre
    best = np.argsort(scores)[::-1][:CANDIDATES]
    print(
        f"{prompt}: SAM3 score {score:.2f}, {len(points)} points, best grasp {scores[best[0]]:.2f}"
    )
    points = solid(points)
    # GraspGenX stops with the object at the fingertips. Slide each grasp along
    # its approach axis (palm z) until the object's centre is between the pads.
    grasps = poses[best]
    approach = grasps[:, :3, 2]
    to_centre = np.einsum("ij,ij->i", points.mean(axis=0) - grasps[:, :3, 3], approach)
    grasps[:, :3, 3] += (to_centre - PAD_DEPTH)[:, None] * approach
    return (
        pointcloud.transform_points(odom_from_camera, points),
        pointcloud.transform_poses(odom_from_camera, grasps),
    )


def solid(points):
    """The camera sees only an object's front. Assume it is as deep as it is
    wide and mirror that surface behind it (camera frame: x right, z depth)."""
    front = np.percentile(points[:, 2], 2)
    left, right = np.percentile(points[:, 0], [2, 98])
    back = points.copy()
    back[:, 2] = 2 * front + (right - left) - points[:, 2]
    return np.vstack([points, back])


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
    box.primitive_poses = [Pose()]
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    # An empty id detaches whatever a previous pick left in the hand.
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(
            object=CollisionObject(operation=CollisionObject.REMOVE)
        )
    ]
    scene.world.collision_objects = [box]
    # Empty the octomap before applying anything, or move_group dies here.
    #
    # Applying a diff makes move_group republish monitored_planning_scene with
    # the whole world in it, octomap included. At the 1 cm resolution
    # config/moveit/sensors_xtion.yaml asks for, a map of everywhere the robot
    # has driven reaches ~110 MB -- past CycloneDDS's 100 MB MaxSampleSize in
    # .devcontainer/cyclonedds_sim.xml. The publish raises
    # rclcpp::exceptions::RCLError("failed to publish data") on a publisher
    # thread where nothing catches it, so move_group aborts with SIGABRT and
    # every later call reports "/apply_planning_scene timed out".
    #
    # The clear below is the one that matters to planning: it rebuilds the map
    # around the box. This one only keeps the message on the wire small, and is
    # why the two are not redundant.
    call(node, Empty, "/clear_octomap", Empty.Request())
    if not call(
        node,
        ApplyPlanningScene,
        "/apply_planning_scene",
        ApplyPlanningScene.Request(scene=scene),
    ).success:
        raise RuntimeError("move_group rejected the target box")
    depth_relay(node, True)
    call(node, Empty, "/clear_octomap", Empty.Request())
    time.sleep(3.0)  # the octomap integrates depth at 1 Hz
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
        except RuntimeError:
            handle.cancel_goal_async()
            raise
    finally:
        client.destroy()


def close_hand(node):
    goal = GripperApplyEffort.Goal(effort=CLOSE_EFFORT)
    closed = run_action(node, GripperApplyEffort, "/gripper_controller/grasp", goal, 30.0)
    if closed.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError("the hand did not close")


def mtc_node():
    """An rclcpp node carrying move_group's robot model and planner settings."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml") as params:
        yaml.safe_dump(
            {"/**": {"ros__parameters": planning_model.moveit_params()}}, params
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
        self.ompl = core.PipelinePlanner(node, "move_group")
        self.cartesian = core.CartesianPath()
        self.cartesian.step_size = 0.005  # enough steps for MoveIt's joint-jump check
        for planner in (self.ompl, self.cartesian):
            planner.max_velocity_scaling_factor = 0.15
            planner.max_acceleration_scaling_factor = 0.15

    def task(self, name):
        task = core.Task()
        task.name = name
        task.loadRobotModel(self.node)
        task.add(stages.CurrentState("current state"))
        return task

    def reach(self, grasps, base):
        """Open the hand, then reach the first grasp that has a collision-free
        path to its pregrasp and a straight approach from there."""
        task = self.task("reach")
        # The box is only a coarse hull of the visible surface; let the fingers enter it.
        allow = stages.ModifyPlanningScene("allow finger contact")
        allow.allowCollisions(TARGET, FINGERS, True)
        task.add(allow)
        # OMPL, unlike joint interpolation, tolerates a start state slightly out
        # of bounds, as when the finger springs jitter below 0 rad in simulation.
        open_hand = stages.MoveTo("open hand", self.ompl)
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
        back_off = np.eye(4)
        back_off[2, 3] = -APPROACH
        # Best-scored first; stop at the first grasp that works.
        candidates = core.Fallbacks("grasp candidates")
        for index, grasp in enumerate(grasps):
            candidate = core.SerialContainer(f"grasp {index}")
            pregrasp = stages.MoveTo("pregrasp", self.ompl)
            pregrasp.group = GROUP
            pregrasp.ik_frame = stamped(HAND)
            pregrasp.setGoal(stamped(BASE_FRAME, grasp @ back_off))
            pregrasp.path_constraints = parked
            pregrasp.timeout = 3.0
            candidate.insert(pregrasp)
            candidate.insert(
                move_hand(
                    "approach",
                    self.cartesian,
                    (0.0, 0.0, 1.0),
                    HAND,
                    APPROACH - 0.01,
                    APPROACH,
                )
            )
            candidates.insert(candidate)
        task.add(candidates)
        return task

    def lift(self):
        """Attach the closed-on object to the hand and raise it."""
        task = self.task("lift")
        attach = stages.ModifyPlanningScene("attach target")
        attach.attachObject(TARGET, HAND)
        task.add(attach)
        task.add(
            move_hand("lift", self.cartesian, (0.0, 0.0, 1.0), BASE_FRAME, 0.03, 0.08)
        )
        return task


def execute(node, task):
    """Plan task and have move_group execute the best solution.

    Sent through rclpy: task.execute() once reported an abort while
    move_group went on to execute the motion."""
    if not task.plan(max_solutions=1):
        raise RuntimeError(f"MTC found no {task.name} plan")
    solution = task.solutions[0].toMsg()
    # The plan carries move_group's own octomap back to it, twice: 110 MB in
    # the lift, over the DDS message limit. move_group ignores an empty one.
    solution.start_scene.world.octomap = OctomapWithPose()
    for sub in solution.sub_trajectory:
        sub.scene_diff.world.octomap = OctomapWithPose()
    goal = ExecuteTaskSolution.Goal(solution=solution)
    done = run_action(node, ExecuteTaskSolution, "/execute_task_solution", goal, 300.0)
    if done.result.error_code.val != 1:
        raise RuntimeError(
            f"executing {task.name} failed: MoveIt error {done.result.error_code.val}"
        )


def pick(prompt):
    rclpy.init()
    rclcpp.init()
    node = rclpy.create_node("pick")
    try:
        points, grasps = perceive(prompt)
        model_target(node, points)
        planner = Pick(mtc_node())
        execute(node, planner.reach(grasps, joint_positions(node)))
        close_hand(node)
        execute(node, planner.lift())
        gap = fingertip_gap(node)
        hand = joint_positions(node)["hand_motor_joint"]
        print(f"fingertips {gap * 100:.1f} cm apart, hand_motor_joint {hand:.2f}")
        if gap < EMPTY_GAP:
            raise RuntimeError("the hand closed on nothing")
        print(f"holding the {prompt}")
        return True
    except RuntimeError as error:
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
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    raise SystemExit(0 if pick(sys.argv[1]) else 1)
