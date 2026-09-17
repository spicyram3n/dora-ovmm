"""Check what a real-robot grasp needs before moving anything.

    python3 realrobot/live/grasp_preflight.py            # robot, RX and model servers
    python3 realrobot/live/grasp_preflight.py --moveit   # also this PC's move_group

Run it in the container with ROS_DOMAIN_ID and CYCLONEDDS_URI set for the
robot, RX up, and the model servers up. Each line is one check; the exit code
is 1 if a required one failed. Nothing here commands the robot: subscribing to
RX's topics is what makes TX start streaming, so the first images take a
moment.
"""

import argparse
import json
import os
import sys
import time

os.environ.setdefault("HSR_REAL_ROBOT", "1")
# realrobot/live/<this file>, so three levels up is the repository root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import rclpy  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from control_msgs.msg import JointTrajectoryControllerState  # noqa: E402
from moveit_task_constructor_msgs.action import ExecuteTaskSolution  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_srvs.srv import Empty  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from tmc_control_msgs.action import GripperApplyEffort  # noqa: E402

from core.perception.camera_ros2 import (  # noqa: E402
    BASE_FRAME, CAMERA_INFO_TOPIC, DEPTH_TOPIC, RGB_TOPIC,
)

CAMERA_FRAME = "head_rgbd_sensor_rgb_frame"
# core/grasping/pick.py rejects transforms older than this.
MAX_TF_AGE = 0.5
TF_WAIT = 20.0
ACTIONS = [
    ("/arm_trajectory_controller/follow_joint_trajectory", FollowJointTrajectory),
    ("/head_trajectory_controller/follow_joint_trajectory", FollowJointTrajectory),
    ("/gripper_controller/follow_joint_trajectory", FollowJointTrajectory),
    ("/gripper_controller/grasp", GripperApplyEffort),
]
MODELS = {"sam3": 7447, "graspgenx": 7448}


class Report:
    def __init__(self):
        self.failed = False

    def line(self, ok, name, detail, required=True):
        mark = "ok  " if ok else ("FAIL" if required else "warn")
        self.failed |= required and not ok
        print(f"[{mark}] {name}: {detail}", flush=True)


def latest(node, kind, topic, seconds, qos=qos_profile_sensor_data, count=1):
    """Collect up to `count` messages from topic within `seconds`."""
    received = []
    subscription = node.create_subscription(kind, topic, received.append, qos)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and len(received) < count:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(subscription)
    return received


def age_of(node, stamp):
    return (node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9


def check_joint_states(node, report):
    messages = latest(node, JointState, "/joint_states", 5.0)
    if not messages:
        report.line(False, "/joint_states", "nothing in 5 s; DDS domain, interface or peer wrong?")
        return
    names = set(messages[0].name)
    missing = {"arm_lift_joint", "hand_motor_joint", "head_pan_joint"} - names
    report.line(not missing, "/joint_states", f"{len(names)} joints, age {age_of(node, messages[0].header.stamp):.2f} s"
                + (f", missing {sorted(missing)}" if missing else ""))
    base = latest(node, JointTrajectoryControllerState, "/omni_base_controller/state", 5.0, qos=1)
    report.line(bool(base), "/omni_base_controller/state", "feeds odom_x/y/t into the planning model"
                if base else "nothing in 5 s; the whole_body group cannot plan without it")


def check_tf(node, report):
    buffer = Buffer()
    TransformListener(buffer, node)
    # Static transforms are latched, but the camera chain crosses three extra
    # calibration publishers on the robot, and their history reaches a new
    # listener over the LAN several seconds after discovery.
    started = time.monotonic()
    deadline = started + TF_WAIT
    for child in (CAMERA_FRAME, "hand_palm_link"):
        while time.monotonic() < deadline and not buffer.can_transform(BASE_FRAME, child, Time()):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not buffer.can_transform(BASE_FRAME, child, Time()):
            report.line(False, f"tf {BASE_FRAME} -> {child}", f"not available in {TF_WAIT:.0f} s")
            continue
        transform = buffer.lookup_transform(BASE_FRAME, child, Time())
        age = age_of(node, transform.header.stamp)
        report.line(abs(age) <= MAX_TF_AGE, f"tf {BASE_FRAME} -> {child}",
                    f"after {time.monotonic() - started:.1f} s, age {age:+.3f} s against this PC's clock "
                    f"(limit {MAX_TF_AGE} s; beyond it, sync the clocks with chrony/NTP)")


def check_camera(node, report):
    infos = latest(node, CameraInfo, CAMERA_INFO_TOPIC, 5.0)
    if infos:
        info = infos[0]
        report.line(info.header.frame_id == CAMERA_FRAME, CAMERA_INFO_TOPIC,
                    f"{info.width}x{info.height}, frame {info.header.frame_id}, fx {info.k[0]:.1f}")
    else:
        report.line(False, CAMERA_INFO_TOPIC, "nothing in 5 s; must reach this PC over DDS")
    for topic, kind in ((RGB_TOPIC, "rgb"), (DEPTH_TOPIC, "depth")):
        # Subscribing registers demand with RX; TX then starts encoding.
        images = latest(node, Image, topic, 15.0, count=10)
        if not images:
            report.line(False, topic, "nothing in 15 s; is RX up, and TX reachable from it?")
            continue
        first, last = images[0], images[-1]
        span = (Time.from_msg(last.header.stamp) - Time.from_msg(first.header.stamp)).nanoseconds / 1e9
        rate = (len(images) - 1) / span if span > 0 else 0.0
        expected = ("bgr8", "rgb8") if kind == "rgb" else ("16UC1", "32FC1")
        matched = (not infos) or (last.width, last.height, last.header.frame_id) == (
            infos[0].width, infos[0].height, infos[0].header.frame_id)
        report.line(last.encoding in expected and matched, topic,
                    f"{last.width}x{last.height} {last.encoding}, frame {last.header.frame_id}, "
                    f"{rate:.1f} Hz, age {age_of(node, last.header.stamp):.2f} s"
                    + ("" if matched else "; does not match camera_info"))


def check_actions(node, report):
    for name, kind in ACTIONS:
        client = ActionClient(node, kind, name)
        ok = client.wait_for_server(timeout_sec=5.0)
        client.destroy()
        report.line(ok, name, "action server up" if ok else "no action server in 5 s")


def check_moveit(node, report):
    client = node.create_client(Empty, "/clear_octomap")
    ok = client.wait_for_service(timeout_sec=5.0)
    node.destroy_client(client)
    report.line(ok, "move_group", "/clear_octomap answers" if ok else "not running (launch/realrobot/grasp_real.launch.py)")
    executor = ActionClient(node, ExecuteTaskSolution, "/execute_task_solution")
    ok = executor.wait_for_server(timeout_sec=5.0)
    executor.destroy()
    report.line(ok, "/execute_task_solution", "MTC capability loaded" if ok else "missing")
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(GetParameters, "/octomap_depth_camera/get_parameters")
    if client.wait_for_service(timeout_sec=5.0):
        future = client.call_async(GetParameters.Request(names=["depth_topic", "info_topic", "sensor_data_qos"]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        values = future.result().values if future.done() else []
        wired = len(values) == 3 and values[0].string_value == DEPTH_TOPIC and values[2].bool_value
        report.line(wired, "depth relay", ", ".join(
            [values[0].string_value, values[1].string_value, f"sensor_data_qos={values[2].bool_value}"]
            if len(values) == 3 else ["parameters unreadable"]))
    else:
        report.line(False, "depth relay", "octomap_depth_camera not running")
    node.destroy_client(client)


def check_models(report):
    import zenoh
    endpoints = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447,tcp/127.0.0.1:7448").split(",")
    for name, port in MODELS.items():
        endpoint = next((e for e in endpoints if e.endswith(f":{port}")), f"tcp/127.0.0.1:{port}")
        config = zenoh.Config()
        config.insert_json5("connect/endpoints", json.dumps([endpoint]))
        config.insert_json5("scouting/multicast/enabled", "false")
        ready = False
        try:
            with zenoh.open(config) as session:
                for reply in session.get(f"{name}/health", timeout=3):
                    ready = bool(reply.ok) and reply.ok.payload.to_bytes() == b"ready"
        except Exception as error:  # zenoh raises on an unreachable endpoint
            detail = f"{endpoint}: {error}"
        else:
            detail = f"{endpoint}: {'ready' if ready else 'no ready reply in 3 s'}"
        report.line(ready, name, detail)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--moveit", action="store_true", help="also check this PC's move_group and depth relay")
    parser.add_argument("--no-models", action="store_true", help="skip the SAM3 and GraspGenX probes")
    args = parser.parse_args()
    report = Report()
    print(f"domain {os.environ.get('ROS_DOMAIN_ID', '0')}, "
          f"{os.environ.get('CYCLONEDDS_URI', 'default CycloneDDS config')}", flush=True)
    rclpy.init()
    node = rclpy.create_node("grasp_preflight", parameter_overrides=[Parameter("use_sim_time", value=False)])
    try:
        check_joint_states(node, report)
        check_tf(node, report)
        check_camera(node, report)
        check_actions(node, report)
        if args.moveit:
            check_moveit(node, report)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not args.no_models:
        check_models(report)
    print("NOT READY: fix the FAIL lines before picking" if report.failed else "READY", flush=True)
    return 1 if report.failed else 0


if __name__ == "__main__":
    sys.exit(main())
