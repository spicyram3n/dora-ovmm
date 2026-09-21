"""Web dashboard for the behaviour-tree mission: type what to fetch, watch it run.

    python3 -m web.server
    # then open http://localhost:8090  (override with WEB_PORT=...)

Run it in its own terminal, so it outlives the sim: `ros2 launch
launch/search.launch.py web:=true` starts the stack without a mission, and the
page's queries run against it. Every argument here is passed on to
core.pipeline.mission_tree, which runs once per query in its own process. Its
[EVENT] lines (core.utils.events) feed the page; the rest of its output goes to
this terminal as before. Each run is saved to outputs/web_runs/ when it ends, with
its full log, so any past query can be opened again, even with the sim closed.

On the real robot the same page serves requests in plain language. The stack comes
up without a mission when no target is given, and this server needs the robot's
switches in its own environment, since it hands them to every mission it starts:

    export RECORDING=<name> DEEPSEEK_API_KEY=<key>
    ros2 launch /home/ws/launch/realrobot/search_real.launch.py
    HSR_REAL_ROBOT=1 python3 -m web.server --natural-language true --top-k 2 --startup-timeout 600
"""

import json
import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import rclpy
import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from flask import Flask, abort, jsonify, request, send_from_directory
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tmc_control_msgs.action import GripperApplyEffort
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener

from core.perception.camera_ros2 import RGB_TOPIC
from core.pipeline.actions import home_goal
from core.utils.events import PREFIX

ROOT = Path(__file__).resolve().parents[1]
# Local only: anyone who can open the page can drive the robot.
# Override with WEB_PORT when 8090 is taken. Stay above 1024 (binding lower
# needs root) and clear of 6665-6669, which Chrome and Firefox refuse to open.
HOST = os.environ.get("WEB_HOST", "127.0.0.1")
PORT = int(os.environ.get("WEB_PORT", "8090"))
# Every run is saved here when it ends, to inspect after the sim is gone.
RUNS = ROOT / "outputs/web_runs"
# Joint samples kept while the arm homes: enough for a smooth replay, and small.
SAMPLE_PERIOD = 0.1
# The steps that drive the base; the path and track are kept for them. Navigate-only
# drives in "Go there", the full mission inside its search and when parking.
DRIVE_STEPS = ("Go there", "Find target", "Park")
# How often the base's map pose is read, and how far apart path waypoints are drawn.
TRACK_PERIOD = 0.2
WAYPOINT_SPACING = 0.25
# A base transform that has not changed for this long means the sim stopped or restarted.
STALE_TF = 2.0
# Width the head camera image is sent at, for the page's camera window; never upscaled.
CAMERA_WIDTH = 640
# A frame older than this, in wall seconds, means the camera stopped: the sim closed or hung.
CAMERA_STALE = 2.0
# How long Reset, and quitting the server, wait for a stopped mission to exit and be saved.
STOP_TIMEOUT = 30
# Sent with every state: a page that sees it change reloads, so it never runs old code.
BOOT = time.time()

# The page: static/index.html, style.css and app.js, served at the site root.
app = Flask(__name__, static_folder="static", static_url_path="")
# The page polls /state four times a second; logging each request buries the mission's log.
logging.getLogger("werkzeug").setLevel(logging.ERROR)
lock = threading.Lock()
mission_args = sys.argv[1:]
urdf = ""
process = None
follower = None     # the thread that saves the current run when it ends
arm = None
gripper = None      # hand_motor_joint trajectory: opens the hand
grasp = None        # Toyota's effort grasp: closes it
nav_manage = None   # Nav2's lifecycle manager: pauses and resumes the motion nodes
nav_state = None    # controller_server's lifecycle state, shown on the Nav2 button
nav2 = "down"       # "active", "paused" or "down", refreshed every NAV2_PERIOD seconds
NAV2_PERIOD = 2.0
# As core/grasping/pick.py: wider breaks the distal finger limit; negative closes.
OPEN_HAND = 1.1
CLOSE_EFFORT = -0.3
tf_buffer = Buffer()
tf_listener = None  # fills tf_buffer; held here so it lives as long as the server
tf_stamp = None     # stamp of the last base transform, and when it last changed
tf_seen = 0.0
bridge = CvBridge()
image = None        # the latest head camera frame, encoded only when the page asks
image_seen = 0.0    # when it arrived, on the monotonic clock


def fresh(target=None):
    """Everything the page shows about one run."""
    return {"id": None, "target": target, "running": False, "exit": None, "tree": None, "leaf": None,
            "events": [], "homing": [], "log": "", "joints": {},
            "pose": None, "track": [], "path": []}


run = fresh()


def follow(child, current):
    """Sort the mission's output into page state and its log until it exits, then save the run."""
    # Line-buffered, so the log holds everything up to a crash of this server.
    with open(RUNS / f"{current['id']}.log", "w", buffering=1) as log:
        for line in child.stdout:
            if line.startswith(PREFIX):
                event = json.loads(line[len(PREFIX):])
                with lock:
                    if event["kind"] == "tree":
                        current["tree"] = event["tree"]
                        current["leaf"] = event["running"]
                    else:
                        current["events"].append(event)
                continue
            print(line, end="", flush=True)
            log.write(line)
            if line.strip():
                with lock:
                    current["log"] = line.strip()
    code = child.wait()
    with lock:
        current["running"] = False
        current["exit"] = code
        (RUNS / f"{current['id']}.json").write_text(json.dumps(current))


def on_joints(message):
    now = time.monotonic()
    with lock:
        run["joints"] = dict(zip(message.name, message.position))
        # Record the homing itself, so the step can be replayed after it ends.
        if run["running"] and run["leaf"] == "Home arm":
            if not run["homing"] or now - run["homing"][-1]["t"] >= SAMPLE_PERIOD:
                run["homing"].append({"t": now, "joints": run["joints"]})


def on_plan(message):
    """Nav2's global plan, thinned to waypoints; it replans once a second while driving."""
    points = []
    for stamped in message.poses:
        x, y = stamped.pose.position.x, stamped.pose.position.y
        if not points or math.hypot(x - points[-1][0], y - points[-1][1]) >= WAYPOINT_SPACING:
            points.append([x, y])
    with lock:
        if run["running"] and run["leaf"] in DRIVE_STEPS:
            run["path"] = points


def on_image(message):
    global image, image_seen
    image, image_seen = message, time.monotonic()


def on_tick():
    """The base's map pose, recorded while driving so the drive can be replayed."""
    global tf_stamp, tf_seen
    try:
        stamped = tf_buffer.lookup_transform("map", "base_footprint", Time())
    except TransformException:
        return
    now = time.monotonic()
    if stamped.header.stamp != tf_stamp:
        tf_stamp, tf_seen = stamped.header.stamp, now
    elif now - tf_seen > STALE_TF:
        # A restarted sim's clock starts over, and the buffer would refuse every new
        # transform as older than the last one it holds.
        tf_buffer.clear()
        return
    transform = stamped.transform
    q = transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    pose = [transform.translation.x, transform.translation.y, yaw]
    with lock:
        run["pose"] = pose
        if run["running"] and run["leaf"] in DRIVE_STEPS:
            run["track"].append({"t": time.monotonic(), "pose": pose})


@app.get("/")
def index():
    return app.send_static_file("index.html")


@app.get("/state")
def state():
    with lock:
        return jsonify(dict(run, boot=BOOT, camera_topic=RGB_TOPIC, nav2=nav2))


def take_over():
    """The operator's click outranks the mission: stop a running one and wait for it,
    so the two never command the arm together. None, or the JSON error to return when
    it will not stop, in which case the click is not carried out either."""
    with lock:
        running = run["running"]
        if running:
            try:
                # The mission cancels its Nav2 goal and its pick on SIGINT, as for Stop.
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass  # already exiting; the follower has yet to notice
    if running:
        follower.join(timeout=STOP_TIMEOUT)
        if follower.is_alive():
            return jsonify(error="The mission has not stopped yet; try again"), 504
    return None


def begin(title, command):
    """Run `command` as the page's current run, or return the JSON error if one is going."""
    global process, run, follower
    with lock:
        if run["running"]:
            return jsonify(error="A mission is already running"), 409
        run = fresh(title)
        run["running"] = True
        run["id"] = time.strftime("%Y%m%d-%H%M%S")
        RUNS.mkdir(parents=True, exist_ok=True)
        # Its own session, so Stop reaches anything the mission starts, as Ctrl+C would.
        process = subprocess.Popen(
            command,
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace",
            # The run id names the mission's folder of saved detections as well.
            env=dict(os.environ, MISSION_EVENTS="1", MISSION_RUN_ID=run["id"]), start_new_session=True,
        )
        follower = threading.Thread(target=follow, args=(process, run), daemon=True)
        follower.start()
    return jsonify(ok=True)


@app.post("/run")
def start():
    query = request.get_json(force=True)
    target = str(query.get("target", "")).strip()
    furniture = str(query.get("furniture", "")).strip()
    if not target and not furniture:
        return jsonify(error="Type an object to fetch, a piece of furniture to drive to, or both"), 400
    # The full mission by default: search, park and pick, so SAM3 and GraspGenX
    # must be up. The server's own arguments come later and win, so
    # `python3 -m web.server --navigate-only true` still gets drive-only runs,
    # as does any other mission_tree flag passed to the server.
    command = [sys.executable, "-u", "-m", "core.pipeline.mission_tree", "--target", target or furniture,
               "--navigate-only", "false", *mission_args]
    if furniture:
        # That one piece only: no remembered places and no DeepSeek (mission_tree --furniture).
        command += ["--furniture", furniture]
    if not target:
        # Furniture alone tests the drive there. These come last, so they win over the
        # server's own flags: nothing to detect or pick, and no request to understand.
        command += ["--navigate-only", "true", "--natural-language", "false"]
    return begin(target or f"drive to {furniture}", command)


@app.post("/recover")
def recover():
    """Free the arm after a pick that stopped part-way, as realrobot/live/recover_home.py
    does it: open the hand, forget the stale scene, back straight out, then plan home.
    The Home button folds the arm with no collision check, which drags a hand that is
    still around the object through it. Runs like a mission: logged, saved, stoppable."""
    if (stuck := take_over()) is not None:
        return stuck
    return begin("recover arm", [sys.executable, "-u", str(ROOT / "realrobot/live/recover_home.py")])


@app.post("/stop")
def stop():
    with lock:
        if run["running"]:
            # The mission cancels its Nav2 goal on SIGINT, exactly as for Ctrl+C.
            os.killpg(process.pid, signal.SIGINT)
    return jsonify(ok=True)


@app.post("/reset")
def reset():
    """Clear the page's current run, stopping the mission first if one is going.

    Saved runs are kept, a stopped one included: it is saved before the clear."""
    global run
    with lock:
        running = run["running"]
        if running:
            try:
                # The mission cancels its Nav2 goal on SIGINT, exactly as for Stop.
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass  # already exiting; the follower has yet to notice
    if running:
        # The follower marks the run over and saves it once the mission has exited.
        follower.join(timeout=STOP_TIMEOUT)
        if follower.is_alive():
            return jsonify(error="The mission has not stopped yet; try Reset again"), 504
    with lock:
        # Another page may have started a mission while this one waited.
        if run["running"]:
            return jsonify(error="A mission is already running"), 409
        run = fresh()
    return jsonify(ok=True)


@app.get("/runs")
def saved_runs():
    """Saved runs, newest first, for the page's list."""
    listing = []
    for path in sorted(RUNS.glob("*.json"), reverse=True):
        saved = json.loads(path.read_text())
        listing.append({"id": saved["id"], "target": saved["target"], "exit": saved["exit"]})
    return jsonify(listing)


@app.get("/runs/<name>")
def saved_run(name):
    """One saved run: <id>.json for the page, <id>.log for its full log."""
    return send_from_directory(RUNS, name)


@app.post("/home")
def home():
    """Send the arm to the pose the mission's Home step uses; the page shows it live."""
    if (stuck := take_over()) is not None:
        return stuck
    if not arm.server_is_ready():
        return jsonify(error="Arm controller is not ready"), 503
    arm.send_goal_async(home_goal())
    return jsonify(ok=True)


def hand_free():
    """None when the hand may be moved by hand, else the JSON error to return."""
    return take_over()


@app.post("/gripper/open")
def gripper_open():
    """Open the hand fully, as the pick does before its approach."""
    if (busy := hand_free()) is not None:
        return busy
    if not gripper.server_is_ready():
        return jsonify(error="Gripper controller is not ready"), 503
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ["hand_motor_joint"]
    goal.trajectory.points = [JointTrajectoryPoint(positions=[OPEN_HAND], time_from_start=Duration(sec=2))]
    gripper.send_goal_async(goal)
    return jsonify(ok=True)


@app.post("/gripper/close")
def gripper_close():
    """Close the hand with the pick's grasp effort, so it holds whatever is in it."""
    if (busy := hand_free()) is not None:
        return busy
    if not grasp.server_is_ready():
        return jsonify(error="Gripper grasp action is not ready"), 503
    grasp.send_goal_async(GripperApplyEffort.Goal(effort=CLOSE_EFFORT))
    return jsonify(ok=True)


def ask(client, message, timeout=10.0):
    """One service call from a Flask thread; the executor thread completes it. None when
    the service is absent or silent."""
    if not client.service_is_ready():
        return None
    done = threading.Event()
    future = client.call_async(message)
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        client.remove_pending_request(future)
        return None
    return future.result()


def on_nav2_tick():
    """Keep `nav2` current for the page's button, without blocking the executor."""
    global nav2
    if not nav_state.service_is_ready():
        nav2 = "down"
        return

    def answered(future):
        global nav2
        state = future.result().current_state.id
        nav2 = {State.PRIMARY_STATE_ACTIVE: "active", State.PRIMARY_STATE_INACTIVE: "paused"}.get(state, "down")
    nav_state.call_async(GetState.Request()).add_done_callback(answered)


def switch_nav2(command):
    # As the hand and arm buttons: the mission is stopped first. Resumed during a
    # pick, Nav2 would fight MoveIt for the base; paused mid-drive, the mission would
    # only see its goal fail.
    if (stuck := take_over()) is not None:
        return stuck
    reply = ask(nav_manage, ManageLifecycleNodes.Request(command=command), timeout=30.0)
    if reply is None:
        return jsonify(error="Nav2's lifecycle manager is not answering; is the stack up?"), 503
    if not reply.success:
        return jsonify(error="Nav2 refused"), 502
    return jsonify(ok=True)


@app.post("/nav2/pause")
def nav2_pause():
    """Deactivate Nav2's motion nodes; the map and localization stay up."""
    return switch_nav2(ManageLifecycleNodes.Request.PAUSE)


@app.post("/nav2/resume")
def nav2_resume():
    return switch_nav2(ManageLifecycleNodes.Request.RESUME)


@app.get("/camera.jpg")
def camera():
    # The last frame outlives the sim; an old one means the camera has stopped.
    if image is None or time.monotonic() - image_seen > CAMERA_STALE:
        abort(503)
    frame = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    width = min(CAMERA_WIDTH, frame.shape[1])
    height = round(frame.shape[0] * width / frame.shape[1])
    _, jpeg = cv2.imencode(".jpg", cv2.resize(frame, (width, height)))
    return jpeg.tobytes(), 200, {"Content-Type": "image/jpeg", "Cache-Control": "no-store"}


@app.get("/robot.urdf")
def robot():
    return urdf, 200, {"Content-Type": "application/xml"}


@app.get("/package/<package>/<path:rest>")
def package_file(package, rest):
    """package:// meshes the URDF refers to."""
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError:
        abort(404)
    return send_from_directory(share, rest)


def main():
    global urdf, arm, gripper, grasp, tf_listener, nav_manage, nav_state
    # The description the simulation spawns (launch/search.launch.py).
    share = get_package_share_directory("hsrc_description")
    urdf = xacro.process_file(os.path.join(share, "robots/hsrc1s.urdf.xacro")).toxml()
    # Ctrl+C stays a plain KeyboardInterrupt, so the finally below stops the mission.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("mission_dashboard")
    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    arm = ActionClient(node, FollowJointTrajectory,
                       "/arm_trajectory_controller/follow_joint_trajectory")
    gripper = ActionClient(node, FollowJointTrajectory,
                           "/gripper_controller/follow_joint_trajectory")
    grasp = ActionClient(node, GripperApplyEffort, "/gripper_controller/grasp")
    node.create_subscription(NavPath, "/plan", on_plan, 10)
    node.create_subscription(Image, RGB_TOPIC, on_image, qos_profile_sensor_data)
    tf_listener = TransformListener(tf_buffer, node)
    node.create_timer(TRACK_PERIOD, on_tick)
    nav_manage = node.create_client(ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")
    nav_state = node.create_client(GetState, "/controller_server/get_state")
    node.create_timer(NAV2_PERIOD, on_nav2_tick)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    print(f"[WEB] open http://localhost:{PORT}", flush=True)
    try:
        app.run(host=HOST, port=PORT, threaded=True)
    finally:
        with lock:
            running = run["running"]
        if running:
            os.killpg(process.pid, signal.SIGINT)
            # The follower saves the run once the mission has exited.
            follower.join(timeout=STOP_TIMEOUT)
        # Exiting while the thread is still inside spin() aborts the process in C++.
        executor.shutdown()
        spinner.join()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
