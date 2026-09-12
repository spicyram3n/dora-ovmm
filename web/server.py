"""Web dashboard for the behaviour-tree mission: type what to fetch, watch it run.

    python3 -m web.server
    # then open http://localhost:8080

Run it in its own terminal, so it outlives the sim: `ros2 launch
launch/search.launch.py web:=true` starts the stack without a mission, and the
page's queries run against it. Every argument here is passed on to
core.pipeline.mission_tree, which runs once per query in its own process. Its
[EVENT] lines (core.utils.events) feed the page; the rest of its output goes to
this terminal as before. Each run is saved to outputs/web_runs/ when it ends, with
its full log, so any past query can be opened again, even with the sim closed.
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
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from flask import Flask, abort, jsonify, request, send_from_directory
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

from core.perception.camera_ros2 import RGB_TOPIC
from core.pipeline.mission import home_goal
from core.utils.events import PREFIX

ROOT = Path(__file__).resolve().parents[1]
# Local only: anyone who can open the page can drive the robot.
HOST, PORT = "127.0.0.1", 8080
# Every run is saved here when it ends, to inspect after the sim is gone.
RUNS = ROOT / "outputs/web_runs"
# Joint samples kept while the arm homes: enough for a smooth replay, and small.
SAMPLE_PERIOD = 0.1
# The step that drives the base to the place; the path and track are kept for it.
DRIVE_STEP = "Go there"
# How often the base's map pose is read, and how far apart path waypoints are drawn.
TRACK_PERIOD = 0.2
WAYPOINT_SPACING = 0.25
# A base transform that has not changed for this long means the sim stopped or restarted.
STALE_TF = 2.0
# Width of the head camera image in the page's small window.
CAMERA_WIDTH = 320
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
tf_buffer = Buffer()
tf_listener = None  # fills tf_buffer; held here so it lives as long as the server
tf_stamp = None     # stamp of the last base transform, and when it last changed
tf_seen = 0.0
bridge = CvBridge()
image = None        # the latest head camera frame, encoded only when the page asks


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
        if run["running"] and run["leaf"] == DRIVE_STEP:
            run["path"] = points


def on_image(message):
    global image
    image = message


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
        if run["running"] and run["leaf"] == DRIVE_STEP:
            run["track"].append({"t": time.monotonic(), "pose": pose})


@app.get("/")
def index():
    return app.send_static_file("index.html")


@app.get("/state")
def state():
    with lock:
        return jsonify(dict(run, boot=BOOT))


@app.post("/run")
def start():
    global process, run, follower
    target = str(request.get_json(force=True).get("target", "")).strip()
    if not target:
        return jsonify(error="Type an object to fetch"), 400
    with lock:
        if run["running"]:
            return jsonify(error="A mission is already running"), 409
        run = fresh(target)
        run["running"] = True
        run["id"] = time.strftime("%Y%m%d-%H%M%S")
        RUNS.mkdir(parents=True, exist_ok=True)
        # The page covers home, reasoning and navigation so far: no SAM3 or GraspGenX.
        # Its own session, so Stop reaches anything the mission starts, as Ctrl+C would.
        process = subprocess.Popen(
            [sys.executable, "-u", "-m", "core.pipeline.mission_tree", "--target", target,
             "--navigate-only", "true", *mission_args],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace",
            env=dict(os.environ, MISSION_EVENTS="1"), start_new_session=True,
        )
        follower = threading.Thread(target=follow, args=(process, run), daemon=True)
        follower.start()
    return jsonify(ok=True)


@app.post("/stop")
def stop():
    with lock:
        if run["running"]:
            # The mission cancels its Nav2 goal on SIGINT, exactly as for Ctrl+C.
            os.killpg(process.pid, signal.SIGINT)
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
    with lock:
        if run["running"]:
            return jsonify(error="The mission is using the arm"), 409
    if not arm.server_is_ready():
        return jsonify(error="Arm controller is not ready"), 503
    arm.send_goal_async(home_goal())
    return jsonify(ok=True)


@app.get("/camera.jpg")
def camera():
    if image is None:
        abort(503)
    frame = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    height = round(frame.shape[0] * CAMERA_WIDTH / frame.shape[1])
    _, jpeg = cv2.imencode(".jpg", cv2.resize(frame, (CAMERA_WIDTH, height)))
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
    global urdf, arm, tf_listener
    # The description the simulation spawns (launch/search.launch.py).
    share = get_package_share_directory("hsrc_description")
    urdf = xacro.process_file(os.path.join(share, "robots/hsrc1s.urdf.xacro")).toxml()
    # Ctrl+C stays a plain KeyboardInterrupt, so the finally below stops the mission.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("mission_dashboard")
    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    arm = ActionClient(node, FollowJointTrajectory,
                       "/arm_trajectory_controller/follow_joint_trajectory")
    node.create_subscription(NavPath, "/plan", on_plan, 10)
    node.create_subscription(Image, RGB_TOPIC, on_image, qos_profile_sensor_data)
    tf_listener = TransformListener(tf_buffer, node)
    node.create_timer(TRACK_PERIOD, on_tick)
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
            follower.join(timeout=30)
        # Exiting while the thread is still inside spin() aborts the process in C++.
        executor.shutdown()
        spinner.join()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
