"""The query-to-grasp mission as a behaviour tree, alongside core.pipeline.mission.

    python3 -m core.pipeline.mission_tree --target pringles
    python3 -m core.pipeline.mission_tree --target pringles --navigate-only true
    python3 -m core.pipeline.mission_tree --render      # picture of the tree, no ROS

Same stack, checks and steps as mission.py; the tree decides what runs next.
Each leaf runs one existing step to its end within a single tick. The mission
is a strict sequence, and Ctrl+C still cancels a Nav2 goal inside the step.

    Mission                                   Mission (--navigate-only true)
    ├─ Ready                                  ├─ Ready             no SAM3
    ├─ Home arm                               ├─ Home arm
    ├─ Find target                            ├─ Choose location   target reasoning
    ├─ Park            once                   └─ Go there          first reachable view
    ├─ Pause Nav2
    ├─ Pick            up to ATTEMPTS tries
    └─ Home arm after pick

Watch it live, with ROS sourced, once this is running:
    py-trees-tree-watcher      # in the terminal
    py-trees-tree-viewer       # in a window
"""

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import py_trees
import py_trees_ros
import rclpy
from rclpy.executors import SingleThreadedExecutor

from core.navigation.nav2_client import Navigator
from core.pipeline import mission, search
from core.scene_graph import graph as sg

ROOT = Path(__file__).resolve().parents[2]
# Each pick re-segments the object and asks for fresh grasps, so a retry is a new try.
ATTEMPTS = 3
# The failing leaf decides the exit code, matching core.pipeline's, which
# launch/search.launch.py reads. Anything unexpected raises and exits 3.
EXIT_CODES = {"Choose location": 1, "Find target": 1, "Go there": 2, "Park": 2, "Pick": 4}


class Step(py_trees.behaviour.Behaviour):
    """A leaf that runs `action()` once per visit: True is success, False failure."""

    def __init__(self, name, action):
        super().__init__(name)
        self.action = action
        self.started = False

    def initialise(self):
        self.started = False

    def update(self):
        # Report RUNNING for one tick first. A tick that only ever sees finished
        # steps runs the whole mission in one go, and the viewers, which update
        # after each tick, would see nothing until the end.
        if not self.started:
            self.started = True
            return py_trees.common.Status.RUNNING
        if self.action():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


def build(steps, grasp=True, navigate_only=False):
    """The mission tree over `steps`, a dict of name -> action."""
    children = [Step("Ready", steps["ready"]), Step("Home arm", steps["home"])]
    if navigate_only:
        children.append(Step("Choose location", steps["choose"]))
        children.append(Step("Go there", steps["go"]))
        return py_trees.composites.Sequence("Mission", memory=True, children=children)
    children.append(Step("Find target", steps["find"]))
    # Parking already tries every base pose the IK solver returns; once is enough.
    children.append(Step("Park", steps["park"]))
    if grasp:
        children.append(Step("Pause Nav2", steps["pause"]))
        children.append(py_trees.decorators.Retry(
            "Pick, retried", Step("Pick", steps["pick"]), num_failures=ATTEMPTS))
        children.append(Step("Home arm after pick", steps["home"]))
    return py_trees.composites.Sequence("Mission", memory=True, children=children)


def exit_code(root):
    if root.status == py_trees.common.Status.SUCCESS:
        return 0
    return EXIT_CODES.get(root.tip().name, 3)


def mission_steps(navigator, scene, target, graph_path, top_k, bearings, timeout, perception):
    """The existing pipeline functions, as the tree's actions."""
    chosen = {}

    def ready():
        mission.get_ready(navigator, target, timeout, perception=perception)
        return True

    def home():
        _, wait = mission.waiter(navigator, time.monotonic() + 60)
        mission.home_arm(navigator, wait)
        return True

    def choose():
        chosen["location"] = next(search.targets(scene, target, robot_xy=navigator.robot_xy(),
                                                 top_k=top_k), None)
        if chosen["location"] is None:
            print(f"[REASON] no location to search for {target!r}", flush=True)
            return False
        location = chosen["location"]
        print(f"[REASON] {target!r}: {location.label} in {location.room} ({location.source})", flush=True)
        return True

    def go():
        location = chosen["location"]
        poses, _ = search.plan(scene, location, navigator.robot_xy())
        for pose in poses:
            if navigator.reachable(*pose) and navigator.drive_to(*pose):
                # Face where the target should be, as the search does before looking.
                for aim in search.look_points(scene, location, pose):
                    if navigator.look_at(aim):
                        break
                return True
        return False

    def find():
        navigator.resume_navigation_if_paused()
        status, chosen["object"] = search.search(scene, target, navigator, top_k=top_k)
        if chosen["object"] is not None:
            # Keep the observation even if a later step fails.
            sg.save(scene, graph_path)
        return status == search.FOUND

    def park():
        status = search.make_graspable(scene, chosen["object"], navigator, bearings=bearings)
        return status == search.READY

    def pause():
        # MoveIt moves the base during the pick; Nav2 must not fight it.
        navigator.pause_navigation()
        return True

    def pick():
        return search.pick_up(target) == search.PICKED

    return {"ready": ready, "home": home, "choose": choose, "go": go,
            "find": find, "park": park, "pause": pause, "pick": pick}


def render(grasp, navigate_only):
    """Save the tree as a picture under outputs/, without ROS or the robot."""
    placeholders = {}
    for name in ("ready", "home", "choose", "go", "find", "park", "pause", "pick"):
        placeholders[name] = None  # never run: drawing needs names only
    root = build(placeholders, grasp=grasp, navigate_only=navigate_only)
    name = "mission_tree_navigate" if navigate_only else "mission_tree"
    files = py_trees.display.render_dot_tree(root, name=name, target_directory=str(ROOT / "outputs"))
    print(py_trees.display.unicode_tree(root))
    for path in files.values():
        print("wrote", path)
    return 0


def run(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target")
    parser.add_argument("--graph", type=Path, default=ROOT / "outputs/scene_graph/apartment.json")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--bearings", type=int, default=6)
    parser.add_argument("--startup-timeout", type=float, default=180)
    parser.add_argument("--grasp", default="true", choices=["true", "false"])
    parser.add_argument("--navigate-only", default="false", choices=["true", "false"],
                        help="reason and drive to the target's place; no SAM3 or GraspGenX")
    parser.add_argument("--render", action="store_true", help="save a picture of the tree and exit")
    args = parser.parse_args(argv)
    grasp, navigate_only = args.grasp == "true", args.navigate_only == "true"
    if args.render:
        return render(grasp, navigate_only)
    if not args.target or not args.target.strip() or args.top_k < 1 or args.bearings < 1:
        parser.error("target must be nonempty; top-k and bearings must be positive")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error("startup-timeout must be finite and positive")

    # Validate the graph before waiting for any robot or model service.
    scene = sg.load(args.graph)
    sg.require_map(scene)
    rclpy.init()
    navigator = Navigator()
    steps = mission_steps(navigator, scene, args.target, args.graph, args.top_k,
                          args.bearings, args.startup_timeout, perception=not navigate_only)
    tree = py_trees_ros.trees.BehaviourTree(build(steps, grasp=grasp, navigate_only=navigate_only))
    viewers = SingleThreadedExecutor()
    try:
        tree.setup(node_name="mission_tree", timeout=15.0)
        # The tree's node answers the watcher and viewer from its own thread, so
        # they can connect while a long step (Ready, a drive, a pick) is running.
        viewers.add_node(tree.node)
        threading.Thread(target=viewers.spin, daemon=True).start()
        while True:
            tree.tick()
            if tree.root.status != py_trees.common.Status.RUNNING:
                break
            time.sleep(0.1)
        print(py_trees.display.unicode_tree(tree.root, show_status=True), flush=True)
        print(f"[RESULT] {tree.root.status.value}, last step: {tree.root.tip().name}", flush=True)
        return exit_code(tree.root)
    finally:
        viewers.shutdown()
        tree.shutdown()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print(f"[ERROR] {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        raise SystemExit(3)
