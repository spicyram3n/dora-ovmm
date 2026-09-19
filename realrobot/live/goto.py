"""Drive to one scene-graph furniture piece. Nothing else.

    python3 realrobot/live/goto.py --furniture 107
    python3 realrobot/live/goto.py --furniture 107 --dry-run

No reasoner and no DeepSeek: the furniture is named outright, so this is the
navigation leg of the mission on its own, for testing that the graph, the
costmap and the base agree.

`actions.plan` rings the piece with observation poses the way the mission would,
drops the ones Nav2's costmap calls blocked, orders them nearest-first, and the
base drives to the first one `compute_path_to_pose` can actually reach.

Needs, in this order:
  1. the robot's own stack up, on ROS_DOMAIN_ID=5
  2. python3 realrobot/live/localize.py     -- once per boot, before Nav2
  3. ros2 launch launch/realrobot/nav2_real.launch.py
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import rclpy  # noqa: E402

from core.navigation import base_placement, standoff  # noqa: E402
from core.navigation.nav2_client import Navigator  # noqa: E402
from core.pipeline import actions  # noqa: E402
from core.scene_graph import graph as sg  # noqa: E402
from core.utils.recording import Paths  # noqa: E402

DEFAULT_GRAPH = Paths().graph
CLEAR_COSTMAP = ("ros2 service call /global_costmap/clear_entirely_global_costmap "
                 "nav2_msgs/srv/ClearEntireCostmap {}")
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--furniture", required=True,
                        help="furniture to drive to, by name, label or node ID")
    parser.add_argument("--graph", type=Path, default=DEFAULT_GRAPH)
    parser.add_argument("--dry-run", action="store_true",
                        help="report the poses and what Nav2 can plan to, drive nowhere")
    parser.add_argument("--look", action="store_true",
                        help="turn the head towards the piece after arriving")
    parser.add_argument("--timeout", type=float, default=300,
                        help="seconds for the drive itself")
    args = parser.parse_args()

    scene = sg.load(args.graph)
    sg.require_map(scene)
    # Resolved before anything talks to the robot: a label like "chair" matches
    # a dozen pieces, and the node ID is the way to say which one.
    try:
        location = next(actions.targets(scene, None, args.furniture))
    except ValueError as error:
        matches = [f"{data['name'] or data['label']} ({node})"
                   for node, data in sg.furniture(scene).items()
                   if args.furniture in (data["name"], data["label"], str(node))]
        print(f"[target] {error}")
        print(f"         {args.furniture!r} matches {len(matches)}: "
              f"{', '.join(matches) if matches else 'nothing in the graph'}")
        print("         plan_overlay.py --survey draws every piece with its ID")
        return 1
    data = scene.nodes[location.furniture_id]
    centre = data["centroid"]
    print(f"[target] {data['name'] or data['label']} ({location.furniture_id}) "
          f"at ({centre[0]:.2f}, {centre[1]:.2f}) in map")

    rclpy.init()
    navigator = Navigator(use_sim_time=False)  # no /clock on the robot
    try:
        # Both of these say plainly which piece of the stack is missing.
        start = navigator.robot_xy()
        print(f"[robot] at ({start[0]:.2f}, {start[1]:.2f}), "
              f"{((start[0] - centre[0]) ** 2 + (start[1] - centre[1]) ** 2) ** 0.5:.1f} m away")
        grid = base_placement.costmap_grid(navigator)

        # Theta* refuses a plan whose *start* is an obstacle, and then every goal
        # reads as "no path" however clear it is. Say so once, here.
        if not standoff.free(grid, start):
            print("[costmap] the base is standing on a blocked cell: Theta* rejects "
                  "the start, so every goal below will say 'no path'")
            escape = nearest_free(grid, start)
            if escape:
                distance, (x, y) = escape
                print(f"          nearest free cell is {distance:.2f} m away, "
                      f"towards ({x:.2f}, {y:.2f})")
            else:
                print("          nothing free in the 2.5 m box around the base")
            print(f"          stale marks?  {CLEAR_COSTMAP}")
            print("          drifted?      realrobot/live/localize.py --dry-run")

        # actions.plan already drops poses on blocked cells and poses that would
        # look at the piece through a wall (standoff.unobstructed).
        poses, views = actions.plan(scene, location, start, grid,
                                    base_placement.sight_grid(navigator))
        if not poses:
            print("[plan] no observation pose survives the costmap; "
                  "run plan_overlay.py --furniture "
                  f"{args.furniture} offline to see why")
            return 1

        for index, pose in enumerate(poses, 1):
            if not navigator.reachable(*pose):
                print(f"[plan] {index}/{len(poses)} ({pose[0]:.2f}, {pose[1]:.2f}, "
                      f"{pose[2]:.2f}): no path")
                continue
            print(f"[plan] {index}/{len(poses)} ({pose[0]:.2f}, {pose[1]:.2f}, "
                  f"{pose[2]:.2f}): Nav2 has a path")
            if args.dry_run:
                print("[dry-run] stopping here; drop --dry-run to drive it")
                return 0
            if not navigator.drive_to(*pose, timeout=args.timeout):
                # Nav2 gives drive_to a bool, so say where it actually ended up:
                # the abort reason itself is in the controller_server log.
                stopped = navigator.robot_xy()
                travelled = np.hypot(stopped[0] - start[0], stopped[1] - start[1])
                short = np.hypot(pose[0] - stopped[0], pose[1] - stopped[1])
                print(f"[drive] stopped at ({stopped[0]:.2f}, {stopped[1]:.2f}): "
                      f"{travelled:.1f} m travelled, {short:.1f} m short of the goal")
                # Refreshed: the base has moved, so the grid from before is stale.
                grid = base_placement.costmap_grid(navigator)
                if not standoff.free(grid, stopped):
                    print("[drive] it is on a blocked cell now, so the poses below "
                          "will all say 'no path' whatever their own clearance")
                    escape = nearest_free(grid, stopped)
                    if escape:
                        distance, (x, y) = escape
                        print(f"        nearest free cell {distance:.2f} m away, "
                              f"towards ({x:.2f}, {y:.2f})")
                print("[drive] why it gave up is in the nav2 terminal: MPPI, or the "
                      "progress checker wanting 0.5 m every 10 s")
                continue
            offset, error = navigator.residual(*pose)
            print(f"[drive] arrived, {offset:.2f} m and {error:.2f} rad off the goal")
            if args.look:
                for aim in actions.look_points(scene, location, pose):
                    if navigator.look_at(aim):
                        break
            return 0

        print(f"[plan] Nav2 refused all {len(poses)} poses ({views} views)")
        if standoff.free(grid, navigator.robot_xy()):
            print("       the base is on free floor, so it is the goals Nav2 "
                  "dislikes; the planner_server log names which")
        return 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
