"""Which furniture in a scene graph can actually be grasped from, and how.

A quick offline check -- no ROS, no Gazebo. Reads the saved scene graph and
asks reach.py whether the top surface of each piece is within the arm's
envelope at all, before any driving is planned. A piece whose surface is out
of reach is one the search loop should never be sent to expecting a grasp.

Read the shelves with care. "Top surface" here is the top of the bounding
box, which for a table is the work surface and is exactly right, but for a
multi-shelf unit is the topmost shelf rather than where anything necessarily
sits. `high_shelf01` coming back unreachable means its top shelf is
unreachable, not the whole unit -- objects on its lower shelves are fine. The
number that actually decides is the object's own height once SAM3 has found
it, at which point `reach.base_annulus` answers exactly.

From core/:
    python3 reach_report.py
"""

import sys
from pathlib import Path

from navigation import reach
from scene_graph import graph as sg


def _pose(rotation, centroid, height):
    return reach.hand_pose((centroid[0], centroid[1], height), rotation)


def report(scene, robot=reach.DEFAULT_ROBOT):
    rows = []
    for _, data in sorted(sg.furniture(scene).items(), key=lambda kv: kv[1]["name"]):
        surface = data["centroid"][2] + data["dimensions"][2] / 2
        top = reach.base_annulus(_pose(reach.TOP_DOWN, data["centroid"], surface), robot)
        side = reach.base_annulus(_pose(reach.SIDE_ON, data["centroid"], surface), robot)
        rows.append((data["name"], surface, top, side))
    return rows


def main(name="apartment"):
    graph_path = Path(__file__).resolve().parents[1] / "outputs/scene_graph" / name / "graph.json"
    print(f"{'furniture':<20} {'top surface':>11}  {'top-down':>18}  {'side-on':>18}")
    for piece, surface, top, side in report(sg.load(graph_path)):
        def show(annulus):
            return "out of reach" if annulus is None else \
                   f"r=[{annulus.radius_min:.2f}, {annulus.radius_max:.2f}]"
        flag = "  <-- unreachable" if top is None and side is None else ""
        print(f"{piece:<20} {surface:>10.2f} m  {show(top):>18}  {show(side):>18}{flag}")


if __name__ == "__main__":
    main(*sys.argv[1:])
