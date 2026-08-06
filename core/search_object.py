"""Drive the robot to where a queried object should be.

Usage:
  python3 search_object.py "pringles"                  # scene graph first, then the LLM
  python3 search_object.py "pringles" --dry-run        # print the plan, touch no robot
  python3 search_object.py --furniture high_shelf01    # skip the query, just drive there

Scenario B is tried first: if the graph already holds the object, its recorded
furniture is the first place to go and no LLM call is made. Only when that
comes back empty is the model asked, and its answer is cached per object under
outputs/scene_graph/<name>/locations/.

This ends where docs/navigation-and-grasping.md picks up: the robot is parked
in front of the right furniture, facing it. Detection and grasping are not
wired in here yet.
"""

import argparse
from pathlib import Path

import numpy as np

from navigation import standoff
from scene_graph import graph as sg
from spatial_reasoning import query_sg
from spatial_reasoning.query_sg import Location

REPO = Path(__file__).resolve().parent.parent
OUTPUTS = REPO / "outputs" / "scene_graph"


def furniture_named(scene, name):
    for node, data in sg.furniture(scene).items():
        if data["name"] == name or data["label"] == name:
            return node
    raise SystemExit(f"no furniture called {name!r}; have: "
                     f"{sorted(d['name'] for d in sg.furniture(scene).values())}")


def targets(scene, obj, furniture, cache, robot_xy, top_k):
    """Places to try, in order, produced lazily.

    Lazily is the whole point. In scenario B the graph already knows where the
    object is and the LLM must not be called at all, so this must not be
    collected into a list -- doing that pulls the next item, which is the
    model, and waits a minute for an answer nobody needed.
    """
    if furniture:
        node = furniture_named(scene, furniture)
        data = scene.nodes[node]
        yield Location(node, data["label"], data["room"], "at", "--furniture")
        return
    yield from query_sg.search_order(scene, obj, top_k=top_k, cache=cache, near=robot_xy)


def plan(scene, location, robot_xy):
    data = scene.nodes[location.furniture_id]
    poses = standoff.candidates(data["centroid"], data["dimensions"], robot_xy=robot_xy)
    reach = np.hypot(poses[0][0] - data["centroid"][0], poses[0][1] - data["centroid"][1])
    print(f"target: {data['name']} ({data['label']}) at "
          f"{[round(v, 2) for v in data['centroid'][:2]]}, room {data['room']} "
          f"-- from {location.source}")
    if location.reason:
        print(f"  because: {location.reason}")
    print(f"  {len(poses)} candidates, best stands {reach:.2f} m from the centroid")
    return poses


def main(obj, furniture, name, dry_run, top_k):
    scene = sg.load(OUTPUTS / name / "graph.json")

    navigator = None
    robot_xy = None
    if not dry_run:
        import rclpy
        from navigation.nav2_client import Navigator
        rclpy.init()
        navigator = Navigator()
        robot_xy = navigator.robot_xy()
        print(f"robot at {robot_xy[0]:.2f}, {robot_xy[1]:.2f}")

    try:
        for location in targets(scene, obj, furniture, OUTPUTS / name / "locations",
                                robot_xy, top_k):
            poses = plan(scene, location, robot_xy)
            if dry_run:
                for x, y, yaw in poses[:3]:
                    print(f"    ({x:6.2f}, {y:6.2f}) facing {yaw:5.2f} rad")
                return          # stop here too, or the next location is pulled
                                # from the generator and that costs an LLM call
            if navigator.go_to_first_reachable(poses):
                print("arrived. Look for the object from here.")
                return
            print("  no reachable pose here, trying the next location")
        if not dry_run:
            print("no location worked out")
    finally:
        if navigator is not None:
            import rclpy
            navigator.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Navigate to where an object should be.")
    parser.add_argument("object", nargs="?", default="", help='what to look for, e.g. "pringles"')
    parser.add_argument("--furniture", help="go straight to this furniture, no LLM")
    parser.add_argument("--name", default="apartment", help="which saved scene graph to use")
    parser.add_argument("--dry-run", action="store_true", help="print the plan, do not drive")
    parser.add_argument("--top-k", type=int, default=3)
    arguments = parser.parse_args()
    if not arguments.object and not arguments.furniture:
        parser.error("give an object to search for, or --furniture")
    main(arguments.object, arguments.furniture, arguments.name,
         arguments.dry_run, arguments.top_k)
