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

from navigation import reach, standoff
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
    # Every other piece is something the base cannot stand inside -- a coffee
    # table's own sofas are the case that matters.
    blockers = [(f["centroid"], f["dimensions"]) for node, f in sg.furniture(scene).items()
                if node != location.furniture_id]
    poses = standoff.candidates(data["centroid"], data["dimensions"],
                                robot_xy=robot_xy, blockers=blockers)
    print(f"target: {data['name']} ({data['label']}) at "
          f"{[round(v, 2) for v in data['centroid'][:2]]}, room {data['room']} "
          f"-- from {location.source}")
    if location.reason:
        print(f"  because: {location.reason}")
    if not poses:
        print("  no standoff clear of the surrounding furniture")
        return poses
    reach = np.hypot(poses[0][0] - data["centroid"][0], poses[0][1] - data["centroid"][1])
    print(f"  {len(poses)} candidates, best stands {reach:.2f} m from the centroid")
    return poses


def locate(obj):
    """Where `obj` actually is, in the map frame, or None if it is not in
    view from here.

    Perception, not the scene graph, even when the graph claims to know: a
    remembered centroid is where the object was last seen, and the whole point
    of driving somewhere is to find out whether it is still there.
    """
    # Imported here, not at the top: --dry-run must not need ROS or the
    # SAM3 server, and neither must the tests.
    from navigation import nav2_client
    from perception import pointcloud, sam3_client
    from perception.camera_ros2 import grab_rgbd

    rgb, depth_m, k, map_from_camera = grab_rgbd(target_frame=nav2_client.FRAME)
    try:
        mask, score = sam3_client.detect(rgb, obj)
    except RuntimeError as error:
        print(f"  {error}")
        return None
    points = pointcloud.deproject(depth_m, k, mask)
    if len(points) == 0:
        print("  found it, but no valid depth inside the mask")
        return None
    print(f"  detected (score {score:.2f}), {len(points)} points")
    return pointcloud.transform_points(map_from_camera, points).mean(axis=0)


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
            if not navigator.go_to_first_reachable(poses):
                print("  no reachable pose here, trying the next location")
                continue
            if not obj:                       # --furniture with nothing to find
                print("arrived.")
                return
            centroid = locate(obj)
            if centroid is None:
                print("  not here, trying the next location")
                continue
            furniture = [(f["centroid"], f["dimensions"])
                         for f in sg.furniture(scene).values()]
            if reach.move_into_reach(navigator, reach.hand_pose(centroid), furniture):
                print(f"in reach of {obj} at {[round(v, 2) for v in centroid]}. Ready to grasp.")
                return
            print("  found it but cannot stand anywhere that reaches it")
            robot_xy = navigator.robot_xy()   # it has moved; rank the next one from here
        if not dry_run:
            print("no location worked out")
    finally:
        if navigator is not None:
            import rclpy
            navigator.destroy_node()
            if rclpy.ok():          # Ctrl-C: rclpy's own handler got there first
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
