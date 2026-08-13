"""Prompt in, arm at the grasp out. The whole pipeline, one process.

    python3 fetch.py "pringles can"
    python3 fetch.py "pringles can" --execute      # actually move the arm
    python3 fetch.py "pringles can" --top-k 3

Running search_object, then run_pipeline, then pick by hand costs three camera
grabs, three ROS contexts and three chances to hand the next stage stale data
-- the saved grasps are only valid for the base pose they were generated from,
and driving anywhere invalidates them. This keeps one context open and one
current view, so what gets planned is what the robot is looking at now.

For each of the top-k places the object might be:

    drive to a standoff  ->  look  ->  not there? next place
                                   ->  there? close in, look again, grasp

The second look matters. The first is from 0.8 m away, which the arm cannot
reach from; the grasps that get planned come from the closer one.
"""

import argparse
from pathlib import Path

import numpy as np

from grasping import graspgenx_client, select
from navigation import reach, standoff
from perception import detect, head
from scene_graph import graph as sg
from spatial_reasoning import query_sg

OUTPUTS = Path(__file__).resolve().parent.parent / "outputs" / "scene_graph"

# Grasps are planned in the frame MoveIt plans in; navigation goals go in the
# frame Nav2 plans in. They coincide at start-up and drift apart afterwards, so
# neither can stand in for the other.
GRASP_FRAME = "odom"


def look_around(prompt, frame, head_node, aims):
    """Try each head aim in turn, stopping at the first detection. Returns
    (points, centroid, score) or None.

    Aiming is not optional and one aim is often not enough. The camera's
    vertical field is 46.8 degrees; a table's work surface fits in that, but a
    shelf unit running floor to 1.82 m spans nearly 87 degrees from its own
    standoff. Aiming once at such a piece puts most of it out of frame, and a
    detector that sees nothing reports "not here" rather than "not looked at".
    """
    for view, (pan, tilt) in enumerate(aims, 1):
        head_node.point(pan, tilt)
        try:
            points, score = detect.find(prompt, frame)
        except detect.NotFound as reason:
            print(f"  view {view}/{len(aims)}: {reason}")
            continue
        print(f"  view {view}/{len(aims)}: detected (score {score:.2f}), "
              f"{len(points)} points")
        return points, points.mean(axis=0), score
    return None


def grasps_for(points):
    """GraspGenX on an object-centred cloud, back in the cloud's own frame."""
    centroid = points.mean(axis=0)
    poses, scores = graspgenx_client.generate(points - centroid, "hsrc_hand")
    poses = poses.copy()
    poses[:, :3, 3] += centroid
    return poses, scores


def try_here(navigator, picker, head_node, prompt, extent, blockers, execute):
    """Everything that happens once the robot is parked at a standoff pose.
    `extent` is (x, y, low_z, high_z) for whatever is being searched. True if
    the arm ended up at a grasp."""
    x, y, low_z, high_z = extent
    pose = navigator.robot_pose()
    seen = look_around(prompt, "map", head_node,
                       head.scan((x, y), low_z, high_z, pose[:2], pose[2]))
    if seen is None:
        return False
    _, centroid_map, _ = seen

    if not reach.move_into_reach(navigator, reach.hand_pose(centroid_map), blockers):
        print("  cannot stand anywhere that reaches it from this side")
        return False

    # Look again from the new pose: this is the view the grasps come from, and
    # the head has to be re-aimed because the base moved under it. Now the
    # object's own height is known, so this is one aim rather than a sweep.
    pose = navigator.robot_pose()
    seen = look_around(prompt, GRASP_FRAME, head_node,
                       head.scan(centroid_map[:2], centroid_map[2], centroid_map[2],
                                 pose[:2], pose[2]))
    if seen is None:
        print("  lost sight of it after closing in")
        return False
    points, _, _ = seen

    poses, scores = grasps_for(points)
    print(f"  {len(scores)} grasps, best score {scores.max():.2f}")

    base = picker.base_xy(GRASP_FRAME)
    print(f"  filter from ({base[0]:.2f}, {base[1]:.2f}): "
          f"{select.report(poses, scores, points, base)}")
    order = select.feasible(poses, scores, points, base)
    if not order:
        short = select.shortfall(poses, scores, points, base)
        if short:
            i, gap, where = min(short, key=lambda row: row[1])
            print(f"  closest candidate is {gap:.2f} m out of band; "
                  f"({where[0]:.2f}, {where[1]:.2f}) would work")
        return False

    for rank, (i, so) in enumerate(order, 1):
        print(f"  candidate {rank}/{len(order)} (score {scores[i]:.2f}, "
              f"standoff {so:.2f} m): planning...")
        trajectory, code = picker.plan(select.pregrasp(poses[i], so), GRASP_FRAME)
        if trajectory is None:
            print(f"    no plan (MoveItErrorCodes {code})")
            continue
        from pick import BASE_DRIFT_LIMIT, _base_drift
        drift = _base_drift(trajectory)
        if drift > BASE_DRIFT_LIMIT:
            print(f"    rejected: plan drives the base {drift:.2f} m")
            continue
        if not execute:
            print("    planned. --execute not given, stopping here")
            return True
        if picker.execute(trajectory):
            print("    at the pregrasp. The approach segment is not built yet.")
            return True
        print("    execution failed")
    return False


def main(prompt, name, top_k, execute):
    import rclpy
    from navigation.nav2_client import Navigator
    from perception.head_ros2 import Head
    from pick import Picker

    scene = sg.load(OUTPUTS / name / "graph.json")
    blockers = [(f["centroid"], f["dimensions"]) for f in sg.furniture(scene).values()]

    rclpy.init()
    navigator, picker, head_node = Navigator(), Picker("whole_body_weighted"), Head()
    try:
        robot_xy = navigator.robot_xy()
        print(f"robot at {robot_xy[0]:.2f}, {robot_xy[1]:.2f}")

        for location in query_sg.search_order(scene, prompt, top_k=top_k,
                                              cache=OUTPUTS / name / "locations",
                                              near=robot_xy):
            data = scene.nodes[location.furniture_id]
            print(f"\n{data['name']} -- from {location.source}"
                  + (f": {location.reason}" if location.reason else ""))

            others = [b for b in blockers if b[0] is not data["centroid"]]
            poses = standoff.candidates(data["centroid"], data["dimensions"],
                                        robot_xy=robot_xy, blockers=others)
            if not poses:
                print("  no standoff clear of the surrounding furniture")
                continue
            if navigator.go_to_first_reachable(poses) is None:
                print("  could not get to any standoff pose")
                robot_xy = navigator.robot_xy()
                continue

            # If the graph remembers the object, search only its own height;
            # otherwise sweep the whole piece, since which shelf it is on is
            # exactly what is not known.
            node = sg.find_object(scene, prompt, near=robot_xy)
            if node is not None and sg.location_of(scene, node)[0] == location.furniture_id:
                obj = scene.nodes[node]["centroid"]
                extent = (obj[0], obj[1], obj[2], obj[2])
                print(f"  graph puts it at z={obj[2]:.2f}; one view")
            else:
                c, dim = data["centroid"], data["dimensions"]
                extent = (c[0], c[1], c[2] - dim[2] / 2, c[2] + dim[2] / 2)
            if try_here(navigator, picker, head_node, prompt, extent, blockers, execute):
                print("\ndone.")
                return
            robot_xy = navigator.robot_xy()

        print("\nran out of places to look")
    finally:
        navigator.destroy_node()
        picker.destroy_node()
        head_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Find an object and put the arm on it.")
    parser.add_argument("prompt", help='what to fetch, e.g. "pringles can"')
    parser.add_argument("--name", default="apartment", help="which saved scene graph")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--execute", action="store_true", help="actually move the arm")
    arguments = parser.parse_args()
    main(arguments.prompt, arguments.name, arguments.top_k, arguments.execute)
