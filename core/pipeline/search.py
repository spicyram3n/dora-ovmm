"""Search remembered or LLM-ranked locations with Nav2 and SAM3, park the base
where the arm can reach what was found, and with --grasp pick it up."""

import argparse
import subprocess
import sys
from pathlib import Path
import numpy as np
from core.navigation import standoff
from core.scene_graph import graph as sg
from core.reasoner.query import Location, search_order

ROOT = Path(__file__).resolve().parents[2]
# Mission outcomes, from the object never being seen to standing ready to grasp.
NOT_FOUND = "not_found"
ARRIVED = "arrived"
FOUND = "found"
NOT_GRASPABLE = "not_graspable"
READY = "ready"
PICKED = "picked"
NOT_PICKED = "not_picked"
EXIT_CODES = {NOT_FOUND: 1, ARRIVED: 0, FOUND: 0, NOT_GRASPABLE: 2, READY: 0,
              PICKED: 0, NOT_PICKED: 4}
# A same-label detection further than this from a remembered one is another instance.
SAME_INSTANCE = 1.0
# Nothing the HSR gripper can pick up measures more than this across.
MAX_SPAN = 0.6
# Taller furniture holds things on shelves the head cannot see from above.
TALL = 1.2
STORAGE_WORDS = ("shelf", "shelves", "bookcase", "rack")
# Views planned this run, per place, with what happened at each pose, so a later
# look at the same place (another query, active perception) can reuse them.
# Held in memory only: they go when the run ends.
VIEWS = {}


def box(data):
    """The box centre differs from a segmented cloud's mean; prefer saved bounds."""
    lower, upper = np.asarray(data["bounds"])
    return ((lower + upper) / 2, upper - lower)


def is_storage(data):
    """Shelved furniture is searched level by level instead of from above."""
    height = data["bounds"][1][2] - data["bounds"][0][2]
    return height > TALL or any(word in data["label"].lower() for word in STORAGE_WORDS)


def targets(scene, obj, furniture=None, robot_xy=None, top_k=3):
    if furniture:
        matches = []
        for node_id, data in sg.furniture(scene).items():
            if furniture in (data["name"], data["label"], str(node_id)):
                matches.append((node_id, data))
        if len(matches) != 1:
            raise ValueError("Use a unique furniture instance name or node ID")
        node, data = matches[0]
        yield Location(
            furniture_id=node,
            label=data["label"],
            room=data["room"],
            relation="near",
            source="furniture",
            centroid=data["centroid"],
        )
    else:
        yield from search_order(scene, obj, near=robot_xy, top_k=top_k)


def view_key(location):
    """One place to look: furniture, or a remembered object where it was last seen."""
    return (location.furniture_id, location.object_id,
            tuple(np.round(location.centroid, 2).tolist()))


def _views(scene, location):
    """Every clear pose around a place, facing it, and the surface it has to see."""
    pieces = sg.furniture(scene)
    blockers = []
    for data in pieces.values():
        blockers.append(sg.footprint(data))
    if location.furniture_id is None:
        centre, dimensions, yaw = (location.centroid, [0, 0, 0], 0.0)
        storage = False
    else:
        centre, dimensions, yaw = sg.footprint(pieces[location.furniture_id])
        storage = is_storage(pieces[location.furniture_id])
    poses = standoff.candidates(
        centre, dimensions, yaw, blockers=blockers, storage=storage
    )
    points = standoff.surface_points(centre, dimensions, yaw)
    if location.object_id is not None:
        # Stand outside the furniture, but face the remembered object itself.
        aimed = []
        for x, y, yaw in poses:
            yaw = float(np.arctan2(location.centroid[1] - y, location.centroid[0] - x))
            aimed.append((x, y, yaw))
        poses = aimed
        points = np.array([location.centroid[:2]])
    return (poses, points)


def plan(scene, location, robot_xy=None):
    """Observation poses in visiting order, and how many views the location needs."""
    sg.require_map(scene)
    if location.frame_id != "map":
        raise ValueError("Search locations must be in map")
    key = view_key(location)
    if key not in VIEWS:
        poses, points = _views(scene, location)
        VIEWS[key] = {"poses": poses, "points": points, "outcomes": {}}
    # Ordered afresh each time: the nearest pose depends on where the robot is now.
    poses, views = standoff.order(VIEWS[key]["poses"], VIEWS[key]["points"], robot_xy)
    print(
        f"{location.source}: {location.label}, room={location.room},"
        f" {len(poses)} observation poses, {views} views needed"
    )
    if location.reason:
        print(f"  {location.reason}")
    return (poses, views)


def look_points(scene, location, pose):
    """Where the head aims from `pose`: the remembered object, just above a
    surface, or each height of shelved furniture."""
    if location.object_id is not None or location.furniture_id is None:
        return [location.centroid]
    data = scene.nodes[location.furniture_id]
    centre, dimensions, yaw = sg.footprint(data)
    x, y = standoff.aim_point(pose[:2], centre, dimensions, yaw)
    lower, upper = (data["bounds"][0][2], data["bounds"][1][2])
    if not is_storage(data):
        # Objects lie on top; aiming just above it keeps the whole top in view.
        return [[x, y, upper + 0.1]]
    distance = np.hypot(x - pose[0], y - pose[1])
    points = []
    for height in standoff.shelf_heights(lower, upper, distance):
        points.append([x, y, height])
    return points


def observed_furniture(scene, points):
    from core.scene_graph.instance import Instance, from_box
    from core.scene_graph.relations import classify

    ids = []
    pieces = []
    for node, data in sg.furniture(scene).items():
        centre, dimensions = box(data)
        ids.append(node)
        pieces.append(from_box(data["label"], centre, dimensions))
    index, relation = classify(Instance("target", points), pieces, near_limit=0.5)
    if index is None:
        return None, "near"
    return ids[index], relation


def locate(obj):
    from core.perception import pointcloud, sam3_client
    from core.perception.camera_ros2 import grab_rgbd

    rgb, depth, intrinsics, transform = grab_rgbd(target_frame="map")
    try:
        mask, score = sam3_client.detect(rgb, obj)
    except sam3_client.ObjectNotFound:
        return None
    # The segmentation edge straddles object and background, so its pixels carry
    # the background's depth. Deprojected, they stretch the box down the view ray.
    core = pointcloud.shrink(mask)
    points = pointcloud.deproject(depth, intrinsics, core)
    if len(points) < 100 or len(points) < 0.25 * np.count_nonzero(core):
        print("Object detected, but depth is insufficient; trying another view.")
        return None
    points = pointcloud.largest_cluster(points)
    if points is None or len(points) < 100:
        print("Detection did not form one solid body; trying another view.")
        return None
    print(f"Detected {obj}, score={score:.2f}, {len(points)} points")
    return pointcloud.transform_points(transform, points)


def make_graspable(scene, node_id, navigator, obstacles="--costmap", bearings=6):
    """Refine the observation pose into one the IK solver certifies for the arm."""
    from core.navigation import graspable

    centre, dimensions = box(scene.nodes[node_id])
    blockers = []
    for data in sg.furniture(scene).values():
        blockers.append(sg.footprint(data))
    outcome = graspable.reposition(
        navigator, centre, dimensions, obstacles=obstacles, bearings=bearings,
        blockers=blockers,
    )
    if outcome is None:
        print(
            "No clear, navigable IK base pose was reached within position and yaw "
            "tolerances. The detected object remains saved in the scene graph."
        )
        return NOT_GRASPABLE
    pose, offset = outcome
    if offset > graspable.PRECISE_TOLERANCE:
        print(
            f"Parked {offset:.3f} m from the certified pose, outside the"
            f" {graspable.PRECISE_TOLERANCE:.2f} m the solver certifies. The object is"
            " recorded, but the arm is not proven to reach it from here."
        )
        return NOT_GRASPABLE
    # Keep the object in view so grasp generation can capture it from here.
    if not navigator.look_at(centre):
        print("Warning: the head cannot aim at the object from this pose.")
    print(
        f"Ready to grasp from ({pose[0]:.2f}, {pose[1]:.2f}, yaw {pose[2]:.2f}),"
        f" parked {offset:.3f} m from it."
    )
    return READY


def pick_up(obj):
    """Re-observe the object from where the base is parked and pick it up.

    Runs core/grasping/pick.py in its own process, which keeps MoveIt Task
    Constructor's C++ node apart from this process's rclpy context."""
    pick = subprocess.run([sys.executable, "-m", "core.grasping.pick", obj], cwd=ROOT)
    return PICKED if pick.returncode == 0 else NOT_PICKED


def search(scene, obj, navigator, furniture=None, top_k=3, observe=locate, views=2):
    """Try at least `views` viewpoints per location, more when the furniture is too
    large for them to cover. Perception/server errors stop the run.

    Returns (status, recorded object node), the node being None when nothing was seen."""
    if views < 1:
        raise ValueError("views must be positive")
    # Advancing this iterator requests LLM fallback only when needed.
    for location in targets(scene, obj, furniture, navigator.robot_xy(), top_k):
        poses, needed = plan(scene, location, navigator.robot_xy())
        limit = max(views, needed)
        outcomes = VIEWS[view_key(location)]["outcomes"]
        observations = 0
        for pose in poses:
            if not navigator.reachable(*pose):
                outcomes[pose] = "no path"
                continue
            if not navigator.drive_to(*pose):
                outcomes[pose] = "drive failed"
                continue
            if not obj:
                return (ARRIVED, None)
            # A spent view is spent whether the head or the detector failed;
            # otherwise an unaimable target burns every candidate pose.
            observations += 1
            points = None
            aimed = False
            for aim in look_points(scene, location, pose):
                # A shelf height beyond the head's tilt is skipped; the others still count.
                if not navigator.look_at(aim):
                    continue
                aimed = True
                # A missed detection allows retry; communication errors still stop us.
                points = observe(obj)
                if points is not None:
                    break
            if not aimed:
                print("The head cannot aim at the target from this pose.")
                outcomes[pose] = "cannot aim"
            elif points is None:
                outcomes[pose] = "not detected"
            else:
                outcomes[pose] = "detected"
            if points is not None:
                points = np.asarray(points, dtype=float)
                if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
                    raise ValueError("Detection must contain finite map-frame 3D points")
                lower, upper = (points.min(axis=0), points.max(axis=0))
                span = float((upper - lower).max())
                if span > MAX_SPAN:
                    # Recording this would corrupt the remembered box and every
                    # standoff and probe pose later derived from it.
                    print(
                        f"Detection spans {span:.2f} m, beyond the {MAX_SPAN:.2f} m"
                        " a graspable object can measure; discarding this view."
                    )
                    if observations >= limit:
                        break
                    continue
                # A fallback detection updates the remembered object, not a duplicate.
                # Past the limit it is a different instance and deserves its own node.
                object_id = location.object_id
                if object_id is not None and np.linalg.norm(
                    points.mean(axis=0)[:2]
                    - np.asarray(scene.nodes[object_id]["centroid"])[:2]
                ) > SAME_INSTANCE:
                    object_id = None
                if object_id is None:
                    object_id = sg.find_object(
                        scene, obj, near=points.mean(axis=0)[:2], limit=SAME_INSTANCE
                    )
                label, name = (obj, "")
                if object_id is not None:
                    # Keep the asset label. Renaming it to the query splits the label
                    # space, and sibling instances stop matching each other.
                    label = scene.nodes[object_id]["label"]
                    name = scene.nodes[object_id]["name"]
                furniture_id, relation = observed_furniture(scene, points)
                object_id = sg.record_object(
                    scene,
                    label,
                    (lower + upper) / 2,
                    upper - lower,
                    furniture_id,
                    frame_id="map",
                    node_id=object_id,
                    relation=relation,
                    name=name,
                )
                print(f"Found {obj} in map at {points.mean(axis=0).round(3).tolist()}")
                return (FOUND, object_id)
            if observations >= limit:
                break
        if observations == 0:
            print("Location could not be observed: navigation failed; trying the next location.")
        else:
            print("Object not detected in completed views; trying the next location.")
    return (NOT_FOUND, None)


def execute_mission(scene, obj, navigator, graph_path, *, furniture=None, top_k=3,
                    bearings=6, obstacles="--costmap", refine=True, grasp=True):
    """Search, persist the observation, park, then optionally perform a verified pick.

    Both CLI entry points use this sequence. Errors propagate so a failed stage
    cannot silently advance to robot motion in the next stage.
    """
    if grasp and not refine:
        raise ValueError("Pickup requires arm-reachable base placement")
    navigator.resume_navigation_if_paused()
    print(f"[SEARCH] target={obj!r}; graph={graph_path}", flush=True)
    status, object_id = search(scene, obj, navigator, furniture, top_k)
    if object_id is not None:
        # Preserve the observation even if the subsequent IK/model call fails.
        sg.save(scene, graph_path)
    if status == FOUND and refine:
        if object_id is None:
            raise RuntimeError("Search reported found without a recorded object")
        print('[APPROACH] finding and reaching an arm-reachable base pose', flush=True)
        status = make_graspable(scene, object_id, navigator, obstacles, bearings)
    if status == READY and grasp:
        navigator.pause_navigation()
        print('[GRASP] generating grasps and picking the object up', flush=True)
        status = pick_up(obj)
    print(f'[RESULT] {status}', flush=True)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("object", nargs="?", default="")
    parser.add_argument(
        "--graph", type=Path, default=ROOT / "outputs/scene_graph/apartment.json"
    )
    parser.add_argument("--furniture", help="unique furniture instance name or node ID")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="show first location; unknown objects still call DeepSeek",
    )
    parser.add_argument(
        "--grasp",
        action="store_true",
        help="generate grasps and pick the object up once parked",
    )
    parser.add_argument(
        "--no-refine",
        action="store_true",
        help="stop at detection instead of parking where the arm can reach",
    )
    parser.add_argument(
        "--bearings",
        type=int,
        default=6,
        help="horizontal probe directions asked of the IK solver",
    )
    obstacles = parser.add_mutually_exclusive_group()
    obstacles.add_argument("--map", action="store_const", const="--map", dest="source")
    obstacles.add_argument(
        "--costmap", action="store_const", const="--costmap", dest="source"
    )
    parser.set_defaults(source="--costmap")
    args = parser.parse_args()
    if not args.object and (not args.furniture):
        parser.error("give an object or --furniture")
    if args.top_k < 1:
        parser.error("--top-k must be positive")
    if args.bearings < 1:
        parser.error("--bearings must be positive")
    if args.grasp and (args.no_refine or not args.object):
        parser.error("--grasp requires an object and base refinement")
    scene = sg.load(args.graph)
    if args.dry_run:
        first = next(
            targets(scene, args.object, args.furniture, top_k=args.top_k), None
        )
        if first is None:
            return 1
        poses, _ = plan(scene, first)
        for pose in poses[:3]:
            print(" ", pose)
        return 0
    import rclpy
    from core.navigation.nav2_client import Navigator

    rclpy.init()
    navigator = Navigator()
    try:
        status = execute_mission(
            scene, args.object, navigator, args.graph, furniture=args.furniture,
            top_k=args.top_k, bearings=args.bearings, obstacles=args.source,
            refine=not args.no_refine, grasp=args.grasp,
        )
        return EXIT_CODES[status]
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
