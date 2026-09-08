"""Search remembered or LLM-ranked locations using Nav2 and SAM3."""

import argparse
from pathlib import Path
import numpy as np
from navigation import standoff
from scene_graph import graph as sg
from reasoner.query import Location, search_order

ROOT = Path(__file__).resolve().parent.parent


def box(data):
    """The box centre differs from a segmented cloud's mean; prefer saved bounds."""
    lower, upper = np.asarray(data["bounds"])
    return ((lower + upper) / 2, upper - lower)


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


def plan(scene, location, robot_xy=None):
    pieces = sg.furniture(scene)
    blockers = []
    for data in pieces.values():
        blockers.append(box(data))
    if location.furniture_id is None:
        centre, dimensions = (location.centroid, [0, 0, 0])
    else:
        centre, dimensions = box(pieces[location.furniture_id])
    sg.require_map(scene)
    if location.frame_id != "map":
        raise ValueError("Search locations must be in map")
    poses = standoff.candidates(
        centre, dimensions, robot_xy=robot_xy, blockers=blockers
    )
    if location.object_id is not None:
        # Stand outside the furniture, but face the remembered object itself.
        aimed = []
        for x, y, yaw in poses:
            yaw = float(np.arctan2(location.centroid[1] - y, location.centroid[0] - x))
            aimed.append((x, y, yaw))
        poses = aimed
    print(
        f"{location.source}: {location.label}, room={location.room}, {len(poses)} observation poses"
    )
    if location.reason:
        print(f"  {location.reason}")
    return poses


def look_point(scene, location):
    if location.object_id is not None or location.furniture_id is None:
        return location.centroid
    data = scene.nodes[location.furniture_id]
    centre, dimensions = box(data)
    # Tables expose their top; storage furniture is first inspected at mid-height.
    if any(word in data["label"].lower() for word in ("table", "desk", "counter")):
        centre[2] = data["bounds"][1][2] + 0.1
    return centre


def observed_furniture(scene, points):
    from scene_graph.instance import Instance, from_box
    from scene_graph.relations import classify

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
    from perception import pointcloud, sam3_client
    from perception.camera_ros2 import grab_rgbd

    rgb, depth, intrinsics, transform = grab_rgbd(target_frame="map")
    try:
        mask, score = sam3_client.detect(rgb, obj)
    except sam3_client.ObjectNotFound:
        return None
    points = pointcloud.deproject(depth, intrinsics, mask)
    if len(points) < 100 or len(points) < 0.25 * np.count_nonzero(mask):
        print("Object detected, but depth is insufficient; trying another view.")
        return None
    print(f"Detected {obj}, score={score:.2f}")
    return pointcloud.transform_points(transform, points)


def search(scene, obj, navigator, furniture=None, top_k=3, observe=locate, views=2):
    """Try bounded viewpoints per location. Perception/server errors stop the run."""
    if views < 1:
        raise ValueError("views must be positive")
    # Advancing this iterator requests LLM fallback only when needed.
    for location in targets(scene, obj, furniture, navigator.robot_xy(), top_k):
        poses = plan(scene, location, navigator.robot_xy())
        observations = 0
        for pose in poses:
            if not navigator.reachable(*pose) or not navigator.drive_to(*pose):
                continue
            if not obj:
                return True
            if not navigator.look_at(look_point(scene, location)):
                continue
            # A missed detection allows retry; communication errors still stop us.
            points = observe(obj)
            observations += 1
            if points is not None:
                points = np.asarray(points, dtype=float)
                if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
                    raise ValueError("Detection must contain finite map-frame 3D points")
                lower, upper = (points.min(axis=0), points.max(axis=0))
                # A fallback detection updates the remembered object, not a duplicate.
                object_id = location.object_id
                if object_id is None:
                    object_id = sg.find_object(scene, obj, near=points.mean(axis=0)[:2])
                furniture_id, relation = observed_furniture(scene, points)
                sg.record_object(
                    scene,
                    obj,
                    (lower + upper) / 2,
                    upper - lower,
                    furniture_id,
                    frame_id="map",
                    node_id=object_id,
                    relation=relation,
                )
                print(f"Found {obj} in map at {points.mean(axis=0).round(3).tolist()}")
                return True
            if observations >= views:
                break
        if observations == 0:
            print("Location could not be observed: navigation failed; trying the next location.")
        else:
            print("Object not detected in completed views; trying the next location.")
    return False


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
    args = parser.parse_args()
    if not args.object and (not args.furniture):
        parser.error("give an object or --furniture")
    if args.top_k < 1:
        parser.error("--top-k must be positive")
    scene = sg.load(args.graph)
    if args.dry_run:
        first = next(
            targets(scene, args.object, args.furniture, top_k=args.top_k), None
        )
        if first is None:
            return 1
        for pose in plan(scene, first)[:3]:
            print(" ", pose)
        return 0
    import rclpy
    from navigation.nav2_client import Navigator

    rclpy.init()
    navigator = Navigator()
    try:
        found = search(scene, args.object, navigator, args.furniture, args.top_k)
        if found and args.object:
            sg.save(scene, args.graph)
        if found:
            return 0
        else:
            return 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
