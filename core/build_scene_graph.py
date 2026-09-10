"""Build the apartment graph using an explicit Gazebo-world-to-map transform."""

import argparse
import json
import sys
import numpy as np
from pathlib import Path
from scene_graph import gazebo, graph as sg

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "ros2_ws/src/tmc_gazebo/tmc_gazebo_worlds/worlds/apartment.world"


def planar_pose(x, y, yaw):
    c, s = (np.cos(yaw), np.sin(yaw))
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


def register(argv):
    import rclpy
    from navigation.nav2_client import Navigator

    parser = argparse.ArgumentParser(description="Save Gazebo-to-map calibration")
    parser.add_argument(
        "--world-base",
        type=float,
        nargs=3,
        required=True,
        metavar=("X", "Y", "YAW"),
        help="current base_footprint pose in Gazebo world",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("config/map/world_to_map.json")
    )
    args = parser.parse_args(argv)
    rclpy.init()
    navigator = Navigator()
    try:
        map_base = navigator.robot_pose()
        # Both poses must describe the same stationary base at calibration time.
        transform = planar_pose(*map_base) @ np.linalg.inv(
            planar_pose(*args.world_base)
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(
                {"source_frame": "gazebo_world", "map_from_source": transform.tolist()},
                indent=2,
            )
        )
        print(f"Map base pose: {map_base}; wrote {args.output}")
        print("Check alignment against the occupancy map before navigation.")
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    if sys.argv[1:2] == ["register"]:
        register(sys.argv[2:])
        return

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, default=WORLD)
    parser.add_argument(
        "--transform",
        type=Path,
        required=True,
        help="JSON containing source_frame and map_from_source (4x4)",
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "outputs/scene_graph/apartment.json"
    )
    parser.add_argument(
        "--rooms", action="store_true", help="ask DeepSeek to assign rooms"
    )
    args = parser.parse_args()
    registration = json.loads(args.transform.read_text())
    if registration["source_frame"] != "gazebo_world":
        raise ValueError("The world-file loader requires source_frame=gazebo_world")
    scene = sg.build(
        gazebo.load_world(args.world),
        source_frame=registration["source_frame"],
        map_from_source=registration["map_from_source"],
    )
    scene.graph["source"] = str(args.world.resolve())
    scene.graph["snapshot"] = "initial_world_file"
    if args.rooms:
        from reasoner import deepseek
        from reasoner.deepseek import get_client

        deepseek.assign(scene, get_client())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sg.save(scene, args.output)
    print(
        f"{len(sg.furniture(scene))} furniture, {len(sg.objects(scene))} objects -> {args.output}"
    )


if __name__ == "__main__":
    main()
