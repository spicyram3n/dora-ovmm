"""Build the scene graph from Gazebo's ground truth and save it.

Usage: python3 build_scene_graph.py [--world PATH] [--name NAME] [--no-rooms]

Writes outputs/scene_graph/<name>/graph.json. Runs offline: it reads the
.world file, so Gazebo does not have to be up.

Room names come from an LLM and so need DEEPSEEK_API_KEY; --no-rooms skips
that step and leaves every room null, which is enough to navigate by.
"""

import argparse
from pathlib import Path

from scene_graph import gazebo, graph as sg
from spatial_reasoning import rooms
from spatial_reasoning._deepseek import get_client

REPO = Path(__file__).resolve().parent.parent
DEFAULT_WORLD = REPO / "ros2_ws/src/tmc_gazebo/tmc_gazebo_worlds/worlds/apartment.world"
OUTPUTS = REPO / "outputs" / "scene_graph"


def main(world, name, with_rooms):
    instances = gazebo.load_world(world)
    scene = sg.build(instances)
    print(f"{len(instances)} models -> {scene.number_of_nodes()} nodes "
          f"({len(sg.furniture(scene))} furniture, {len(sg.objects(scene))} objects)")

    if with_rooms:
        assignment = rooms.assign(scene, get_client())
        print(f"rooms: {', '.join(sorted(set(assignment.values())))}")

    directory = OUTPUTS / name
    directory.mkdir(parents=True, exist_ok=True)
    sg.save(scene, directory / "graph.json")
    print(f"saved {directory / 'graph.json'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build a scene graph from a Gazebo world.")
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--name", help="output folder name, defaults to the world's")
    parser.add_argument("--no-rooms", action="store_true", help="skip the LLM room naming")
    arguments = parser.parse_args()
    main(arguments.world, arguments.name or arguments.world.stem, not arguments.no_rooms)
