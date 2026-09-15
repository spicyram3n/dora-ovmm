"""View a saved scene graph and its object-to-furniture edges in native Rerun.

Usage: python3 visualization/scene_graph.py --graph GRAPH.json --output VIEW.rrd [--scene SCENE.ply]
"""

import argparse
import colorsys
import sys
from pathlib import Path

import numpy as np
import rerun as rr

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.scene_graph import graph as sg  # noqa: E402


def write_view(scene, output, scene_path=None):
    rr.init("hrl_scene_graph", strict=True)
    rr.save(str(output))
    rr.log(
        "description",
        rr.TextDocument(
            f"Frame: {scene.graph['frame_id']} (metres). "
            "Colors identify nodes. Labels include model scores. "
            "Edges are geometric hypotheses; near does not imply on or inside."
        ),
        static=True,
    )
    if scene_path:
        import trimesh

        points = np.asarray(trimesh.load(scene_path, process=False).vertices)
        points = points[
            np.linspace(0, len(points) - 1, min(len(points), 100000), dtype=int)
        ]
        rr.log(
            "scene/context",
            rr.Points3D(points, colors=[130, 130, 130], radii=0.002),
            static=True,
        )
    for node, data in scene.nodes(data=True):
        # Furniture turns with its footprint; objects have none and stay axis-aligned.
        centre, dimensions, yaw = sg.footprint(data)
        color = [
            int(c * 255) for c in colorsys.hsv_to_rgb((node * 0.618034) % 1, 0.7, 0.95)
        ]
        rr.log(
            f"scene/nodes/{node}",
            rr.Boxes3D(
                centers=[centre],
                half_sizes=[dimensions / 2],
                rotation_axis_angles=[
                    rr.RotationAxisAngle(axis=[0, 0, 1], radians=yaw)
                ],
                colors=color,
                labels=[f"{node}: {data['label']} ({data['confidence']:.3f})"],
                show_labels=True,
            ),
            static=True,
        )
    for source, target, edge in scene.edges(data=True):
        start, end = scene.nodes[source]["centroid"], scene.nodes[target]["centroid"]
        rr.log(
            f"scene/relations/{source}_to_{target}",
            rr.Arrows3D(
                origins=[start],
                vectors=[np.asarray(end) - start],
                colors=[200, 200, 200],
                radii=0.008,
                labels=[edge["relation"]],
                show_labels=True,
            ),
            static=True,
        )
    rr.get_global_data_recording().flush()  # rerun >= 0.22 has no module-level flush
    rr.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", required=True, type=Path)
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Recording to write, e.g. scene_graph.rrd (replaced on rerun)",
    )
    parser.add_argument(
        "--save-only",
        action="store_true",
        help="Save recording without opening a desktop window",
    )
    parser.add_argument(
        "--scene", type=Path, help="Optional map-frame PLY shown in gray"
    )
    args = parser.parse_args()
    scene = sg.load(args.graph)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_view(scene, args.output, args.scene)
    print(
        f'{scene.number_of_nodes()} nodes, {scene.number_of_edges()} edges in {scene.graph["frame_id"]}'
    )
    print(f"Recording: {args.output}")
    if not args.save_only:
        import subprocess
        import sys

        subprocess.run([sys.executable, "-m", "rerun", str(args.output)], check=True)


if __name__ == "__main__":
    main()
