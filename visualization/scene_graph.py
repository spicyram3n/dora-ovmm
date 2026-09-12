"""View OpenYOLO instances and graph connections in native Rerun.

Usage: python3 visualization/scene_graph.py --instances INSTANCES.json --output DIR"""
import argparse
import colorsys
import json
import sys
from pathlib import Path

import networkx as nx
import numpy as np
import rerun as rr

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.scene_graph.openyolo3d import load_instances  # noqa: E402
from core.scene_graph import graph as sg  # noqa: E402
from core.scene_graph.relations import distance_to  # noqa: E402


def preview_graph(instances, frame):
    """Proximity-only inspection graph; never claim unregistered points are in map."""
    graph = nx.DiGraph(frame_id=frame, source_frame=frame, units='m',
                       preview=True, registration_verified=False,
                       relation_method='nearest furniture bounding box within 2 metres; proximity only')
    for i, item in enumerate(instances):
        graph.add_node(i, name=item.name, label=item.label, confidence=float(item.confidence),
                       movable=bool(item.movable), centroid=item.centroid.tolist(),
                       dimensions=item.dimensions.tolist(),
                       bounds=[item.lower.tolist(), item.upper.tolist()], room=None)
    furniture = [i for i, item in enumerate(instances) if not item.movable]
    for i, item in enumerate(instances):
        if item.movable and furniture:
            target = min(furniture, key=lambda j: distance_to(item.centroid, instances[j]))
            distance = distance_to(item.centroid, instances[target])
            if distance <= 2.0:
                graph.add_edge(i, target, relation='near', distance_to_box_m=round(distance, 3))
    return graph


def write_view(instances, graph, output, max_points, scene_path=None, matrix=None):
    rr.init('hrl_scene_graph', strict=True)
    rr.save(str(output))
    rr.log('description', rr.TextDocument(
        f"Frame: {graph.graph['frame_id']} (metres). "
        "Colors identify instances. Labels include model scores. "
        "Edges are geometric hypotheses; near does not imply on or inside."), static=True)
    if scene_path:
        import trimesh
        points = np.asarray(trimesh.load(scene_path, process=False).vertices)
        points = points[np.linspace(0, len(points)-1, min(len(points), 100000), dtype=int)]
        if matrix is not None:
            points = points @ matrix[:3, :3].T + matrix[:3, 3]
        rr.log('scene/context', rr.Points3D(points, colors=[130, 130, 130], radii=0.002), static=True)
    by_name = {item.name: item for item in instances}
    for node, data in graph.nodes(data=True):
        item = by_name[data['name']]
        points = item.points
        points = points[np.linspace(0, len(points)-1, min(len(points), max_points), dtype=int)]
        color = [int(c * 255) for c in colorsys.hsv_to_rgb((node * 0.618034) % 1, 0.7, 0.95)]
        path = f"scene/instances/{data['name']}"
        rr.log(path + '/points', rr.Points3D(points, colors=color, radii=0.003), static=True)
        rr.log(path + '/box', rr.Boxes3D(
            centers=[(item.lower + item.upper) / 2], half_sizes=[item.dimensions / 2],
            colors=color, labels=[f"{item.name}: {item.label} ({item.confidence:.3f})"],
            show_labels=True), static=True)
    for source, target, edge in graph.edges(data=True):
        start, end = graph.nodes[source]['centroid'], graph.nodes[target]['centroid']
        rr.log(f'scene/relations/{source}_to_{target}', rr.Arrows3D(
            origins=[start], vectors=[np.asarray(end)-start], colors=[200, 200, 200],
            radii=0.008, labels=[edge['relation']], show_labels=True), static=True)
    rr.flush()
    rr.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--instances', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='Directory for graph.json and scene.rrd (replaced on rerun)')
    parser.add_argument('--save-only', action='store_true', help='Save recording without opening a desktop window')
    parser.add_argument('--transform', type=Path, help='Measured registration JSON; omit for scan preview')
    parser.add_argument('--scene', type=Path, help='Optional original PLY shown in gray')
    parser.add_argument('--max-points', type=int, default=30000, help='Display limit per instance; all points used for graph')
    args = parser.parse_args()
    if args.max_points < 1:
        parser.error('--max-points must be positive')
    instances, frame = load_instances(args.instances)
    if not instances or len({item.name for item in instances}) != len(instances):
        parser.error('Instances must be nonempty and have unique names')
    matrix = None
    if args.transform:
        registration = json.loads(args.transform.read_text())
        if registration['source_frame'] != frame:
            parser.error('Registration and instance source frames differ')
        matrix = sg.rigid_transform(registration['map_from_source'])
        graph = sg.build(instances, source_frame=frame, map_from_source=matrix, drop_structure=False)
        instances = [sg.transform_instance(item, matrix) for item in instances]
    else:
        graph = preview_graph(instances, frame)
    args.output.mkdir(parents=True, exist_ok=True)
    if matrix is None:
        (args.output/'graph.json').write_text(json.dumps(nx.node_link_data(graph, edges='edges'), indent=2))
    else:
        sg.save(graph, args.output/'graph.json')
    write_view(instances, graph, args.output/'scene.rrd', args.max_points, args.scene, matrix)
    print(f'{graph.number_of_nodes()} nodes, {graph.number_of_edges()} edges in {graph.graph["frame_id"]}')
    print(f'Recording: {args.output / "scene.rrd"}')
    if not args.save_only:
        import subprocess
        import sys
        subprocess.run([sys.executable, '-m', 'rerun', str(args.output / 'scene.rrd')], check=True)


if __name__ == '__main__':
    main()
