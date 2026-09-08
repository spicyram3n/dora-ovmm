"""Build a map-frame graph with object-to-furniture edges and save it as JSON."""

import json
import networkx as nx
import numpy as np
from . import relations
from .instance import Instance, from_box, is_structure, match_score

_EDGES = "edges"


def rigid_transform(matrix):
    """Validate T_map_source. Scale must be corrected before registration."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("map_from_source must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if (
        not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-06)
        or (not np.isclose(np.linalg.det(rotation), 1, atol=1e-06))
    ):
        raise ValueError(
            "map_from_source must be a rigid transform without scale or reflection"
        )
    return matrix


def transform_instance(item, matrix):
    points = item.points @ matrix[:3, :3].T + matrix[:3, 3]
    return Instance(item.label, points, item.confidence, item.movable, item.name)


def _attributes(label, centroid, dimensions, movable, confidence, name, room=None):
    rounded_centroid = []
    for value in np.asarray(centroid, float).reshape(3):
        rounded_centroid.append(round(float(value), 3))
    rounded_dimensions = []
    for value in np.asarray(dimensions, float).reshape(3):
        rounded_dimensions.append(round(float(value), 3))
    return {
        "label": label,
        "name": name,
        "movable": bool(movable),
        "confidence": round(float(confidence), 3),
        "centroid": rounded_centroid,
        "dimensions": rounded_dimensions,
        "room": room,
    }


def build(instances, *, source_frame, map_from_source, drop_structure=True, **kwargs):
    """Transform instances into map and connect objects to furniture."""
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ValueError("source_frame must be named explicitly")
    matrix = rigid_transform(map_from_source)
    if source_frame == "map" and (not np.allclose(matrix, np.eye(4))):
        raise ValueError("points already in map require an identity transform")
    # Store geometry in map once; source coordinates remain provenance only.
    previous_instances = instances
    instances = []
    for item in previous_instances:
        instances.append(transform_instance(item, matrix))
    if drop_structure:
        previous_instances = instances
        instances = []
        for item in previous_instances:
            if not is_structure(item.label):
                instances.append(item)
    graph = nx.DiGraph(
        frame_id="map",
        units="m",
        source_frame=source_frame,
        map_from_source=matrix.tolist(),
    )
    for node_id, item in enumerate(instances):
        graph.add_node(
            node_id,
            **_attributes(
                item.label,
                item.centroid,
                item.dimensions,
                item.movable,
                item.confidence,
                item.name,
            )
        )
        graph.nodes[node_id]["bounds"] = [item.lower.tolist(), item.upper.tolist()]
    furniture_ids = []
    for index, item in enumerate(instances):
        if not item.movable:
            furniture_ids.append(index)
    pieces = []
    for index in furniture_ids:
        pieces.append(instances[index])
    shadows = []
    for piece in pieces:
        shadows.append(relations.footprint(piece))
    for node_id, item in enumerate(instances):
        if not item.movable:
            continue
        index, relation = relations.classify(item, pieces, shadows, **kwargs)
        if index is not None:
            graph.add_edge(node_id, furniture_ids[index], relation=relation)
    return graph


def require_map(graph):
    if graph.graph.get("frame_id") != "map" or graph.graph.get("units") != "m":
        raise ValueError("scene graph must explicitly use map coordinates in metres")


def furniture_listing(graph):
    results = []
    for node_id, data in furniture(graph).items():
        results.append(
            dict(
                id=node_id,
                label=data["label"],
                room=data["room"],
                centroid=data["centroid"],
            )
        )
    return results


def furniture(graph):
    """The nodes the LLM chooses between, and that Nav2 goals derive from."""
    results = {}
    for node_id, data in graph.nodes(data=True):
        if not data["movable"]:
            results[node_id] = data
    return results


def objects(graph):
    results = {}
    for node_id, data in graph.nodes(data=True):
        if data["movable"]:
            results[node_id] = data
    return results


def find_object(graph, label, near=None):
    """Choose the best label match, then break ties by confidence or distance."""
    scored_objects = []
    best_score = 0.0
    for node_id, data in objects(graph).items():
        score = match_score(data["label"], label)
        scored_objects.append((score, node_id))
        best_score = max(best_score, score)
    if best_score == 0.0:
        return None
    matches = []
    for score, node_id in scored_objects:
        if score == best_score:
            matches.append(node_id)
    if near is None:

        def confidence(node_id):
            return graph.nodes[node_id]["confidence"]

        return max(matches, key=confidence)
    robot_xy = np.asarray(near, float)[:2]

    def distance(node_id):
        object_xy = np.array(graph.nodes[node_id]["centroid"][:2])
        return np.linalg.norm(object_xy - robot_xy)

    return min(matches, key=distance)


def location_of(graph, node_id):
    """(furniture id, relation), or (None, None) if unattached."""
    for _, target, data in graph.out_edges(node_id, data=True):
        return (target, data["relation"])
    return (None, None)


def record_object(
    graph,
    label,
    centroid,
    dimensions,
    furniture_id,
    *,
    frame_id,
    node_id=None,
    relation="on",
    confidence=1.0,
    name=""
):
    """Record a map-frame box. Supply node_id to update; otherwise create a node."""
    require_map(graph)
    if frame_id != "map":
        raise ValueError("Transform the observation into map before recording it")
    if furniture_id is not None and furniture_id not in furniture(graph):
        raise ValueError("furniture_id must identify furniture")
    if relation not in ("on", "in", "near"):
        raise ValueError("relation must be on, in, or near")
    if node_id is not None and node_id not in objects(graph):
        raise ValueError("node_id must identify an existing object")
    item = from_box(
        label, centroid, dimensions, movable=True, confidence=confidence, name=name
    )
    if node_id is None:
        node_id = max(graph.nodes, default=-1) + 1
    if furniture_id is not None:
        room = graph.nodes[furniture_id]["room"]
    else:
        room = None
    graph.add_node(
        node_id,
        **_attributes(
            label, item.centroid, item.dimensions, True, confidence, name, room
        )
    )
    graph.nodes[node_id]["bounds"] = [item.lower.tolist(), item.upper.tolist()]
    graph.remove_edges_from(list(graph.out_edges(node_id)))
    if furniture_id is not None:
        graph.add_edge(node_id, furniture_id, relation=relation)
    return node_id


def set_rooms(graph, assignment):
    """Apply {furniture id: room name}, propagating each room to the objects that belong to that furniture."""
    for node_id, room in assignment.items():
        if node_id in graph:
            graph.nodes[node_id]["room"] = room
    for node_id in objects(graph):
        target, _ = location_of(graph, node_id)
        if target is not None:
            graph.nodes[node_id]["room"] = graph.nodes[target]["room"]


def save(graph, path):
    require_map(graph)
    with open(path, "w") as handle:
        json.dump(nx.node_link_data(graph, edges=_EDGES), handle, indent=2)


def load(path):
    with open(path) as handle:
        graph = nx.node_link_graph(json.load(handle), edges=_EDGES)
    require_map(graph)
    return graph
