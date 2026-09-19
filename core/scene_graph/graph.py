"""Build a map-frame graph with object-to-furniture edges and save it as JSON."""

import json
import networkx as nx
import numpy as np
from . import relations
from .instance import Instance, from_box, is_structure, match_score
from core.utils import geometry

_EDGES = "edges"


def rigid_transform(matrix):
    """Validate T_map_source. Scale must be corrected before registration."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("map_from_source must be a finite 4x4 matrix")
    # Reject scale, reflection, or malformed transforms before moving any points.
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
    # Move the cloud into the new frame while preserving its label and metadata.
    points = item.points @ matrix[:3, :3].T + matrix[:3, 3]
    return Instance(item.label, points, item.confidence, item.movable, item.name)


def _attributes(label, centroid, dimensions, movable, confidence, name, room=None):
    # Convert NumPy values to rounded Python numbers for compact JSON storage.
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


def _fitted_footprint(points):
    """A furniture node's turned floor outline, rounded like its other attributes."""
    # Fit a rotated floor rectangle instead of using a larger axis-aligned outline.
    centre, size, yaw = geometry.fit_footprint(points)
    rounded_centre = []
    for value in centre:
        rounded_centre.append(round(float(value), 3))
    rounded_size = []
    for value in size:
        rounded_size.append(round(float(value), 3))
    return {"centre": rounded_centre, "size": rounded_size, "yaw": round(float(yaw), 4)}


def footprint(data):
    """A furniture node as (centre, dimensions along its own axes, yaw in map).

    Graphs saved before footprints were fitted fall back to the axis-aligned bounds."""
    lower, upper = np.asarray(data["bounds"], dtype=float)
    centre, dimensions = ((lower + upper) / 2, upper - lower)
    # Use the old axis-aligned box when a graph has no fitted footprint.
    if "footprint" not in data:
        return (centre, dimensions, 0.0)
    centre[:2] = data["footprint"]["centre"]
    dimensions[:2] = data["footprint"]["size"]
    return (centre, dimensions, float(data["footprint"]["yaw"]))


# An object's box underside this close to z = 0 means it rests on the floor:
# map z = 0 is base_footprint, the floor the base drives on.
FLOOR = 0.15
# Shorter than this and the base rides over it: a cable, a mat, a threshold.
MIN_BLOCKER_HEIGHT = 0.05
# Taller than this and a box touching the floor is not an object standing on it:
# it is a door leaf, a mirror or a wall-mounted whiteboard whose detection ran
# floor to ceiling. Those matter more than their height suggests, because an
# object blocks by its axis-aligned bounds: a 5 cm panel at 40 deg to the map
# axes inflates into a square about a metre on a side. One such box, a door
# recorded open, sealed the only way into the kitchen on lab_20260919. The
# tallest real floor-standing object measured across both recordings is a
# 1.17 m computer tower; the two artefacts were 2.12 m and 2.36 m.
MAX_BLOCKER_HEIGHT = 1.8
# Classes the live laser already handles better than a recording can. Everything
# else here is static between the recording and the run, so its recorded box is
# still true; a person is not, and their box spans the 0.19 m scan height, so
# Nav2's obstacle layer marks them while they stand there and clears them when
# they leave. Keeping the recorded box would instead hold a keep-out over floor
# nobody is on, and standoff.blocks is pure geometry: no sensor can clear it.
TRANSIENT = ("person",)


def standing_on_floor(data):
    """Whether an object's box is something standing on the floor that the base hits.

    The graph's own `on`/`in` edge will not do: a computer tower is edged `in` the
    desk because it is inside the desk's box, and it is standing under it on the
    floor. Only furniture gets a fitted yaw, so an object blocks by its bounds.

    The height has to fall between MIN_BLOCKER_HEIGHT and MAX_BLOCKER_HEIGHT:
    below the band the base rides over it, above it the box is a panel fixed to a
    wall or a door, which touches the floor without standing on it."""
    lower, upper = np.asarray(data["bounds"], dtype=float)
    height = upper[2] - lower[2]
    return lower[2] <= FLOOR and MIN_BLOCKER_HEIGHT <= height <= MAX_BLOCKER_HEIGHT


def blockers(graph, exclude=()):
    """{node id: (centre, dimensions, yaw)} for every shape the base can run into.

    Furniture by its fitted footprint, plus objects standing on the floor by their
    axis-aligned bounds. Pass the target's node id in `exclude`: a bin is wider
    than the parking clearance, so left in it would rule out its own approach."""
    shapes = {}
    for node_id, data in graph.nodes(data=True):
        if node_id in exclude:
            continue
        if not data["movable"]:
            shapes[node_id] = footprint(data)
        elif data["label"] not in TRANSIENT and standing_on_floor(data):
            lower, upper = np.asarray(data["bounds"], dtype=float)
            shapes[node_id] = ((lower + upper) / 2, upper - lower, 0.0)
    return shapes


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
    # Optionally remove walls and other structural geometry from the graph.
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
        if not item.movable:
            # The bounds of a turned piece are larger than it; views follow its sides.
            graph.nodes[node_id]["footprint"] = _fitted_footprint(item.points)
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
    # Connect each movable object to the furniture that supports or contains it.
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
    # Send the reasoner only the furniture ID, label, room, and map position.
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
        # Furniture nodes are the fixed places the robot can search around.
        if not data["movable"]:
            results[node_id] = data
    return results


def objects(graph):
    results = {}
    for node_id, data in graph.nodes(data=True):
        # Movable nodes represent the objects being searched for.
        if data["movable"]:
            results[node_id] = data
    return results


def find_objects(graph, label, near=None):
    """Every best-label match, nearest `near` first, or most confident first without it."""
    scored_objects = []
    best_score = 0.0
    for node_id, data in objects(graph).items():
        # Match both the shared class and the more specific instance name.
        score = max(match_score(data["label"], label), match_score(data["name"], label))
        scored_objects.append((score, node_id))
        best_score = max(best_score, score)
    # Return no match when neither the label nor instance name shares query words.
    if best_score == 0.0:
        return []
    matches = []
    for score, node_id in scored_objects:
        if score == best_score:
            matches.append(node_id)
    # Break equal label matches by confidence, or by distance if a location was supplied.
    if near is None:

        def confidence(node_id):
            return -graph.nodes[node_id]["confidence"]

        return sorted(matches, key=confidence)

    def distance(node_id):
        return _distance_to(graph, node_id, near)

    return sorted(matches, key=distance)


def _distance_to(graph, node_id, near):
    origin = np.asarray(near, float)[:2]
    object_xy = np.array(graph.nodes[node_id]["centroid"][:2])
    return float(np.linalg.norm(object_xy - origin))


def find_object(graph, label, near=None, limit=None):
    """The best label match, or None. `limit` rejects a match that far from `near`."""
    if limit is not None and near is None:
        raise ValueError("limit needs a position to measure the match against")
    matches = find_objects(graph, label, near)
    if not matches:
        return None
    # Beyond the limit this is a different instance, not the remembered one.
    if limit is not None and _distance_to(graph, matches[0], near) > limit:
        return None
    return matches[0]


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
    # Allocate a new node ID only when this is not an update to an existing object.
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
    # Replace the object's old furniture link with the latest observation.
    graph.remove_edges_from(list(graph.out_edges(node_id)))
    if furniture_id is not None:
        graph.add_edge(node_id, furniture_id, relation=relation)
    return node_id


def set_rooms(graph, assignment):
    """Apply {furniture id: room name}, propagating each room to the objects that belong to that furniture."""
    for node_id, room in assignment.items():
        if node_id in graph:
            graph.nodes[node_id]["room"] = room
    # Give each object the room assigned to its furniture.
    for node_id in objects(graph):
        target, _ = location_of(graph, node_id)
        if target is not None:
            graph.nodes[node_id]["room"] = graph.nodes[target]["room"]


def save(graph, path):
    require_map(graph)
    # Store graph metadata, nodes, and relations together in one JSON file.
    with open(path, "w") as handle:
        json.dump(nx.node_link_data(graph, edges=_EDGES), handle, indent=2)


def load(path):
    # Restore the saved graph, then require metre-valued map coordinates.
    with open(path) as handle:
        graph = nx.node_link_graph(json.load(handle), edges=_EDGES)
    require_map(graph)
    return graph
