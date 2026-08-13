"""Instances in, scene graph out, plain JSON on disk.

networkx does the bookkeeping stretch-compose hand-rolled: parallel
ingoing/outgoing dicts, repairing edges on delete, four bespoke json writers.

Shape: every instance is a node, every *movable* one has exactly one outgoing
edge to the furniture it is in/on/near. So the graph is a forest rooted at the
furniture, and "where is the apple" is one hop. Attributes stay json-native,
so `save` needs no encoder and the LLM prompt is a slice of this same
structure rather than a second one that can drift from it.
"""

import json

import networkx as nx
import numpy as np

from scene_graph import relations
from scene_graph.instance import is_structure, match_score

# networkx 3.4 warns unless this is explicit, and flips the default in 3.6.
_EDGES = "edges"


def _attributes(label, centroid, dimensions, movable, confidence, name, room=None):
    return {
        "label": label,
        "name": name,
        "movable": bool(movable),
        "confidence": round(float(confidence), 3),
        "centroid": [round(float(v), 3) for v in np.asarray(centroid, float).reshape(3)],
        "dimensions": [round(float(v), 3) for v in np.asarray(dimensions, float).reshape(3)],
        "room": room,
    }


def build(instances, drop_structure=True, **kwargs):
    """`kwargs` go to relations.classify, so noisier input can loosen the
    support gap without editing this module.

    Walls, floors and doors are dropped by default: they are not places to
    look for anything, and a wall that becomes furniture is a wall the LLM can
    send the robot at.
    """
    if drop_structure:
        instances = [item for item in instances if not is_structure(item.label)]

    graph = nx.DiGraph()
    for node_id, item in enumerate(instances):
        graph.add_node(node_id, **_attributes(item.label, item.centroid, item.dimensions,
                                              item.movable, item.confidence, item.name))

    furniture_ids = [i for i, item in enumerate(instances) if not item.movable]
    pieces = [instances[i] for i in furniture_ids]
    # Once per piece, not once per object: hulls are the expensive part.
    shadows = [relations.footprint(piece) for piece in pieces]

    for node_id, item in enumerate(instances):
        if not item.movable:
            continue
        index, relation = relations.classify(item, pieces, shadows, **kwargs)
        if index is not None:
            graph.add_edge(node_id, furniture_ids[index], relation=relation)
    return graph


def furniture(graph):
    """The nodes the LLM chooses between, and that Nav2 goals derive from."""
    return {n: d for n, d in graph.nodes(data=True) if not d["movable"]}


def objects(graph):
    return {n: d for n, d in graph.nodes(data=True) if d["movable"]}


def find_object(graph, label, near=None):
    """Node id of the `label` the caller means, or None.

    Matched via `same_object`, so 'Apple', 'apple_2' and 'hsr_apple' are all
    the same object; a genuine miss is scenario A, which the LLM handles.

    The apartment holds two pringles cans, and which one "the pringles" means
    depends on where the robot is standing. Pass `near` as an (x, y) to get the
    closest; without it the most confident wins, which is arbitrary between
    ground-truth instances that are all equally certain. Distance is measured
    in the plane, since the robot drives on the floor and a can on a high
    shelf is no further away for being high up.
    """
    scored = [(match_score(d["label"], label), n) for n, d in objects(graph).items()]
    best = max((s for s, _ in scored), default=0.0)
    if best == 0.0:
        return None
    matches = [n for s, n in scored if s == best]
    if near is None:
        return max(matches, key=lambda node: graph.nodes[node]["confidence"])
    point = np.asarray(near, float)[:2]
    return min(matches, key=lambda node:
               np.linalg.norm(np.array(graph.nodes[node]["centroid"][:2]) - point))


def location_of(graph, node_id):
    """(furniture id, relation), or (None, None) if unattached."""
    for _, target, data in graph.out_edges(node_id, data=True):
        return target, data["relation"]
    return None, None


def record_object(graph, label, centroid, dimensions, furniture_id,
                  relation="on", confidence=1.0, name=""):
    """Write a just-found object in, updating in place if already present.
    This is what makes the next search for it a scenario B. Returns its id."""
    if furniture_id is not None and furniture_id not in graph:
        # add_edge would otherwise invent the node, silently, without
        # attributes, and every later furniture() call would die on it.
        raise KeyError(f"no furniture node {furniture_id} in the graph")

    # Nearest, not most confident: the robot is recording the instance it just
    # looked at, which may not be the one the graph is most sure about.
    node_id = find_object(graph, label, near=centroid)
    if node_id is None:
        node_id = max(graph.nodes, default=-1) + 1
    room = graph.nodes[furniture_id]["room"] if furniture_id is not None else None
    graph.add_node(node_id, **_attributes(label, centroid, dimensions, True,
                                          confidence, name, room))
    # One outgoing edge per object is the invariant the rest of this relies on.
    graph.remove_edges_from(list(graph.out_edges(node_id)))
    if furniture_id is not None:
        graph.add_edge(node_id, furniture_id, relation=relation)
    return node_id


def set_rooms(graph, assignment):
    """Apply {furniture id: room name}, propagating each room to the objects
    that belong to that furniture."""
    for node_id, room in assignment.items():
        if node_id in graph:
            graph.nodes[node_id]["room"] = room
    for node_id in objects(graph):
        target, _ = location_of(graph, node_id)
        if target is not None:
            graph.nodes[node_id]["room"] = graph.nodes[target]["room"]


def save(graph, path):
    with open(path, "w") as handle:
        json.dump(nx.node_link_data(graph, edges=_EDGES), handle, indent=2)


def load(path):
    with open(path) as handle:
        return nx.node_link_graph(json.load(handle), edges=_EDGES)
