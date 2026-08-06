"""Where should the robot look for an object?

Two scenarios, and the seam between them:
  B  the object is already a node, so its recorded furniture is the first
     place to try and costs no LLM call.
  A  it is not, so the model is asked which of *this* scene's furniture it
     would most likely be at.
If B's memory turns out to be stale (the robot looks and the object is gone)
the search falls through to A. `search_order` is a generator for that reason:
the model is only asked once the remembered location has actually failed.
"""

import json
from dataclasses import dataclass
from pathlib import Path

from pydantic import BaseModel

from scene_graph import graph as sg
from scene_graph.instance import normalize
from spatial_reasoning._deepseek import ask_json, get_client

SYSTEM = (
    "You are given the furniture in one home, each with an id, a label, a room "
    "and an x y z centroid in metres, plus one object to find. Name the {k} "
    "places the object is most likely to be, most likely first, using only ids "
    "from the list. Reply as json: "
    '{{"locations": [{{"furniture_id": <int>, "relation": "on|in|near", '
    '"reason": "<short>"}}]}}'
)


@dataclass
class Location:
    furniture_id: int
    label: str
    room: str
    relation: str
    source: str        # "scene_graph" or "llm"
    reason: str = ""


class _Guess(BaseModel):
    furniture_id: int
    relation: str = "on"
    reason: str = ""


class _Guesses(BaseModel):
    locations: list[_Guess]


def _location(scene, furniture_id, relation, source, reason=""):
    data = scene.nodes[furniture_id]
    return Location(furniture_id, data["label"], data["room"], relation, source, reason)


def known(scene, obj, near=None):
    """Scenario B: what the graph already remembers, or None. `near` is the
    robot's (x, y), which decides between two objects of the same name."""
    node_id = sg.find_object(scene, obj, near)
    if node_id is None:
        return None
    furniture_id, relation = sg.location_of(scene, node_id)
    if furniture_id is None:
        return None
    return _location(scene, furniture_id, relation, "scene_graph")


def _cache_file(cache, obj):
    return Path(cache) / f"{normalize(obj).replace(' ', '_') or 'object'}.json"


def predict(scene, obj, client=None, top_k=3, hint="", exclude=(), model="deepseek-reasoner",
            cache=None):
    """Scenario A: the model's ranked guesses, filtered down to furniture that
    exists and has not already been searched.

    Given a `cache` directory the model is asked once per object and the
    ranking is kept on disk; delete the file to force a fresh answer. What is
    stored is the whole ranking rather than the filtered result, so the same
    file still serves a later search that has already ruled some furniture out.
    """
    pieces = sg.furniture(scene)
    path = _cache_file(cache, obj) if cache is not None else None

    if path is not None and path.exists():
        guesses = json.loads(path.read_text())
    else:
        listing = [{"id": n, "label": d["label"], "room": d["room"], "centroid": d["centroid"]}
                   for n, d in pieces.items()]
        user = json.dumps({"furniture": listing, "object": obj, "hint": hint})
        # Built here rather than by the caller, so that a scenario B search
        # never needs DEEPSEEK_API_KEY set at all.
        answer = ask_json(client or get_client(), SYSTEM.format(k=top_k), user, _Guesses, model)
        guesses = [guess.model_dump() for guess in answer.locations]
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(guesses, indent=2))

    # A model can name an id that is not in the scene, which would be a
    # KeyError at best and the wrong room at worst.
    return [_location(scene, g["furniture_id"], g["relation"], "llm", g["reason"])
            for g in guesses
            if g["furniture_id"] in pieces and g["furniture_id"] not in exclude][:top_k]


def search_order(scene, obj, client=None, top_k=3, hint="", model="deepseek-reasoner",
                 cache=None, near=None):
    """The whole flow as one lazy sequence: the remembered location first, and
    the model only asked if the robot comes back from it empty-handed.

    `near` is the robot's (x, y), used to pick between same-named objects.
    """
    first = known(scene, obj, near)
    if first is None:
        yield from predict(scene, obj, client, top_k, hint, (), model, cache)
        return
    yield first
    yield from predict(scene, obj, client, top_k, hint, (first.furniture_id,), model, cache)
