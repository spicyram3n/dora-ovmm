"""Group the furniture into rooms.

The LLM does this because it is the part that needs world knowledge: "sofa
plus low table" reads as a living room in a way no geometric rule knows, and
coordinates alone can never produce a *name*. What it is not asked to do is
the arithmetic. stretch-compose's prompt tells the model to "cluster using
k-means on the x-y-center-coordinates", which is asking a language model to
approximate an algorithm; here the positions go in as numbers and only the
grouping and the naming come back.
"""

import json

from pydantic import BaseModel

from scene_graph import graph as sg
from spatial_reasoning._deepseek import ask_json

SYSTEM = (
    "You are given the furniture in one home, each with an id, a label and an "
    "x y z centroid in metres. Group them into rooms, using both how close "
    "they are and what kind of room their labels imply, and give each room a "
    "name. Every id must appear exactly once. "
    'Reply as json: {"rooms": {"<id>": "<room name>"}}'
)


class _Rooms(BaseModel):
    rooms: dict[str, str]


def assign(scene, client, model="deepseek-reasoner"):
    """Fill in every furniture node's room, and its objects' rooms with it.
    Returns the {id: room} mapping that was applied."""
    listing = [{"id": node_id, "label": data["label"], "centroid": data["centroid"]}
               for node_id, data in sg.furniture(scene).items()]
    answer = ask_json(client, SYSTEM, json.dumps(listing), _Rooms, model)

    # Ids come back as json keys, so as strings, and a model will occasionally
    # name one that is not in the scene at all.
    assignment = {int(k): v for k, v in answer.rooms.items()
                  if k.lstrip("-").isdigit() and int(k) in scene}
    sg.set_rooms(scene, assignment)
    return assignment
