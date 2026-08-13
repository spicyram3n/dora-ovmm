"""The contract every scene source satisfies: a labelled blob of points in the
map frame. Gazebo's world file and Mask3D's masks both reduce to this, and
nothing downstream can tell which one it got.

Points rather than a box, because points are what Mask3D produces natively and
a box is one line to derive from them.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

import numpy as np

# Static things the robot searches *at*: the destinations of a search, and
# what the LLM gets to choose between. Appliances count, since "is it in the
# fridge" is the same question as "is it in the cabinet" as far as the robot
# is concerned.
FURNITURE = frozenset({
    "armchair", "bed", "bench", "bookcase", "bookshelf", "cabinet", "cart",
    "chair", "counter", "countertop", "couch", "cupboard", "desk",
    "dishwasher", "drawers", "dresser", "freezer", "fridge", "island",
    "microwave", "nightstand", "oven", "pantry", "rack", "refrigerator",
    "shelf", "shelves", "sideboard", "sink", "sofa", "stand", "stool",
    "stove", "table", "trolley", "vanity", "wagon", "wardrobe", "washer",
    "worktop",
})

# Building fabric. Never a node: a wall is not somewhere to look for a mug,
# and letting one become furniture means the LLM can send the robot at it.
STRUCTURE = frozenset({
    "banister", "beam", "blinds", "ceiling", "column", "curtain", "door",
    "doorframe", "floor", "ground", "pillar", "railing", "roof", "stairs",
    "staircase", "wall", "window",
})


def normalize(label):
    """'High_Shelf', 'sofa01' and 'coffee-table 2' all land on the same words
    a Mask3D category would use, so one vocabulary serves every source."""
    words = re.sub(r"[^a-z0-9]+", " ", label.lower()).split()
    return " ".join(word for word in (re.sub(r"\d+$", "", w) for w in words) if word)


def _matches(label, vocabulary):
    """Two naming conventions collide here, and the separator tells them apart.

    A natural-language category ('coffee table', 'table lamp') is an English
    compound, so the head noun is last and only the head may decide -- that is
    what keeps a table lamp from counting as a table. An asset name
    ('high_shelf', 'sofa-fix') puts the category first and a variant after, so
    any word may decide.

    Where that guesses wrong it errs towards furniture, because a shelf that
    never becomes a search destination is a broken run, whereas a lamp that
    wrongly becomes one only wastes a look. Sources that know better should
    pass `movable` outright rather than lean on any of this.
    """
    text = normalize(label)
    if text in vocabulary:
        return True
    words = text.split()
    if not words:
        return False
    candidates = words if re.search(r"[_-]", label) else words[-1:]
    return any(word in vocabulary
               or any(word.endswith(term) for term in vocabulary if len(term) >= 4)
               for word in candidates)


def match_score(label, wanted):
    """How well `label` names the thing `wanted` asks for; 0.0 for not at all.

    Word overlap, not equality, because both sides carry words the other will
    not: an asset name prefixes ('hsr_pringles'), and a person adds the
    category ('pringles can'). Requiring either to be a subset of the other
    sent every two-word query to the LLM for an object already in the graph.

    Scored rather than boolean so 'pringles can' prefers 'hsr_pringles' over a
    'trash_can' that happens to share the word 'can'.
    """
    mine, theirs = set(normalize(label).split()), set(normalize(wanted).split())
    shared = mine & theirs
    return len(shared) / len(mine | theirs) if shared else 0.0


def same_object(label, wanted):
    """Does `label` name the thing `wanted` asks for?"""
    return match_score(label, wanted) > 0.0


def is_furniture(label, vocabulary=FURNITURE):
    return _matches(label, vocabulary)


def is_structure(label, vocabulary=STRUCTURE):
    return _matches(label, vocabulary)


@dataclass
class Instance:
    label: str
    points: np.ndarray            # (N, 3), map frame, metres
    confidence: float = 1.0
    movable: bool | None = None   # None: decide from the label
    name: str = ""                # the source's own id, for traceability

    lower: np.ndarray = field(init=False)
    upper: np.ndarray = field(init=False)
    centroid: np.ndarray = field(init=False)

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float).reshape(-1, 3)
        if len(self.points) == 0:
            raise ValueError(f"instance {self.label!r} has no points")
        self.lower, self.upper = self.points.min(axis=0), self.points.max(axis=0)
        self.centroid = self.points.mean(axis=0)
        # A source that knows better says so; the vocabulary is the fallback,
        # and no heuristic over names will ever be right for every scene.
        if self.movable is None:
            self.movable = not is_furniture(self.label)

    @property
    def dimensions(self):
        return self.upper - self.lower


def from_box(label, centre, dimensions, **kwargs):
    """An Instance from a box, for sources that have one already. The eight
    corners carry everything the graph asks for."""
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    points = np.asarray(centre, float) + corners * np.asarray(dimensions, float) / 2
    return Instance(label, points, **kwargs)
