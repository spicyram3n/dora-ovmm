"""A labelled point set in metres. The caller supplies its frame to graph.build."""

from __future__ import annotations
import re
from dataclasses import dataclass, field
import numpy as np
from core.utils.geometry import box_corners

FURNITURE = frozenset(
    {
        "armchair",
        "bed",
        "bench",
        "bookcase",
        "bookshelf",
        "cabinet",
        "cart",
        "chair",
        "counter",
        "countertop",
        "couch",
        "cupboard",
        "desk",
        "dishwasher",
        "drawers",
        "dresser",
        "freezer",
        "fridge",
        "island",
        "microwave",
        "nightstand",
        "oven",
        "pantry",
        "rack",
        "refrigerator",
        "shelf",
        "shelves",
        "sideboard",
        "sink",
        "sofa",
        "stand",
        "stool",
        "stove",
        "table",
        "trolley",
        "vanity",
        "wagon",
        "wardrobe",
        "washer",
        "worktop",
    }
)
STRUCTURE = frozenset(
    {
        "banister",
        "beam",
        "blinds",
        "ceiling",
        "column",
        "curtain",
        "door",
        "doorframe",
        "floor",
        "ground",
        "pillar",
        "railing",
        "roof",
        "stairs",
        "staircase",
        "wall",
        "window",
    }
)


def normalize(label):
    """Turn asset names into lowercase words without trailing instance numbers."""
    lowercase_label = label.lower()
    separated_words = re.sub("[^a-z0-9]+", " ", lowercase_label)
    words = separated_words.split()
    clean_words = []
    for word in words:
        clean_word = re.sub("\\d+$", "", word)
        if clean_word:
            clean_words.append(clean_word)
    return " ".join(clean_words)


def _matches(label, vocabulary):
    """Match natural-language nouns or words in a Gazebo asset name."""
    text = normalize(label)
    if text in vocabulary:
        return True
    words = text.split()
    if not words:
        return False
    if re.search("[_-]", label):
        candidates = words
    else:
        candidates = words[-1:]
    for word in candidates:
        if word in vocabulary:
            return True
        for term in vocabulary:
            if len(term) >= 4 and word.endswith(term):
                return True
    return False


def match_score(label, wanted):
    """Word overlap lets 'pringles can' match the asset label 'hsr_pringles'."""
    label_words, query_words = (
        set(normalize(label).split()),
        set(normalize(wanted).split()),
    )
    shared = label_words & query_words
    if shared:
        return len(shared) / len(label_words | query_words)
    else:
        return 0.0


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
    points: np.ndarray
    confidence: float = 1.0
    movable: bool | None = None
    name: str = ""
    lower: np.ndarray = field(init=False)
    upper: np.ndarray = field(init=False)
    centroid: np.ndarray = field(init=False)

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float).reshape(-1, 3)
        if not np.isfinite(self.points).all():
            raise ValueError("instance points must be finite")
        if len(self.points) == 0:
            raise ValueError(f"instance {self.label!r} has no points")
        self.lower, self.upper = (self.points.min(axis=0), self.points.max(axis=0))
        self.centroid = self.points.mean(axis=0)
        if self.movable is None:
            self.movable = not is_furniture(self.label)

    @property
    def dimensions(self):
        return self.upper - self.lower


def from_box(label, centre, dimensions, **kwargs):
    """Create an instance from the eight corners of a box."""
    dimensions = np.asarray(dimensions, float)
    valid_shape = dimensions.shape == (3,)
    valid_values = np.isfinite(dimensions).all()
    if not valid_shape or not valid_values or np.any(dimensions < 0):
        raise ValueError("dimensions must be three finite nonnegative lengths")
    points = box_corners(np.asarray(centre, float), dimensions)
    return Instance(label, points, **kwargs)
