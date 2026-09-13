"""A labelled point set in metres. The caller supplies its frame to graph.build."""

from __future__ import annotations
import re
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import yaml
from core.utils.geometry import box_corners

LABELS = Path(__file__).resolve().parents[2] / "config/scene_graph/scannet200.yaml"
ROLES = ("furniture", "object", "structure")


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


def _load(path):
    """ScanNet200 class -> role, and every other name, normalized -> its class."""
    data = yaml.safe_load(Path(path).read_text())
    classes = {}
    for role_name in ROLES:
        for name in data[role_name]:
            if name in classes:
                raise ValueError(f"{name!r} is listed under two roles in {path}")
            classes[name] = role_name
    aliases = {}
    for section in ("gazebo", "synonyms"):
        for target, names in data[section].items():
            if target not in classes:
                raise ValueError(f"{target!r} in {path} is not a ScanNet200 class")
            for name in names:
                if aliases.setdefault(normalize(name), target) != target:
                    raise ValueError(f"{name!r} names two classes in {path}")
    return classes, aliases


CLASSES, ALIASES = _load(LABELS)


def scannet_class(label):
    """The ScanNet200 class a label names, or None. The whole name is tried
    first, then its last word: English puts the noun last, so a kitchen chair
    is a chair, while "tv stand" stays a class of its own."""
    words = normalize(label).split()
    if not words:
        return None
    for candidate in (" ".join(words), words[-1]):
        if candidate in CLASSES:
            return candidate
        if candidate in ALIASES:
            return ALIASES[candidate]
    return None


def role(label):
    """furniture, object or structure. A label outside the dictionary, such as a
    search query, names an object."""
    return CLASSES.get(scannet_class(label), "object")


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


def is_furniture(label):
    return role(label) == "furniture"


def is_structure(label):
    return role(label) == "structure"


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
