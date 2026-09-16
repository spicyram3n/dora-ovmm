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
    # Replace punctuation with spaces before removing numeric instance suffixes.
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
    # Resolve simulator names and synonyms to the same class vocabulary.
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
    """Try the complete label first, then its last word, using known classes and aliases."""
    words = normalize(label).split()
    if not words:
        return None
    # Prefer full names such as tv stand before trying a general noun such as stand.
    for candidate in (" ".join(words), words[-1]):
        if candidate in CLASSES:
            return candidate
        if candidate in ALIASES:
            return ALIASES[candidate]
    return None


def role(label):
    """Return furniture, object, or structure; treat unknown labels as objects."""
    return CLASSES.get(scannet_class(label), "object")


def match_score(label, wanted):
    """Word overlap lets 'pringles can' match the asset label 'hsr_pringles'."""
    label_words, query_words = (
        set(normalize(label).split()),
        set(normalize(wanted).split()),
    )
    # Score the fraction of unique words shared by the label and query.
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
        # Store the cloud bounds and mean position for later spatial checks.
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
    # Require three valid nonnegative dimensions before constructing box corners.
    if not valid_shape or not valid_values or np.any(dimensions < 0):
        raise ValueError("dimensions must be three finite nonnegative lengths")
    points = box_corners(np.asarray(centre, float), dimensions)
    return Instance(label, points, **kwargs)
