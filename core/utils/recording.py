"""Where one recording's files live, so a new bag does not mean editing paths.

Every real-robot tool defaults to the same recording, and each of them used to
spell out its own path to it. Switching to another bag meant finding all of them.

    export RECORDING=lab_20260912       # one variable, every tool follows
    python3 realrobot/offline/plan_overlay.py --survey

The `--map`, `--graph` and `--keyframes` flags still win over this, so a one-off
mix of two recordings needs no export. The layout is the convention the data prep
and Boxer steps already write to; `docker/boxer/README.md` has the full tree.
"""

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = "lab_20260811"


def name():
    """The recording every default refers to; $RECORDING overrides it."""
    return os.getenv("RECORDING", DEFAULT).strip() or DEFAULT


def name_from_map(path):
    """The recording a Nav2 map belongs to, for naming that map's outputs.

    Maps live at `config/realrobot/map/<recording>/map.yaml`, so the recording is
    the parent directory, not the stem -- every map.yaml has the stem "map". A
    map from anywhere else keeps falling back to its stem, which is what the
    flat `<recording>.yaml` layout used before.
    """
    path = Path(path)
    return path.parent.name if path.stem == "map" else path.stem


class Paths:
    """One recording's files. Missing ones are still named: callers report them."""

    def __init__(self, recording=None):
        self.name = recording or name()
        config = ROOT / "config/realrobot"
        outputs = ROOT / "outputs/realrobot" / self.name
        # What the robot runs with. The map and its two registrations sit in one
        # folder per recording, so a second scan adds a directory under map/
        # instead of four more files beside the first scan's.
        self.map_dir = config / "map" / self.name
        self.map = self.map_dir / "map.yaml"
        self.registration = self.map_dir / "boxer_to_map.json"
        self.scan_registration = self.map_dir / "scan_to_map.json"
        self.graph = config / "scene_graph" / f"{self.name}.json"
        # What the bag produced.
        self.keyframes = outputs / "keyframes"
        self.slam = outputs / "slam"
        self.boxer = outputs / "boxer"
        self.sequence = outputs / "boxer/scannet" / self.name
        self.boxes = outputs / "boxer" / self.name / "boxer_3dbbs_fused.csv"
        self.nav = outputs / "nav"

    def __repr__(self):
        return f"Paths({self.name!r})"
