"""Load Boxer's static 3D boxes into the existing scene graph builder.

Runs in the dev container, not in the Boxer image: it needs core/ and the
graph dependencies, not CUDA. From the repository root:

    python3 -m docker.boxer.to_scene_graph --boxes ... --transform ... --output ...
"""

import argparse
import csv
import json
from itertools import product
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from core.scene_graph.instance import Instance, scannet_class
from core.scene_graph import graph as sg

# Boxer writes one row per box: centre, wxyz quaternion and the extents along
# the box's own axes. utils/file_io.py ObbCsvWriter2 upstream.
COLUMNS = ("tx_world_object", "qw_world_object", "scale_x", "name", "prob")
_SIGNS = np.array(list(product((-1, 1), repeat=3)), dtype=float)


def box_corners(row):
    """The eight corners of one turned box. Corners, not a centre and size,
    because fit_footprint reads a furniture node's yaw back off the points."""
    centre = np.array([float(row["t" + axis + "_world_object"]) for axis in "xyz"])
    size = np.array([float(row["scale_" + axis]) for axis in "xyz"])
    quaternion = [float(row["q" + part + "_world_object"]) for part in "xyzw"]
    return centre + (_SIGNS * size / 2) @ Rotation.from_quat(quaternion).as_matrix().T


def load_boxes(path, min_confidence=0.0):
    """Instances in Boxer's sequence frame, one per row of a static OBB CSV."""
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or not all(column in rows[0] for column in COLUMNS):
        raise ValueError(f"{path} is not a Boxer 3D box CSV")
    # Per-frame boxer_3dbbs.csv repeats each object once per frame it was seen
    # in; only the static files hold one row per object.
    if len({row["time_ns"] for row in rows}) > 1:
        raise ValueError("Boxes span several frames: rerun with --fuse or --track")
    instances = []
    for row in rows:
        confidence = float(row["prob"])
        if confidence < min_confidence:
            continue
        # The prompt Boxer was given is more specific than the class it falls
        # under, so it stays as the name that object search also matches on.
        prompt = row["name"].strip()
        instances.append(
            Instance(
                scannet_class(prompt) or prompt,
                box_corners(row),
                confidence=confidence,
                name=prompt,
            )
        )
    return instances


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boxes", required=True, type=Path,
                        help="boxer_3dbbs_fused.csv or boxer_3dbbs_tracked.csv")
    parser.add_argument("--transform", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--min-confidence", type=float, default=0.0)
    args = parser.parse_args()
    registration = json.loads(args.transform.read_text())
    scene = sg.build(
        load_boxes(args.boxes, args.min_confidence),
        source_frame=registration["source_frame"],
        map_from_source=registration["map_from_source"],
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sg.save(scene, args.output)
    print(f"{len(sg.furniture(scene))} furniture, {len(sg.objects(scene))} objects"
          f" in {registration['source_frame']} -> {args.output}")


if __name__ == "__main__":
    main()
