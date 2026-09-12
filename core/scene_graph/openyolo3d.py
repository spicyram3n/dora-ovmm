"""Load OpenYOLO3D point sets into the existing scene graph builder."""

import argparse
import json
from pathlib import Path

import numpy as np
from .instance import Instance
from . import graph as sg


def load_instances(manifest):
    manifest = Path(manifest)
    data = json.loads(manifest.read_text())
    if data["units"] != "m" or not data["source_frame"]:
        raise ValueError("Instances require metre units and an explicit source frame")
    instances = [
        Instance(
            item["label"],
            np.load(manifest.parent / item["points"], allow_pickle=False),
            confidence=item["confidence"],
            name=item["name"],
        )
        for item in data["instances"]
    ]
    return instances, data["source_frame"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instances", required=True, type=Path)
    parser.add_argument("--transform", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    instances, source_frame = load_instances(args.instances)
    registration = json.loads(args.transform.read_text())
    if registration["source_frame"] != source_frame:
        raise ValueError("Registration and instance source frames differ")
    scene = sg.build(
        instances,
        source_frame=source_frame,
        map_from_source=registration["map_from_source"],
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sg.save(scene, args.output)


if __name__ == "__main__":
    main()
