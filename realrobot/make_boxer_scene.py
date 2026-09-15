"""Keyframes from extract_rgbd.py as a sequence Boxer's ScanNet loader reads.

The keyframes already use ScanNet's conventions (optical camera->map poses,
uint16 mm depth, a 4x4 K), so they are hardlinked into its layout. The loader
moves the origin to the first camera; the --registration JSON moves it back.
"""

import argparse
import json
import os
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="extract_rgbd.py output")
    parser.add_argument("--output", type=Path, required=True, help="outputs/realrobot/<recording>/boxer/scannet/<recording>")
    parser.add_argument("--registration", type=Path, required=True,
                        help="config/realrobot/map/<recording>_boxer_to_map.json")
    parser.add_argument("--stride", type=int, default=1, help="keep every Nth keyframe")
    args = parser.parse_args()
    # run_boxer.py picks its ScanNet loader only for a path containing /scannet/.
    if args.output.parent.name != "scannet":
        parser.error("--output must be inside a folder named scannet/")

    frames = args.output / "frames"
    for folder in ("color", "depth", "pose", "intrinsic"):
        (frames / folder).mkdir(parents=True)
    total = len(list((args.source / "poses").glob("*.txt")))
    kept = range(0, total, args.stride)
    for new, old in enumerate(kept):  # renumbered 0..N-1: the loader's origin is frame 0
        os.link(args.source / "color" / f"{old}.jpg", frames / "color" / f"{new}.jpg")
        os.link(args.source / "depth" / f"{old}.png", frames / "depth" / f"{new}.png")
        os.link(args.source / "poses" / f"{old}.txt", frames / "pose" / f"{new}.txt")
    os.link(args.source / "intrinsics.txt", frames / "intrinsic" / "intrinsic_color.txt")

    map_from_source = np.eye(4)
    map_from_source[:3, 3] = np.loadtxt(frames / "pose" / "0.txt")[:3, 3]
    args.registration.write_text(json.dumps(
        {"source_frame": f"boxer_{args.output.name}", "map_from_source": map_from_source.tolist()},
        indent=2))
    print(f"Prepared {len(kept)}/{total} keyframes at {args.output}")


if __name__ == "__main__":
    main()
