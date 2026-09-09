"""Prepare this repository's data_hrl scan without modifying the original files."""

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

from app import validate_scene


def link_or_copy(source, target):
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=root / "data_hrl")
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).parent / "data/hrl"
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=5,
        help="Keep every Nth frame; 10 keeps 40 of this scan’s 395 frames",
    )
    parser.add_argument(
        "--intrinsic-divisor",
        type=float,
        default=3,
        help="Verified for this data_hrl export, not a general calibration rule",
    )
    args = parser.parse_args()
    if (
        args.stride < 1
        or not np.isfinite(args.intrinsic_divisor)
        or args.intrinsic_divisor <= 0
    ):
        parser.error("stride and intrinsic divisor must be positive")
    src, out = args.source.resolve(), args.output.resolve()
    if out.exists():
        parser.error(f"Output already exists: {out}; choose a new --output")
    ids = {}
    for folder, extension in [("color", ".jpg"), ("depth", ".png"), ("pose", ".txt")]:
        ids[folder] = {
            int(f.stem)
            for f in (src / folder).glob("*" + extension)
            if f.stem.isdigit()
        }
    if not ids["color"] or not ids["color"] == ids["depth"] == ids["pose"]:
        raise ValueError("RGB, depth and numbered pose IDs must match")
    ordered = sorted(ids["color"])
    for i in ordered:
        with Image.open(src / "color" / f"{i}.jpg") as rgb, Image.open(
            src / "depth" / f"{i}.png"
        ) as depth:
            if rgb.size != (640, 480) or depth.size != rgb.size:
                raise ValueError("This HRL preparation expects aligned 640x480 RGB-D")
        pose = np.loadtxt(src / "pose" / f"{i}.txt")
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise ValueError(f"Invalid pose {i}")
    intrinsic = np.loadtxt(src / "intrinsic/intrinsic_color.txt")
    if intrinsic.shape != (4, 4) or not np.isfinite(intrinsic).all():
        raise ValueError("Invalid intrinsic matrix")
    intrinsic[:2, :] /= args.intrinsic_divisor
    selected = ordered[:: args.stride]
    out.mkdir(parents=True)
    try:
        for folder in ["color", "depth", "poses"]:
            (out / folder).mkdir()
        link_or_copy(src / "scene.ply", out / "scene.ply")
        for new_id, old_id in enumerate(selected):
            for old_folder, new_folder, extension in [
                ("color", "color", ".jpg"),
                ("depth", "depth", ".png"),
                ("pose", "poses", ".txt"),
            ]:
                link_or_copy(
                    src / old_folder / f"{old_id}{extension}",
                    out / new_folder / f"{new_id}{extension}",
                )
        np.savetxt(out / "intrinsics.txt", intrinsic)
        validate_scene(out)
        (out / "preparation.json").write_text(
            json.dumps(
                dict(
                    source=str(src),
                    source_frame="hrl_scan",
                    depth_scale=1000,
                    intrinsic_divisor=args.intrinsic_divisor,
                    stride=args.stride,
                    original_frame_ids=selected,
                    note="Hardlinked originals where possible; treat prepared files as read-only.",
                ),
                indent=2,
            )
        )
    except Exception:
        shutil.rmtree(out)
        raise
    print(f"Prepared {len(selected)}/{len(ordered)} frames at {out}")
    print("Intrinsics:\n", intrinsic)


if __name__ == "__main__":
    main()
