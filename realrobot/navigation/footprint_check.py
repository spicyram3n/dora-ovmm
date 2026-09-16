"""How much of each scene-graph footprint the laser map already knows about.

    python3 realrobot/navigation/footprint_check.py

Boxer's boxes come from RGB-D at head height; the Nav2 map comes from a laser at
19 cm. Neither knows about the other, so the overlap between them is a free check
on the boxes, and the disagreement is what the scene graph adds to navigation.

Per furniture piece, and then per object plan_overlay.standing_on_floor picks out:
  occ / free / unknown   what the laser map says about the cells under the footprint
  edge                   metres from the footprint to the nearest occupied cell; 0
                         means the piece is anchored to something the laser saw
  yaw                    the fitted footprint's angle in the map
  laser                  whether the box spans the 19 cm laser plane, so the map
                         could have seen it at all

The room is not square to the map, so compare the yaw column against the wall
angle the summary prints rather than against zero.

No ROS: numpy, scipy, opencv, matplotlib and the repo's core/ are enough.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from core.scene_graph import graph as sg  # noqa: E402
from core.utils import geometry  # noqa: E402
from realrobot.navigation.plan_overlay import FLOOR, standing_on_floor  # noqa: E402

DEFAULT_GRAPH = ROOT / "config/realrobot/scene_graph/lab_20260811.json"
DEFAULT_MAP = ROOT / "config/realrobot/map/lab_20260811.yaml"
FREE, UNKNOWN, OCCUPIED = 254, 205, 0
# Neighbourhood a wall cell is fitted over, and how straight that fit has to be
# before its direction counts towards the room angle.
WALL_RADIUS, WALL_STRAIGHTNESS = 0.25, 3.0
# base_range_sensor_link above base_footprint, from /tf_static: the one height the
# Nav2 map was ever able to see. realrobot/visualize.py uses the same number.
LASER_Z = 0.19


def read_map(yaml_path):
    meta = yaml.safe_load(yaml_path.read_text())
    image = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_UNCHANGED)
    return np.flipud(image), float(meta["resolution"]), np.array(meta["origin"][:2], float)


def cells_under(corners, resolution, origin, shape):
    mask = np.zeros(shape, np.uint8)
    cells = (np.asarray(corners) - origin) / resolution
    cv2.fillPoly(mask, [np.round(cells).astype(np.int32)], 1)
    return mask.astype(bool)


def wall_angle(image, resolution, origin, centre, radius=6.0):
    """The dominant direction of the occupied cells around `centre`, modulo 90
    degrees, fitted locally so a few stray points cannot set it."""
    rows, columns = np.nonzero(image == OCCUPIED)
    points = np.column_stack([origin[0] + columns * resolution, origin[1] + rows * resolution])
    points = points[np.hypot(*(points - np.asarray(centre)).T) <= radius]
    tree, angles = cKDTree(points), []
    for point in points:
        near = points[tree.query_ball_point(point, WALL_RADIUS)]
        if len(near) < 5:
            continue
        _, spread, directions = np.linalg.svd(near - near.mean(0), full_matrices=False)
        if spread[0] < WALL_STRAIGHTNESS * spread[1]:
            continue  # a blob, not a stretch of wall
        angles.append(math.degrees(math.atan2(*directions[0][::-1])) % 90)
    if not angles:
        return None, 0
    counts, edges = np.histogram(angles, bins=18, range=(0, 90))
    return edges[counts.argmax()] + 2.5, len(angles)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--graph", type=Path, default=DEFAULT_GRAPH)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    args = parser.parse_args()

    scene = sg.load(args.graph)
    image, resolution, origin = read_map(args.map)
    to_occupied = distance_transform_edt(image != OCCUPIED) * resolution

    print(f"{'id':>3}  {'label':<14} {'size [m]':>11} {'yaw':>6} {'cells':>6}"
          f" {'occ':>5} {'free':>5} {'unk':>5} {'edge':>5}")
    shares, yaws = [], []
    for node, data in sg.furniture(scene).items():
        centre, dimensions, yaw = sg.footprint(data)
        mask = cells_under(geometry.footprint_corners(centre[:2], dimensions[:2], yaw),
                           resolution, origin, image.shape)
        if not mask.any():
            print(f"{node:>3}  {data['label']:<14} off the map")
            continue
        share = [(image[mask] == value).mean() for value in (OCCUPIED, FREE, UNKNOWN)]
        shares.append(share)
        yaws.append(math.degrees(yaw))
        print(f"{node:>3}  {data['label']:<14}"
              f" {dimensions[0]:>5.2f}x{dimensions[1]:<5.2f} {math.degrees(yaw):>5.1f}°"
              f" {mask.sum():>6}" + "".join(f" {value:>4.0%}" for value in share)
              + f" {to_occupied[mask].min():>5.2f}")

    shares = np.array(shares)
    print(f"\n{len(shares)} furniture pieces, mean"
          f" occupied {shares[:, 0].mean():.0%},"
          f" free {shares[:, 1].mean():.0%},"
          f" unknown {shares[:, 2].mean():.0%}")
    centre = np.mean([sg.footprint(d)[0][:2] for d in sg.furniture(scene).values()], axis=0)
    angle, samples = wall_angle(image, resolution, origin, centre)
    if angle is not None:
        # A rectangle repeats every 90 degrees, so both are compared mod 90.
        agreed = sum(abs((y % 90) - angle) <= 10 for y in yaws)
        print(f"walls within 6 m of the furniture run at {angle:.0f}° mod 90"
              f" ({samples} fitted cells); {agreed} of {len(yaws)} footprint yaws"
              f" sit within 10° of that, median {np.median([y % 90 for y in yaws]):.0f}°")
    print(f"\n{shares[:, 1].mean():.0%} of the furniture area is free floor in the laser"
          f" map: a base planning on that map alone drives through it.")

    print(f"\nObjects standing on the floor (bounds reach z <= {FLOOR} m):")
    print(f"{'id':>3}  {'label':<14} {'z span [m]':>12} {'cells':>6}"
          f" {'occ':>5} {'free':>5} {'unk':>5}  laser")
    floor_shares = []
    for node, data in sg.objects(scene).items():
        if not standing_on_floor(data):
            continue
        lower, upper = np.asarray(data["bounds"], float)
        mask = cells_under(geometry.footprint_corners((lower[:2] + upper[:2]) / 2,
                                                      (upper - lower)[:2], 0.0),
                           resolution, origin, image.shape)
        share = [(image[mask] == value).mean() for value in (OCCUPIED, FREE, UNKNOWN)]
        floor_shares.append(share)
        print(f"{node:>3}  {data['label']:<14} {lower[2]:>5.2f} to {upper[2]:<4.2f}"
              f" {mask.sum():>6}" + "".join(f" {value:>4.0%}" for value in share)
              + ("  crosses it" if lower[2] <= LASER_Z <= upper[2] else "  misses it"))
    if floor_shares:
        occupied = np.array(floor_shares)[:, 0].mean()
        print(f"\nThese stand tall enough to cross the laser plane, and the map already"
              f" has {occupied:.0%} of their area occupied. Stamping them in sharpens"
              f" boxes the laser saw only one side of; the furniture above is the part"
              f" the map genuinely lacks.")


if __name__ == "__main__":
    main()
