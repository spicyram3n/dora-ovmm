"""Figures and numbers that check the map, the trajectory and the fused scene.

Writes into --figures:
  map_trajectory.png   occupancy map + base path
  map_keyframes.png    occupancy map + keyframe camera positions
  cloud_on_map.png     RGB-D points at laser height over the laser map
  scene_topdown.png    the fused cloud in colour, seen from above
  keyframes.png        RGB and depth of evenly spaced keyframes
  checks.json          the alignment and frame-to-model numbers
"""

import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import open3d as o3d
import rclpy.time
import tf2_py
import yaml
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from extract_rgbd import load_tf  # noqa: E402

INK, MUTED, PATH, CAM = "#1f2933", "#7b8794", "#d14d1f", "#2464b0"
LASER_Z = 0.19  # base_range_sensor_link above base_footprint, from /tf_static
FREE, UNKNOWN, OCCUPIED = 254, 205, 0  # nav2 trinary PGM values


def read_map(yaml_path):
    meta = yaml.safe_load(yaml_path.read_text())
    image = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_UNCHANGED)
    return image, meta["resolution"], np.array(meta["origin"][:2], dtype=float)


def map_axes(ax, image, resolution, origin):
    """Draw the PGM in map metres."""
    height, width = image.shape
    extent = [origin[0], origin[0] + width * resolution, origin[1], origin[1] + height * resolution]
    ax.imshow(image, cmap="gray", vmin=0, vmax=255, extent=extent, interpolation="nearest")
    ax.set_xlabel("map x [m]", color=MUTED)
    ax.set_ylabel("map y [m]", color=MUTED)
    ax.tick_params(colors=MUTED)
    for spine in ax.spines.values():
        spine.set_color("#d9dde2")


def base_path(buffer, start, end, hz=5.0):
    """(stamp, x, y) of base_footprint in the map, sampled at `hz`."""
    points = []
    for stamp in np.arange(start, end, 1.0 / hz):
        try:
            t = buffer.lookup_transform_core("map", "base_footprint", rclpy.time.Time(seconds=stamp))
        except (tf2_py.ExtrapolationException, tf2_py.LookupException):
            continue
        points.append([stamp, t.transform.translation.x, t.transform.translation.y])
    return np.array(points)


def alignment(cloud_xyz, image, resolution, origin):
    """Distance from RGB-D points at laser height to the nearest laser-occupied cell."""
    slab = cloud_xyz[np.abs(cloud_xyz[:, 2] - LASER_Z) < 0.05]
    distance = distance_transform_edt(image != OCCUPIED) * resolution
    col = ((slab[:, 0] - origin[0]) / resolution).astype(int)
    row = (image.shape[0] - 1 - (slab[:, 1] - origin[1]) / resolution).astype(int)  # row 0 = top
    inside = (col >= 0) & (col < image.shape[1]) & (row >= 0) & (row < image.shape[0])
    d = distance[row[inside], col[inside]]
    return dict(points=int(inside.sum()), median_m=round(float(np.median(d)), 3),
                within_5cm=round(float((d <= 0.05).mean()), 3),
                within_10cm=round(float((d <= 0.10).mean()), 3),
                within_20cm=round(float((d <= 0.20).mean()), 3)), slab


def frame_to_model(scene, cloud_xyz, ids, k, step=4):
    """Unproject a keyframe's depth with its pose; distance to the nearest scene point.

    Checks the pose/intrinsics/depth-scale contract Boxer's ScanNet loader relies on: a
    correct triple lands every pixel on the fused surface, about one voxel away."""
    tree = cKDTree(cloud_xyz)
    results = []
    for i in ids:
        pose = np.loadtxt(scene / "poses" / f"{i}.txt")
        depth = cv2.imread(str(scene / "depth" / f"{i}.png"), cv2.IMREAD_UNCHANGED) / 1000.0
        v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
        z = depth[v, u]
        keep = (z > 0) & (z <= 4.0)  # the range the TSDF was given
        x = (u[keep] - k[0, 2]) / k[0, 0] * z[keep]
        y = (v[keep] - k[1, 2]) / k[1, 1] * z[keep]
        world = (pose @ np.c_[x, y, z[keep], np.ones(keep.sum())].T).T[:, :3]
        distance = np.minimum(tree.query(world, distance_upper_bound=1.0)[0], 1.0)
        results.append(dict(keyframe=int(i), pixels=int(keep.sum()),
                            median_mm=round(float(np.median(distance)) * 1000, 1),
                            within_5cm=round(float((distance <= 0.05).mean()), 3)))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True, help="extract_rgbd.py output")
    parser.add_argument("--map", type=Path, required=True, help="map YAML")
    parser.add_argument("--slam-tf", type=Path, required=True, help=".../slam/tf_mapping")
    parser.add_argument("--bag", type=Path, default="bags/playable_bag")
    parser.add_argument("--figures", type=Path, required=True)
    args = parser.parse_args()
    args.figures.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.titlecolor": INK})

    image, resolution, origin = read_map(args.map)
    with open(args.scene / "keyframes.csv") as f:
        keyframes = np.array([[float(x) for x in row] for row in list(csv.reader(f))[1:]])
    cloud = o3d.io.read_point_cloud(str(args.scene / "scene.ply"))
    xyz, rgb = np.asarray(cloud.points), np.asarray(cloud.colors)
    k = np.loadtxt(args.scene / "intrinsics.txt")[:3, :3]
    buffer, _ = load_tf(args.bag, args.slam_tf, span=800.0)
    path = base_path(buffer, keyframes[0, 1] - 5, keyframes[-1, 1] + 5)
    path_length = float(np.linalg.norm(np.diff(path[:, 1:], axis=0), axis=1).sum())
    figure_size = (9, 9 * image.shape[0] / image.shape[1])

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(args.figures / name, dpi=150)
        plt.close(fig)

    def on_map(name, title, draw):
        """One figure: the map, whatever `draw` adds, and the start marker."""
        fig, ax = plt.subplots(figsize=figure_size)
        map_axes(ax, image, resolution, origin)
        draw(ax)
        ax.plot(path[0, 1], path[0, 2], "o", color=INK, markersize=7, label="start")
        ax.legend(loc="upper left", frameon=False)  # the top-left of every map is empty
        ax.set_title(title)
        save(fig, name)

    on_map("map_trajectory.png", "SLAM map with the base trajectory",
           lambda ax: ax.plot(path[:, 1], path[:, 2], color=PATH, linewidth=1,
                              label=f"base path, {path_length:.0f} m"))
    on_map("map_keyframes.png", "SLAM map with the keyframe camera positions",
           lambda ax: ax.scatter(keyframes[:, 2], keyframes[:, 3], s=4, color=CAM,
                                 label=f"{len(keyframes)} keyframe cameras"))
    align, slab = alignment(xyz, image, resolution, origin)
    on_map("cloud_on_map.png", "Fused RGB-D against the laser map",
           lambda ax: ax.scatter(slab[:, 0], slab[:, 1], s=0.3, color=CAM, alpha=0.5, rasterized=True,
                                 label=f"RGB-D points at laser height, "
                                       f"{align['within_10cm']:.0%} within 10 cm of a wall cell"))

    # Coloured top-down view: a random subset, highest points drawn last.
    keep = np.random.default_rng(0).choice(len(xyz), min(len(xyz), 600_000), replace=False)
    order = keep[np.argsort(xyz[keep, 2])]
    fig, ax = plt.subplots(figsize=figure_size)
    ax.scatter(xyz[order, 0], xyz[order, 1], c=rgb[order], s=0.2, rasterized=True)
    ax.set_aspect("equal")
    ax.set_facecolor("#f4f5f7")
    ax.set_xlabel("map x [m]", color=MUTED)
    ax.set_ylabel("map y [m]", color=MUTED)
    ax.tick_params(colors=MUTED)
    ax.set_title(f"scene.ply from above, {len(xyz):,} points")
    save(fig, "scene_topdown.png")

    # Keyframe strip: RGB above, depth below.
    ids = np.linspace(0, len(keyframes) - 1, 6).astype(int)
    fig, axes = plt.subplots(2, len(ids), figsize=(2.6 * len(ids), 4.2))
    for column, i in enumerate(ids):
        color = cv2.cvtColor(cv2.imread(str(args.scene / "color" / f"{i}.jpg")), cv2.COLOR_BGR2RGB)
        depth = cv2.imread(str(args.scene / "depth" / f"{i}.png"), cv2.IMREAD_UNCHANGED) / 1000.0
        axes[0, column].imshow(color)
        axes[0, column].set_title(f"keyframe {i}, t+{keyframes[i, 1] - keyframes[0, 1]:.0f}s")
        axes[1, column].imshow(np.ma.masked_equal(depth, 0), cmap="Blues_r", vmin=0.4, vmax=4.0)
        for ax in axes[:, column]:
            ax.axis("off")
    fig.suptitle("Keyframes: RGB above, depth (0.4-4 m, white = none) below")
    save(fig, "keyframes.png")

    checks = dict(path_length_m=round(path_length, 1),
                  map_cells={name: int((image == value).sum()) for name, value in
                             (("free", FREE), ("occupied", OCCUPIED), ("unknown", UNKNOWN))},
                  laser_height_alignment=align,
                  frame_to_model=frame_to_model(args.scene, xyz, ids, k))
    (args.figures / "checks.json").write_text(json.dumps(checks, indent=2))
    print(json.dumps(checks, indent=2))


if __name__ == "__main__":
    main()
