"""Keyframes + a text prompt -> a TSDF cube around the detected target.

Offline replay of active_grasp's perception on extract_rgbd.py keyframes:
SAM3 finds the target in each colour image, its masked depth gives the
target's box in the map frame (active_grasp gets this box from a detector
too), and every depth image is fused into a cube placed from that box.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from core.active_perception.bbox import AABBox
from core.active_perception.tsdf import UniformTSDFVolume, base_from_task
from core.perception.pointcloud import deproject, largest_cluster, shrink, transform_points
from core.perception.sam3_client import detect_all


def parse_frames(text):
    """'457-468' or '457,460,468' -> [457, ...]."""
    frames = []
    for part in text.split(","):
        if "-" in part:
            first, last = (int(v) for v in part.split("-"))
            frames.extend(range(first, last + 1))
        else:
            frames.append(int(part))
    return frames


def load(keyframes, i):
    color = cv2.imread(str(keyframes / "color" / f"{i}.jpg"))
    depth_mm = cv2.imread(str(keyframes / "depth" / f"{i}.png"), cv2.IMREAD_UNCHANGED)
    map_from_camera = np.loadtxt(keyframes / "poses" / f"{i}.txt")
    return color, depth_mm.astype(np.float32) / 1000.0, map_from_camera


def target_points(color, depth, k, map_from_camera, prompt, conf):
    """The target's points in the map frame from the best SAM3 instance, or None."""
    result = detect_all(color, prompt, conf)
    if not len(result["scores"]):
        return None, None
    best = int(np.argmax(result["scores"]))
    points = largest_cluster(deproject(depth, k, shrink(result["masks"][best])))
    if points is None:
        return None, None
    detection = dict(
        score=float(result["scores"][best]),
        box=result["boxes"][best].tolist(),
        mask=result["masks"][best],
    )
    return transform_points(map_from_camera, points), detection


def save_overlay(color, detection, path):
    overlay = color.copy()
    overlay[detection["mask"]] = 0.5 * overlay[detection["mask"]] + (0, 96, 0)
    x0, y0, x1, y1 = (int(v) for v in detection["box"])
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 255, 0), 2)
    cv2.putText(overlay, f"{detection['score']:.2f}", (x0, max(y0 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(str(path), overlay)


def write_cloud(points, path, colors=None):
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    if colors is not None:
        cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(str(path), cloud)


def render(scene, reference, bbox, origin, length, path):
    """Top and side views of the fused cube next to the full-scene reference."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    panels = [("fused TSDF surface", scene), ("scene.ply reference, same cube", reference)]
    for column, (title, points) in enumerate(panels):
        for row, (a, b, name) in enumerate([(0, 1, "x y"), (0, 2, "x z")]):
            ax = axes[row, column]
            if len(points):
                ax.scatter(points[:, a], points[:, b], c=points[:, 2], s=1, cmap="viridis")
            ax.add_patch(Rectangle((origin[a], origin[b]), length, length, fill=False, color="k", lw=1))
            ax.add_patch(Rectangle((bbox.min[a], bbox.min[b]), bbox.size[a], bbox.size[b],
                                   fill=False, color="r", lw=1.5, ls="--"))
            ax.set_title(f"{title} ({name})")
            ax.set_xlim(origin[a] - 0.05, origin[a] + length + 0.05)
            ax.set_ylim(origin[b] - 0.05, origin[b] + length + 0.05)
            ax.set_aspect("equal")
    fig.suptitle("black: TSDF cube   red dashed: target box")
    fig.tight_layout()
    fig.savefig(path, dpi=120)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keyframes", type=Path, default="outputs/realrobot/lab_20260811/keyframes")
    parser.add_argument("--frames", default="457-468", help="e.g. 457-468 or 457,460,468")
    parser.add_argument("--prompt", default="printer")
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--length", type=float, default=0.3, help="cube side, m (VGN: 0.3)")
    parser.add_argument("--resolution", type=int, default=40, help="voxels per side (VGN: 40)")
    parser.add_argument("--output", type=Path, default="outputs/active_perception/tsdf_test")
    args = parser.parse_args()

    out = args.output
    (out / "detections").mkdir(parents=True, exist_ok=True)
    k = np.loadtxt(args.keyframes / "intrinsics.txt")[:3, :3]
    frames = parse_frames(args.frames)
    data = {i: load(args.keyframes, i) for i in frames}

    # 1. The target's box from every frame that sees it.
    points, detections = [], {}
    for i, (color, depth, map_from_camera) in data.items():
        found, detection = target_points(color, depth, k, map_from_camera, args.prompt, args.conf)
        if found is None:
            print(f"frame {i}: no '{args.prompt}'")
            continue
        points.append(found)
        detections[i] = detection
        save_overlay(color, detection, out / "detections" / f"{i}.jpg")
        print(f"frame {i}: score {detection['score']:.2f}, {len(found)} points")
    if not points:
        raise SystemExit(f"'{args.prompt}' was not found in any frame")
    bbox = AABBox.from_points(np.concatenate(points))
    print(f"bbox min {bbox.min.round(3)} max {bbox.max.round(3)} size {bbox.size.round(3)}")

    # 2. Fuse every frame into the cube placed from the box.
    map_from_task = base_from_task(bbox, args.length)
    task_from_map = np.linalg.inv(map_from_task)
    tsdf = UniformTSDFVolume(args.length, args.resolution)
    for color, depth, map_from_camera in data.values():
        tsdf.integrate(depth, k, np.linalg.inv(task_from_map @ map_from_camera))

    # 3. Outputs, back in the map frame.
    origin = map_from_task[:3, 3]
    scene = np.asarray(tsdf.get_scene_cloud().points) + origin
    map_cloud = tsdf.get_map_cloud()
    voxels = np.asarray(map_cloud.points) + origin
    tsdf_values = np.asarray(map_cloud.colors)[:, 0]
    grid = tsdf.get_grid()
    write_cloud(scene, out / "scene_cloud.ply")
    write_cloud(voxels, out / "map_cloud.ply", np.stack([tsdf_values, 1 - tsdf_values, 0 * tsdf_values], 1))
    np.save(out / "grid.npy", grid)

    reference = np.asarray(o3d.io.read_point_cloud(str(args.keyframes / "scene.ply")).points)
    reference = reference[np.all((reference > origin) & (reference < origin + args.length), axis=1)]
    render(scene, reference, bbox, origin, args.length, out / "tsdf_views.png")

    summary = dict(
        prompt=args.prompt, frames=frames, detected=sorted(detections),
        scores={i: round(d["score"], 3) for i, d in detections.items()},
        bbox=bbox.to_dict(), length=args.length, resolution=args.resolution,
        voxel_size=tsdf.voxel_size, task_origin_in_map=origin.tolist(),
        surface_points=len(scene), voxels_near_surface=len(voxels),
        grid_nonzero=int((grid > 0).sum()),
        bbox_inside_cube=bool(np.all(bbox.min >= origin) and np.all(bbox.max <= origin + args.length)),
    )
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
