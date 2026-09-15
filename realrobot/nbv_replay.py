"""Replay recorded keyframes through the next-best-view policy.

The robot's path is fixed in a recording, so at each step the policy fuses the
frame the robot actually took, scores every view the HSR head could take next,
and we log what it would have chosen against the gain of the view the robot
took instead. The target box comes from target_tsdf.py's summary.json; with
GraspGenX running, grasps are predicted after every fusion as in active_grasp.
"""

import argparse
import glob
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import open3d as o3d

from core.active_perception.bbox import AABBox
from core.active_perception.grasps import graspgenx_grasps
from core.active_perception.nbv import NextBestView
from core.active_perception.record import Recording
from core.active_perception.views import ViewHalfSphere
from core.navigation.standoff import ROBOT_RADIUS
from realrobot.target_tsdf import load, parse_frames


def occupancy_from_scene(keyframes, resolution=0.05, heights=(0.1, 1.0), self_radius=0.5):
    """Nav2's costmap stands in offline: cells within ROBOT_RADIUS of anything
    the SLAM run reconstructed between knee and head height are not free.
    Points within `self_radius` of a keyframe camera are the robot itself."""
    from scipy.ndimage import binary_dilation
    from scipy.spatial import cKDTree

    points = np.asarray(o3d.io.read_point_cloud(str(keyframes / "scene.ply")).points)
    points = points[(points[:, 2] > heights[0]) & (points[:, 2] < heights[1])]
    cameras = np.array([np.loadtxt(p)[:3, 3] for p in glob.glob(str(keyframes / "poses" / "*.txt"))])
    distance, _ = cKDTree(cameras[:, :2]).query(points[:, :2])
    points = points[distance >= self_radius]
    origin = points[:, :2].min(axis=0) - 1.0
    cells = np.floor((points[:, :2] - origin) / resolution).astype(int)
    width, height = cells.max(axis=0) + int(1.0 / resolution)
    occupied = np.zeros((height, width), dtype=bool)
    occupied[cells[:, 1], cells[:, 0]] = True
    radius = int(np.ceil(ROBOT_RADIUS / resolution))
    yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
    occupied = binary_dilation(occupied, xx ** 2 + yy ** 2 <= radius ** 2)
    info = SimpleNamespace(resolution=resolution, width=width, height=height,
                           origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])))
    return SimpleNamespace(info=info, data=np.where(occupied, 100, 0).ravel())


def frustum(ax, pose, depth, k, shape, **style):
    """Draw a camera as its image-plane pyramid at `depth`."""
    h, w = shape
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=float)
    rays = np.c_[(corners[:, 0] - k[0, 2]) / k[0, 0], (corners[:, 1] - k[1, 2]) / k[1, 1], np.ones(4)] * depth
    pts = rays @ pose[:3, :3].T + pose[:3, 3]
    eye = pose[:3, 3]
    for a, b in [(eye, p) for p in pts] + list(zip(pts, np.roll(pts, 1, axis=0))):
        ax.plot(*zip(a, b), **style)


def box_edges(ax, low, high, **style):
    low, high = (np.asarray(low), np.asarray(high))
    for axis in range(3):
        for corner in [(0, 0), (0, 1), (1, 0), (1, 1)]:
            a = np.array([low, high])
            start, end = (a[[0], :].copy()[0], a[[1], :].copy()[0])
            other = [i for i in range(3) if i != axis]
            start[other] = high[other] * np.array(corner) + low[other] * (1 - np.array(corner))
            end[other] = start[other]
            start[axis], end[axis] = (low[axis], high[axis])
            ax.plot(*zip(start, end), **style)


def robot(ax, xy, camera_z, **style):
    """A stylised HSR: base cylinder, torso, head at the camera height."""
    angle = np.linspace(0, 2 * np.pi, 24)
    ring = np.c_[xy[0] + ROBOT_RADIUS * np.cos(angle), xy[1] + ROBOT_RADIUS * np.sin(angle)]
    for z in (0.0, 0.3):
        ax.plot(ring[:, 0], ring[:, 1], np.full(len(ring), z), **style)
    for x, y in ring[::6]:
        ax.plot([x, x], [y, y], [0.0, 0.3], **style)
    for dx, dy in [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1)]:
        ax.plot([xy[0] + dx] * 2, [xy[1] + dy] * 2, [0.3, camera_z - 0.1], **style)
    ax.plot(*zip(*[[xy[0] + dx, xy[1] + dy, camera_z - 0.1] for dx, dy in
                   [(-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1)]]), **style)


def render_overview(policy, k, shape, data, grid, out):
    """One figure: fused surface, box, cube, view sphere, scored candidates,
    the recorded path with the robot at its last pose, and the chosen view."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    sphere = policy.view_sphere
    bbox, origin, length = (policy.bbox, policy.base_from_task[:3, 3], policy.length)
    views, gains, best = (policy.info["views"], policy.info["gains"], policy.info["best"])
    scene = np.asarray(policy.tsdf.get_scene_cloud().points) + origin
    path = np.array([pose[:3, 3] for _, _, pose in data])
    cmap = plt.get_cmap("plasma")
    norm = plt.Normalize(0, max(gains.max(), 1))

    fig = plt.figure(figsize=(16, 8))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.scatter(scene[:, 0], scene[:, 1], scene[:, 2], c=scene[:, 2], cmap="viridis", s=2, alpha=0.8)
    box_edges(ax, bbox.min, bbox.max, color="r", lw=1.5, ls="--")
    box_edges(ax, origin, origin + length, color="k", lw=1)
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 25), np.linspace(0, np.pi / 2 + 0.4, 9))
    ax.plot_wireframe(sphere.center[0] + sphere.r * np.sin(v) * np.cos(u),
                      sphere.center[1] + sphere.r * np.sin(v) * np.sin(u),
                      sphere.center[2] + sphere.r * np.cos(v), color="gray", lw=0.4, alpha=0.5)
    for view in sphere.all_views():
        if not sphere.feasible(view):
            frustum(ax, view, 0.15, k, shape, color="lightgray", lw=0.6)
    for view, gain in zip(views, gains):
        frustum(ax, view, 0.2, k, shape, color=cmap(norm(gain)), lw=1.2)
    frustum(ax, views[best], 0.3, k, shape, color="lime", lw=2.5)
    robot(ax, views[best][:3, 3], views[best][2, 3], color="lime", lw=0.8, alpha=0.6)
    ax.plot(path[:, 0], path[:, 1], path[:, 2], "b.-", ms=4, lw=0.8)
    for _, _, pose in data:
        frustum(ax, pose, 0.12, k, shape, color="b", lw=0.5, alpha=0.6)
    robot(ax, path[-1, :2], path[-1, 2], color="b", lw=0.8)
    span = sphere.r + 0.7
    ax.set_xlim(bbox.center[0] - span, bbox.center[0] + span)
    ax.set_ylim(bbox.center[1] - span, bbox.center[1] + span)
    ax.set_zlim(0, 2.0)
    ax.set_box_aspect((1, 1, 2.0 / (2 * span)))
    ax.view_init(elev=24, azim=-125)
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]"), ax.set_zlabel("z [m]")
    ax.set_title("fusion, candidate views and the next best view")
    fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, shrink=0.45, pad=0.02,
                 label="information gain (rear-side voxels in the box)")

    top = fig.add_subplot(1, 2, 2)
    info = grid.info
    occupied = np.array(grid.data).reshape(info.height, info.width) > 0
    extent = [info.origin.position.x, info.origin.position.x + info.width * info.resolution,
              info.origin.position.y, info.origin.position.y + info.height * info.resolution]
    top.imshow(occupied, origin="lower", extent=extent, cmap="Greys", alpha=0.35)
    top.scatter(scene[:, 0], scene[:, 1], c=scene[:, 2], cmap="viridis", s=1)
    top.add_patch(Rectangle(bbox.min[:2], *bbox.size[:2], fill=False, color="r", ls="--", lw=1.5))
    top.add_patch(Rectangle(origin[:2], length, length, fill=False, color="k"))
    for r in sphere.radii:
        top.add_patch(plt.Circle(sphere.center[:2], r, fill=False, color="gray", ls=":"))
    for view in sphere.all_views():
        if not sphere.feasible(view):
            top.plot(view[0, 3], view[1, 3], "x", color="gray", ms=5)
    eyes = np.array([v[:3, 3] for v in views])
    fwd = np.array([v[:3, 2] for v in views])
    top.scatter(eyes[:, 0], eyes[:, 1], c=gains, cmap="plasma", norm=norm, s=70, zorder=3, edgecolors="k")
    top.quiver(eyes[:, 0], eyes[:, 1], fwd[:, 0], fwd[:, 1], color="k", scale=15, width=0.003)
    top.plot(eyes[best, 0], eyes[best, 1], "*", color="lime", ms=20, mec="k", zorder=4)
    top.plot(path[:, 0], path[:, 1], "b.-", ms=5, lw=0.8)
    top.add_patch(plt.Circle(path[-1, :2], ROBOT_RADIUS, fill=False, color="b"))
    top.set_xlim(bbox.center[0] - span, bbox.center[0] + span)
    top.set_ylim(bbox.center[1] - span, bbox.center[1] + span)
    top.set_aspect("equal")
    top.set_xlabel("x [m]"), top.set_ylabel("y [m]")
    top.set_title("top view: reachable (coloured) and unreachable (x) views")
    fig.tight_layout()
    fig.savefig(out, dpi=150)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keyframes", type=Path, default="outputs/realrobot/lab_20260811/keyframes")
    parser.add_argument("--summary", type=Path,
                        default="outputs/active_perception/tsdf_test/fit_cube_0.9m/summary.json")
    parser.add_argument("--frames", default=None, help="default: the summary's frames")
    parser.add_argument("--min-z-dist", type=float, default=0.6, help="closest camera range, m")
    parser.add_argument("--phis", type=int, default=16, help="azimuths on the sphere (ETH: 8)")
    parser.add_argument("--downsample", type=int, default=20, help="ETH: 10 sim, 20 hardware")
    parser.add_argument("--no-grasps", action="store_true", help="skip GraspGenX after each fusion")
    parser.add_argument("--output", type=Path, default="outputs/active_perception/tsdf_test/nbv_replay")
    parser.add_argument("--figure", type=Path, default="docs/figures/nbv_overview.png")
    parser.add_argument("--rerun", action="store_true", help="also open the Rerun viewer live")
    args = parser.parse_args()

    summary = json.loads(args.summary.read_text())
    bbox = AABBox(summary["bbox"]["min"], summary["bbox"]["max"])
    frames = parse_frames(args.frames) if args.frames else summary["frames"]
    k = np.loadtxt(args.keyframes / "intrinsics.txt")[:3, :3]
    args.output.mkdir(parents=True, exist_ok=True)
    args.figure.parent.mkdir(parents=True, exist_ok=True)

    policy = NextBestView(k, length=summary["length"], resolution=summary["resolution"],
                          downsample=args.downsample, grasp_fn=None if args.no_grasps else graspgenx_grasps)
    grid = occupancy_from_scene(args.keyframes)
    sphere = ViewHalfSphere(bbox, args.min_z_dist, grid=grid, phis=args.phis)
    policy.activate(bbox, sphere)
    print(f"{len(sphere.candidates())} feasible views on spheres of r={np.round(sphere.radii, 2)} m "
          f"({len(ViewHalfSphere(bbox, args.min_z_dist, phis=args.phis).candidates())} before the occupancy check)")

    data = [load(args.keyframes, i) for i in frames]
    recording = Recording(k, data[0][1].shape, path=args.output / "nbv_replay.rrd", spawn=args.rerun)
    recording.scene(policy)
    steps = []
    for step, (i, (_, depth, map_from_camera)) in enumerate(zip(frames, data), start=1):
        started = time.perf_counter()
        policy.update(depth, map_from_camera)
        seconds = time.perf_counter() - started
        recording.step(step, policy, depth, map_from_camera, image=data[step - 1][0])
        if not policy.info:
            print(f"frame {i}: policy done before scoring any view")
            break
        recorded_next = data[step][2] if step < len(data) else None
        record = dict(
            step=step, frame=i, seconds=round(seconds, 3),
            candidates=len(policy.info["views"]),
            nbv_gain=int(policy.info["gains"][policy.info["best"]]),
            nbv_eye=policy.x_d[:3, 3].round(3).tolist(),
            gains=policy.info["gains"].astype(int).tolist(),
            recorded_next_gain=None if recorded_next is None else policy.ig_fn(recorded_next),
            best_grasp_quality=None if policy.best_grasp is None else round(policy.best_grasp[1], 3),
            best_grasp_position=None if policy.best_grasp is None else policy.best_grasp[0][:3, 3].round(3).tolist(),
            done=policy.done,
        )
        steps.append(record)
        print(f"frame {i}: {record['candidates']} views in {seconds:.2f}s, best gain {record['nbv_gain']} "
              f"at {record['nbv_eye']}, robot's next view would gain {record['recorded_next_gain']}, "
              f"best grasp quality {record['best_grasp_quality']}")
        if policy.done:
            print("policy done: gain below min_gain or grasp prediction stable")
            break

    if steps:
        render_overview(policy, k, data[0][1].shape, data, grid, args.figure)
        (args.output / "nbv_overview.png").write_bytes(args.figure.read_bytes())
    result = dict(summary=str(args.summary), frames=frames, min_z_dist=args.min_z_dist, phis=args.phis,
                  downsample=args.downsample, sphere_radius=sphere.r, grasps=not args.no_grasps, steps=steps)
    (args.output / "nbv_replay.json").write_text(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
