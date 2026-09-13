"""Plot the viewpoints a query selects, before anything moves.

Usage: python3 visualization/viewpoints.py 'pringles' [--index 0] [--output plot.png]

The poses drawn come from core.pipeline.actions.plan, the same call the mission drives,
so this shows the real choice rather than a redrawing of it. Nothing here talks
to ROS: reachability is Nav2's answer at run time and is not known yet."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from core.navigation import graspable, standoff
from core.scene_graph import graph as sg
from core.pipeline.actions import ROOT, box, is_storage, look_points, plan, targets
from core.utils import geometry

ARROW = 0.35
# Probes are drawn longer than APPROACH_DEPTH purely so they are visible at room scale.
PROBE_ARROW = 0.22


def draw_box(axes, centre, dimensions, yaw, colour, width=0.8, alpha=1.0):
    """The 12 edges of a box standing on its turned footprint."""
    corners = geometry.footprint_corners(centre, dimensions, yaw)
    bottom, top = (centre[2] - dimensions[2] / 2, centre[2] + dimensions[2] / 2)
    for index in range(4):
        (x0, y0), (x1, y1) = (corners[index], corners[(index + 1) % 4])
        for z in (bottom, top):
            axes.plot([x0, x1], [y0, y1], [z, z], color=colour, linewidth=width, alpha=alpha)
        axes.plot([x0, x0], [y0, y0], [bottom, top], color=colour, linewidth=width, alpha=alpha)


def unfiltered_ring(scene, location):
    """The ring before furniture filtering, so the plot can show what was dropped.

    Mirrors only the inputs plan() derives; the ring itself is the real function."""
    if location.furniture_id is None:
        return standoff.candidates(location.centroid, [0, 0, 0])
    data = sg.furniture(scene)[location.furniture_id]
    return standoff.candidates(*sg.footprint(data), storage=is_storage(data))


def draw(scene, location, poses, axes):
    target = location.furniture_id
    for node, data in sg.furniture(scene).items():
        chosen = node == target
        draw_box(
            axes,
            *sg.footprint(data),
            "#1f77b4" if chosen else "#cccccc",
            width=1.6 if chosen else 0.6,
            alpha=1.0 if chosen else 0.55,
        )
        if chosen:
            axes.text(*data["centroid"], f" {data['name'] or data['label']}",
                      color="#1f77b4", fontsize=8)
    if location.object_id is not None:
        centre, dimensions = box(scene.nodes[location.object_id])
        draw_box(axes, centre, dimensions, 0.0, "#d62728", width=2.0)

    kept = {(round(x, 6), round(y, 6)) for x, y, _ in poses}
    dropped = []
    for x, y, _ in unfiltered_ring(scene, location):
        if (round(x, 6), round(y, 6)) not in kept:
            dropped.append((x, y))
    if dropped:
        dropped = np.array(dropped)
        axes.scatter(dropped[:, 0], dropped[:, 1], np.zeros(len(dropped)),
                     marker="x", color="#d62728", s=28, depthshade=False,
                     label=f"blocked by furniture ({len(dropped)})")

    for order, (x, y, yaw) in enumerate(poses, 1):
        axes.quiver(x, y, 0, ARROW * np.cos(yaw), ARROW * np.sin(yaw), 0,
                    color="#2ca02c", linewidth=1.4, arrow_length_ratio=0.35)
        axes.text(x, y, 0.06, str(order), color="#2ca02c", fontsize=8, weight="bold")
    if poses:
        first = poses[0]
        # Shelves get one look-at point per height; surfaces and objects get one.
        aims = np.asarray(look_points(scene, location, first), dtype=float)
        for aim in aims:
            axes.plot([first[0], aim[0]], [first[1], aim[1]], [0, aim[2]],
                      color="#2ca02c", linestyle=":", linewidth=1.0)
        axes.scatter(aims[:, 0], aims[:, 1], aims[:, 2], marker="*", color="#000000",
                     s=70, depthshade=False, label="first view's look-at points")

    if location.object_id is not None:
        centre, dimensions = box(scene.nodes[location.object_id])
        for pose in graspable.probes(centre, dimensions):
            approach, palm = (pose[:3, 2], pose[:3, 3])
            axes.quiver(*(palm - PROBE_ARROW * approach), *(PROBE_ARROW * approach),
                        color="#9467bd", linewidth=1.2, arrow_length_ratio=0.35)
        axes.scatter([], [], [], color="#9467bd", marker="^",
                     label="IK reach probes (hand approach)")
    axes.scatter([], [], [], color="#2ca02c", marker=">",
                 label=f"observation poses in order ({len(poses)})")


def draw_plan(scene, location, poses, axes):
    """The same thing from above, where base placement is actually readable."""
    for node, data in sg.furniture(scene).items():
        chosen = node == location.furniture_id
        axes.add_patch(
            plt.Polygon(
                geometry.footprint_corners(*sg.footprint(data)),
                facecolor="#1f77b4" if chosen else "#dddddd",
                edgecolor="#1f77b4" if chosen else "#bbbbbb",
                alpha=0.45 if chosen else 0.7,
            )
        )
    if location.object_id is not None:
        lower, upper = scene.nodes[location.object_id]["bounds"]
        axes.add_patch(
            plt.Rectangle(lower[:2], upper[0] - lower[0], upper[1] - lower[1],
                          facecolor="#d62728", edgecolor="#d62728")
        )
    kept = {(round(x, 6), round(y, 6)) for x, y, _ in poses}
    for x, y, _ in unfiltered_ring(scene, location):
        if (round(x, 6), round(y, 6)) not in kept:
            axes.plot(x, y, marker="x", color="#d62728", markersize=8)
    for order, (x, y, yaw) in enumerate(poses, 1):
        # Each view centres on the part of the furniture in front of it.
        aim = np.asarray(look_points(scene, location, (x, y, yaw))[0], dtype=float)
        axes.plot([x, aim[0]], [y, aim[1]], color="#2ca02c", linestyle=":",
                  linewidth=0.7, alpha=0.6)
        axes.arrow(x, y, ARROW * np.cos(yaw), ARROW * np.sin(yaw), color="#2ca02c",
                   width=0.012, head_width=0.09, length_includes_head=True)
        axes.annotate(str(order), (x, y), textcoords="offset points", xytext=(-11, -4),
                      color="#2ca02c", fontsize=8, weight="bold")
        # The base footprint is what standoff filters against, so show its radius.
        axes.add_patch(plt.Circle((x, y), standoff.ROBOT_RADIUS, fill=False,
                                  edgecolor="#2ca02c", alpha=0.3, linewidth=0.7))
    axes.set_aspect("equal")
    axes.set_xlabel("x (map, m)")
    axes.set_ylabel("y (map, m)")
    axes.set_title(f"plan view -- circles are the {standoff.ROBOT_RADIUS:.2f} m base radius",
                   fontsize=9)
    axes.grid(alpha=0.25, linewidth=0.5)


def frame(axes, scene, location, poses):
    points = [np.asarray(location.centroid, dtype=float)]
    for x, y, _ in poses:
        points.append([x, y, 0.0])
    points = np.array(points)
    centre = (points.min(axis=0) + points.max(axis=0)) / 2
    span = max(float((points.max(axis=0) - points.min(axis=0)).max()), 1.0) / 2 + 0.6
    axes.set_xlim(centre[0] - span, centre[0] + span)
    axes.set_ylim(centre[1] - span, centre[1] + span)
    axes.set_zlim(0, max(2.0, float(location.centroid[2]) + 0.5))
    axes.set_box_aspect((1, 1, 0.45))
    axes.set_xlabel("x (map, m)")
    axes.set_ylabel("y (map, m)")
    axes.set_zlabel("z (m)")
    axes.legend(loc="upper left", fontsize=7, framealpha=0.9)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("object", nargs="?", default="")
    parser.add_argument("--graph", type=Path,
                        default=ROOT / "outputs/scene_graph/apartment.json")
    parser.add_argument("--furniture", help="unique furniture instance name or node ID")
    parser.add_argument("--robot", type=float, nargs=2, metavar=("X", "Y"),
                        default=(0.0, 0.0),
                        help="where the robot is, which orders the candidates")
    parser.add_argument("--index", type=int, default=0,
                        help="which search location to plot; past the remembered "
                             "ones this calls DeepSeek")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/viewpoints.png")
    args = parser.parse_args()
    if not args.object and not args.furniture:
        parser.error("give an object or --furniture")
    if args.index < 0:
        parser.error("--index must not be negative")
    scene = sg.load(args.graph)
    location = None
    for order, candidate in enumerate(
        targets(scene, args.object, args.furniture, tuple(args.robot))
    ):
        if order == args.index:
            location = candidate
            break
    if location is None:
        print(f"No search location at index {args.index}")
        return 1
    poses, views = plan(scene, location, tuple(args.robot))
    figure = plt.figure(figsize=(15, 7))
    axes = figure.add_subplot(1, 2, 1, projection="3d")
    draw(scene, location, poses, axes)
    frame(axes, scene, location, poses)
    plan_axes = figure.add_subplot(1, 2, 2)
    draw_plan(scene, location, poses, plan_axes)
    plan_axes.set_xlim(axes.get_xlim())
    plan_axes.set_ylim(axes.get_ylim())
    figure.suptitle(
        f"{args.object or args.furniture}: {location.source} location"
        f"{f', {location.room}' if location.room else ''} -- {len(poses)} poses, {views} views needed,"
        f" robot at ({args.robot[0]:.1f}, {args.robot[1]:.1f})"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=130, bbox_inches="tight")
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
