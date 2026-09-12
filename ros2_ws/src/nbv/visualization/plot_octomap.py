#!/usr/bin/env python3
"""Save a figure of the octomap and the sphere it is confined to.

    ros2 run nbv plot_octomap.py --ros-args -p path:=/home/ws/outputs/octomap.png

Needs build_octomap.py running. Three views of the same cells, coloured by
occupancy probability.
"""

import matplotlib
matplotlib.use("Agg")  # there is no display in the container

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.patches import Circle
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from visualization_msgs.msg import Marker

# Occupancy is a magnitude, so one sequential ramp; viridis is perceptually
# uniform and stays readable in greyscale and for colour-blind readers.
COLOURS = "viridis"
INK = "#222222"
FAINT = "#9aa0a6"
STYLE = {
    "font.size": 8.5, "axes.labelsize": 8.5, "axes.titlesize": 9.5,
    "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "axes.edgecolor": FAINT, "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": FAINT, "ytick.color": FAINT,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": "white", "savefig.facecolor": "white",
}


def flat_view(axes, cells, centre, radius, first, second, labels):
    """One orthographic panel: cells as squares, the sphere as its outline."""
    axes.add_patch(Circle((centre[first], centre[second]), radius, fill=False,
                          linestyle=(0, (4, 3)), linewidth=0.9, edgecolor=FAINT))
    dots = axes.scatter(cells[:, first], cells[:, second], c=cells[:, 3],
                        cmap=COLOURS, vmin=0.5, vmax=1.0, s=7, marker="s",
                        linewidths=0)
    axes.set(xlabel=f"{labels[0]} (m)", ylabel=f"{labels[1]} (m)")
    axes.set_aspect("equal")
    axes.grid(True, linewidth=0.4, alpha=0.35, color=FAINT)
    axes.set_axisbelow(True)
    for axis, value in ((axes.set_xlim, centre[first]), (axes.set_ylim, centre[second])):
        axis(value - 1.15 * radius, value + 1.15 * radius)
    return dots


def main():
    rclpy.init()
    node = Node("plot_octomap")
    path = node.declare_parameter("path", "/home/ws/outputs/octomap.png").value
    received, marker = wait_for_message(Marker, node, "/nbv/sphere", time_to_wait=30.0)
    if not received:
        raise SystemExit("no /nbv/sphere; is build_octomap.py running?")
    # The node publishes once a second from the moment it starts, so the first
    # few messages are empty: wait for one with something in it.
    for _ in range(30):
        received, cloud = wait_for_message(PointCloud2, node, "/nbv/occupied",
                                           time_to_wait=5.0)
        if received and cloud.width:
            break
    else:
        raise SystemExit("/nbv/occupied stayed empty; is the camera seeing the sphere?")

    cells = read_points_numpy(cloud, ["x", "y", "z", "intensity"])
    centre = np.array([getattr(marker.pose.position, axis) for axis in "xyz"])
    radius = marker.scale.x / 2

    with plt.rc_context(STYLE):
        figure = plt.figure(figsize=(10.5, 3.9))
        grid = figure.add_gridspec(1, 3, width_ratios=[1.25, 1, 1],
                                   wspace=0.34, left=0.03, right=0.89,
                                   bottom=0.14, top=0.86)

        space = figure.add_subplot(grid[0], projection="3d")
        # Sparse wireframe: enough to read as a sphere, not enough to compete
        # with the cells inside it.
        u, v = np.mgrid[0:2 * np.pi:13j, 0:np.pi:7j]
        space.plot_wireframe(centre[0] + radius * np.cos(u) * np.sin(v),
                             centre[1] + radius * np.sin(u) * np.sin(v),
                             centre[2] + radius * np.cos(v),
                             color=FAINT, alpha=0.4, linewidth=0.4)
        space.scatter(cells[:, 0], cells[:, 1], cells[:, 2], c=cells[:, 3],
                      cmap=COLOURS, vmin=0.5, vmax=1.0, s=5, marker="s",
                      linewidths=0, depthshade=False)
        space.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        space.text2D(0.0, 1.0, "(a) perspective", transform=space.transAxes,
                     va="top", fontsize=9.5)
        space.view_init(elev=20, azim=-62)
        space.set_box_aspect((1, 1, 1))
        for axis in (space.xaxis, space.yaxis, space.zaxis):
            axis.pane.set_facecolor("white")
            axis.pane.set_edgecolor(FAINT)
            axis.pane.set_alpha(0.25)
            axis._axinfo["grid"].update(color=FAINT, linewidth=0.3, alpha=0.3)
        for limit, value in ((space.set_xlim, centre[0]), (space.set_ylim, centre[1]),
                             (space.set_zlim, centre[2])):
            limit(value - radius, value + radius)

        top = figure.add_subplot(grid[1])
        dots = flat_view(top, cells, centre, radius, 0, 1, ("x", "y"))
        top.text(0.0, 1.06, "(b) top down", transform=top.transAxes, fontsize=9.5)
        side = figure.add_subplot(grid[2])
        flat_view(side, cells, centre, radius, 0, 2, ("x", "z"))
        side.text(0.0, 1.06, "(c) side", transform=side.transAxes, fontsize=9.5)

        bar = figure.colorbar(dots, ax=[side], fraction=0.09, pad=0.12)
        bar.set_label("occupancy probability", fontsize=8.5)
        bar.outline.set_edgecolor(FAINT)
        figure.suptitle(
            f"Occupancy map of the target sphere  ·  {len(cells)} occupied cells  ·  "
            f"r = {radius:.2f} m at ({', '.join(f'{v:.2f}' for v in centre)}) "
            f"in {cloud.header.frame_id}", x=0.04, ha="left", fontsize=10)
        figure.savefig(path, dpi=220)
    print(f"{len(cells)} cells -> {path}")


if __name__ == "__main__":
    main()
