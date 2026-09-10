"""Plot an object point cloud with its candidate grasps, colored by score,
and the real gripper mesh at the best one. Matplotlib only, so no display
server is needed."""

from pathlib import Path

import numpy as np
import trimesh
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

GRIPPERS_DIR = Path(__file__).resolve().parents[2] / "docker" / "graspgenx" / "x_grippers"
AXIS_LENGTH = 0.03  # meters, per drawn grasp frame
AXIS_COLORS = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # x, y, z
MAX_POINTS = 20_000  # subsample above this, just for plot speed


def _axis_lines(poses):
    """One origin -> tip segment per axis per grasp, plus a color for each."""
    lines, colors = [], []
    for pose in poses:
        origin = pose[:3, 3]
        for axis, color in enumerate(AXIS_COLORS):
            lines.append([origin, origin + pose[:3, axis] * AXIS_LENGTH])
            colors.append(color)
    return lines, colors


def _gripper_mesh_at(gripper, pose):
    """Load the gripper's visual mesh and place it at `pose`. GraspGenX poses
    live in a canonical grasp frame (+Z = approach). The registration wizard
    already applies base_rotation when exporting vis_mesh.obj."""
    gripper_dir = GRIPPERS_DIR / gripper
    mesh = trimesh.load(gripper_dir / "vis_mesh.obj", force="mesh")
    mesh.apply_transform(pose)
    return mesh


def plot_grasps(points, poses, scores, path, gripper=None):
    """Save a 3D plot of the cloud in gray, every grasp as a score-colored
    origin with an rgb axis triad, and the gripper at the best grasp."""
    if len(points) > MAX_POINTS:
        points = points[np.random.choice(len(points), MAX_POINTS, replace=False)]

    figure = Figure(figsize=(9, 8))
    axes = figure.add_subplot(projection="3d")
    axes.scatter(*points.T, s=1, c="gray", alpha=0.3)

    best = int(np.argmax(scores))
    lines, colors = _axis_lines(poses)
    widths = [3.0] * len(lines)
    widths[best * 3:best * 3 + 3] = [6.0] * 3  # thicken the best grasp's axes
    axes.add_collection(Line3DCollection(lines, colors=colors, linewidths=widths))

    origins = axes.scatter(*poses[:, :3, 3].T, c=scores, cmap="viridis", s=20, depthshade=False)
    figure.colorbar(origins, ax=axes, label="grasp score", shrink=0.6)

    if gripper:
        mesh = _gripper_mesh_at(gripper, poses[best])
        axes.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], alpha=0.4, facecolor="red"))

    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_zlabel("z [m]")
    axes.set_title(f"{len(poses)} grasp candidates (best score {scores.max():.2f})")
    figure.tight_layout()
    figure.savefig(path, dpi=150)
