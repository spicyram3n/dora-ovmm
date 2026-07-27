"""Plot an object point cloud with candidate grasp poses, colored by score,
and the real gripper mesh at the top-scoring grasp.

Uses matplotlib only (no display server needed) and saves a PNG.
"""

import numpy as np
import trimesh
from matplotlib import cm
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

AXIS_LEN = 0.03  # meters, length of each grasp's drawn coordinate frame
AXIS_COLORS = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # x, y, z
MAX_POINTS = 20_000  # subsample the cloud above this, just for plot speed


def _grasp_axis_lines(grasps):
    """One (origin -> tip) line segment per axis per grasp, for a Line3DCollection."""
    lines, colors = [], []
    for pose in grasps:
        origin = pose[:3, 3]
        for axis, color in enumerate(AXIS_COLORS):
            lines.append([origin, origin + pose[:3, axis] * AXIS_LEN])
            colors.append(color)
    return lines, colors


def _gripper_mesh_at(mesh_path, base_rotation, grasp_pose):
    """Load the gripper's visual mesh and place it at `grasp_pose`.

    GraspGenX poses are given in a canonical grasp frame (+Z = approach).
    `base_rotation`, from the gripper's x_grippers/<name>/config.json, maps
    the mesh's own native (URDF) frame into that canonical frame.
    """
    mesh = trimesh.load(mesh_path, force="mesh")
    mesh.apply_transform(np.asarray(base_rotation, dtype=np.float64))
    mesh.apply_transform(grasp_pose)
    return mesh


def plot_grasps(points, grasps, scores, out_path, gripper_mesh_path=None, gripper_base_rotation=None):
    """Save a 3D plot: `points` as a gray cloud, each grasp as a colored
    (by score) origin plus red/green/blue axis triad, and the gripper mesh
    at the best-scoring grasp."""
    if len(points) > MAX_POINTS:
        points = points[np.random.choice(len(points), MAX_POINTS, replace=False)]

    fig = Figure(figsize=(9, 8))
    ax = fig.add_subplot(projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c="gray", alpha=0.3)

    if len(grasps):
        best = int(np.argmax(scores))

        lines, colors = _grasp_axis_lines(grasps)
        widths = [3.0] * len(lines)
        widths[best * 3:best * 3 + 3] = [6.0, 6.0, 6.0]  # highlight the best grasp's axes
        ax.add_collection(Line3DCollection(lines, colors=colors, linewidths=widths))

        sc = ax.scatter(grasps[:, 0, 3], grasps[:, 1, 3], grasps[:, 2, 3],
                         c=scores, cmap=cm.get_cmap("viridis"), s=20, depthshade=False)
        fig.colorbar(sc, ax=ax, label="grasp score", shrink=0.6)

        if gripper_mesh_path is not None:
            mesh = _gripper_mesh_at(gripper_mesh_path, gripper_base_rotation, grasps[best])
            ax.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], alpha=0.4, facecolor="red"))

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    title = f"{len(grasps)} grasp candidates (best score={scores.max():.2f})" if len(grasps) else "no grasps found"
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
