"""Depth image + mask + intrinsics -> 3D points, in the camera's optical frame (meters, +Z forward)."""

import numpy as np
import trimesh


def deproject(depth_m, k, mask):
    """Back-project every masked pixel that has valid depth into (N, 3) points."""
    fx = k[0, 0]
    fy = k[1, 1]
    cx = k[0, 2]
    cy = k[1, 2]
    # Missing depth cannot define a 3D object point.
    rows, cols = np.nonzero(mask & np.isfinite(depth_m) & (depth_m > 0))
    z = depth_m[rows, cols]
    x = (cols - cx) * z / fx
    y = (rows - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def transform_points(matrix, points):
    """Apply a (4, 4) transform to (N, 3) points."""
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def transform_poses(matrix, poses):
    """Apply a (4, 4) transform to (N, 4, 4) poses."""
    return matrix @ poses


def save_ply(points, path):
    trimesh.PointCloud(points).export(str(path))
