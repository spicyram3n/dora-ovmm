"""Depth image + mask + camera intrinsics -> 3D point cloud, in the camera's
optical frame (meters, +Z forward)."""

import numpy as np


def deproject(depth_m, k, mask):
    """Back-project every masked, valid-depth pixel to a 3D point.

    depth_m: (H, W) float32 depth in meters (0 or NaN = invalid).
    k:       (3, 3) camera intrinsic matrix (fx, fy, cx, cy on the usual spots).
    mask:    (H, W) bool, same resolution as depth_m.

    Returns (N, 3) float32 points.
    """
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[0, 2], k[1, 2]

    valid = mask & np.isfinite(depth_m) & (depth_m > 0)
    v, u = np.nonzero(valid)
    z = depth_m[v, u]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def center(points):
    """Subtract the centroid, since GraspGenX expects an object-centered
    cloud. Returns (centered_points, centroid). Add centroid back to any
    result (e.g. grasp translations) to return to the original frame."""
    centroid = points.mean(axis=0)
    return points - centroid, centroid


def transform_points(matrix, points):
    """Apply a (4, 4) homogeneous transform to (N, 3) points."""
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def transform_poses(matrix, poses):
    """Apply a (4, 4) homogeneous transform to (N, 4, 4) poses."""
    return matrix @ poses
