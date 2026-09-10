"""Depth image + mask + intrinsics -> 3D points, in the camera's optical
frame (meters, +Z forward)."""

import numpy as np
import trimesh
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def object_depth_mask(depth_m, mask, max_neighbor_depth_step=0.02):
    """Keep the largest depth-connected part of the semantic mask.

    SAM boundaries can include wall pixels behind the object. A 2 cm jump
    between adjacent pixels splits those away without flattening the object's
    gradually curved surface or assuming its dimensions.
    """
    valid = mask & np.isfinite(depth_m) & (depth_m > 0)
    count = int(valid.sum())
    if count == 0:
        raise RuntimeError('no valid depth inside the mask')
    indices = np.full(mask.shape, -1, dtype=np.int32)
    indices[valid] = np.arange(count)
    starts, ends = [], []
    for first, second in ((np.s_[:-1, :], np.s_[1:, :]),
                          (np.s_[:, :-1], np.s_[:, 1:])):
        connected = (valid[first] & valid[second] &
                     (np.abs(depth_m[first] - depth_m[second]) < max_neighbor_depth_step))
        starts.append(indices[first][connected])
        ends.append(indices[second][connected])
    rows, cols = np.concatenate(starts), np.concatenate(ends)
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(count, count))
    _, labels = connected_components(graph, directed=False)
    sizes = np.bincount(labels)
    if sizes.max() < count / 2:
        raise RuntimeError('target depth is fragmented; acquire another view')
    result = np.zeros_like(valid)
    result[valid] = labels == sizes.argmax()
    return result


def deproject(depth_m, k, mask):
    """Back-project every masked pixel that has valid depth into (N, 3) points."""
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
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
