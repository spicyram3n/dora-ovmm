"""Depth image + mask + intrinsics -> 3D points, in the camera's optical frame (meters, +Z forward)."""

import cv2
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


def shrink(mask, pixels=3):
    """Drop the mask's boundary ring, where depth mixes object and background.

    A segmentation edge sits astride both, so those pixels carry the background's
    depth and deproject metres down the view ray, stretching the object's box."""
    if pixels < 1:
        return np.asarray(mask, dtype=bool)
    eroded = cv2.erode(
        np.asarray(mask, dtype=np.uint8), np.ones((3, 3), np.uint8), iterations=pixels
    ).astype(bool)
    # A thin or distant object can erode away entirely; keep the detector's mask then.
    if eroded.any():
        return eroded
    return np.asarray(mask, dtype=bool)


def largest_cluster(points, gap=0.03, minimum=20):
    """The biggest group of points joined through `gap`-sized voxels, or None.

    Whatever survives erosion still lands on the surface behind the object, far
    from the object's own points. Distance separates them; the mask cannot.
    Linkage runs over voxels rather than points, because a mask covering a sofa
    carries hundreds of thousands of them and neighbour search would not scale."""
    from scipy.ndimage import label

    points = np.asarray(points, dtype=float)
    if len(points) < minimum:
        return None
    voxels = np.floor(points / gap).astype(np.int64)
    voxels -= voxels.min(axis=0)
    occupied = np.zeros(voxels.max(axis=0) + 1, dtype=bool)
    occupied[tuple(voxels.T)] = True
    # A 3x3x3 structure joins voxels touching at a corner too, which is the same
    # surface seen at an angle.
    labels, groups = label(occupied, structure=np.ones((3, 3, 3)))
    if groups > 1:
        # Biggest by point count, not by voxel count: density is the evidence.
        per_point = labels[tuple(voxels.T)]
        points = points[per_point == np.bincount(per_point).argmax()]
    if len(points) < minimum:
        return None
    return points


def object_depth_mask(depth_m, mask, step=0.02):
    """Keep the largest depth-connected part of `mask`.

    An alternative to shrink()+largest_cluster() for grasping, where the mask
    must stay in pixel space: it splits the background off across the depth
    discontinuity at the object's silhouette instead of eroding the boundary,
    so the full visible surface survives for the cylinder fit."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    valid = mask & np.isfinite(depth_m) & (depth_m > 0)
    count = int(valid.sum())
    if count == 0:
        raise RuntimeError("no valid depth inside the mask")
    indices = np.full(mask.shape, -1, dtype=np.int32)
    indices[valid] = np.arange(count)
    rows, columns = ([], [])
    for first, second in ((np.s_[:-1, :], np.s_[1:, :]), (np.s_[:, :-1], np.s_[:, 1:])):
        # A jump larger than `step` between neighbours is an edge, not a surface.
        joined = valid[first] & valid[second] & (np.abs(depth_m[first] - depth_m[second]) < step)
        rows.append(indices[first][joined])
        columns.append(indices[second][joined])
    rows, columns = (np.concatenate(rows), np.concatenate(columns))
    linkage = coo_matrix((np.ones(len(rows)), (rows, columns)), shape=(count, count))
    _, labels = connected_components(linkage, directed=False)
    sizes = np.bincount(labels)
    if sizes.max() < count / 2:
        raise RuntimeError("target depth is fragmented; acquire another view")
    result = np.zeros_like(valid)
    result[valid] = labels == sizes.argmax()
    return result


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
