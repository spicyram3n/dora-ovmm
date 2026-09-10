"""Depth image + mask + intrinsics -> 3D points, in the camera's optical frame (meters, +Z forward)."""

import cv2
import numpy as np
import trimesh


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
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    points = np.asarray(points, dtype=float)
    if len(points) < minimum:
        return None
    voxels, inverse = np.unique(
        np.floor(points / gap).astype(np.int64), axis=0, return_inverse=True
    )
    inverse = inverse.ravel()
    index = {}
    for position, voxel in enumerate(voxels):
        index[tuple(voxel)] = position
    # Half of the 26-neighbourhood: each adjacency is undirected, so once is enough.
    offsets = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if (dx, dy, dz) > (0, 0, 0):
                    offsets.append((dx, dy, dz))
    rows, columns = ([], [])
    for position, voxel in enumerate(voxels):
        for dx, dy, dz in offsets:
            neighbour = index.get((voxel[0] + dx, voxel[1] + dy, voxel[2] + dz))
            if neighbour is not None:
                rows.append(position)
                columns.append(neighbour)
    linkage = coo_matrix(
        (np.ones(len(rows)), (rows, columns)), shape=(len(voxels), len(voxels))
    )
    count, labels = connected_components(linkage, directed=False)
    if count > 1:
        per_point = labels[inverse]
        # Biggest by point count, not by voxel count: density is the evidence.
        points = points[per_point == int(np.bincount(per_point).argmax())]
    if len(points) < minimum:
        return None
    return points


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
