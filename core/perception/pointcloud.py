"""Depth image + mask + intrinsics -> 3D points, in the camera's optical frame (meters, +Z forward)."""

import cv2
import numpy as np
import trimesh


def shrink(mask, pixels=3):
    """Remove mask-edge pixels that may contain background depth."""
    if pixels < 1:
        return np.asarray(mask, dtype=bool)
    # Peel away one pixel around the mask for each requested iteration.
    eroded = cv2.erode(
        np.asarray(mask, dtype=np.uint8), np.ones((3, 3), np.uint8), iterations=pixels
    ).astype(bool)
    # A thin or distant object can erode away entirely; keep the detector's mask then.
    if eroded.any():
        return eroded
    return np.asarray(mask, dtype=bool)


def largest_cluster(points, gap=0.03, minimum=20):
    """Keep the largest point group connected through gap-sized voxels, or return None."""
    from scipy.ndimage import label

    points = np.asarray(points, dtype=float)
    if len(points) < minimum:
        return None
    # Group points into small voxels to make connected-region detection cheaper.
    voxels = np.floor(points / gap).astype(np.int64)
    voxels -= voxels.min(axis=0)
    # Build a 3D occupancy array and mark the voxels containing points.
    occupied = np.zeros(voxels.max(axis=0) + 1, dtype=bool)
    occupied[tuple(voxels.T)] = True
    # Join neighbouring voxels, including those touching only at a corner.
    labels, groups = label(occupied, structure=np.ones((3, 3, 3)))
    if groups > 1:
        # Biggest by point count, not by voxel count: density is the evidence.
        per_point = labels[tuple(voxels.T)]
        points = points[per_point == np.bincount(per_point).argmax()]
    if len(points) < minimum:
        return None
    return points


class InvalidTargetDepth(RuntimeError):
    """A segmented view has no usable connected target depth."""


def object_depth_mask(depth_m, mask, step=0.02):
    """Keep the largest region joined by similar neighbouring depths, preserving pixel locations."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    # Keep masked pixels only when their depth is finite and positive.
    valid = mask & np.isfinite(depth_m) & (depth_m > 0)
    count = int(valid.sum())
    if count == 0:
        raise InvalidTargetDepth("no valid depth inside the mask")
    # Give each valid pixel an index for the neighbour graph.
    indices = np.full(mask.shape, -1, dtype=np.int32)
    indices[valid] = np.arange(count)
    rows, columns = ([], [])
    # Compare vertically and horizontally adjacent pixels to build depth connections.
    for first, second in ((np.s_[:-1, :], np.s_[1:, :]), (np.s_[:, :-1], np.s_[:, 1:])):
        # A jump larger than `step` between neighbours is an edge, not a surface.
        joined = valid[first] & valid[second] & (np.abs(depth_m[first] - depth_m[second]) < step)
        rows.append(indices[first][joined])
        columns.append(indices[second][joined])
    rows, columns = (np.concatenate(rows), np.concatenate(columns))
    linkage = coo_matrix((np.ones(len(rows)), (rows, columns)), shape=(count, count))
    # Group pixels linked by similar depth and keep the largest group.
    _, labels = connected_components(linkage, directed=False)
    sizes = np.bincount(labels)
    # Reject a fragmented detection if no connected region contains half the pixels.
    if sizes.max() < count / 2:
        raise InvalidTargetDepth("target depth is fragmented; acquire another view")
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
    # Use each selected pixel's measured depth as its camera Z coordinate.
    z = depth_m[rows, cols]
    # Use the camera calibration to turn pixel offsets and depth into metres.
    x = (cols - cx) * z / fx
    y = (rows - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def transform_points(matrix, points):
    """Apply a (4, 4) transform to (N, 3) points."""
    # Rotate the points, then add the translation into the destination frame.
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def transform_poses(matrix, poses):
    """Apply a (4, 4) transform to (N, 4, 4) poses."""
    return matrix @ poses


def save_ply(points, path):
    # Write the XYZ points as a point-cloud file for later inspection.
    trimesh.PointCloud(points).export(str(path))
