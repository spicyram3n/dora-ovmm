"""Request VGN grasps for a TSDF grid. See docker/vgn/app.py for the protocol
and core/perception/tsdf.py for the grid.

VGN's grasp frame is the Franka hand's, at the base of its fingers: z is the
approach and the fingers close along y. hand_palm_link has the same axes
(bugs/grasp-pose-convention.md) and its pads span 1.5-5.4 cm in front of the
palm, so a VGN pose serves as the palm pose unchanged.
"""

import numpy as np

from core.perception.tsdf import RESOLUTION, VOXEL
from core.utils.zenoh_rpc import query

# How far the fingers reach past a VGN grasp origin (upstream's finger_depth).
FINGER_DEPTH = 0.05
SHAPE = (RESOLUTION,) * 3


def predict(grid, threshold=0.9, timeout=10, *, quality=False, metadata=None):
    """Return grid-frame grasp poses, widths in metres, and scores, best first.

    With quality=True, also return the dense quality grid. Zero grasps is valid."""
    # Pack the TSDF in the float32 grid layout expected by VGN.
    grid = np.ascontiguousarray(grid, dtype=np.float32)
    if grid.shape != SHAPE or not np.isfinite(grid).all():
        raise ValueError(f"Expected a finite {SHAPE} TSDF grid")
    selector = f"vgn/predict?threshold={threshold};voxel_size={VOXEL};quality={int(quality)}"
    meta, body = query(selector, grid.tobytes(), timeout, metadata=metadata)
    count = meta.get("num_grasps")
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise RuntimeError("Malformed VGN grasp count")
    # Find the byte ranges for poses, widths, scores, and the optional quality grid.
    poses_end = count * 16 * 4
    widths_end = poses_end + count * 4
    qualities_end = widths_end + count * 4
    volume = int(np.prod(SHAPE)) * 4 if quality else 0
    if len(body) != qualities_end + volume:
        raise RuntimeError("Malformed VGN reply")
    # Decode candidate transforms and matching width and quality arrays.
    poses = np.frombuffer(body[:poses_end], np.float32).reshape(count, 4, 4).astype(np.float64)
    widths = np.frombuffer(body[poses_end:widths_end], np.float32)
    qualities = np.frombuffer(body[widths_end:qualities_end], np.float32)
    if not np.isfinite(poses).all():
        raise RuntimeError("VGN returned nonfinite grasps")
    if not quality:
        return poses, widths, qualities
    return poses, widths, qualities, np.frombuffer(body[qualities_end:], np.float32).reshape(SHAPE)


def fingertips(poses):
    """(M, 3) points midway between each grasp's fingertips."""
    # Move from each grasp origin along approach Z to the fingertip centre.
    return poses[:, :3, 3] + FINGER_DEPTH * poses[:, :3, 2]


def inside(poses, low, high):
    """Check which grasp fingertip centres lie inside the target box [low, high]."""
    # Keep grasps whose fingertip centre falls inside the target box.
    tips = fingertips(poses)
    return np.all((tips >= low) & (tips <= high), axis=1)
