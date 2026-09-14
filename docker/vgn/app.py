"""
VGN inference server, over zenoh: take a TSDF of the space around a target
and return 6-DOF parallel-jaw grasps in it.

VGN (Breyer et al., CoRL 2020; https://github.com/ethz-asl/vgn, devel branch,
the one active_grasp builds on) is a 3D CNN: a 40^3 TSDF in, and for every
voxel a grasp quality, orientation and opening width out. The filtering is
upstream's own: VGN.predict smooths the quality and keeps only voxels just
outside an observed surface; select_local_maxima thresholds it and keeps the
local maxima.

The TSDF is fused on the ROS side (core/perception/tsdf.py), from depth images
and TF this container never sees, so the server stays stateless like SAM3 and
GraspGenX: one grid in, grasps out.

Zenoh queryable key: "vgn/predict"
    query payload:     TSDF grid, raw float32 bytes, shape (40, 40, 40) indexed
                        [x, y, z]: (tsdf + 1) / 2 for observed voxels near a
                        surface, 0 everywhere else (unseen, or far from one)
    query parameters:  threshold  (optional, default 0.9: minimum quality)
                        voxel_size (optional, default 0.0075: the 0.3 m cube
                                    VGN was trained on)
                        quality    (optional, 1 also returns the quality volume)

Reply payload: raw bytes = poses.tobytes() + widths.tobytes() + qualities.tobytes()
                            [+ quality.tobytes()]
    poses:     (M, 4, 4) float32 grasp frames in the grid's frame, metres from
               its corner: z is the approach, the fingers close along y and
               reach 5 cm past the origin
    widths:    (M,) float32 opening between the fingers, metres
    qualities: (M,) float32 in [0, 1], best first
    quality:   (40, 40, 40) float32 filtered per-voxel quality, when asked
Reply attachment: json {"num_grasps": M, "voxel_size": ..., "quality": bool}
No grasp above threshold is a valid reply (M = 0), not an error: it is what
tells a next-best-view loop to keep looking.
"""

import os
from pathlib import Path

import numpy as np
from zenoh_rpc import serve

MODEL = Path(os.environ.get("VGN_MODEL", "/opt/vgn/assets/models/vgn_conv.pth"))
SHAPE = (40, 40, 40)
VOXEL_SIZE = 0.3 / 40

NET = None


def _load_model():
    global NET
    from vgn.detection import VGN

    print(f"[vgn] loading {MODEL}...")
    NET = VGN(MODEL)
    print(f"[vgn] model loaded on {NET.device}.")


def _warmup():
    """One pass on an empty grid, so CUDA picks its 3D conv kernels now rather
    than on the first real request."""
    print("[vgn] warming up...")
    NET.predict(np.zeros(SHAPE, np.float32))
    print("[vgn] warmup done - ready for requests.")


def _on_predict(query):
    from vgn.detection import select_local_maxima

    grid = np.frombuffer(query.payload.to_bytes(), dtype=np.float32)
    if grid.size != np.prod(SHAPE):
        raise ValueError(f"expected {np.prod(SHAPE)} float32 TSDF values, got {grid.size}")
    # A copy: torch.from_numpy wants writable memory.
    grid = grid.reshape(SHAPE).copy()
    threshold = float(query.parameters.get("threshold", "0.9"))
    voxel_size = float(query.parameters.get("voxel_size", str(VOXEL_SIZE)))
    want_quality = query.parameters.get("quality", "0") == "1"
    if not np.isfinite(grid).all() or grid.min() < 0 or grid.max() > 1:
        raise ValueError("TSDF values must be finite and in [0, 1]")
    if not 0 <= threshold <= 1 or not 0 < voxel_size < 1:
        raise ValueError("threshold must be in [0, 1] and voxel_size in (0, 1) m")

    out = NET.predict(grid)
    grasps, qualities = select_local_maxima(voxel_size, out, threshold)
    order = np.argsort(qualities)[::-1]
    poses = np.array([grasps[i].pose.as_matrix() for i in order], np.float32).reshape(-1, 4, 4)
    widths = np.array([grasps[i].width for i in order], np.float32)
    qualities = np.asarray(qualities, np.float32)[order]

    best = f", best {qualities[0]:.2f}" if len(qualities) else ""
    print(f"[vgn] {np.count_nonzero(grid)} observed voxels -> {len(qualities)} grasp(s){best}")
    body = poses.tobytes() + widths.tobytes() + qualities.tobytes()
    if want_quality:
        body += out.qual.astype(np.float32).tobytes()
    meta = {"num_grasps": len(qualities), "voxel_size": voxel_size, "quality": want_quality}
    return meta, body


def main():
    _load_model()
    _warmup()
    serve("vgn/predict", _on_predict, os.environ.get("ZENOH_LISTEN", "tcp/0.0.0.0:7449"))


if __name__ == "__main__":
    main()
