"""Offline alignment diagnostic for an upright cylindrical target in odom.

Uses segmented RGB-D points, never simulator object poses. This is deliberately
not applied to arbitrary open-vocabulary objects or automatically executed.
"""
import numpy as np


def fit_upright_can(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 100:
        raise ValueError("Need at least 100 object depth points")
    if not np.isfinite(points).all():
        raise ValueError("Nonfinite object depth")
    bottom, top = np.quantile(points[:, 2], [.01, .99])
    # Select by physical height, not point-count percentiles: a camera above
    # a low table sees many lid pixels, which otherwise enter the side fit.
    lo = bottom + .20 * (top - bottom)
    hi = top - .20 * (top - bottom)
    if hi - lo < .04:
        raise ValueError("Insufficient upright side surface")
    xy = points[(points[:, 2] > lo) & (points[:, 2] < hi), :2]
    if len(xy) < 100:
        raise ValueError("Need at least 100 side-surface depth points")
    origin = xy.mean(axis=0)
    local = xy - origin
    a = np.column_stack((2 * local, np.ones(len(local))))
    if np.linalg.matrix_rank(a) < 3:
        raise ValueError("Side surface does not constrain a circle")
    solution = np.linalg.lstsq(a, np.sum(local ** 2, axis=1), rcond=None)[0]
    radius_squared = solution[2] + solution[:2] @ solution[:2]
    if radius_squared <= 0:
        raise ValueError("Invalid cylinder radius")
    radius = float(np.sqrt(radius_squared))
    center = origin + solution[:2]
    residual = float(np.sqrt(np.mean((np.linalg.norm(xy - center, axis=1) - radius) ** 2)))
    # Bound this diagnostic to a can-sized shape with substantial visible arc.
    directions = (xy - center) / radius
    if not .015 <= radius <= .06 or residual > .0015:
        raise ValueError("Depth does not support a can-sized circular section")
    if np.linalg.norm(directions.mean(axis=0)) > .95:
        raise ValueError("Too little visible arc to infer the can centre")
    return center, radius, residual


def align_side_grasp(pose, center_xy, radius, closing_profile, level=False):
    """Return a candidate translated in XY to centre pads around the can.

    Preserve generated orientation and world height. Profile entries contain
    actual canonical pad centres and free-motion apertures measured from URDF.
    The result still requires fresh observations and full collision checking.
    """
    pose = np.asarray(pose, dtype=float)
    rotation = pose[:3, :3].copy()
    if abs(rotation[2, 0]) > .15 or abs(rotation[2, 2]) > .15:
        raise ValueError("Only nearly horizontal side grasps are supported")
    original_rotation = rotation.copy()
    if level:
        approach = rotation[:, 2].copy()
        approach[2] = 0.
        approach /= np.linalg.norm(approach)
        closing = np.cross([0., 0., 1.], approach)
        if np.dot(closing, rotation[:, 0]) < 0:
            closing *= -1
        rotation = np.column_stack([closing, np.cross(approach, closing), approach])
    profile = sorted(closing_profile, key=lambda sample: sample['gap'][0])
    gaps = np.array([sample['gap'][0] for sample in profile])
    # Projection of the cylinder diameter onto the slightly tilted closing axis.
    width = 2 * radius / np.sqrt(1 - rotation[2, 0] ** 2)
    if not gaps[0] <= width <= gaps[-1] - .01:
        raise ValueError("Can does not fit the calibrated gripper aperture")
    pad = np.array([np.interp(width, gaps, [s['center'][i] for s in profile]) for i in range(3)])
    contact = pose[:3, 3] + rotation @ pad
    # Level about the pad center, preserving its height rather than moving
    # the fingers vertically when correcting wrist orientation.
    original_contact = pose[:3, 3] + original_rotation @ pad
    shift = np.r_[np.asarray(center_xy) - contact[:2],
                  original_contact[2] - contact[2] if level else 0.]
    if np.linalg.norm(shift) > .05:
        raise ValueError("Required correction exceeds 5 cm; regenerate grasps")
    corrected = pose.copy()
    corrected[:3, :3] = rotation
    corrected[:3, 3] += shift
    return corrected, shift, pad
