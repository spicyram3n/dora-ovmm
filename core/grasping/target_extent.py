"""Complete a fitted cylinder's visible top from the same RGB-D observation.

This does not invent hidden geometry or use simulator model dimensions.
Only depth inside the fitted cross-section and a contiguous upper surface is
considered. The lower bound is never expanded into the supporting surface.
"""
import numpy as np

def complete_cylinder_extent(geometry, scene_points, maximum_extension=.05):
    points = np.asarray(scene_points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('Expected Nx3 scene points')
    result = dict(geometry)
    radial = np.hypot(points[:, 0]-geometry['center_x'], points[:, 1]-geometry['center_y'])
    start = geometry['max_z'] - .01
    selected = points[np.isfinite(points).all(axis=1) & (radial <= geometry['radius'] + .002)
        & (points[:, 2] >= start) & (points[:, 2] <= geometry['max_z'] + maximum_extension)]
    if len(selected) < 30:
        return result
    edges = np.arange(start, geometry['max_z'] + maximum_extension + .005, .005)
    counts, _ = np.histogram(selected[:, 2], edges)
    # Require observations connecting to the existing target. Never bridge a
    # gap to a shelf or a separate object above it.
    if len(counts) < 3 or min(counts[:2]) < 5:
        return result
    last = 1
    for index in range(2, len(counts)):
        if counts[index] < 5:
            break
        last = index
    # If the search ends inside a surface, its extent is unknown; stop rather
    # than truncate it and claim the complete target has been measured.
    if last == len(counts)-1:
        raise ValueError('Target upper extent exceeds bounded RGB-D search; observe again')
    upper = selected[selected[:, 2] < edges[last+1], 2].max() + .001
    result['max_z'] = max(geometry['max_z'], float(upper))
    return result
