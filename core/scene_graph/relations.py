"""Infer on/in/near using support height, footprint overlap and proximity."""

import numpy as np
from scipy.spatial import ConvexHull, Delaunay, QhullError


def footprint(instance):
    """Return a function that tests whether XY points lie inside the footprint."""
    xy = instance.points[:, :2]
    try:
        boundary = ConvexHull(xy)
        boundary_points = xy[boundary.vertices]
        # Triangulate the floor outline so point-inside checks are fast.
        triangles = Delaunay(boundary_points)

        def inside_hull(query):
            triangle_ids = triangles.find_simplex(query)
            return triangle_ids >= 0

        return inside_hull
    # Use a bounding box when the points cannot form a 2D hull.
    except QhullError:
        lower = xy.min(axis=0)
        upper = xy.max(axis=0)

        def inside_box(query):
            within_bounds = (query >= lower) & (query <= upper)
            return np.all(within_bounds, axis=1)

        return inside_box


def overlap(obj, shadow):
    """Fraction of `obj`'s points standing over a footprint."""
    return float(np.mean(shadow(obj.points[:, :2])))


def distance_to(point, piece):
    """Distance from a point to a piece's box, zero inside it."""
    # Measure only how far the point lies beyond each box face.
    outside = np.maximum(np.maximum(piece.lower - point, point - piece.upper), 0.0)
    return float(np.linalg.norm(outside))


def classify(
    obj,
    furniture,
    shadows=None,
    gap=0.1,
    min_overlap=0.4,
    tolerance=0.05,
    near_limit=2.0,
):
    """Check support first, containment second, and proximity last."""
    if not furniture:
        return (None, None)
    if shadows is None:
        shadows = []
        for piece in furniture:
            shadows.append(footprint(piece))
    shares = []
    for shadow in shadows:
        shares.append(overlap(obj, shadow))
    # Look for furniture whose top touches the object and overlaps its footprint.
    on = []
    for index, piece in enumerate(furniture):
        if shares[index] >= min_overlap and abs(obj.lower[2] - piece.upper[2]) <= gap:
            on.append((-shares[index], index))
    # Choose the supporting surface with the greatest footprint overlap.
    if on:
        return (min(on)[1], "on")
    # Otherwise choose the smallest furniture box that contains the object vertically.
    inside = []
    for index, piece in enumerate(furniture):
        if (
            shares[index] >= min_overlap
            and obj.lower[2] >= piece.lower[2] - tolerance
            and (obj.upper[2] <= piece.upper[2] + tolerance)
        ):
            inside.append((np.prod(piece.dimensions), index))
    # Prefer the smallest enclosing furniture when several pieces contain the object.
    if inside:
        return (min(inside)[1], "in")
    # If neither test passes, choose nearby furniture within the distance limit.
    distances = []
    for piece in furniture:
        distances.append(distance_to(obj.centroid, piece))
    nearest = int(np.argmin(distances))
    if distances[nearest] <= near_limit:
        return (nearest, "near")
    else:
        return (None, None)
