"""Which furniture an object belongs to, and how.

stretch-compose uses "nearest furniture centroid", which fails exactly where
it matters: a mug on a low table beside a tall shelf binds to the shelf, whose
centroid sits high and close. Testing support and containment first fixes
that, and the in/on/near word then falls out of the geometry instead of being
something the LLM has to invent.

Everything here works off the convex hull of an instance's points rather than
its axis-aligned box, because furniture in a real scan is rarely axis aligned
and a sofa at forty degrees has a box half again its own size -- easily enough
to swallow objects that are merely standing beside it.
"""

import numpy as np
from scipy.spatial import ConvexHull, Delaunay


def footprint(instance):
    """A test for "does this xy point stand over the instance's floor shadow".

    Returns a callable taking (N, 2) and giving (N,) bools, so the hull is
    triangulated once per piece of furniture rather than once per object.
    """
    xy = instance.points[:, :2]
    try:
        hull = Delaunay(xy[ConvexHull(xy).vertices])
        return lambda query: hull.find_simplex(query) >= 0
    except Exception:
        # Fewer than three distinct points, or all of them collinear: no hull
        # exists and the rectangle is the best available answer.
        lower, upper = xy.min(axis=0), xy.max(axis=0)
        return lambda query: np.all((query >= lower) & (query <= upper), axis=1)


def overlap(obj, shadow):
    """Fraction of `obj`'s points standing over a footprint."""
    return float(np.mean(shadow(obj.points[:, :2])))


def distance_to(point, piece):
    """Distance from a point to a piece's box, zero inside it. Better than
    centroid to centroid for anything long: the far end of a three metre sofa
    is not two metres from an object beside it."""
    outside = np.maximum(np.maximum(piece.lower - point, point - piece.upper), 0.0)
    return float(np.linalg.norm(outside))


def classify(obj, furniture, shadows=None, gap=0.10, min_overlap=0.4,
             tolerance=0.05, near_limit=2.0):
    """Pick the furniture `obj` belongs to, in descending order of how
    specific the test is: resting on something, inside something, else merely
    near it.

    shadows:     footprints for `furniture`, computed here if not supplied
    gap:         how far an underside may float above a surface and still rest on it
    min_overlap: how much of the object has to be over that surface
    near_limit:  past this, "near" means nothing and the object is unattached.
                 Scene-scale dependent, so worth raising for a large room.

    Returns (index into `furniture`, relation), or (None, None).
    """
    if not furniture:
        return None, None
    if shadows is None:
        shadows = [footprint(piece) for piece in furniture]
    shares = [overlap(obj, shadow) for shadow in shadows]

    # Support first because it keys on the top face specifically, which
    # containment does not: a wallet lying on a table is thinner than the
    # containment tolerance, so the other order would call it "in".
    # Most-overlapped surface wins, settling an object bridging two tables.
    on = [(-shares[i], i) for i, piece in enumerate(furniture)
          if shares[i] >= min_overlap and abs(obj.lower[2] - piece.upper[2]) <= gap]
    if on:
        return min(on)[1], "on"

    # Smallest container wins: an apple in a shelf standing against a
    # wall-sized cabinet binds to the shelf.
    inside = [(np.prod(piece.dimensions), i) for i, piece in enumerate(furniture)
              if shares[i] >= min_overlap
              and obj.lower[2] >= piece.lower[2] - tolerance
              and obj.upper[2] <= piece.upper[2] + tolerance]
    if inside:
        return min(inside)[1], "in"

    distances = [distance_to(obj.centroid, piece) for piece in furniture]
    nearest = int(np.argmin(distances))
    return (nearest, "near") if distances[nearest] <= near_limit else (None, None)
