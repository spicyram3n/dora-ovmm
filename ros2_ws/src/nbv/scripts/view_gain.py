#!/usr/bin/env python3
"""What a candidate view is worth: the unknown cells it would uncover.

An unknown cell sits at occupancy 0.5 and so holds exactly one bit; a known one
is clamped near 0.12 or 0.97, and another look cannot move it. So the gain of a
view, in bits, is the number of unknown cells of the target it can actually see.

The work is per target cell, not per camera pixel: a cell is visible when it
falls inside the image, within range, and nothing occupied stands between it and
the lens. That makes a score independent of how finely the frustum is sampled,
and costs the same whether the map is one room or a whole flat.

Which cells are the target is the caller's business — build_octomap.sphere_cells
gives the sphere ones. No ROS here, so a view can be scored against a tree built
in memory:

    python3 -m pytest -p no:anyio ros2_ws/src/nbv/test/test_view_gain.py
"""

import numpy as np

# As build_octomap.py: these two install as programs, with no module between
# them to share a constant.
UNKNOWN, FREE, OCCUPIED = -1, 0, 1
MAX_RANGE = 2.5


def in_view(origin, rotation, cells, info, max_range=MAX_RANGE):
    """Which cells fall inside the image, before anything blocking is considered.

    `rotation` holds the camera's axes as columns in the map's frame, so that
    `rotation @ [0, 0, 1]` is where it points.
    """
    # The camera's own frame: x right, y down, z forward.
    local = (cells - origin) @ rotation
    with np.errstate(invalid="ignore", divide="ignore"):
        columns = info.k[0] * local[:, 0] / local[:, 2] + info.k[2]
        rows = info.k[4] * local[:, 1] / local[:, 2] + info.k[5]
    return ((local[:, 2] > 0) & (local[:, 2] <= max_range)
            & (0 <= columns) & (columns < info.width)
            & (0 <= rows) & (rows < info.height))


def unoccluded(tree, origin, cells):
    """Those cells with no surface between them and the camera."""
    end = np.empty(3)
    clear = []
    for cell in cells:
        towards = cell - origin
        distance = np.linalg.norm(towards)
        # Unknown space does not block: it is the thing being measured, and
        # treating it as solid would score every first look at nothing.
        clear.append(not tree.castRay(origin, towards / distance, end, True, distance))
    return cells[clear]


def view_gain(tree, origin, rotation, cells, info, max_range=MAX_RANGE):
    """Bits the view at (`origin`, `rotation`) would uncover among `cells`."""
    origin = np.asarray(origin, dtype=np.float64)
    cells = np.asarray(cells, dtype=np.float64)
    # Narrowest test first: the image is a cheap array of comparisons, asking
    # the map costs a lookup per cell, and casting a ray costs the most.
    framed = cells[in_view(origin, rotation, cells, info, max_range)]
    unknown = framed[tree.getLabels(framed) == UNKNOWN]
    return len(unoccluded(tree, origin, unknown))
