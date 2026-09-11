"""Log-odds occupancy over one sphere of the workspace, in plain numpy.

A dense array rather than an octree: an octree pays for itself by compressing
the empty space outside the region of interest, and next best view crops that
away before anything is stored. A 1 m sphere at 5 cm is 64k cells, so every
query is a numpy mask. The update rule and clamps are octomap's, so a cell
here means what it means there.
"""

import numpy as np

HIT = 0.85
MISS = -0.4
CLAMP = (-2.0, 3.5)


class SphereGrid:
    """Cells are held flat, so deduplicating them is np.unique over one int."""

    def __init__(self, centre, radius, resolution):
        self.centre = np.asarray(centre, dtype=np.float32)
        self.radius = radius
        self.resolution = resolution
        self.size = int(np.ceil(2 * radius / resolution))
        # Corner of the cube around the sphere, so a cell is a division away.
        self.origin = self.centre - self.size * resolution / 2
        self.log_odds = np.zeros(self.size ** 3, dtype=np.float32)
        self.in_sphere = np.linalg.norm(
            self.centres(np.arange(self.size ** 3)) - self.centre, axis=1) <= radius

    def cells(self, points):
        """Flat indices of the cells holding `points`, dropping those outside."""
        grid = np.floor((points - self.origin) / self.resolution).astype(np.int32)
        grid = grid[((grid >= 0) & (grid < self.size)).all(axis=1)]
        return (grid[:, 0] * self.size + grid[:, 1]) * self.size + grid[:, 2]

    def centres(self, cells):
        grid = np.stack(np.unravel_index(cells, (self.size,) * 3), axis=-1)
        return self.origin + (grid.astype(np.float32) + 0.5) * self.resolution

    def integrate(self, sensor, points, empty=None):
        """Mark `points` occupied and the rays from `sensor` to them free.

        `empty` are the endpoints of rays that came back with no surface on
        them: free along their whole length, with nothing occupied at the end.
        Without them the volume in front of a pixel that saw nothing stays
        unknown for ever, and information gain keeps proposing views of it."""
        sensor = np.asarray(sensor, dtype=np.float32).reshape(3)
        hits = self.cells(np.asarray(points, dtype=np.float32).reshape(-1, 3))
        # One ray per cell, not per point. This is the downsample that matters:
        # a frame puts hundreds of points in the same cell, all with the same
        # ray behind them, and the raycast is the only expensive step left.
        hits = np.unique(hits[self.in_sphere[hits]])
        ends = self.centres(hits)
        # Stop a cell short of the surface, so an endpoint is only ever a hit.
        stop = np.linalg.norm(ends - sensor, axis=1) - self.resolution
        if empty is not None and len(empty):
            empty = np.asarray(empty, dtype=np.float32).reshape(-1, 3)
            ends = np.vstack([ends, empty])
            stop = np.concatenate([stop, np.linalg.norm(empty - sensor, axis=1)])
        if len(ends) == 0:
            return

        direction = ends - sensor
        direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-6)
        # Empty rays run to the sensor's full range, which is mostly outside a
        # sphere this size: drop the ones that miss it, and stop the rest at
        # its far side. Everything beyond that is somebody else's business.
        to_centre = self.centre - sensor
        along = direction @ to_centre
        offset = to_centre @ to_centre - along ** 2
        reaches = (offset <= self.radius ** 2) & (along + self.radius > 0)
        direction, stop = (direction[reaches], stop[reaches])
        stop = np.minimum(stop, np.linalg.norm(to_centre) + self.radius)
        if len(direction) == 0:
            return

        # March every ray at once. Half a cell per step: fine enough not to
        # skip a cell crossed diagonally, coarse enough to keep this small.
        step = self.resolution / 2
        distance = np.arange(step, stop.max(), step, dtype=np.float32)
        samples = sensor + direction * distance[:, None, None]
        # Scatter into a mask rather than np.unique: the samples repeat a cell
        # many times over and sorting them to find that out costs more than
        # one pass over the whole grid.
        free = np.zeros(self.log_odds.size, dtype=bool)
        free[self.cells(samples[distance[:, None] < stop])] = True

        # Free first, then hits, so a cell that is both ends up occupied.
        self.log_odds[free] += MISS
        self.log_odds[hits] += HIT
        np.clip(self.log_odds, *CLAMP, out=self.log_odds)


    def occupied(self):
        return (self.log_odds > 0) & self.in_sphere

    def unknown(self):
        """Never seen. The quantity next best view is trying to shrink."""
        return (self.log_odds == 0) & self.in_sphere
