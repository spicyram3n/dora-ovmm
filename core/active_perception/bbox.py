"""The target's axis-aligned box: active_grasp/bbox.py without the ROS messages."""

import itertools

import numpy as np


class AABBox:
    def __init__(self, bbox_min, bbox_max):
        self.min = np.asarray(bbox_min, dtype=float)
        self.max = np.asarray(bbox_max, dtype=float)
        # Use the midpoint as the box centre and the bound differences as its size.
        self.center = 0.5 * (self.min + self.max)
        self.size = self.max - self.min

    @classmethod
    def from_points(cls, points):
        """The tightest box around (N, 3) points of the target."""
        points = np.asarray(points, dtype=float)
        # Find the smallest and largest coordinate along each axis.
        return cls(points.min(axis=0), points.max(axis=0))

    @property
    def corners(self):
        # Choose the low or high value on each axis to get all eight corners.
        return list(itertools.product(*np.vstack((self.min, self.max)).T))

    def to_dict(self):
        return {"min": self.min.tolist(), "max": self.max.tolist()}
