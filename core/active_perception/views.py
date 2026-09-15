"""Candidate camera views the HSR head can take: ETH's ViewHalfSphere plus
the reachability check their generate_views() gets from an IK solver.

The Panda carries its camera on a 7-DoF arm, so ETH sample a half sphere
above the target at fixed polar angles and keep the views IK can reach. The
HSR's RGB-D camera sits on a pan-tilt head over the torso lift of a mobile
base, so a view is reachable when: the camera height lies in the lift's range,
the tilt lies in the head's range, the image is level (the head has no roll),
the base stands clear of the target and of every furniture footprint, and,
given a costmap, on a free cell. The footprints matter: the laser costmap holds
a table's legs only, so Nav2 would happily drive the base under its top.
Every view is base_from_camera in the optical convention (z forward, y down).
"""

import numpy as np

from core.navigation.standoff import ROBOT_RADIUS, blocks, free

# HSRC: camera 1.015 m over the floor with the torso down (lab_20260811
# keyframes); the torso lift adds up to 0.345 m (hsrb_description torso_v0).
CAMERA_HEIGHT = 1.015
TORSO_LIFT = 0.345
# head_tilt_joint limits (hsrb_description head_v2): looking down is negative.
TILT_MIN, TILT_MAX = (-1.57, 0.52)
# The camera stands over the base centre, near enough for footprint clearance.
CLEARANCE = ROBOT_RADIUS + 0.05


def spherical_to_cartesian(r, theta, phi):
    return np.r_[r * np.sin(theta) * np.cos(phi), r * np.sin(theta) * np.sin(phi), r * np.cos(theta)]


def look_at(eye, center, up):
    """vgn.utils.look_at: a camera at `eye` with its optical axis on `center`."""
    eye, center = (np.asarray(eye, dtype=float), np.asarray(center, dtype=float))
    forward = center - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    m = np.eye(4)
    m[:3, 0] = right
    m[:3, 1] = -up
    m[:3, 2] = forward
    m[:3, 3] = eye
    return m


class ViewHalfSphere:
    """ETH's sphere around the target, sampled where the HSR head can be."""

    def __init__(self, bbox, min_z_dist, floor_z=0.0, grid=None, blockers=(), phis=8, rings=3, ring_step=0.3):
        """`blockers` are furniture footprints (centre, dimensions, yaw) in the
        base frame, as core.scene_graph.graph.footprint gives them."""
        self.bbox = bbox
        self.blockers = list(blockers)
        self.rejected = []  # eyes Nav2 refused to drive under; not offered again
        self.center = bbox.center
        self.r = 0.5 * bbox.size[2] + min_z_dist
        # A base cannot hover over the table like the Panda's camera, so the
        # sphere is repeated at larger radii until it stands clear of furniture.
        self.radii = self.r + ring_step * np.arange(rings)
        self.floor_z = floor_z
        self.grid = grid
        self.phis = np.arange(phis) * (2 * np.pi / phis)

    def get_view(self, theta, phi, r):
        eye = self.center + spherical_to_cartesian(r, theta, phi)
        return look_at(eye, self.center, up=np.r_[0.0, 0.0, 1.0])

    def thetas(self, r):
        """Polar angles that put the camera at the torso's lowest and highest
        height, in place of ETH's fixed 15 and 30 degrees."""
        heights = self.floor_z + CAMERA_HEIGHT + np.array([0.0, TORSO_LIFT])
        cos = (heights - self.center[2]) / r
        return np.arccos(cos[np.abs(cos) < 1.0])

    def feasible(self, view):
        eye, forward = (view[:3, 3], view[:3, 2])
        height = eye[2] - self.floor_z
        tilt = np.arcsin(np.clip(forward[2], -1.0, 1.0))
        clear = np.any((eye[:2] < self.bbox.min[:2] - CLEARANCE) | (eye[:2] > self.bbox.max[:2] + CLEARANCE))
        return (
            CAMERA_HEIGHT - 1e-6 <= height <= CAMERA_HEIGHT + TORSO_LIFT + 1e-6
            and TILT_MIN <= tilt <= TILT_MAX
            and bool(clear)
            and not blocks(eye[:2], self.blockers)
            and not any(np.linalg.norm(eye - e) < 0.01 for e in self.rejected)
            and (self.grid is None or free(self.grid, eye[:2]))
        )

    def all_views(self):
        for r in self.radii:
            for theta in self.thetas(r):
                for phi in self.phis:
                    yield self.get_view(theta, phi, r)

    def candidates(self):
        """NextBestView.generate_views() with feasible() in place of solve_cam_ik()."""
        return [view for view in self.all_views() if self.feasible(view)]
