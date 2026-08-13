"""Where to point the head to see a given place.

Everyone assumes finding an object needs a pan/tilt sweep. On this robot and
this apartment it does not: the head camera's vertical field of view is 46.8
degrees, and the widest angular span any piece of furniture's top surface
subtends from its own standoff pose is about 20 degrees. One view covers all of
it -- but only if the tilt is computed. Left at zero the camera looks straight
out and the near half of a low table falls below the frame, which fails quietly
rather than obviously.

Geometry from the expanded hsrc1s URDF: head_rgbd_sensor_link sits 1.010 m
above base_footprint with the head joints at zero, and the depth camera is
640x480 over a 60 degree horizontal field of view.
"""

import math

# head_rgbd_sensor_link height above base_footprint, head joints at zero.
CAMERA_HEIGHT = 1.010

# asus_xtion.gazebo.xacro: 1.047 rad across 640 px, so fx = 320/tan(fov/2) and
# the vertical field follows from the 480 px height.
FOV_H = 1.047
FOV_V = 0.8164
FOCAL_PX = 554.4

# hsrc1s.urdf.xacro joint limits. Tilt is negative looking down, and reaches
# only 30 degrees up, which is what saturates on the tall shelves.
PAN_LIMITS = (-3.84, 1.75)
TILT_LIMITS = (-1.57, 0.52)


def look_at(target, base_xy, base_yaw):
    """Head (pan, tilt) that centres `target` -- an (x, y, z) in the same frame
    as `base_xy` -- clamped to the joints' real limits.

    The pan and tilt axes are a few centimetres from the camera itself, so this
    is exact only for a camera at the pan axis. At the 0.8 m and beyond this is
    used for, that offset is under 4 degrees against a 60 degree field, so the
    target lands well inside frame either way.
    """
    dx, dy = target[0] - base_xy[0], target[1] - base_xy[1]
    # into the base's own frame, where pan is measured from straight ahead
    forward = math.cos(-base_yaw) * dx - math.sin(-base_yaw) * dy
    left = math.sin(-base_yaw) * dx + math.cos(-base_yaw) * dy

    pan = math.atan2(left, forward)
    tilt = -math.atan2(CAMERA_HEIGHT - target[2], math.hypot(forward, left))
    return (_clamp(pan, PAN_LIMITS), _clamp(tilt, TILT_LIMITS))


def _clamp(value, limits):
    return max(limits[0], min(limits[1], value))


def covers(target, base_xy, base_yaw, margin=0.0):
    """Would `target` actually be in frame at the pose `look_at` returns?

    False means the joint limits clamped the aim short -- the tall shelves are
    the case that does this, since the head only tilts 30 degrees up. Worth
    asking before blaming the detector for finding nothing.
    """
    pan, tilt = look_at(target, base_xy, base_yaw)
    dx, dy = target[0] - base_xy[0], target[1] - base_xy[1]
    forward = math.cos(-base_yaw) * dx - math.sin(-base_yaw) * dy
    left = math.sin(-base_yaw) * dx + math.cos(-base_yaw) * dy

    want_pan = math.atan2(left, forward)
    want_tilt = -math.atan2(CAMERA_HEIGHT - target[2], math.hypot(forward, left))
    return (abs(want_pan - pan) <= FOV_H / 2 - margin
            and abs(want_tilt - tilt) <= FOV_V / 2 - margin)


def pixel_size(size_m, distance_m):
    """How many pixels across a `size_m` object is at `distance_m`. Below about
    20 px a segmenter has little to work with, which is the real limit on how
    far back standoff.py may park."""
    return size_m / distance_m * FOCAL_PX


def tilt_for(target_z, distance):
    """Tilt that centres a point `target_z` high, `distance` away in the plane."""
    return -math.atan2(CAMERA_HEIGHT - target_z, distance)


def tilt_sweep(low_z, high_z, distance, overlap=0.15):
    """Tilt angles covering everything between `low_z` and `high_z`, fewest
    first, clamped to the joint.

    One view is enough for a table: its work surface is a plane, and the whole
    of it subtends about 20 degrees against a 46.8 degree field. A shelf unit
    is not a plane -- `high_shelf02` runs from the floor to 1.82 m and spans
    nearly 87 degrees from its own standoff, so aiming once puts most of it
    out of frame. Which shelf the object is on is exactly what is not known
    when the search starts, so the sweep covers all of them.
    """
    # Tilt is negative looking down, so the *low* end of the extent is the
    # most negative tilt and the span runs upward from it.
    top, bottom = tilt_for(high_z, distance), tilt_for(low_z, distance)
    span = top - bottom
    views = max(1, math.ceil(span / (FOV_V * (1.0 - overlap))))
    return [_clamp(bottom + span * (i + 0.5) / views, TILT_LIMITS)
            for i in range(views)]


def scan(target_xy, low_z, high_z, base_xy, base_yaw):
    """(pan, tilt) pairs that between them see everything at `target_xy`
    between `low_z` and `high_z`.

    Pass a known object's own height as both bounds and this is a single aim;
    pass a piece of furniture's full extent and it is however many views that
    takes. Which one the caller has is the difference between remembering
    where the object was and only knowing which furniture to search.
    """
    dx, dy = target_xy[0] - base_xy[0], target_xy[1] - base_xy[1]
    forward = math.cos(-base_yaw) * dx - math.sin(-base_yaw) * dy
    left = math.sin(-base_yaw) * dx + math.cos(-base_yaw) * dy
    pan = _clamp(math.atan2(left, forward), PAN_LIMITS)
    distance = math.hypot(forward, left)
    return [(pan, tilt) for tilt in tilt_sweep(low_z, high_z, distance)]
