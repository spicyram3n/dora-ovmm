#!/usr/bin/env python3
"""Seed the robot's localizer by matching the live /scan against the map.

The HSR's laser_2d_localizer is not seeded at boot: it reports the map origin
until something publishes a pose. RViz's "2D Pose Estimate" is the manual way;
this is the automatic one, so a fresh start does not need a hand-typed pose.

    python3 realrobot/live/localize.py                 # search the whole map
    python3 realrobot/live/localize.py --near 2.0 -3.5 # search 3 m around a guess
    python3 realrobot/live/localize.py --dry-run       # report the pose, publish nothing
    python3 realrobot/live/localize.py --watch         # measure drift while it drives

It publishes on both /laser_2d_correct_pose (what Toyota's localizer reads) and
/initialpose (what AMCL reads), so it needs nothing else running and can be used
before Nav2 starts -- which is the right order, because a costmap built while
the pose was wrong keeps phantom obstacles. Publish nothing and say so when the
match is poor: a confident wrong pose is worse than none.

Check the result before trusting it. The map is from 2026-08-11 and the lab's
desks and chairs have moved since, so clutter matches badly and only the walls
are dependable; a symmetric corridor has several equally good poses and this
picks one of them. --near disambiguates. Cross-check against the robot's own
/laser_2d_localizer/score, which runs on the robot's current map: about -0.01
at the right pose, -0.07 at a wrong one.
"""

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from core.utils.recording import Paths  # noqa: E402

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image
from rclpy.node import Node
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MAP = str(Paths().map)
SCAN = "/scan"
# Both, so this works whichever localizer is up and whether or not Nav2 is
# running: laser_2d_localizer reads the first, AMCL and RViz the second, and
# the launch's relay only exists while Nav2 does.
OUTPUT = ["/laser_2d_correct_pose", "/initialpose"]
BASE = "base_footprint"

# A pose is accepted when the median beam lands this close to a mapped wall.
# At the right pose in the lab this reads 0 cm with 81% of beams within 10 cm;
# at the map origin, 35 cm with 13%. The gap is wide, so the threshold is loose.
ACCEPT_MEDIAN = 0.15

# Beam-to-wall spread of the likelihood field, in metres: AMCL's sigma_hit.
# config/nav2/nav2_params.yaml uses 0.2; 0.1 is sharper, which is what a
# from-scratch search needs to separate poses a cluttered room makes similar.
SIGMA = 0.1

# The base cannot stand closer than its own radius to a wall, so candidate
# positions nearer than this are not searched. Matches nav2's robot_radius.
ROBOT_RADIUS = 0.30


def read_map(path):
    """The occupancy grid as a wall-distance field, in metres, plus its origin."""
    import yaml

    with open(path) as handle:
        meta = yaml.safe_load(handle)
    image = path.rsplit("/", 1)[0] + "/" + meta["image"]
    pixels = np.array(Image.open(image)).astype(float) / 255.0
    # PGM row 0 is the top of the image, which is max y; the grid starts at min y.
    pixels = np.flipud(pixels)
    occupied = (1.0 - pixels) > meta.get("occupied_thresh", 0.65)
    free = (1.0 - pixels) < meta.get("free_thresh", 0.25)
    resolution = meta["resolution"]
    origin = meta["origin"][:2]
    return distance_transform_edt(~occupied) * resolution, free, resolution, origin


def beams(scan, keep, max_range):
    """Finite returns, thinned to `keep` of them, as (range, angle)."""
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    usable = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < max_range)
    ranges, angles = ranges[usable], angles[usable]
    step = max(1, len(ranges) // keep)
    return ranges[::step], angles[::step]


def score(positions, yaw, ranges, angles, field, resolution, origin):
    """Cost of each candidate: AMCL's likelihood field, negated so lower is better.

    A median over beam-to-wall distances saturates in a cluttered room -- any
    pose that lands its beams somewhere among the desks reads about 5 cm, and
    poses 1.6 m and 50 degrees apart tie. exp(-d^2) does not saturate: a beam
    on a wall counts for much more than one merely near clutter, so the sum
    separates the true pose from its near misses.
    """
    height, width = field.shape
    ends_x = ranges * np.cos(angles + yaw)
    ends_y = ranges * np.sin(angles + yaw)
    # (positions, beams): every candidate scored against every beam at once.
    cols = ((positions[:, 0, None] + ends_x - origin[0]) / resolution).astype(int)
    rows = ((positions[:, 1, None] + ends_y - origin[1]) / resolution).astype(int)
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    distances = field[np.clip(rows, 0, height - 1), np.clip(cols, 0, width - 1)]
    likelihood = np.where(inside, np.exp(-(distances ** 2) / (2 * SIGMA ** 2)), 0.0)
    # A pose that throws most of its scan off the map is not a candidate.
    enough = np.count_nonzero(inside, axis=1) > 0.6 * len(ranges)
    return np.where(enough, -likelihood.sum(axis=1) / len(ranges), np.inf)


def search(ranges, angles, candidates, yaws, field, resolution, origin):
    """The lowest-cost (position, heading) over the candidate grid."""
    best = (np.inf, 0.0, 0.0, 0.0)
    for yaw in yaws:
        medians = score(candidates, yaw, ranges, angles, field, resolution, origin)
        index = int(np.argmin(medians))
        if medians[index] < best[0]:
            best = (float(medians[index]), candidates[index, 0], candidates[index, 1], yaw)
    return best


def refine(best, ranges, angles, field, resolution, origin):
    """Three passes, each halving the step, around the coarse winner."""
    cost, x, y, yaw = best
    for level in range(4):
        span = 0.25 / 2 ** level
        turn = math.radians(4) / 2 ** level
        offsets = np.arange(-3, 4)
        grid = np.array([[x + dx * span, y + dy * span]
                         for dx in offsets for dy in offsets])
        found = search(ranges, angles, grid, yaw + offsets * turn,
                       field, resolution, origin)
        if found[0] <= cost:
            cost, x, y, yaw = found
    return cost, x, y, yaw


def quality(x, y, yaw, ranges, angles, field, resolution, origin):
    cols = ((x + ranges * np.cos(angles + yaw) - origin[0]) / resolution).astype(int)
    rows = ((y + ranges * np.sin(angles + yaw) - origin[1]) / resolution).astype(int)
    height, width = field.shape
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    near = field[rows[inside], cols[inside]]
    return float(np.median(near)), float((near <= 0.10).mean())


class Localizer(Node):

    def __init__(self, scan_topic, output_topics):
        super().__init__("scan_localizer")
        self.scan = None
        self.create_subscription(LaserScan, scan_topic,
                                 lambda message: setattr(self, "scan", message), 1)
        self.publishers_ = [self.create_publisher(PoseWithCovarianceStamped, topic, 1)
                            for topic in output_topics]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def wait(self, attribute, timeout):
        deadline = time.time() + timeout
        while getattr(self, attribute) is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
        return getattr(self, attribute)

    def laser_offset(self, laser_frame, timeout=15.0):
        """Where the laser sits in the base frame, so a laser match becomes a base pose."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            try:
                found = self.buffer.lookup_transform(BASE, laser_frame, rclpy.time.Time())
            except Exception:
                continue
            return found.transform.translation.x, found.transform.translation.y
        raise RuntimeError(f"no TF {BASE} -> {laser_frame}; is the robot up?")

    def map_from_odom(self, timeout=3.0):
        """The correction the localizer is applying, or None. Constant while the
        base drives means nothing is correcting: pure odometry from here on."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            try:
                found = self.buffer.lookup_transform("map", "odom", rclpy.time.Time())
            except Exception:
                continue
            return found.transform.translation.x, found.transform.translation.y
        return None

    def believed_pose(self, timeout=3.0):
        """Where the running localizer currently thinks the base is, or None.

        Absent before anything publishes map->odom, which is the normal state
        the first time this script runs after a boot."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            try:
                found = self.buffer.lookup_transform("map", BASE, rclpy.time.Time())
            except Exception:
                continue
            translation, rotation = found.transform.translation, found.transform.rotation
            return (translation.x, translation.y,
                    math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                               1 - 2 * (rotation.y ** 2 + rotation.z ** 2)))
        return None


def watch(node, options, field, resolution, origin, offset_x, offset_y):
    """Score the live scan at the pose the stack believes, over and over.

    No search and nothing published: this asks one question, "does /scan fit the
    map where the localizer says we are", and answers it while the base drives.
    A median that climbs as the robot moves is the localizer failing to correct,
    which is what pulls the odom-frame local costmap off the map-frame global
    one. Watch map->odom beside it: frozen while the base moves is the proof."""
    print(f"watching every {options.watch:.0f} s; Ctrl-C to stop\n")
    print(f"{'time':>6}  {'believed x,y':>18}  {'map->odom':>16}  "
          f"{'median':>7}  {'<10cm':>6}")
    started = time.time()
    while True:
        node.scan = None
        scan = node.wait("scan", timeout=5.0)
        believed = node.believed_pose()
        correction = node.map_from_odom()
        if scan is None or believed is None:
            print(f"{time.time() - started:>6.0f}  "
                  f"{'no scan' if scan is None else 'no map->base TF':>18}")
            time.sleep(options.watch)
            continue
        ranges, angles = beams(scan, options.beams, options.max_range)
        # quality() wants the laser's pose; believed_pose() gives the base's.
        base_x, base_y, yaw = believed
        laser_x = base_x + offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
        laser_y = base_y + offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
        median, within = quality(laser_x, laser_y, yaw, ranges, angles,
                                 field, resolution, origin)
        drift = ("     --" if correction is None
                 else f"{correction[0]:>7.3f},{correction[1]:>7.3f}")
        print(f"{time.time() - started:>6.0f}  {base_x:>8.3f},{base_y:>8.3f}  "
              f"{drift:>16}  {median * 100:>5.0f} cm  {within * 100:>5.0f}%")
        time.sleep(options.watch)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=MAP)
    parser.add_argument("--scan", default=SCAN)
    parser.add_argument("--topic", action="append", dest="topics", metavar="TOPIC",
                        help=f"Publish here instead. Repeatable. Default: {' '.join(OUTPUT)}")
    parser.add_argument("--near", nargs=2, type=float, metavar=("X", "Y"),
                        help="Search 3 m around this map position instead of the whole map.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the pose without publishing it.")
    parser.add_argument("--watch", type=float, nargs="?", const=2.0, metavar="SECONDS",
                        help="Do not search or publish. Instead score the live scan "
                             "against the map at the pose the stack believes, every "
                             "SECONDS (default 2), and print map->odom beside it.")
    parser.add_argument("--accept", type=float, default=ACCEPT_MEDIAN,
                        help="Largest median beam-to-wall distance to accept, in metres.")
    parser.add_argument("--beams", type=int, default=240)
    parser.add_argument("--max-range", type=float, default=10.0,
                        help="Ignore returns past this; far beams drift with small yaw errors.")
    options = parser.parse_args()

    field, free, resolution, origin = read_map(options.map)

    rclpy.init()
    node = Localizer(options.scan, options.topics or OUTPUT)
    scan = node.wait("scan", timeout=30.0)
    if scan is None:
        print(f"no {options.scan} in 30 s; is the robot up and on this ROS_DOMAIN_ID?")
        return 3
    offset_x, offset_y = node.laser_offset(scan.header.frame_id)

    ranges, angles = beams(scan, options.beams, options.max_range)
    print(f"{len(ranges)} beams from {scan.header.frame_id}")

    if options.watch:
        try:
            watch(node, options, field, resolution, origin, offset_x, offset_y)
        except KeyboardInterrupt:
            print("\nstopped.")
        return 0

    rows, cols = np.nonzero(free)
    positions = np.stack([origin[0] + (cols + 0.5) * resolution,
                          origin[1] + (rows + 0.5) * resolution], axis=1)
    clear = field[rows, cols] >= ROBOT_RADIUS
    positions = positions[clear]
    if options.near:
        guess = np.array(options.near)
        positions = positions[np.hypot(*(positions - guess).T) <= 3.0]
        spacing = 0.2
    else:
        spacing = 0.4
    # Thin to the coarse spacing; refine() recovers the resolution afterwards.
    keys = np.round(positions / spacing).astype(np.int64)
    positions = positions[np.unique(keys, axis=0, return_index=True)[1]]
    if not len(positions):
        print("no free space to search; wrong map, or --near is off the map")
        return 3
    print(f"searching {len(positions)} positions x 72 headings")

    started = time.time()
    coarse = search(ranges, angles, positions,
                    np.arange(-math.pi, math.pi, math.radians(5)),
                    field, resolution, origin)
    _, x, y, yaw = refine(coarse, ranges, angles, field, resolution, origin)
    median, within = quality(x, y, yaw, ranges, angles, field, resolution, origin)
    print(f"searched in {time.time() - started:.0f} s")

    base_x = x - offset_x * math.cos(yaw) + offset_y * math.sin(yaw)
    base_y = y - offset_x * math.sin(yaw) - offset_y * math.cos(yaw)
    print(f"{BASE} x={base_x:.3f} y={base_y:.3f} yaw={yaw:.4f} rad "
          f"({math.degrees(yaw):.1f} deg)")
    print(f"median beam-to-wall {median * 100:.0f} cm, "
          f"{within * 100:.0f}% of beams within 10 cm")

    # The gap between this match and the pose the stack is already acting on. A
    # large one with a good match means everything downstream -- the costmap,
    # every plan, every goal -- is being computed at the wrong place in the map.
    believed = node.believed_pose()
    if believed is None:
        print("nothing publishes map->base_footprint yet; nothing to compare against")
    else:
        drift = math.hypot(base_x - believed[0], base_y - believed[1])
        turn = abs(math.atan2(math.sin(yaw - believed[2]), math.cos(yaw - believed[2])))
        print(f"the stack currently believes ({believed[0]:.3f}, {believed[1]:.3f}), "
              f"{drift:.2f} m and {math.degrees(turn):.0f} deg from this match")
        if drift > 0.5 and median <= options.accept:
            print("  -> the running localizer is wrong; publish this and clear the "
                  "costmap afterwards, or every plan keeps using the old pose")

    if median > options.accept:
        print(f"rejected: worse than {options.accept * 100:.0f} cm. Nothing published. "
              "Give --near, or place the robot where the map has walls in view.")
        return 1
    if options.dry_run:
        print("--dry-run: nothing published.")
        return 0

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.pose.position.x = base_x
    pose.pose.pose.position.y = base_y
    pose.pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.pose.orientation.w = math.cos(yaw / 2)
    # AMCL reads this; laser_2d_localizer ignores it. Loose enough that a few
    # centimetres of matching error do not have to be argued with.
    pose.pose.covariance[0] = pose.pose.covariance[7] = 0.25
    pose.pose.covariance[35] = 0.068
    # Publishing the instant the publisher is created sends the pose into a
    # topic nobody has matched yet and it is silently lost -- the localizer goes
    # on reporting the map origin. Wait for the subscriber to show up first.
    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(publisher.get_subscription_count() for publisher in node.publishers_):
            break
    for publisher in node.publishers_:
        if not publisher.get_subscription_count():
            print(f"warning: nothing subscribes to {publisher.topic_name}")
        publisher.publish(pose)
    # Give the samples a moment to leave before the process exits.
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    print("published on " + ", ".join(options.topics or OUTPUT))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    finally:
        rclpy.try_shutdown()
