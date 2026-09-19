"""Bag -> keyframes (color, depth, poses) in the SLAM map frame + a fused scene.ply.

Camera pose = map->odom (recorded from slam_toolbox by slam_replay.sh)
             @ odom->...->head_rgbd_sensor_rgb_frame (from the bag's own /tf).
That camera frame is optical (x right, y down, z forward): the bag's registered
cloud equals the pinhole unprojection of its depth image. So the poses follow
ScanNet's conventions, which Boxer's ScanNet loader reads (make_boxer_scene.py),
and the scene frame *is* the map frame (identity registration).
"""

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import rclpy.duration
import rclpy.time
import rosbag2_py
import tf2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation

RGB = "/head_rgbd_sensor/rgb/image_rect_color"
DEPTH = "/head_rgbd_sensor/depth_registered/image_rect_raw"
INFO = "/head_rgbd_sensor/rgb/camera_info"
CAMERA = "head_rgbd_sensor_rgb_frame"
# The moving joints between odom and the head camera; every other link is static.
CHAIN_PARENTS = {"odom", "base_link", "torso_lift_link", "head_pan_link"}


def messages(bag, topics):
    """Deserialized (topic, message) pairs of `topics`, in bag order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def load_tf(bag, slam_tf_bag, span):
    """A tf2 buffer with the bag's statics and head chain plus SLAM's map->odom.

    `span` must cover the whole bag: the buffer forgets older transforms.

    SLAM's map->odom corrects whichever odometry SLAM was fed, so the chain below
    must use that same one. slam_replay.sh notes it beside tf_mapping; a map built
    before the note existed was built on the bag's own odom->base_footprint."""
    from tf_filter import wheel_odometry
    note = Path(slam_tf_bag).parent / "odometry"
    wheel = note.exists() and note.read_text().strip() == "wheel"
    buffer = tf2_py.BufferCore(rclpy.duration.Duration(seconds=span))
    counts = {"static": 0, "chain": 0, "map->odom": 0}
    odometry = {}
    for topic, message in messages(bag, ["/tf", "/tf_static"]):
        for t in (wheel_odometry(message.transforms, odometry) if wheel and topic == "/tf" else message.transforms):
            if topic == "/tf_static":
                buffer.set_transform_static(t, "bag")
                counts["static"] += 1
            elif t.header.frame_id in CHAIN_PARENTS:
                buffer.set_transform(t, "bag")
                counts["chain"] += 1
    # The bag's own map->odom was dropped during SLAM; only SLAM's counts.
    seen, conflicts = {}, 0
    for _, message in messages(slam_tf_bag, ["/tf"]):
        for t in message.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                stamp = stamp_of(t.header)
                value = (t.transform.translation.x, t.transform.translation.y)
                conflicts += seen.setdefault(stamp, value) != value
                buffer.set_transform(t, "slam")
                counts["map->odom"] += 1
    # One node may restamp within a clock tick; many conflicts mean two nodes were alive.
    if conflicts > 0.01 * len(seen):
        raise ValueError(f"{conflicts} conflicting map->odom stamps: rerun slam_replay.sh")
    return buffer, counts


def matrix(transform):
    """geometry_msgs Transform -> 4x4."""
    t, q = transform.translation, transform.rotation
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    pose[:3, 3] = [t.x, t.y, t.z]
    return pose


def moved_enough(previous, current, translation, degrees):
    """Keyframe gate: skip frames taken from (almost) the same viewpoint."""
    if previous is None:
        return True
    delta = np.linalg.norm(current[:3, 3] - previous[:3, 3])
    angle = Rotation.from_matrix(previous[:3, :3].T @ current[:3, :3]).magnitude()
    return delta >= translation or np.degrees(angle) >= degrees


def pairs(bag, tolerance):
    """(rgb, depth) messages whose stamps agree within `tolerance` seconds.

    The two streams interleave by arrival, so an RGB frame waits until a depth
    frame stamped later than itself has arrived; only then is the nearest picked."""
    depths, pending = [], []

    def flush(final=False):
        while pending:
            stamp = stamp_of(pending[0].header)
            if not final and stamp_of(depths[-1].header) < stamp + tolerance:
                return  # a closer depth frame may still come
            rgb = pending.pop(0)
            depth = min(depths, key=lambda d: abs(stamp_of(d.header) - stamp))
            if abs(stamp_of(depth.header) - stamp) <= tolerance:
                yield rgb, depth

    for topic, message in messages(bag, [RGB, DEPTH]):
        if topic == DEPTH:
            depths.append(message)
            depths = depths[-10:]
            yield from flush()
        else:
            pending.append(message)
    if depths:
        yield from flush(final=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default="bags/playable_bag")
    parser.add_argument(
        "--slam-tf",
        type=Path,
        required=True,
        help="bag recorded by slam_replay.sh: .../slam/tf_mapping",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--registration",
        type=Path,
        help="also write the scene->map registration JSON here",
    )
    parser.add_argument(
        "--min-translation", type=float, default=0.10, help="m between keyframes"
    )
    parser.add_argument(
        "--min-rotation", type=float, default=5.0, help="deg between keyframes"
    )
    parser.add_argument(
        "--pair-tolerance",
        type=float,
        default=0.035,
        help="max rgb/depth stamp gap, s (one 30 Hz period)",
    )
    parser.add_argument(
        "--max-depth", type=float, default=4.0, help="m fused into the TSDF"
    )
    parser.add_argument(
        "--voxel", type=float, default=0.02, help="TSDF voxel = scene.ply spacing, m"
    )
    args = parser.parse_args()
    out = args.output
    if out.exists():
        parser.error(f"Output already exists: {out}; choose a new --output")

    started = time.time()
    buffer, tf_counts = load_tf(args.bag, args.slam_tf, span=800.0)  # bag is 792 s
    print(f"tf loaded in {time.time() - started:.0f}s: {tf_counts}")

    info = next(m for _, m in messages(args.bag, [INFO]))
    k = np.array(info.p).reshape(3, 4)[:, :3]  # images are rectified: P, not K
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        info.width, info.height, k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    )

    for folder in ("color", "depth", "poses"):
        (out / folder).mkdir(parents=True)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel,
        sdf_trunc=4 * args.voxel,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )
    counts = dict(pairs=0, no_tf=0, keyframes=0)
    gaps, rows, previous = [], [], None
    for rgb, depth in pairs(args.bag, args.pair_tolerance):
        counts["pairs"] += 1
        gaps.append(abs(stamp_of(rgb.header) - stamp_of(depth.header)))
        stamp = stamp_of(depth.header)  # the depth image carries the geometry
        try:
            transform = buffer.lookup_transform_core(
                "map", CAMERA, rclpy.time.Time(seconds=stamp)
            ).transform
        except (tf2_py.ExtrapolationException, tf2_py.LookupException):
            counts["no_tf"] += 1  # before SLAM's first map->odom
            continue
        pose = matrix(transform)
        if not moved_enough(previous, pose, args.min_translation, args.min_rotation):
            continue
        previous = pose
        i = counts["keyframes"]
        counts["keyframes"] += 1

        color = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
        depth_mm = np.frombuffer(depth.data, np.uint16).reshape(
            depth.height, depth.width
        )
        cv2.imwrite(
            str(out / "color" / f"{i}.jpg"),
            cv2.cvtColor(color, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 95],
        )
        cv2.imwrite(str(out / "depth" / f"{i}.png"), depth_mm)  # lossless, mm
        np.savetxt(out / "poses" / f"{i}.txt", pose)  # camera -> map
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(color)),
            o3d.geometry.Image(np.ascontiguousarray(depth_mm)),
            depth_scale=1000.0,
            depth_trunc=args.max_depth,
            convert_rgb_to_intensity=False,
        )
        volume.integrate(
            rgbd, intrinsic, np.linalg.inv(pose)
        )  # Open3D wants map -> camera
        rows.append([i, f"{stamp:.6f}", *np.round(pose[:3, 3], 4)])
    print(f"frames done in {time.time() - started:.0f}s: {counts}")

    cloud = volume.extract_point_cloud().voxel_down_sample(args.voxel)
    o3d.io.write_point_cloud(str(out / "scene.ply"), cloud)
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out / "scene_mesh.ply"), mesh)
    intrinsic4 = np.eye(4)
    intrinsic4[:3, :3] = k
    np.savetxt(out / "intrinsics.txt", intrinsic4)
    with open(out / "keyframes.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "stamp", "cam_x", "cam_y", "cam_z"])
        writer.writerows(rows)
    stats = dict(
        bag=str(args.bag),
        slam_tf=str(args.slam_tf),
        source_frame="map",
        depth_scale=1000,
        tf=tf_counts,
        **counts,
        pair_gap_ms=dict(
            median=round(float(np.median(gaps)) * 1e3, 1),
            max=round(float(np.max(gaps)) * 1e3, 1),
        ),
        keyframe_gate=dict(
            translation_m=args.min_translation, rotation_deg=args.min_rotation
        ),
        tsdf=dict(voxel_m=args.voxel, max_depth_m=args.max_depth),
        scene_points=len(cloud.points),
        mesh_vertices=len(mesh.vertices),
        scene_extent_m=np.round(
            cloud.get_axis_aligned_bounding_box().get_extent(), 2
        ).tolist(),
        seconds=round(time.time() - started),
    )
    (out / "stats.json").write_text(json.dumps(stats, indent=2))
    if args.registration:
        # Poses are already in map coordinates, so scene -> map is the identity.
        args.registration.write_text(
            json.dumps(
                dict(source_frame="map", map_from_source=np.eye(4).tolist()), indent=2
            )
        )
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
