"""A TSDF of a cube around the grasp target: vgn/perception.py in numpy frames.

active_grasp fuses depth into a fixed cube (0.3 m, 40^3 voxels for VGN) that
sits in a `task` frame placed from the target's bounding box: centred on the
box in x and y, its floor 5 cm below the box's bottom, axes parallel to the
base frame (Policy.calibrate_task_frame). Every pose here is a (4, 4) matrix
named `a_from_b`; `camera_from_task` is what Open3D calls the extrinsic.
"""

import numpy as np
import open3d as o3d


class UniformTSDFVolume:
    def __init__(self, length, resolution):
        self.length = length
        self.resolution = resolution
        # Divide the cube side length by the number of cells along one axis.
        self.voxel_size = self.length / self.resolution
        self.sdf_trunc = 4 * self.voxel_size
        self.o3dvol = o3d.pipelines.integration.UniformTSDFVolume(
            length=self.length,
            resolution=self.resolution,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
        )

    def integrate(self, depth_img, intrinsic, extrinsic):
        """Fuse metre-valued depth using camera intrinsics and a camera_from_task transform."""
        depth_img = np.ascontiguousarray(depth_img, dtype=np.float32)
        height, width = depth_img.shape
        # Package metre-valued depth for Open3D; this volume does not use colour.
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.empty_like(depth_img)),
            o3d.geometry.Image(depth_img),
            depth_scale=1.0,
            convert_rgb_to_intensity=False,
        )
        # Build Open3D's camera model from image size, focal lengths, and image centre.
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height, intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]
        )
        self.o3dvol.integrate(rgbd, intrinsic, np.asarray(extrinsic, dtype=np.float64))

    def get_scene_cloud(self):
        """Points on the fused surface, in the task frame."""
        return self.o3dvol.extract_point_cloud()

    def get_map_cloud(self):
        """Voxel centres near a surface; colour channel 0 holds (tsdf + 1) / 2."""
        return self.o3dvol.extract_voxel_point_cloud()

    def get_grid(self):
        map_cloud = self.get_map_cloud()
        points = np.asarray(map_cloud.points)
        # Read normalized signed distances from the voxel cloud's first colour channel.
        distances = np.asarray(map_cloud.colors)[:, [0]]
        return map_cloud_to_grid(self.voxel_size, self.resolution, points, distances)


def map_cloud_to_grid(voxel_size, resolution, points, distances):
    """Place the stored voxel distances into a dense 3D array."""
    # Start with zero for every cell, including cells with no stored surface data.
    grid = np.zeros((resolution,) * 3, dtype=np.float32)
    # Convert voxel-centre coordinates into integer grid indices.
    indices = (points // voxel_size).astype(int)
    grid[tuple(indices.T)] = distances.squeeze()
    return grid


def base_from_task(bbox, length):
    """Return the cube origin as a 4x4 transform into the base frame."""
    # Centre the cube in XY and place its floor 5 cm below the target.
    xyz = np.r_[bbox.center[:2] - length / 2, bbox.min[2] - 0.05]
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    return matrix

