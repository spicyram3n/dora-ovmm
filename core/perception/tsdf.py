"""A TSDF of the space around a grasp target, in the layout VGN reads.

VGN (docker/vgn/) was trained on Open3D's UniformTSDFVolume: a 0.3 m cube of
40^3 voxels fused from depth images, read back as (tsdf + 1) / 2 in every voxel
Open3D reports near a surface and 0 everywhere else. This builds exactly that,
with the same Open3D calls, so the network sees what it was trained on.

The cube is axis-aligned in a fixed frame (map, odom), centred on the target in
x and y. The training scenes stand their objects on a table at z = 5 cm in the
grid, so its floor goes 5 cm below the surface the target stands on.
"""

import numpy as np
import open3d as o3d

LENGTH = 0.3
RESOLUTION = 40
VOXEL = LENGTH / RESOLUTION
# vgn's simulation: table_height = finger_depth = 0.05.
TABLE_HEIGHT = 0.05


def origin_for(centre, support_z=float("nan")):
    """The cube's corner, in the fixed frame, for a target at `centre` standing
    on a surface at height `support_z`. Without one, the cube is centred on the
    target, and VGN sees no table where it expects one."""
    bottom = support_z - TABLE_HEIGHT if np.isfinite(support_z) else centre[2] - LENGTH / 2
    return np.array([centre[0] - LENGTH / 2, centre[1] - LENGTH / 2, bottom])


class GraspVolume:
    def __init__(self, origin):
        self.origin = np.asarray(origin, dtype=np.float64)
        # Grid coordinates to the fixed frame: the cube does not rotate.
        self.frame_from_grid = np.eye(4)
        self.frame_from_grid[:3, 3] = self.origin
        self.frames = 0
        self._volume = o3d.pipelines.integration.UniformTSDFVolume(
            length=LENGTH,
            resolution=RESOLUTION,
            sdf_trunc=4 * VOXEL,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
        )

    def integrate(self, depth_m, k, frame_from_camera):
        """Fuse one depth image, in metres, seen from `frame_from_camera` (4, 4)
        with intrinsics `k` (3, 3). Missing depth may be 0, NaN or inf."""
        depth = np.nan_to_num(
            np.asarray(depth_m, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0
        )
        height, width = depth.shape
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.zeros_like(depth)),
            o3d.geometry.Image(depth),
            depth_scale=1.0,
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height, k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        )
        grid_from_camera = np.linalg.inv(self.frame_from_grid) @ frame_from_camera
        # Open3D takes the extrinsic the other way round: grid points into the camera.
        self._volume.integrate(rgbd, intrinsic, np.linalg.inv(grid_from_camera))
        self.frames += 1

    def voxels(self):
        """Voxels Open3D reports near a surface: (N, 3) centres in the fixed
        frame, and each one's (tsdf + 1) / 2 in (0, 1)."""
        cloud = self._volume.extract_voxel_point_cloud()
        return (
            np.asarray(cloud.points) + self.origin,
            np.asarray(cloud.colors)[:, 0].astype(np.float32),
        )

    def grid(self):
        """(40, 40, 40) float32, indexed [x, y, z]: VGN's input."""
        cloud = self._volume.extract_voxel_point_cloud()
        grid = np.zeros((RESOLUTION,) * 3, dtype=np.float32)
        points = np.asarray(cloud.points)
        if len(points):
            # Voxel centres, so the floor is exact: vgn.utils.map_cloud_to_grid.
            index = np.clip((points // VOXEL).astype(int), 0, RESOLUTION - 1)
            grid[tuple(index.T)] = np.asarray(cloud.colors)[:, 0]
        return grid

    def surface(self):
        """(N, 3) points on the fused surface, in the fixed frame."""
        return np.asarray(self._volume.extract_point_cloud().points) + self.origin
