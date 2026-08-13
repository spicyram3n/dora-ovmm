#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import yaml
from pathlib import Path

# -----------------------------
# Parameters
# -----------------------------
INPUT = "scene.ply"
OUTPUT = "ipad_map"

RESOLUTION = 0.05  # metres/pixel
HEIGHT = 0.20  # metres above floor
BAND = 0.05  # +/- metres


# -----------------------------
# Load point cloud
# -----------------------------
pcd = o3d.io.read_point_cloud(INPUT)
points = np.asarray(pcd.points)


# -----------------------------
# Find floor
# -----------------------------
plane, inliers = pcd.segment_plane(
    distance_threshold=0.02, ransac_n=3, num_iterations=1000
)

normal = np.array(plane[:3])
normal /= np.linalg.norm(normal)

if abs(normal[2]) < 0.9:
    raise RuntimeError("Detected plane is not horizontal.")

floor_z = points[inliers, 2].mean()

print("Floor height:", floor_z)


# -----------------------------
# Voxelize point cloud
# -----------------------------
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=RESOLUTION)

voxels = np.array([voxel.grid_index for voxel in voxel_grid.get_voxels()])

origin = np.asarray(voxel_grid.origin)


# -----------------------------
# Select height band
# -----------------------------
voxel_z = origin[2] + (voxels[:, 2] + 0.5) * RESOLUTION
height = voxel_z - floor_z

mask = (height >= HEIGHT - BAND) & (height <= HEIGHT + BAND)

voxels = voxels[mask]

if len(voxels) == 0:
    raise RuntimeError("No voxels found at this height.")


# -----------------------------
# Project 3D → 2D
# -----------------------------
xy = voxels[:, :2]

# Shift map so minimum coordinate is (0,0)
xy -= xy.min(axis=0)

x = xy[:, 0]
y = xy[:, 1]


# -----------------------------
# Create occupancy image
# -----------------------------
width = int(x.max()) + 1
height = int(y.max()) + 1

grid = np.full((height, width), 254, dtype=np.uint8)  # free

grid[y, x] = 0  # occupied


# -----------------------------
# Save PGM
# -----------------------------
pgm_file = Path(OUTPUT + ".pgm")

with open(pgm_file, "wb") as f:
    f.write(f"P5\n{width} {height}\n255\n".encode())
    f.write(grid.tobytes())


# -----------------------------
# Save YAML
# -----------------------------
yaml_file = Path(OUTPUT + ".yaml")

with open(yaml_file, "w") as f:
    yaml.safe_dump(
        {
            "image": pgm_file.name,
            "mode": "trinary",
            "resolution": RESOLUTION,
            "origin": [float(origin[0]), float(origin[1]), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        },
        f,
        sort_keys=False,
    )


print(f"Saved: {pgm_file}")
print(f"Saved: {yaml_file}")
