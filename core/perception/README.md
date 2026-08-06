# perception

Getting from the robot's cameras to a point cloud of one named object.

This is the only folder that touches ROS 2, and only in `camera_ros2.py`.
Everything else here is numpy, so swapping middleware means changing one file.

## Files

| File | Purpose |
|---|---|
| `camera_ros2.py` | Grabs one RGB frame, depth frame, intrinsics, and the camera-to-base transform |
| `sam3_client.py` | Segments a text prompt in an image, over zenoh, and returns the best mask |
| `pointcloud.py` | Depth plus mask plus intrinsics to 3D points, and transforms between frames |

## Running

Nothing here is a program; `core/run_pipeline.py` is the entry point.

```python
from perception import pointcloud, sam3_client
from perception.camera_ros2 import grab_rgbd

rgb, depth_m, k, base_from_camera = grab_rgbd()
mask, score = sam3_client.detect(rgb, "pringles can")
points = pointcloud.deproject(depth_m, k, mask)
points = pointcloud.transform_points(base_from_camera, points)
```

`grab_rgbd` blocks until one message has arrived on each camera topic and the
transform is available, so a sourced ROS 2 environment and a running robot or
simulator are both required.

## Things worth knowing

- **Depth units differ between real and simulated robots.** The real driver
  publishes 16UC1 millimetres, Gazebo publishes 32FC1 metres.
  `camera_ros2.py` normalises both to metres.
- **`BASE_FRAME` is `odom`**, matching what the scene graph and Nav2 use.
- **SAM3 runs in its own Docker container** and is reached over zenoh, not ROS
  topics, so it can keep its own CUDA and Python versions. Start it with
  `bash docker/sam3/run_sam3.sh` on the host, not in the dev container.
