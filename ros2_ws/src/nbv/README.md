# nbv — next best view for the HSR-C

Where the robot should look, and stand, to see enough of a target to grasp it. Right now: an OctoMap of what the head camera has seen around the target, and how much of it is still unknown.

## Run it

In a sourced container terminal, with the sim (or robot) running:

| Command | Why |
| --- | --- |
| `ros2 run nbv build_octomap.py --ros-args -p sphere_centre:="[2.0, 1.0, 0.7]" -p sphere_radius:=0.6` | Build the map and report on a sphere around the target |
| `ros2 service call /nbv/unknown_fraction nbv/srv/UnknownFraction "{centre: {x: 2.0, y: 1.0, z: 0.7}, radius: 0.6}"` | Ask how much of a sphere is still unknown |
| `ros2 run nbv plot_octomap.py --ros-args -p path:=/home/ws/outputs/octomap.png` | Save a picture of the map (three views, coloured by occupancy) |
| `ros2 run nbv save_occupied.py --ros-args -p path:=/home/ws/outputs/octomap.ply` | Save the occupied cells as a `.ply` for Rerun or MeshLab |
| `python3 -m pytest -p no:anyio ros2_ws/src/nbv/test` | Test the map node and octomap-python (from `/home/ws`) |

**Parameters:**

| Parameter | Default | Meaning |
| --- | --- | --- |
| `sphere_centre` | `[0.0, 0.0, 0.8]` | Target centre, in `frame` |
| `sphere_radius` | `0.6` | Sphere radius, metres |
| `frame` | `map` | Frame of the map and sphere |

Other settings (5 cm cells, 4-pixel stride, 2.5 m range, depth topics) are constants at the top of [build_octomap.py](scripts/build_octomap.py).

**Talks on:**

| Name | Type | What |
| --- | --- | --- |
| `/nbv/occupied` | PointCloud2 | Occupied cell centres, for RViz |
| `/nbv/octomap` | Octomap | The whole map |
| `/nbv/sphere` | Marker | The target sphere |
| `/nbv/unknown_fraction` | Service | `fraction`, `unknown`, `total` for a given sphere |

Other processes query the service instead of copying the map.

---

## How it works

- **Input:** depth image + CameraInfo, deprojected in the node. A third of the DDS bytes of the `rectified_points` cloud, and the stride skips pixels before they're deprojected.
- **Downsampling:** pixel stride, then a range cut on the raw depth values (which also drops missing depth). OctoMap then updates each cell once per frame. About 15 ms per frame at 5 cm with 19k rays.
- **Every ray goes in,** not only those ending in the sphere. A ray crossing the sphere to a wall behind is what marks that stretch free. OctoMap's bounding-box mode would drop it.
- **Empty rays carve free space** out to `MAX_RANGE`. Otherwise the air in front of the camera stays unknown forever and keeps attracting views.

## Why these choices

| Choice | Why |
| --- | --- |
| OctoMap, not our own grid | A dense numpy grid needed 6.7 GB on 40 real frames (2.5 m, 1 cm); OctoMap needed 154 MB, and the grid hid a free-space bug |
| `octomap-python`, not `pyoctomap` | Matches C++ OctoMap cell for cell; pyoctomap 1.2.2 returns occupied cells half a cell off |
| No hand-written `octomap_msgs/Octomap` | `octomap_server` is installed and can take a cropped cloud |

**Design lineage:** structure from [ActPerMoMa](https://arxiv.org/abs/2310.00433), built for a mobile manipulator with a head camera: choose where the base goes, trading information gain against grasp reach. Mechanics from [active_grasp](https://github.com/ethz-asl/active_grasp) (Breyer et al., IROS 2022): an object-anchored volume, raycast from a candidate view, count the unknown cells it would uncover.

| active_grasp | here |
| --- | --- |
| TSDF volume (`UniformTSDFVolume(0.3, 40)`) | OctoMap via `octomap-python` |
| VGN grasp network | GraspGenX |
| — | SAM3 detects the target |
| Franka + wrist D435, ROS Noetic | HSR-C + head RGB-D, ROS Humble |

## Still to do

- [x] **Free space from empty rays.**
- [ ] **Information gain:** count the unknown cells (`getLabels == -1`) each candidate view would hit, stopping at the first surface. OctoMap's `castRay` stops at the first occupied cell.
- [ ] **Candidate views:** active_grasp samples a sphere (`thetas = [15°, 30°]`, `phis = 8 × 45°`) and keeps IK-reachable views. Ours must use the HSR's full body: head pan/tilt, arm lift/flex, torso.
- [ ] **Base placement:** the base is part of that chain, so candidates include base poses, scored by navigation cost and checked against the map. No active_grasp equivalent.
- [ ] **Graspability:** their utility is `gain/Σgain − cost/Σcost` with cost 1. Weight views that reveal graspable surface (via GraspGenX), not just unknown volume.
- [ ] **Sphere from the scene graph:** the target's centroid, radius from its box plus gripper clearance. Today it is a parameter.
