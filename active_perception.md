# Active perception — next best view

Decide where the robot should look, and stand, to see enough of a target object
to grasp it.

The map is the first piece: an occupancy grid of **one sphere** around the
target. Information gain, view sampling, and base placement come next and are
not built yet.

Package: [`ros2_ws/src/nbv`](ros2_ws/src/nbv/README.md). Main pipeline:
[README.md](README.md).

## What we are following

| Piece | Taken from | Why |
| --- | --- | --- |
| Structure | [ActPerMoMa](https://arxiv.org/abs/2310.00433) | Mobile base + head camera, same as the HSR. The decision is *where the base goes*, and utility trades information gain against grasp reachability. |
| Formulation | [active_grasp](https://github.com/ethz-asl/active_grasp) | Object-anchored volume, raycast forward from a candidate view, count unknown cells. Simpler, and enough. |
| Map | ours | Occupancy, not TSDF. GraspGenX eats points, not a TSDF grid, and occupancy gives `unknown` for free. |

Both papers anchor the volume to **the target object, never the robot**.
active_grasp uses a 0.3 m cube on the target's bounding box; ActPerMoMa counts
gain only inside the target's bbox. Ours is a sphere on the target.

## 1. Prerequisites — once per boot

CycloneDDS asks for 32 MB socket buffers. Without these every ROS node dies at
startup with `rmw handle is invalid`.

```bash
sudo sysctl -w net.core.rmem_max=33554432 net.core.wmem_max=33554432
```

To persist:

```bash
echo -e "net.core.rmem_max=33554432\nnet.core.wmem_max=33554432" | sudo tee /etc/sysctl.d/10-cyclonedds.conf
```

## 2. Start the simulation — container terminal 1

Full pipeline not needed. Gazebo alone is enough.

```bash
cd /home/ws && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

**To park the robot at a specific object**, edit the spawn pose — it is
hardcoded and cannot be passed on the command line:

```bash
sed -i \
 -e 's|"robot_pos_x": "5.0"|"robot_pos_x": "5.18"|' \
 -e 's|"robot_pos_y": "6.6"|"robot_pos_y": "3.89"|' \
 -e 's|"robot_rpy_Y": "0.0"|"robot_rpy_Y": "3.14159"|' \
 ros2_ws/src/hsrb_launch/hsrb_gazebo_launch/launch/hsrb_apartment_world.launch.py
```

That puts the robot 0.51 m east of the kitchen pringles can, facing it. **Revert
before running the full pipeline** — `config/map/world_to_map.json` assumes the
5.0 / 6.6 spawn, and the file is not tracked by git.

## 3. Build the map — container terminal 2

```bash
cd /home/ws && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run nbv build_octomap.py --ros-args \
  -p use_sim_time:=true -p frame:=base_footprint \
  -p sphere_centre:="[0.51, 0.0, 1.16]" -p sphere_radius:=0.5
```

That maps a 0.5 m sphere 0.51 m in front of the robot — the pringles can.

Watch `occupied N  unknown M`. `N` climbing means it is working. Leave it
running; everything below reads from it. **Ctrl+C** to stop.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `sphere_centre` | `[0.0, 0.0, 0.8]` | Sphere centre, in `frame` |
| `sphere_radius` | `0.6` | Sphere radius, m |
| `frame` | `map` | Fixed frame the map lives in |
| `use_sim_time` | `false` | `true` for Gazebo, which drives `/clock` |

Pass `frame:=base_footprint` because `odom` is missing when the sim runs
without navigation. The base does not move here, so it is an equally valid
fixed frame. Use `map` once Nav2 runs.

## 4. See it — container terminal 3

```bash
cd /home/ws && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
rviz2 -d /home/ws/config/rviz/nbv.rviz
```

Preloaded: occupied cells as boxes coloured by probability, the target sphere,
the robot. Fixed frame `base_footprint`.

Click a cell with the **Select** tool to read its occupancy probability.

## 5. Save a figure and a cloud — container terminal 4

With the mapper still running, and `occupied` no longer climbing:

```bash
cd /home/ws && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run nbv plot_octomap.py --ros-args -p path:=/home/ws/outputs/octomap.png
ros2 run nbv save_occupied.py --ros-args -p path:=/home/ws/outputs/octomap.ply
```

`octomap.png` is three views, cells coloured by probability, sphere outlined.
`octomap.ply` opens in rerun or meshlab. Run either again at any time.

## What the cells mean

| State | Meaning | Published? |
| --- | --- | --- |
| **occupied** | A depth ray ended here. Surface. | Yes, `/nbv/occupied` |
| **free** | A ray passed through. Confirmed empty. | No |
| **unknown** | No ray ever touched it. No information. | No |

Unknown is **not** free. It is "never looked at" — occluded, outside the field
of view, or never pointed at.

This is the whole point: a candidate view scores by how many **unknown** cells
it would turn into known ones. Free and occupied cells are worth nothing to
look at again.

## Topics

| Topic | Type | Contents |
| --- | --- | --- |
| `/nbv/occupied` | `PointCloud2` | Occupied cell centres. `intensity` = occupancy probability |
| `/nbv/sphere` | `Marker` | The region being mapped |

## Tuning

Constants at the top of
[`scripts/build_octomap.py`](ros2_ws/src/nbv/scripts/build_octomap.py):

| Constant | Default | Notes |
| --- | --- | --- |
| `RESOLUTION` | `0.05` | 5 cm is coarse for a 9 cm can. Use `0.02` or `0.01` for grasping |
| `STRIDE` | `4` | Keep every 4th pixel each way |
| `MAX_RANGE` | `2.5` | Depth past this is dropped |
| `DEPTH`, `INFO` | head RGBD | The simulator and the real robot name the depth topic differently |

Cost per frame, 0.5 m sphere:

| Resolution | Time |
| --- | --- |
| 5 cm | 14 ms |
| 2 cm | 23 ms |
| 1 cm | 93 ms |

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| Every node dies, `rmw handle is invalid` | Socket buffers, or a dead network interface in `.devcontainer/cyclonedds_sim.xml` | Step 1. The sim profile uses `lo` |
| `incompatible QoS ... RELIABILITY` | Subscription does not match the camera | Already fixed; rebuild the package |
| `"odom" ... does not exist` | No navigation running, so nothing publishes `odom` | Pass `-p frame:=base_footprint` |
| `occupied` stays 0 | Head not aimed, or camera silent | `ros2 topic hz /head_rgbd_sensor/depth_registered/image` |
| `no transform ... extrapolation` | tf arrives just after the image | Harmless if `occupied` is climbing |
| Goal accepted, never finishes | `/clock` not advancing | Unpause Gazebo |

## Status

Done:

- Occupancy map of a sphere around the target, from the depth image.
- Free space carved from rays that returned nothing.
- Per-cell probability published, RViz config, PNG and PLY export.

Next, in order:

1. **Information gain** — raycast forward from a candidate view, count the
   unknown cells it would uncover. Same raycast as `integrate`, counting
   instead of updating.
2. **Candidate views** — through the HSR's full-body kinematics: head pan and
   tilt, arm lift and flex, torso, all at once.
3. **Base placement** — the base is part of that chain. Score against
   navigation cost, check collisions against the map.
4. **Graspability** — weight views that would reveal graspable surface, via
   GraspGenX, not unknown volume alone.
5. **Sphere from the scene graph** — the target's centroid and bounding box,
   instead of typing coordinates.

## Files

| Path | Purpose |
| --- | --- |
| [`ros2_ws/src/nbv/nbv/occupancy.py`](ros2_ws/src/nbv/nbv/occupancy.py) | The grid. Pure numpy, no ROS, testable without a robot |
| [`ros2_ws/src/nbv/scripts/build_octomap.py`](ros2_ws/src/nbv/scripts/build_octomap.py) | ROS node: depth image → map |
| [`ros2_ws/src/nbv/scripts/plot_octomap.py`](ros2_ws/src/nbv/scripts/plot_octomap.py) | PNG figure |
| [`ros2_ws/src/nbv/scripts/save_occupied.py`](ros2_ws/src/nbv/scripts/save_occupied.py) | PLY export |
| [`config/rviz/nbv.rviz`](config/rviz/nbv.rviz) | RViz layout |
| [`bugs/`](bugs/) | Upstream bugs found on the way |
