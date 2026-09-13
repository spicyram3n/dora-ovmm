# HSR Grasping Pipeline

Prompt driven grasping on the HSR robot. SAM3 finds the object in an image,
GraspGenX generates grasp poses for it, and MoveIt executes the grasp.

The upright-can executor now includes bounded base repositioning, fresh grasp
generation after repositioning, camera-view recovery, and planar tracking
correction before feedback closure. A single-command kitchen pickup with
automatic base and tracking recovery passed depth hold verification on
2026-09-10. See [validation and scope](GRASPING_ARCHITECTURE_2026-09-10.md).
This covers local upright-can pickup; arbitrary-object execution and room-scale
search/navigation are not yet integrated.

## 1. Open the dev container

Open this folder in VS Code and reopen it in the container
(`.devcontainer/`). First run clones the ROS 2 packages, builds the
workspace, and installs cyclonedds and zenoh.

## 2. Launch ROS 2

**Simulation:**

```bash
ros2 launch hsrb_gazebo_launch hsrc_apartment_world.launch.py
```

For grasp testing beside the table, use `bash ros2_ws/launch/run_grasp_sim.sh`
instead. In another sourced container terminal, start MoveIt:

```bash
ros2 launch /home/ws/ros2_ws/launch/hsrc_move_group_sim.launch.py
```

MoveIt's octomap now consumes the RGB-D depth image and its matching
`CameraInfo` directly, using TF to place observations in `odom`. It does not
subscribe to a `PointCloud2` topic. The simulation launch selects
`/head_rgbd_sensor/depth_registered/image`; the default real-camera topic is
`/head_rgbd_sensor/depth_registered/image_rect_raw`. RGB still supplies the
object segmentation; depth and calibration supply obstacle geometry.

**Real robot:**

The container's RMW is `rmw_cyclonedds_cpp` by default, matching the robot.
Before connecting, make sure:

- The robot and this machine are on the same private network, not a shared
  public/campus Wi-Fi. Most of those isolate clients from each other, which
  blocks ROS 2 discovery entirely. Use your own router or a hotspot instead.
- `ROS_DOMAIN_ID` matches on both sides (`42` here, see
  `.devcontainer/devcontainer.json`).

No bridge or extra software is needed on the robot itself.

## 3. Start the perception and grasp servers

Run these on the host machine, not inside the dev container. Each builds its
own Docker image on first run.

```bash
bash docker/sam3/run_sam3.sh
bash docker/graspgenx/run_graspgenx.sh
```

GraspGenX needs the HSRC gripper registered once before its first real run.
See `docker/graspgenx/README.md` for that one time setup.

## 4. Run the pipeline

With ROS 2 launched and both servers running:

```bash
python3 core/run_pipeline.py "pringles can" "red cup" --gripper hsrc_hand
```

In this simulation, prefix that command with
`DEPTH_TOPIC=/head_rgbd_sensor/depth_registered/image`.

This grabs one camera frame, finds each prompt in it, and saves grasps under
`config/targets/<slug>/`.

For the bounded upright-can experiment, the same entry point can execute the
whole pickup. Start with an empty gripper, a fresh planning scene, and the can
visible in registered RGB-D:

```bash
python3 -u core/run_pipeline.py "pringles can" \
  --output-dir config/diagnostics/my_shelf_trial/targets \
  --execute --lift .02 --retreat .20
```

This generates fresh grasps, checks the approach, re-detects the can to correct
up to 1 cm of residual pad-centering error, closes gradually using both finger
spring signals, checks a short lift and withdrawal, and verifies the height
change in three fresh depth observations. Failure returns a nonzero status;
an old saved grasp cannot substitute for failed detection. Logs print SAM3,
GraspGenX, and total run times. `pickup/result.json` records verification.

The withdrawal is a bounded translation along **odom -X**, intended for a
robot initially facing this shelf. It is not navigation. Omit `--retreat` for
lift-only tests. A shelf lip can obscure the can's top until it is withdrawn;
occlusion causes verification to fail, not a success claim. The lift permits
only the documented shallow initial target/support contact; all other lift
contacts and all withdrawal contacts are rejected. Target exclusion cleanup,
placing the object, arbitrary shapes, and continuous tracking remain outside
this experiment. The robot finishes holding the can.

SAM3 and GraspGenX remain loaded in their servers. This is fresh perception
per pickup plus a feedback observation at contact, not continuous video-rate
grasp replanning. See `SHELF_PICKUP_2026-09-10.md` for measured results and setup.

## 5. Execute a grasp

```bash
ros2 launch grasp_execution move_to_grasp.launch.py
```

Uses the newest saved grasp file by default. Pass `grasp_file:=<path>` to
pick a specific one.

Saved grasps remain in the canonical GraspGenX frame. The execution launch
right-multiplies each pose by the registered `hsrc_hand` `base_rotation`
before targeting `hand_palm_link`: the URDF closing Y axis aligns with
GraspGenX X, while approach Z stays unchanged. Do not pre-rotate saved YAML
files, or the correction will be applied twice.

Execution requires a populated octomap, opens the gripper with a checked
plan, and requests collision-aware pregrasp IK. The final approach must be
complete, collision-free, and correctly timed. No hand or arm link is
allowed to pass through the octomap. This also means a contact pose that
overlaps the target's occupied voxels is rejected; target-specific contact
modeling is still needed for those grasps, rather than disabling furniture
collision checks.

Read-only verification inside the sourced container:

```bash
python3 -m unittest discover -s ros2_ws/src/grasp_execution/test -v
python3 ros2_ws/src/grasp_execution/test/check_rgbd_scene.py
```

The live check expects the near-table simulation. It checks image-only
map subscriptions and compares checked and unchecked IK at a table probe;
it commands no motion.

## About zenoh

SAM3 and GraspGenX are reached over zenoh instead of ROS 2 topics, since
they run in their own Docker containers with their own CUDA and Python
versions, kept separate from the ROS 2 workspace on purpose.

**No router needed.** Each server opens a plain zenoh session and starts
listening as soon as it starts up, no separate router process required. The
client side (`core/utils/zenoh_rpc.py`) does the same. Discovery between
them happens automatically over local network scouting, since every
container runs with `--net=host` and shares the machine's network directly.

This is independent of whichever ROS 2 RMW is active. Switching between
`rmw_cyclonedds_cpp` and `rmw_zenoh_cpp` for ROS 2 topics has no effect on
the SAM3/GraspGenX zenoh connection, since that connection never goes
through the RMW at all.

Note that scouting relies on multicast reaching all containers, which holds
on one machine but is not guaranteed across separate hosts on a locked down
network. This matters once the ROS 2 side runs on the robot and SAM3/
GraspGenX run on your PC.

**Cross host setup.** Each server listens on a fixed port instead of an
ephemeral one, so it can be reached directly without scouting:

- SAM3: `tcp/0.0.0.0:7447` (override with the `ZENOH_LISTEN` env var)
- GraspGenX: `tcp/0.0.0.0:7448` (same)

On the client side (`core/utils/zenoh_rpc.py`, running wherever the ROS 2
workspace runs), set `ZENOH_CONNECT` to a comma separated list of those
endpoints using the PC's real address, for example:

```bash
export ZENOH_CONNECT="tcp/192.168.1.50:7447,tcp/192.168.1.50:7448"
```

With this set, the client connects to both servers directly and does not
depend on scouting at all. Leave it unset for local, same-machine use, it
falls back to scouting as before.
