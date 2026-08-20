# HSR Grasping Pipeline

Prompt driven grasping on the HSR robot. SAM3 finds the object in an image,
GraspGenX generates grasp poses for it, and MoveIt executes the grasp.

## 1. Open the dev container

Open this folder in VS Code and reopen it in the container
(`.devcontainer/`). First run clones the ROS 2 packages, builds the
workspace, and installs cyclonedds and zenoh.

## 2. Launch ROS 2

**Simulation:**

```bash
ros2 launch hsrb_gazebo_launch hsrc_apartment_world.launch.py
```

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

This grabs one camera frame, finds each prompt in it, and saves grasps under
`config/targets/<slug>/`.

## 5. Execute a grasp

```bash
ros2 launch grasp_execution move_to_grasp.launch.py
```

Uses the newest saved grasp file by default. Pass `grasp_file:=<path>` to
pick a specific one.

## 6. Robot communication setup (one time)

Only needed when running against the real robot over the lab network. Full
detail and troubleshooting is in [zenoh_routing.md](zenoh_routing.md).

**a. Route.** The robot loses its path back to this subnet because a
teammate's USB ethernet adapter installs a lower metric default route. Symptom
is ping to `10.7.3.185` failing while the gateway `10.7.3.254` answers. On the
robot:

```bash
sudo nmcli connection modify "Internet" +ipv4.routes "131.220.7.0/24 10.7.3.254"
sudo nmcli connection up "Internet"
```

**b. Clock.** zenoh drops samples whose timestamp is more than 500 ms off.
Check `date -u` on both machines and sync with `sudo timedatectl set-ntp true`
on the robot if they disagree.

**c. Robot side config.** Copy both files from `.devcontainer/zenoh/`:

```bash
scp .devcontainer/zenoh/bridge_robot.json5 \
    .devcontainer/zenoh/cdds_bridge_robot.xml \
    administrator@10.7.3.185:/tmp/
ssh administrator@10.7.3.185
sudo mkdir -p /etc/zenoh-bridge-ros2dds
sudo cp /tmp/bridge_robot.json5 /etc/zenoh-bridge-ros2dds/
sudo cp /tmp/cdds_bridge_robot.xml /etc/zenoh-bridge-ros2dds/cdds_bridge.xml
```

The robot needs its own minimal CycloneDDS profile. Its full profile crashes
the bridge with a glibc buffer overflow, and running with no profile leaves
the bridge unable to discover any ROS node.

**d. Start.** Robot first, it is the listener:

```bash
# robot
export CYCLONEDDS_URI=file:///etc/zenoh-bridge-ros2dds/cdds_bridge.xml
zenoh-bridge-ros2dds -c /etc/zenoh-bridge-ros2dds/bridge_robot.json5
```

```bash
# this container
export ROS_DOMAIN_ID=1
zenoh-bridge-ros2dds -c /home/ws/.devcontainer/zenoh/bridge_pc.json5
```

Both are foreground processes and must stay running. `CYCLONEDDS_URI` is set
for you by `devcontainer.json`.

**e. Pointcloud.** Only compressed images cross the link, so rebuild the cloud
locally:

```bash
export ROS_DOMAIN_ID=1
ros2 launch /home/ws/zenoh/pointcloud_reconstruct.launch.py
```

This publishes `/head_rgbd_sensor/depth_registered/rectified_points`, the same
topic name the robot uses. In rviz set Fixed Frame to `odom`.

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

On the client side (robot side) (`core/utils/zenoh_rpc.py`, running wherever the ROS 2
workspace runs), set `ZENOH_CONNECT` to a comma separated list of those
endpoints using the PC's real address, for example:

```bash
export ZENOH_CONNECT="tcp/192.168.1.50:7447,tcp/192.168.1.50:7448"
```

With this set, the client connects to both servers directly and does not
depend on scouting at all. Leave it unset for local, same-machine use, it
falls back to scouting as before.
