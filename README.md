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

The PC uses CycloneDDS in domain `9`; the robot uses domain `5`. Run the
Zenoh ROS 2 bridge on both hosts and the RGB-D compressor/receiver in their ROS
containers. See [robot–PC setup](robot_transport/README.md) for commands.

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
