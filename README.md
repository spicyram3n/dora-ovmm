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

## Gazebo open-vocabulary pipeline

Ask for an object by name. The scene graph says where it is, or an LLM guesses,
and the robot drives there.

**1. Launch the simulator and navigation**, in two terminals:

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=True use_navigation:=False robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  initial_orientation_xyzw:=[0,0,0,1] use_sim_time:=True
```

**2. Build the scene graph.** Once, and offline: it reads `apartment.world`
directly, so Gazebo does not have to be running.

```bash
cd core
python3 build_scene_graph.py --no-rooms   # drop --no-rooms to add LLM room names
```

Writes `outputs/scene_graph/apartment/graph.json`.

**3. Search for an object:**

```bash
python3 search_object.py hsr_pringles              # already in the graph, no LLM call
python3 search_object.py pringles_can              # not in the graph, asks the LLM
python3 search_object.py --furniture high_table01  # skip the lookup, just drive
python3 search_object.py hsr_pringles --dry-run    # print the plan, touch no robot
```

For each of the top-3 candidate locations in turn, the robot parks in front of
the furniture, runs SAM3 on the head camera, and moves on if the object is not
there. Once it is found, a second drive closes in until the object is inside
the arm's reach ring -- the standoff ring never is. Grasp execution from there
is not wired in yet (`grasp_execution` was removed in ff59dd1).

Set `DEEPSEEK_API_KEY` for anything that reaches the LLM. Its guesses are
cached per object under `outputs/scene_graph/apartment/locations/`, so the same
search is not paid for twice; delete a file to force a fresh answer.

Scene graph coordinates are in the robot's `odom` frame, not Gazebo's world
frame. See `core/README.md` for the folder layout and
`core/navigation/README.md` for how the standoff pose is picked.

## Grasping

### `arm` MoveIt group — kinematic chain

The `arm` planning group is declared as a chain from `base_link` to
`hand_palm_link`:

```
ros2_ws/src/hsrb_moveit/hsrb_moveit_config/config/hsrb.srdf:13
<chain base_link="base_link" tip_link="hand_palm_link" />
```

Resolving that chain through the URDF gives 5 actuated joints (1 prismatic +
4 revolute) plus one fixed joint at the tip:

| Order | Joint | Type | Parent link → Child link | URDF location |
|---|---|---|---|---|
| 1 | `arm_lift_joint` | prismatic | `base_link` → `arm_lift_link` | `ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrb/hsrb4s.urdf:840-845` |
| 2 | `arm_flex_joint` | revolute | `arm_lift_link` → `arm_flex_link` | `ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrb/hsrb4s.urdf:865-870` |
| 3 | `arm_roll_joint` | revolute | `arm_flex_link` → `arm_roll_link` | `ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrb/hsrb4s.urdf:890-895` |
| 4 | `wrist_flex_joint` | revolute | `arm_roll_link` → `wrist_flex_link` | `ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrb/hsrb4s.urdf:969-974` |
| 5 | `wrist_roll_joint` | revolute | `wrist_flex_link` → `wrist_roll_link` | `ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrb/hsrb4s.urdf:1003-1008` |
| 6 | `hand_palm_joint` | fixed | `wrist_roll_link` → `hand_palm_link` | `ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrb/hsrb4s.urdf:1088-1091` |

The xacro source for this chain (pre-expansion, used when the workspace
builds the real robot description) lives in:

```
ros2_ws/src/hsrb_common/hsrb_description/urdf/arm_v0/arm.urdf.xacro
ros2_ws/src/hsrb_common/hsrb_description/urdf/wrist_v0/wrist.urdf.xacro
ros2_ws/src/hsrb_common/hsrb_description/urdf/hand_v0/hand.urdf.xacro
```

`torso_lift_joint` (also prismatic, `hsrb4s.urdf:438`) is a separate torso
joint and is not part of this chain.

The `arm` group's `neutral` pose sets exactly these 5 actuated joints:

```
ros2_ws/src/hsrb_moveit/hsrb_moveit_config/config/hsrb.srdf:78-84
```

## About zenoh

SAM3 and GraspGenX are reached over zenoh instead of ROS 2 topics, since
they run in their own Docker containers with their own CUDA and Python
versions, kept separate from the ROS 2 workspace on purpose.

**No router needed.** Each server opens a plain zenoh session and starts
listening as soon as it starts up, no separate router process required. The
client side (`core/utils/zenoh_rpc.py`) does the same.

This is independent of whichever ROS 2 RMW is active. Switching between
`rmw_cyclonedds_cpp` and `rmw_zenoh_cpp` for ROS 2 topics has no effect on
the SAM3/GraspGenX zenoh connection, since that connection never goes
through the RMW at all.

**No multicast scouting.** Each server listens on a fixed TCP port instead
of an ephemeral one, and both disable multicast scouting outright:

- SAM3: `tcp/0.0.0.0:2002` (override with the `ZENOH_LISTEN` env var)
- GraspGenX: `tcp/0.0.0.0:2003` (same)

Scouting binds one shared, fixed UDP port (`224.0.0.224:7446`) host-wide.
With multiple `--net=host` containers on the same machine, only one zenoh
session can hold that port at a time, so a second server (or a stray
leftover process) fails at startup with `Address already in use`. Skipping
scouting entirely avoids that.

The client (`core/utils/zenoh_rpc.py`) connects explicitly via
`ZENOH_CONNECT`, which defaults to `tcp/127.0.0.1:2002,tcp/127.0.0.1:2003`
for local, same-machine use. Override it with a comma separated list using
the PC's real address when SAM3/GraspGenX run on a different host than the
client (e.g. the real robot talking to a PC):

```bash
export ZENOH_CONNECT="tcp/192.168.1.50:2002,tcp/192.168.1.50:2003"
```
