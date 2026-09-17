# HSR fetch

Ask for an object. The HSR homes its arm, picks where to look (scene graph first, then DeepSeek), drives there, finds the object with SAM3, parks within reach, and picks it up with GraspGenX and MoveIt.

| Where | What runs there |
| --- | --- |
| **Host** (your PC) | Model servers in Docker: SAM3 (detection), GraspGenX (grasps), VGN (grasps from a TSDF) |
| **Dev container** | ROS 2 Humble: Gazebo sim, Nav2, IK, MoveIt, the mission, the web dashboard |

## Quick start

Already set up? Two commands.

| # | Where | Command | Why |
| --- | --- | --- | --- |
| 1 | Host | `docker compose -f docker/compose.yaml up -d --wait --wait-timeout 900` | Start SAM3, GraspGenX and VGN |
| 2 | Container | `ros2 launch /home/ws/launch/search.launch.py target:="pringles"` | Start the sim stack and run one mission |

Prefer clicking? Use the [web dashboard](#5-web-dashboard) instead of step 2.

---

## 1. One-time setup

**Host needs:** Docker with GPU access (NVIDIA driver + NVIDIA Container Toolkit), VS Code with the **Dev Containers** extension.

```bash
git clone git@github.com:spicyram3n/dora-ovmm.git
cd dora-ovmm
```

Open the folder in VS Code → **Dev Containers: Reopen in Container**. The first start downloads ROS packages and builds the workspace. Wait for it.

The build includes `tf2`/`tf2_ros` 0.25.24 from source (`ros2_ws/src/geometry2`). The apt 0.25.23 deadlocks Nav2's costmaps: the robot then drives blind, circles or hits walls.

**Run this in every new container terminal:**

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

Warning: a terminal opened before the build still uses the broken tf2. `ros2 pkg prefix tf2` must print `/home/ws/ros2_ws/install/tf2`.

**When to rebuild:**

| You changed | Do this |
| --- | --- |
| ROS packages in `ros2_ws/src/` | `ROS2_REBUILD=1 bash /home/ws/.devcontainer/postCreate.sh` |
| `.devcontainer/Dockerfile` | VS Code → **Dev Containers: Rebuild Container** |
| Python, YAML or launch files | Nothing. Restart the process. |
| Model server code in `docker/` | Rebuild the images ([step 3](#3-start-the-model-servers)) |

---

## 2. Build the scene graph

**Once per scene.** The mission's default graph, `config/scene_graph/kitchen_objects.json`, ships with the repo. The steps below build the apartment graph, `outputs/scene_graph/apartment.json`; run it with `graph:=` and a matching `world`, `map` and `params_file`.

**Container terminal 1 — simulation.** Registration needs the robot in the world.

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

**Container terminal 2 — Nav2.** Gives the robot's pose on the map.

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  params_file:=/home/ws/config/nav2/nav2_params.yaml use_sim_time:=true
```

Check in RViz that the laser scan lines up with the map. **Do not move the robot.**

**Container terminal 3 — register, build, check.** Set `DEEPSEEK_API_KEY` first (room names).

```bash
python3 -m core.scene_graph.build register \
  --world-base 5.0 6.6 0.0 --output config/map/world_to_map.json
python3 -m core.scene_graph.build \
  --transform config/map/world_to_map.json --rooms
python3 visualization/viewpoints.py pringles
```

| Command | Why |
| --- | --- |
| `build register --world-base X Y YAW` | Measures the Gazebo-world → map transform from where the robot stands |
| `build --transform ... --rooms` | Builds the graph from the world file; DeepSeek names the rooms |
| `viewpoints.py pringles` | Plots the first place a query would search and its poses. No motion. |

Warning: `5.0 6.6 0.0` is the default fresh spawn only. If the robot moved, use its Gazebo `base_footprint` x, y, yaw.

Then **Ctrl+C terminals 1 and 2** before the full launch.

- **Unknown-object demo:** copy the graph, delete the target's node, keep the furniture. Successful searches update the graph they use.
- **More:** [scene graph README](core/scene_graph/README.md).

---

## 3. Start the model servers

**On the host, from the repository root.** Needs Docker Compose 2.24+.

**Before the first run:**

- [ ] Accept the `facebook/sam3` license on Hugging Face and put `HF_TOKEN=...` in `docker/sam3/.env`.
- [ ] Do the [GraspGenX setup](docker/graspgenx/README.md). Only needed for picking.
- [ ] Stop any SAM3/GraspGenX containers you started by hand, so the ports are free.

| Command | Why |
| --- | --- |
| `docker compose -f docker/compose.yaml up -d --build --wait --wait-timeout 900` | First run, or server code changed: build, start, wait for warmup |
| `docker compose -f docker/compose.yaml up -d --wait --wait-timeout 900` | Every later run |
| `docker compose -f docker/compose.yaml up -d --build --wait --wait-timeout 900 sam3` | Search only: start SAM3 alone |
| `docker compose -f docker/compose.yaml ps sam3 graspgenx vgn` | Health. All `healthy`; `starting` means still loading |
| `docker compose -f docker/compose.yaml logs --tail=100 -f sam3 graspgenx vgn` | See why a server failed. Ctrl+C leaves them running |
| `docker compose -f docker/compose.yaml down` | Stop and remove the containers. Checkpoints and gripper files stay |

Shortcuts that build, start and wait: `bash docker/sam3/run_sam3.sh`, `bash docker/graspgenx/run_graspgenx.sh`, `bash docker/vgn/run_vgn.sh`.

**Readiness probes** (exit code 0 = ready; they check warmup, not a full inference):

```bash
docker compose -f docker/compose.yaml exec -T sam3 python3 zenoh_rpc.py tcp/127.0.0.1:7447 sam3
docker compose -f docker/compose.yaml exec -T graspgenx .venv/bin/python3 zenoh_rpc.py tcp/127.0.0.1:7448 graspgenx
docker compose -f docker/compose.yaml exec -T vgn python3 zenoh_rpc.py tcp/127.0.0.1:7449 vgn
```

| Server | Port | Note |
| --- | --- | --- |
| SAM3 | `7447` | TCP only; multicast is off to avoid clashes on UDP `7446` |
| GraspGenX | `7448` | Same |
| VGN | `7449` | Same. No setup; weights are baked into the image ([README](docker/vgn/README.md)) |

Models on another machine? Set `ZENOH_CONNECT=tcp/<ip>:7447,tcp/<ip>:7448,tcp/<ip>:7449` in the container terminal.

---

## 4. Run the mission

**Container terminal**, with `DEEPSEEK_API_KEY` set for unknown objects:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles"
```

This starts the sim, Nav2, IK and MoveIt, then runs the [behaviour tree](core/pipeline/mission_tree.py):

| Step | What happens |
| --- | --- |
| **Home arm** | Arm to the home pose, whatever pose it is in |
| **Ready** | Waits for Nav2, head, IK, localization, RGB-D and SAM3 (180 s budget) |
| **Find target** | Scene graph first, then DeepSeek's top-k. Drives to views; SAM3 looks. Views Nav2's costmap blocks are skipped; two failed drives move on to the next place. |
| **Park** | Drives to an IK-certified pose within arm reach |
| **Pause Nav2** | So MoveIt can move the base |
| **Pick** (once) | GraspGenX grasps + MoveIt Task Constructor; checks the hand closed on something |
| **Stow** (inside Pick) | Backs the palm straight out, then folds to the carry pose through move_group, with the octomap and the held object in the planning scene. Not a separate tree step: a raw joint command here would drag the load through the surface it was picked from. |

**Launch arguments:**

| Argument | Use it to |
| --- | --- |
| `target:="red cup"` | Choose the object |
| `grasp:=false` | Stop after parking; no pick |
| `navigate_only:=true` | Only reason and drive there; no SAM3 or GraspGenX |
| `top_k:=5` | Ask DeepSeek for more places |
| `graph:=/path/to/scenario.json` | Use another scene graph |
| `world:=` `map:=` `params_file:=` | Use another scene. Defaults: `worlds/kitchen_objects.world`, `config/map/kitchen_objects.yaml`, `config/nav2/kitchen_objects.yaml`. Change all three together. |
| `startup_timeout:=300` | Allow slower startup |
| `bearings:=12` | Set the reach-probe directions for parking (30 deg apart) |
| `use_rviz:=true` | Open RViz with MoveIt |
| `shutdown_when_done:=true` | Close everything when the mission ends |
| `web:=true` | Start the stack with no mission ([dashboard](#5-web-dashboard)) |
| `start_simulation:=false` (also `start_navigation`, `start_ik`, `start_move_group`) | Reuse a stack that is already running |

**Reuse a running stack:**

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles" \
  start_simulation:=false start_navigation:=false start_ik:=false start_move_group:=false
```

**While it runs:**

- **Logs:** `[HOME]` → `[WAIT]` → `[READY]` → search output → `[RESULT]`, then the tree with each step's status.
- **Watch the tree live:** run `py-trees-tree-watcher` (terminal) or `py-trees-tree-viewer` (window) in another sourced terminal.
- The sim stays open afterwards. **Ctrl+C** stops everything.

**Exit codes:**

| Code | Meaning |
| --- | --- |
| `0` | Done |
| `1` | Not found |
| `2` | Found, but could not park within reach |
| `3` | Startup or service error |
| `4` | Parked, but the pick failed; the arm is folded back to the carry pose |
| `5` | The pick failed **and** the arm could not be stowed -- it is still extended, clear it by hand |
| `130` | Stopped with Ctrl+C |

---

## 5. Web dashboard

Type a query in the browser and watch each stage. Runs **navigate-only** for now: home the arm, choose the place, drive there.

| # | Terminal | Command | Why |
| --- | --- | --- | --- |
| 1 | Container A | `python3 -m web.server` | The dashboard. Keep it running; it outlives the sim. |
| 2 | Container B | `ros2 launch /home/ws/launch/search.launch.py web:=true` | The stack, with no mission. Restart it freely. |
| 3 | Browser | <http://localhost:8090> | Type the object, press **Fetch** |

| On the page | Shows |
| --- | --- |
| Left | The mission tree, live, and **Runs**: every saved query |
| **1 · Home** | The robot model homing its arm. **Home arm** homes it by hand. |
| **2 · Reason** | The reasoner: already in the scene graph, or DeepSeek's top-k and why |
| **3 · Navigate** | The target furniture, parking spots, Nav2's path as dots, the robot driving |
| Bottom right | The head camera |

- **Saved runs:** each query is saved to `outputs/web_runs/` (`<start time>.json` + `.log`). Open any under **Runs**, with or without the sim.
- **Replays:** a stage replays once when you open its tab. Click the tab to replay.
- **Options:** extra mission options go after the server command, e.g. `python3 -m web.server --top-k 5`.
- Warning: The browser needs internet (three.js comes from a CDN). The server listens on localhost only.

---

## 6. Pick, MoveIt and RViz

| Command | Why |
| --- | --- |
| `python3 -m core.grasping.pick "pringles can"` | Grasp only, base already parked. Needs MoveIt, SAM3, GraspGenX, and Nav2 paused. Exit 0 = holding it. |
| `ros2 launch /home/ws/launch/move_group.launch.py use_sim_time:=true use_rviz:=true` | MoveIt in its own terminal; then add `start_move_group:=false` to the mission launch |
| `ros2 launch /home/ws/launch/move_group.launch.py rviz_only:=true use_rviz:=true` | RViz on the running MoveIt scene, without a second MoveIt |

**This repo's MoveIt loads:**

| File | Why |
| --- | --- |
| `config/moveit/kinematics.yaml` | Arm and whole-body IK plugins |
| `config/moveit/ompl_planning.yaml` | Planner settings |
| `config/moveit/sensors_xtion.yaml` | 1 cm octomap from `hsr_rgbd`'s 1 Hz depth relay; `config/realrobot/moveit/sensors_xtion_remote.yaml` is the real-robot version ([section 7](#7-real-robot-grasp-test)) |
| `move_group/ExecuteTaskSolutionCapability` | Runs MoveIt Task Constructor plans |

- Warning: `ros2 launch hsrb_moveit_config demo.py` ignores these files. Run only one MoveIt. Config changes need only a MoveIt restart.
- **RViz preset:** `config/rviz/moveit.rviz` shows `odom`, the planning scene, occupied octomap voxels and `whole_body`. The octomap is separate from Nav2's 2D costmap.
- **Before a pick**, the mission pauses Nav2's motion nodes (localization stays on) and refuses to pick if that fails. Nav2 stays paused afterwards; the next mission resumes it.
- **After a pick**, the robot keeps holding the object. The next pick opens the hand first.

---

## 7. Real robot: grasp test

Grasp only, no navigation, no mission. The robot keeps running its own stack; everything of ours runs on the PC. Full write-up with every problem hit so far: [docs/realrobot_grasp_test.md](docs/realrobot_grasp_test.md).

| Where | Runs | Started by |
| --- | --- | --- |
| **Robot** | Toyota's stack: controllers, `/joint_states`, `/tf`, `/omni_base_controller/state`, camera driver, `/head_rgbd_sensor/rgb/camera_info` | already running |
| **Robot** | Sagar's shared vision transport TX on port 7447: encodes RGB and depth onto Zenoh | already running; RX connects to it ([README](vision-transport-poorna/README.md)) |
| **PC host** | SAM3 and GraspGenX | `docker compose -f docker/compose.yaml up -d --wait --wait-timeout 900 sam3 graspgenx` |
| **PC host** | Vision transport RX: Zenoh in, `/remote/head_rgbd_sensor/...` ROS topics out | `docker compose -f deployment/compose.yaml up -d vision_rx` in `vision-transport-poorna/` (domain 5 by default) |
| **PC container** | move_group with the real-robot octomap, the depth relay, the pick | `ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py` |

Heavy topics (RGB, depth) go through vision transport. Small ones (joint states, TF, camera_info, controller actions) go straight over DDS, so the container and RX must share the robot's ROS domain (5) and reach it over the LAN. [.devcontainer/runtime.env](.devcontainer/runtime.env) is the switch; bashrc sources it last, so every new terminal gets it:

```bash
ROS_DOMAIN_ID=5
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/ws/.devcontainer/cyclonedds_profile.xml   # cyclonedds_sim.xml for the simulator
```

Open new terminals after editing it, or `source ~/.bashrc`. Check with `echo $CYCLONEDDS_URI`.

`cyclonedds_profile.xml` talks to the robot at `10.7.3.185` over `enp5s0`; edit both if they changed. The robot's clock and the PC's must agree to within 0.5 s (pick.py rejects older transforms): check with `chronyc tracking` or `date` on both, and the `tf` lines of the preflight below.

**Then, in order:**

| # | Command | Why |
| --- | --- | --- |
| 1 | `python3 realrobot/live/grasp_preflight.py` | Joint states, TF age, camera_info, RX RGB and depth rates, controller actions, SAM3, GraspGenX. Ends with `READY` |
| 2 | `ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py use_rviz:=true` | move_group, wall clock, octomap from `config/realrobot/moveit/sensors_xtion_remote.yaml` |
| 3 | `python3 realrobot/live/grasp_preflight.py --moveit` | Adds move_group, the MTC capability and the relay's topics |
| 4 | `HSR_REAL_ROBOT=1 python3 -m core.grasping.pick "pringles can" --mode grasp` | First trial: close on the can, no lift. Drop `--mode grasp` for the test lift |

Or pass `target:="pringles can" mode:=grasp` to the launch in step 2 and skip step 4. The arm moves under MoveIt and the whole-body group may shuffle the base a few centimetres: keep the emergency stop within reach and clear the table edge.

**What the real-robot switch changes:**

| Piece | Simulation | Real robot |
| --- | --- | --- |
| Octomap config | `config/moveit/sensors_xtion.yaml` | `config/realrobot/moveit/sensors_xtion_remote.yaml`: depth from `/remote/head_rgbd_sensor/depth_registered/image_rect_raw`, calibration from `/head_rgbd_sensor/rgb/camera_info`, best-effort QoS. Rename the topics there when `rx.yaml` changes |
| Depth relay inputs | the `relay:` block of that file | same; `depth_topic:=` and `depth_info_topic:=` on `move_group.launch.py` override it |
| Pick camera | `/head_rgbd_sensor/...` | `HSR_REAL_ROBOT=1` prefixes RGB and depth with `HSR_IMAGE_PREFIX` (default `/remote`, `image_prefix:=` on the launch) and uses the wall clock |
| Clock | `use_sim_time:=true` | `use_sim_time:=false`, set by `realrobot/grasp_real.launch.py` |

- Warning: run only one move_group on the domain. If the robot already runs Toyota's, stop it first, or its services shadow this one.
- RX only streams once something subscribes; the relay and the pick do, the preflight too.
- Camera_info and the hand camera are not in vision transport. The hand camera is only used by visual servoing, which stays off.

---

## 8. Stop

| What | How |
| --- | --- |
| Sim stack and mission | **Ctrl+C** in the launch terminal |
| Model servers | `docker compose -f docker/compose.yaml down` (host) |

Servers restart after a crash unless stopped. Healthchecks only report readiness; they do not restart a stuck server.

---

## Tests

| Command | Tests |
| --- | --- |
| `python3 -m pytest -p no:anyio tests` | Mission tree, web server, events, geometry, standoff poses (local `tests/` folder, not in git) |

Run them from `/home/ws` in a sourced terminal.

## Code map

| Folder | What it does |
| --- | --- |
| `core/pipeline/` | `mission_tree.py` decides when each step runs; `actions.py` does each step |
| `core/navigation/` | Nav2 client, viewing poses, arm-reachable parking ([README](core/navigation/README.md)) |
| `core/perception/` | RGB-D capture, SAM3 client, point clouds, the TSDF VGN reads |
| `core/reasoner/` | Search order and DeepSeek predictions |
| `core/scene_graph/` | Build, save and query the graph ([README](core/scene_graph/README.md)) |
| `core/grasping/` | Pick with MoveIt Task Constructor; GraspGenX and VGN clients |
| `core/utils/` | Geometry, transforms, model transport, dashboard events |
| `web/` | The dashboard: `server.py` and `static/` |
| `launch/` | Sim stack, MoveIt and IK launches |
| `visualization/` | Offline plots of viewpoints and scan graphs |

| Entry point | Run it to |
| --- | --- |
| `python3 -m core.pipeline.mission_tree --target pringles` | Run the mission on an already running stack (`--grasp false`, `--navigate-only true`) |
| `python3 -m core.pipeline.mission_tree --render` | Save a picture of the tree to `outputs/` (no ROS) |
| `python3 -m core.scene_graph.build --transform ...` | Rebuild the scene graph |
| `python3 -m core.grasping.pick "pringles can"` | Pick from where the robot stands |
| `python3 -m web.server` | Start the dashboard |
| `python3 visualization/viewpoints.py pringles` | Plot the poses a query would visit (no ROS) |

## Model server notes

- **Queue:** one inference at a time, two waiting (`MODEL_QUEUE_SIZE`). A full queue answers `BUSY`; a client deadline raises `TimeoutError`. Running GPU work cannot be cancelled.
- **Sessions:** clients reuse one Zenoh session per process.
- **Extra outputs:** `sam3_client.detect_all()` returns every mask, box and score. Both clients take optional `metadata` (`frame_id`, `stamp`, `object_id`); pass `return_metadata=True` to GraspGenX to get it back.
- **Rebuilds:** rebuild every model image after protocol changes. Direct builds use the repository root as context: `docker build -f docker/sam3/Dockerfile -t hrl/sam3:latest .`

## More docs

| Doc | Covers |
| --- | --- |
| [core/navigation/README.md](core/navigation/README.md) | Search, parking, viewpoints, IK settings, troubleshooting |
| [core/scene_graph/README.md](core/scene_graph/README.md) | Graph building, frames, Python use |
| [docker/graspgenx/README.md](docker/graspgenx/README.md) | GraspGenX setup for the HSRC hand |
| [docker/boxer/README.md](docker/boxer/README.md) | Real scans → labelled 3D boxes → scene graph |
| [docker/vgn/README.md](docker/vgn/README.md) | VGN: TSDF in, grasps out, and which objects it can grasp |
| [docs/realrobot_grasp_test.md](docs/realrobot_grasp_test.md) | Real robot grasp test: what runs where, commands, files, every problem hit and its fix |
| [vision-transport-poorna/README.md](vision-transport-poorna/README.md) | Robot camera → PC over Zenoh |
