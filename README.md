# HSR object search

**One command:** home the arm → search scene-graph/LLM locations → navigate → detect with SAM3 → park within arm reach → generate grasps with GraspGenX → approach, close on finger feedback, lift, and verify the hold.

The default scene is now [the door-free kitchen-object world](worlds/README.md).
The robot starts in the **high-table room**, then navigates to the requested
object on the kitchen table. Seven table objects and their assets are saved.
The launch uses the matching map, scene graph, and AMCL startup pose.

```bash
ros2 launch /home/ws/launch/search.launch.py target:="spray bottle" mode:=grasp
# NVIDIA workstation, with GPU access enabled for the ROS container:
bash /home/ws/launch/search_nvidia.sh target:="spray bottle" mode:=grasp
```

`mode:=grasp` holds contact; `mode:=pickup` requests a verified lift.
For the packaged scene, skip the manual scene-registration section below;
that section describes preparing an alternative apartment scene.

## 1. Clone

```bash
git clone git@github.com:spicyram3n/dora-ovmm.git
cd dora-ovmm
```

## 2. Set up Docker and the workspace

On the **host**, install Docker, NVIDIA drivers + NVIDIA Container Toolkit, and VS Code with the **Dev Containers** extension. Docker must work for your user and have GPU access for the model servers.

Open the repository in VS Code → **Dev Containers: Reopen in Container**. First setup downloads ROS packages and builds the workspace. Wait for it to finish.

After changing ROS sources, rebuild inside the **dev container**:

```bash
ROS2_REBUILD=1 bash /home/ws/.devcontainer/postCreate.sh
```

For `.devcontainer/Dockerfile` changes, use **Dev Containers: Rebuild Container**. Workspace Python, YAML, and launch-file edits normally only need a process restart. For model-server changes, rebuild the model images using step 4.

Run this in **each new dev-container terminal**:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

## 3. Prepare the scene graph — once per scene

**Container terminal 1 — simulation:**

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

**Container terminal 2 — Nav2:**

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  params_file:=/home/ws/config/nav2/nav2_params.yaml use_sim_time:=true
```

Check that the laser scan lines up with the map in RViz. Keep the freshly spawned robot stationary.

**Container terminal 3 — register and build:**

Set `DEEPSEEK_API_KEY` in this terminal for room assignment and LLM search.

```bash
python3 -m core.scene_graph.build register \
  --world-base 5.0 6.6 0.0 --output config/map/world_to_map.json
python3 -m core.scene_graph.build \
  --transform config/map/world_to_map.json --rooms
python3 visualization/viewpoints.py pringles --output outputs/viewpoints.png
```

The spawn coordinates above apply only to the default fresh apartment. Check graph alignment before using it. Output: `outputs/scene_graph/apartment.json`.

Stop simulation and Nav2 with **Ctrl+C** in terminals 1 and 2 before the full launch below.

For an unknown-object demo, use a graph copy with the target removed but furniture retained. Successful searches update the selected graph. See [scene graph instructions](core/scene_graph/README.md) for more details.

## 4. Start the model servers — host terminal

Run these commands from the repository root on the **host**, with Docker Compose 2.24+ and GPU access configured.

Before the first run:

- Accept the `facebook/sam3` model license on Hugging Face and save `HF_TOKEN=...` in `docker/sam3/.env`.
- Follow the [GraspGenX setup](docker/graspgenx/README.md) to prepare checkpoints and register the `hsrc_hand` gripper. This is required for pickup; search alone only needs SAM3.
- Stop old manually launched SAM3/GraspGenX containers so their ports are free.

**First run, or after changing model-server code:**

```bash
docker compose -f docker/compose.yaml up -d --build --wait --wait-timeout 900
```

This builds and starts both servers in the background, then waits for model loading and warmup. Continue to the pipeline once the command succeeds.

**Subsequent runs:**

```bash
docker compose -f docker/compose.yaml up -d --wait --wait-timeout 900
```

**Search only:** start just SAM3 instead:

```bash
docker compose -f docker/compose.yaml up -d --build --wait --wait-timeout 900 sam3
```

The existing `bash docker/sam3/run_sam3.sh` and `bash docker/graspgenx/run_graspgenx.sh` commands are shortcuts that build/start the selected server and wait for readiness.

**Check health — host terminal:**

```bash
docker compose -f docker/compose.yaml ps sam3 graspgenx
```

Both should show `healthy`; `starting` is normal during model loading and warmup. Docker checks Zenoh readiness every 30 seconds on SAM3 port `7447` and GraspGenX port `7448`.

Model RPC uses direct TCP connections with multicast discovery disabled to avoid host UDP port `7446` conflicts. Clients default to `tcp/127.0.0.1:7447,tcp/127.0.0.1:7448`; set `ZENOH_CONNECT` to comma-separated TCP endpoints when the models run on another host.

Run the same probes immediately (silent success, exit code `0`; failure is nonzero):

```bash
docker compose -f docker/compose.yaml exec -T sam3 python3 zenoh_rpc.py tcp/127.0.0.1:7447 sam3
docker compose -f docker/compose.yaml exec -T graspgenx .venv/bin/python3 zenoh_rpc.py tcp/127.0.0.1:7448 graspgenx
```

These check readiness after warmup, not a full inference. To investigate failures:

```bash
docker compose -f docker/compose.yaml logs --tail=100 -f sam3 graspgenx
```

Ctrl+C exits log viewing; the servers keep running. Checkpoints and registered grippers persist in their existing host folders. A failed readiness wait does not stop the containers; inspect their logs before retrying.

## 5. Run the pipeline

In a sourced **dev-container terminal**, with `DEEPSEEK_API_KEY` set for LLM predictions:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles" top_k:=3 grasp:=true
```

For search and base placement only, use the same command with `grasp:=false`.

The launch starts simulation, Nav2, IK, and MoveIt; waits for services, localization, RGB-D, and SAM3; then sends the arm home before searching. Pickup is enabled by default; `grasp:=false` stops after base placement.

Watch **`[WAIT]` → `[READY]` → `[HOME]` → `[SEARCH]` → `[APPROACH]` → `[GRASP]` → `[RESULT]`**. Process logs are labelled individually. Simulation stays open after completion; **Ctrl+C** stops the launched processes.

Use the same launch for a contact-only grasp:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="spray bottle" mode:=grasp
```

`mode:=pickup` requests a verified lift; `mode:=auto` lifts cylinders and holds
other shapes. The tree makes one pick attempt and leaves the resulting hold
in place, with navigation paused. It does not home the arm after grasping.
The picker reports `picked`, `grasped`, or failure separately in the logs.

Pickup ([core/grasping/pick.py](core/grasping/pick.py)) keeps SAM3, GraspGenX and
MoveIt Task Constructor. Calibrated finger geometry places side grasps on fitted
cylinders and top grasps across resolved rectangular tops; other shapes use
observed front contact sections. Ambiguous geometry or excessive width stops
pickup before motion.

Before generating grasps, a bounded head scan combines overlapping RGB-D
observations in odom. Image clipping and nearer foreground occlusion are
checked. If the arm blocks the scan, MTC plans one return to the existing
observation/home arm pose, re-aims the head at the target and captures again.
An unsuccessful recovery stops the pick. Detections that do not overlap the
same target are discarded; this is a conservative check, not a guarantee of
complete geometry for every shape or occlusion.

MTC plans opening, pregrasp, and the full 80 mm approach together before execution.
Candidates with an infeasible approach are rejected before motion. Cartesian
approach and lift use coordinated arm and base motion with collision checking.
The palm goal is computed once as `contact - rotation @ calibrated_pad(width)`:
one shared gripper calibration, with a different pad-centre offset for each
measured grip width. This is not a fixed 75 mm addition to every generated
pose. Cylinder diameter, rectangular short edge, or an observed irregular
contact section determines the width. Out-of-range apertures are rejected.
For a controlled insertion trial, `HSR_GRASP_INSERTION_M=0.005` adds 5 mm
along each grasp approach direction. Default is 10 mm for cylinders and zero otherwise; allowed range is 0–10 mm,
also capped at 20% of contact width. This is trial tuning, not validated calibration.
Closure uses bounded position steps and bilateral spring feedback: 0.10 rad
for grasp-only contact and 0.14 rad for pickup. The legacy effort helper is
not used by the default sequence. Contact alone does not verify a pickup.

Closure requires measured arrival at the calibrated palm pose. A short test
lift still requires RGB-D evidence of object motion to report success.

Visual servoing is disabled in the default pickup sequence. The old Servo
experiment remains available in source; its node is opt-in with
`enable_grasp_servo:=true`. The calibration profile lives beside the unchanged
registered meshes in `docker/graspgenx/x_grippers/hsrc_hand/closing_profile.json`.
Geometry and collision checks remain necessary; calibration does not make
all object shapes or poses graspable.

Useful overrides:

| Argument | Purpose |
| --- | --- |
| `target:="red cup"` | Change the queried object |
| `grasp:=true` | Pick the object up once parked |
| `graph:=/path/to/scenario.json` | Select prepared object memory |
| `top_k:=5` | Ask for more candidate locations |
| `startup_timeout:=300` | Allow slower startup |

If simulation, Nav2, IK, and MoveIt are **already running**, reuse them:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles" grasp:=true \
  start_simulation:=false start_navigation:=false start_ik:=false start_move_group:=false
```

### MoveIt with this repository's plugins

The full pipeline launch automatically starts `launch/move_group.launch.py`.
It uses the HSRC description (`hsrc_description`, `hsrc1s.urdf.xacro`) and loads:

- `config/moveit/kinematics.yaml`: arm and whole-body IK plugins.
- `config/moveit/ompl_planning.yaml`: motion planner settings.
- `config/moveit/sensors_xtion.yaml`: MoveIt's depth-image updater and the 1 cm octomap, fed by `hsr_rgbd`'s 1 Hz relay.
- `move_group/ExecuteTaskSolutionCapability`: executes MoveIt Task Constructor plans.

The vendor command `ros2 launch hsrb_moveit_config demo.py` loads the installed
package's configuration; it does not select these repository-local YAML files.
For a separate MoveIt terminal, use this command after sourcing ROS and the workspace:

```bash
ros2 launch /home/ws/launch/move_group.launch.py use_sim_time:=true use_rviz:=true
```

Then run the pipeline in another sourced terminal, reusing that MoveIt instance:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles" grasp:=true start_move_group:=false
```

This still starts simulation, Nav2, and IK. If those are already running too,
use all four `start_...:=false` arguments shown above. Run only one MoveIt
instance. Editing `config/moveit/` settings only requires restarting MoveIt.

To grasp on its own, with the base already parked within reach:

```bash
python3 -m core.grasping.pick "pringles can"
```

## 6. Stop

Press **Ctrl+C** in the pipeline terminal to stop the ROS launch. Model servers run independently; stop them from a **host terminal**:

```bash
docker compose -f docker/compose.yaml down
```

This removes the model containers while keeping downloaded checkpoints and gripper files. Servers otherwise restart after process failure unless explicitly stopped. Healthchecks report readiness; they do not restart an unhealthy process.

## Model client notes

Each model runs one inference at a time with two waiting slots (`MODEL_QUEUE_SIZE`). Full queues return `BUSY`; client deadlines raise `TimeoutError`. Expired queued work is skipped, but running GPU inference cannot be cancelled.

Clients reuse one Zenoh session per process. Existing `detect()` and `generate()` calls keep their return values. `sam3_client.detect_all()` returns all masks, boxes, scores, and metadata. Both clients accept optional `metadata` containing `frame_id`, `stamp` (`sec`/`nanosec`), and `object_id`; use `return_metadata=True` with GraspGenX to receive it. Supply the acquisition stamp and actual cloud/image frame. Centering transforms and object tracking remain the caller's responsibility.

Rebuild both model images after shared protocol changes. Direct Docker builds now use the repository root as context, for example `docker build -f docker/sam3/Dockerfile -t hrl/sam3:latest .`.

## More details

- [Navigation and troubleshooting](core/navigation/README.md)
- [Real scans → Mask3D/OpenYOLO3D → scene graph](docker/openyolo3d/README.md)
- [Pickup with MoveIt Task Constructor](core/grasping/pick.py)
- [Camera transport between machines](vision-transport-poorna/README.md)

## Code organization

Run the complete mission against an already running ROS stack from the repository root:

```bash
python3 -m core.pipeline.mission_tree --target "pringles"
# Search and park only:
python3 -m core.pipeline.mission_tree --target "pringles" --grasp false
```

The main entry point is `launch/search.launch.py`, following the `feat/pipeline` layout:

```text
launch/search.launch.py
└── core/pipeline/mission_tree.py
    └── core/pipeline/actions.py
        └── core/grasping/pick.py
```

The ROS launch above brings up the stack and invokes this behavior tree.
The mission waits for readiness, homes the arm, queries remembered/LLM locations,
searches with Nav2 and SAM3, saves the observation, parks within reach, generates
fresh grasps, and picks the object up with MoveIt Task Constructor.

| Folder | Responsibility |
| --- | --- |
| `core/pipeline/` | Mission readiness, orchestration, and object search |
| `core/grasping/` | MoveIt Task Constructor pickup and the GraspGenX client |
| `core/navigation/` | Nav2, standoff poses, and arm-reachable base placement |
| `core/perception/` | RGB-D capture, SAM3, and point clouds |
| `core/reasoner/` | Query ordering and DeepSeek location predictions |
| `core/scene_graph/` | Graph building, registration, memory, and visualization |
| `core/utils/` | Transforms and model transport |

Use `python3 -m core.<package>.<module>` from the repository root for individual
commands; the former loose scripts have moved into these packages.
Mission exit codes: `0` completed, `1` not found, `2` placement failed,
`3` startup/service error, `4` pickup failed, `130` interrupted.

The grasping package now has this layout:

```text
core/grasping/
├── __init__.py
├── pick.py               # perception, geometry, MTC, closure and lift verification
├── graspgenx_client.py   # model RPC
└── test/                # geometry, visual-control and closure guards
```

### Inspect the running MoveIt scene

Open RViz without starting another MoveIt instance:

```bash
ros2 launch /home/ws/launch/move_group.launch.py rviz_only:=true use_rviz:=true
```

The repository preset `config/rviz/moveit.rviz` selects `odom`,
`/monitored_planning_scene`, occupied octomap voxels, and `whole_body` automatically.
The octomap is part of the planning scene, separate from Nav2's 2D costmap.

To open RViz when starting a new full pipeline, add `use_rviz:=true` to
`search.launch.py`. These options need no rebuild. The RViz-only launch supplies
the same planning robot description and model settings as the MoveIt launch.

Before pickup, the mission pauses `/lifecycle_manager_navigation` and verifies
that the controller, behavior, navigator, and waypoint nodes are inactive.
Localization remains active. A failed pause prevents pickup, and navigation
stays paused after either pickup success or failure. A new mission resumes it.

The octomap relay publishes the latest depth frame at 1 Hz to match the updater.
Full-resolution grasp perception remains on the original camera stream.
