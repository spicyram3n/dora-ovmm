# Search for an object

Search scene-graph memory, drive to a viewing position, detect with SAM3, and park within arm reach. Search does not generate or execute grasps.

## Single launch: simulation to ready for grasping

Prepare and check the registered scene graph using steps 1–2 below once. Start
SAM3 on the **host** (`bash docker/sam3/run_sam3.sh`); the ROS dev container does
not manage that GPU container. Set `DEEPSEEK_API_KEY` in the ROS terminal when
LLM predictions are needed.

With the ROS and workspace setup sourced, run:

```bash
ros2 launch /home/ws/launch/search.launch.py target:=pringles top_k:=3
```

The launch runs [mission_tree.py](../pipeline/mission_tree.py) over
[actions.py](../pipeline/actions.py). Use `grasp:=false` for search and parking
only, or `mode:=grasp` to close and hold without lifting. Grasping runs once;
the arm remains in the resulting pose. Watch the tree with
`py-trees-tree-watcher` in another sourced ROS terminal.

This starts simulation, Nav2 with `config/nav2/nav2_params.yaml`, and IK. The search
worker waits for active Nav2 lifecycle nodes, navigation/head actions, IK,
fresh localization, synchronized RGB-D with TF, and an actual SAM3 response.
The startup SAM3 result is discarded; the search takes fresh observations after
moving. Startup has a shared 180-second deadline; override with
`startup_timeout:=300` for slower startup. No motion is requested before these
checks pass. Readiness does not validate the accuracy of your map registration.

After readiness checks, every launch sends the arm to home positions
`[0.0, 0.0, -1.57, -1.57, 0.0]` for lift, flex, roll, wrist flex, and wrist roll,
with a 3-second trajectory. It waits for successful completion before searching;
rejection, failure, or a 30-second execution timeout stops the mission. This also
runs when attaching to existing simulation with the start flags set to false.
Logs show `[HOME]` followed by `[READY] arm home pose reached`.

The worker searches memory or LLM-ranked furniture, visits viewpoints, detects
the target, saves its updated graph entry, and refines the base position. It
stops before grasp generation and arm execution.

If simulation, Nav2 and IK are already running, avoid launching duplicates:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="red cup" \
  start_simulation:=false start_navigation:=false start_ik:=false
```

Each start flag can be used independently. Other arguments are `graph`, `map`,
`params_file`, and `bearings`. Use a separate prepared `graph:=/path/to/scenario.json`
for each demo scenario: detections update that file, and the launcher does not
rebuild it or change room assignments.

Output is labelled by process (`simulation`, `navigation`, `ik`, `search`), with
individual ROS node names inside their logs. The worker prints `[WAIT]`,
`[READY]`, `[SEARCH]`, `[APPROACH]`, and `[RESULT]`, alongside existing per-location
and per-pose output. Launch also saves logs in its printed log directory.
These are readiness/stage messages, not a percentage-complete dashboard.

After the search exits, simulation stays open for inspection. The final mission
message reports ready (0), not found (1), placement failed (2), or error (3).
This is the **worker** exit status; the interactive ROS launch stays running.
Press Ctrl+C to shut down launched processes; separately started services remain
yours to stop. Rerun with the appropriate start flags to search again.

## 1. Launch simulation and Nav2

Run this setup in **each ROS terminal**:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

**ROS terminal 1 — simulation:**

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

**ROS terminal 2 — Nav2:**

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  params_file:=/home/ws/config/nav2/nav2_params.yaml \
  use_sim_time:=true
```

The loaded controller is direct Omni MPPI, without a rotation shim. It can
strafe during approaches without first aligning to the path. Path-heading costs
still encourage forward-facing travel, and the goal checker requires final yaw.
Restart Nav2 after changing controller plugins.

Run only one Nav2 launch. Check that the laser scan lines up with the map in RViz before continuing. The initial AMCL pose comes from the parameter YAML.

## 2. Register and build the scene graph

In **ROS terminal 3**, before moving the freshly spawned robot:

```bash
python3 -m core.scene_graph.build register \
  --world-base 5.0 6.6 0.0 --output config/map/world_to_map.json
python3 -m core.scene_graph.build --transform config/map/world_to_map.json
python3 visualization/viewpoints.py 'pringles' --output outputs/viewpoints.png
```

`5.0 6.6 0.0` is only for the default fresh apartment spawn. If the robot has moved, use its current Gazebo base-footprint X/Y/yaw. Registration needs correct AMCL localization; check graph furniture against the map and rebuild after changing the transform.

Set `DEEPSEEK_API_KEY` in this terminal for unknown-object searches or fallback after a remembered location fails. Unknown-object dry runs also call the API, but send no motion commands.

To assign room names, rebuild with:

```bash
python3 -m core.scene_graph.build --transform config/map/world_to_map.json --rooms
```

## Real room: from a scan

Build the graph from OpenYOLO3D output with `python -m core.scene_graph.openyolo3d`,
as in the [scan workflow](../../docker/openyolo3d/README.md#3-build-the-scene-graph),
then plot a piece before driving:

```bash
python3 visualization/viewpoints.py --furniture <name> --graph outputs/scene_graph/room1.json
```

Every graph build, scan or Gazebo, fits each furniture piece with the smallest
turned rectangle around its points seen from above, so turned pieces keep their
yaw. Stray points in a mask stretch that rectangle, and box errors move every
viewpoint by the same amount. Graphs built before this fall back to
axis-aligned boxes until rebuilt.

## 3. Start perception and IK

In a **host terminal**, from the repository root:

```bash
bash docker/sam3/run_sam3.sh
```

In another **ROS terminal**:

```bash
ros2 launch /home/ws/launch/ik_solver.launch.py
```

## 4. Search

In ROS terminal 3:

```bash
python3 -m core.pipeline.mission_tree --target 'pringles' --top-k 3 --grasp false
```

| Option | Use it to |
| --- | --- |
| `--grasp false` | Stop after search and parking |
| `--navigate-only true` | Reason and navigate without object detection |
| `--mode grasp` | Hold contact without lifting |
| `--graph /path/to/graph.json` | Use a different scene graph |
| `--bearings 6` | Set horizontal reach-probe directions |

Exit codes: **0** ready/arrived, **1** not found, **2** found but not graspable — either no probe direction is reachable, or the base could not park inside the cluster the IK solver certified.

## Preview the viewpoints

No robot motion; source ROS before running. Draws what the query selects, using the same `plan()` call the mission drives:

```bash
python3 visualization/viewpoints.py 'pringles' --output outputs/viewpoints.png
```

| Option | Use it to |
| --- | --- |
| `--index 1` | Plot the next search location; past the remembered ones this calls DeepSeek |
| `--robot X Y` | Order candidates as if the robot were there (default `0 0`) |
| `--furniture high_table01` | Plot a furniture target instead of an object |
| `--graph room1.json` | Plot a scanned graph |

Reading the plot: green arrows are candidate poses, numbered in visiting
order. The title says how many views the location needs; only that many are
driven to, and later numbers are fallbacks when Nav2 cannot reach one.

Left panel is the 3D scene with the IK reach probes at the object; right panel is the plan view, where the circles are the base radius the standoff filter uses. Red crosses are candidates dropped for hitting furniture. Dotted lines show where each view looks; the title gives how many views the location needs. The filter checks the **base footprint only** — a pose it keeps can still be sighting through another piece of furniture, which is visible in the shelf case.

## What success means

- Candidate views sit every 0.5 m along a line 0.8 m out from the furniture edge, following its yaw, each facing the part in front of it. The first view is the nearest of those showing nearly as much as the best one (within 5%), so it is close without being an end view down a long table; after it, views go in the order that shows the most unseen top from within 1.5 m, until another would add under 5%: 2 for a small table, 3–5 for long ones, at most 6. The second view of a small table comes from the far side.
- Shelved furniture (taller than 1.2 m, or labelled shelf/bookcase/rack) is viewed only from its long faces, with the head aimed at each height band in turn. Heights beyond the head's tilt are skipped. Check head aiming in Gazebo before hardware use.
- A run keeps every place's candidate views, and what happened at each (`no path`, `drive failed`, `cannot aim`, `not detected`, `detected`), in `search.VIEWS`. A later look at the same place reuses them, reordered from where the robot is. They are held in memory, so they go when the process ends; another process, such as the NBV node, cannot see them.
- The camera field of view is assumed to be ±0.45 rad horizontally and ±0.35 rad vertically (`standoff.py`). Check it against the HSR-C `camera_info`.
- Detection needs fresh RGB-D, at least 100 valid masked depth points, and 25% valid masked depth. Bad depth tries another view.
- Search tries every remembered instance of the object, nearest first, before asking DeepSeek to guess furniture. Without an API key it searches memory only and reports not found, rather than failing.
- Successful detection saves map-frame bounds before IK runs. Failed observations leave memory unchanged. Matching uses label and proximity, so instance identity is approximate; a same-label detection more than 1 m from a remembered one becomes a new node instead of overwriting it.
- Base placement uses hand-pose probes. A reachable probe does not prove a stable grasp. The head aims at the object again after placement.
- **Arm–furniture collisions are not checked:** the IK environment is empty. Base costmap and self-collision checks still apply.
- Nav2's standing 25 cm tolerance is larger than the IK robustness neighborhood of about 7.5 cm, so the final approach tightens `general_goal_checker` to 0.08 m at runtime and restores it afterwards. Arrival is then re-measured against the certified pose: past 0.08 m the run reports **2**, not ready. See [IK solver settings](#ik-solver-settings).

## If it stops

| Problem | Next step |
| --- | --- |
| Goal acceptance times out | Stop Nav2 before retrying; the goal state is unknown |
| Cancellation cannot be confirmed | Resolve the active goal before restarting search |
| Camera, SAM3, or IK error | Restore the service; an error does not mean the object is absent or unreachable |
| Old map-to-base TF | Check localization and clock alignment; TF must be within 2 seconds |
| No reachable view | Check localization, furniture bounds, and the costmap |

Planning or driving failures try another candidate. Observation tolerances are 0.30 m/rad, not grasp tolerances.

## IK solver settings

[launch/ik_solver.launch.py](../../launch/ik_solver.launch.py) starts Toyota's solver with the **HSRC** plugin (Toyota's example hardcodes the different HSR-B geometry). Service: `/ik_solver_node/solve_ik_with_collision`.

| Parameter | Value | Effect |
| --- | --- | --- |
| `convolution.ik_base_resolution` | `0.05` m | Base search grid spacing; halving it roughly quadruples grid work |
| `convolution.grid_distance_threhsold` | `1.5` cells | Scores solutions using nearby reachable base cells |
| `convolution.ik_result_joint_distance_threshold` | `1.0` (default) | Joint/base-yaw similarity used for scoring |

Keep the spelling **`threhsold`**. Toyota's code reads that exact key; correcting it silently uses the default.

The solver checks a base-position grid, filters collisions, scores nearby solutions, and returns only those tied for the best score. Border cells without a full scoring neighborhood are skipped. A timeout is a service error, not an unreachable result, and a reachable hand pose does not guarantee a grasp: [core/grasping/pick.py](../grasping/pick.py) plans the actual grasp from the parked pose. Toyota's implementation is under `ros2_ws/src/tmc_manipulation/tmc_ik_solver_node/`.
