# Navigation: search and park

Find the object with Nav2 and SAM3, then park where the arm can reach it. Picking is in [core/grasping/pick.py](../grasping/pick.py).

## Quick run

| Goal | Command |
| --- | --- |
| Full mission (sim included) | `ros2 launch /home/ws/launch/search.launch.py target:=pringles` |
| Search and park, no pick | Add `grasp:=false` |
| Search on a stack you started by hand | `python3 -m core.pipeline.mission_tree --target pringles --top-k 3` |

Set `DEEPSEEK_API_KEY` for objects the scene graph does not know. The scene graph must exist first ([root README, step 2](../../README.md#2-build-the-scene-graph)).

---

## Run the pieces by hand

Source ROS in each container terminal first:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

**Terminal 1 — simulation**

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

**Terminal 2 — Nav2**

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  params_file:=/home/ws/config/nav2/nav2_params.yaml use_sim_time:=true
```

**Host — SAM3:** `bash docker/sam3/run_sam3.sh`

**Terminal 3 — IK solver** (needed for parking): `ros2 launch /home/ws/launch/ik_solver.launch.py`

**Terminal 4 — mission:** `python3 -m core.pipeline.mission_tree --target pringles --top-k 3`

- Run only one Nav2. Check in RViz that the laser scan lines up with the map. The first AMCL pose comes from the params YAML.
- **Controller:** Omni MPPI, no rotation shim. It can strafe on approach; path-heading costs still favour facing forward, and the goal checker wants the final yaw. Restart Nav2 after changing controller plugins.

## Mission options

`python3 -m core.pipeline.mission_tree --target <object> [options]`

| Option | Use it to |
| --- | --- |
| `--top-k 3` | Set how many places DeepSeek suggests |
| `--grasp false` | Stop once parked; no pick |
| `--navigate-only true` | Reason and drive to the first place; no SAM3 or GraspGenX |
| `--graph /path/to/graph.json` | Use another scene graph |
| `--bearings 12` | Set the horizontal reach-probe directions (30 deg apart) |
| `--startup-timeout 180` | Wait this long for the stack at startup |
| `--render` | Save a picture of the tree to `outputs/` (no ROS) |

To see where a query would search without moving, use the [viewpoint preview](#preview-the-viewpoints). Exit codes are [below](#what-the-mission-does-at-startup).

## What the mission does at startup

| Step | Details |
| --- | --- |
| **Home arm** | Lift, flex, roll, wrist flex, wrist roll → `[0.0, 0.0, -1.57, -1.57, 0.0]` in 3 s. A rejection, failure or 30 s timeout stops the mission. Logs `[HOME]`, then `[READY] arm home pose reached`. |
| **Ready** | Waits for active Nav2 nodes, nav and head actions, IK, fresh localization, synced RGB-D with TF, and one SAM3 reply (thrown away). Shared 180 s budget; `startup_timeout:=300` for slow machines. |
| Then | The base moves only after Ready passes. Ready does **not** check that your map registration is accurate. |

- **Logs** are labelled by process: `simulation`, `navigation`, `ik`, `move_group`, `mission_tree`. The mission ends with the tree and each step's status.
- **Exit codes:** `0` done, `1` not found, `2` could not park, `3` startup/service error, `4` pick failed, `130` Ctrl+C. The sim stays open afterwards.
- **Watch live:** `py-trees-tree-watcher` in another sourced terminal. It is a plain command, not `ros2 run`.
- **Detections update the graph file.** Use a separate `graph:=/path/to/scenario.json` per demo scenario. The launch never rebuilds it.

## Preview the viewpoints

No robot, no ROS. Draws what a query would do, using the same `plan()` the mission drives:

```bash
python3 visualization/viewpoints.py 'pringles' --output outputs/viewpoints.png
```

| Option | Use it to |
| --- | --- |
| `--index 1` | Plot the next location. Past the remembered ones, this calls DeepSeek. |
| `--robot X Y` | Order the poses as if the robot stood there (default `0 0`) |
| `--furniture high_table01` | Plot a furniture target |
| `--graph room1.json` | Plot a scanned graph |

**Reading the plot:**

| You see | Means |
| --- | --- |
| Green numbered arrows | Candidate poses, in visiting order |
| Title's "views needed" | Only that many are driven to; later numbers are fallbacks |
| Circles (right panel) | The base radius the furniture filter uses |
| Red crosses | Poses dropped for hitting furniture |
| Dotted lines | Where each view looks |
| Purple arrows (left panel) | IK reach probes at the object |

Warning: the filter checks the **base footprint only**. A kept pose can still look through other furniture, which shows up in the shelf case.

## How search decides

| Topic | Rule |
| --- | --- |
| **View poses** | Every 0.5 m along a line 0.8 m out from the furniture edge, following its yaw, each facing the part in front of it |
| **View order** | First: the nearest view that shows within 5% of the best one. Then whichever shows the most unseen top within 1.5 m, until a view adds under 5%. |
| **How many** | 2 for a small table (the second from the far side), 3–5 for long ones, at most 6 |
| **Shelves** | Taller than 1.2 m, or labelled shelf/bookcase/rack: long faces only, head aimed at each height band. Heights past the head's tilt are skipped. |
| **Search order** | Every remembered instance, nearest first, then DeepSeek. No API key → memory only → "not found". |
| **Detection** | Needs fresh RGB-D, 100+ valid masked depth points and 25%+ valid depth. Bad depth → next view. |
| **Saving** | Map-frame bounds are saved before IK runs. Failed looks change nothing. A same-label detection more than 1 m from a remembered one becomes a new node. |
| **View memory** | Each place's poses and outcomes (`no path`, `drive failed`, `cannot aim`, `not detected`, `detected`) stay in `actions.VIEWS` for the process. Other processes cannot see them. |
| **Camera FOV** | Assumed ±0.45 rad horizontal, ±0.35 rad vertical (`standoff.py`). Check against the HSR-C `camera_info`. |
| **Parking** | Hand-pose probes via IK. Reachable does not mean a stable grasp. The head re-aims at the object after parking. |
| **Parking accuracy** | Nav2's usual tolerance is 25 cm; IK needs about 7.5 cm. The final approach tightens `general_goal_checker` to 0.08 m and restores it after. Parked further than 0.08 m → exit `2`. |
| **Arm collisions** | The IK solver checks the whole robot, arm included, against every scene-graph furniture box except the one the object is on or in (graph boxes are solid, so that one would swallow the hand). Real shapes are not modelled: a shelf's open front or a table's legroom count as solid. |

- Head aiming is tested in Gazebo only. Check it before using hardware.
- Viewing tolerances are 0.30 m / 0.30 rad, not grasp tolerances. A failed plan or drive tries the next pose.

## If it stops

| Problem | Next step |
| --- | --- |
| Goal acceptance times out | Stop Nav2 before retrying; the goal state is unknown |
| Cancellation cannot be confirmed | Resolve the active goal before restarting |
| Camera, SAM3 or IK error | Fix the service. An error does not mean the object is missing or unreachable. |
| Old map-to-base TF | Check localization and clocks; TF must be under 2 s old |
| No reachable view | Check localization, furniture bounds and the costmap |

## IK solver settings

[launch/ik_solver.launch.py](../../launch/ik_solver.launch.py) starts Toyota's solver with the **HSRC** plugin (Toyota's example hardcodes the HSR-B). Service: `/ik_solver_node/solve_ik_with_collision`.

| Parameter | Value | Effect |
| --- | --- | --- |
| `convolution.ik_base_resolution` | `0.05` m | Base grid spacing. Halving it roughly quadruples the work. |
| `convolution.grid_distance_threhsold` | `1.5` cells | Scores solutions by nearby reachable base cells |
| `convolution.ik_result_joint_distance_threshold` | `1.0` (default) | Joint and base-yaw similarity used in scoring |

Warning: keep the spelling **`threhsold`**. Toyota's code reads that exact key; the correct spelling is silently ignored.

- **How it works:** the solver checks a grid of base positions, drops collisions, scores nearby solutions and returns only the best-scoring ones. Border cells without a full neighbourhood are skipped.
- **Timeouts:** a timeout is a service error, not "unreachable".
- **Reach is not a grasp:** a reachable hand pose is not a grasp. `pick.py` plans the real grasp from the parked pose.
- **Source:** `ros2_ws/src/tmc_manipulation/tmc_ik_solver_node/`.

## Real room: from a scan

1. Build the graph from OpenYOLO3D output ([scan workflow](../../docker/openyolo3d/README.md#3-build-the-scene-graph)).
2. Plot a piece before driving:

```bash
python3 visualization/viewpoints.py --furniture <name> --graph outputs/scene_graph/room1.json
```

Every graph build fits each furniture piece with the smallest turned rectangle seen from above, so turned pieces keep their yaw. Stray mask points stretch that rectangle and shift every viewpoint with it. Graphs built before this use axis-aligned boxes until rebuilt.
