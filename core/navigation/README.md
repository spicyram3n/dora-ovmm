# Navigation: search and park

Find the object with Nav2 and SAM3, then park where the arm can reach it. Picking is in [core/grasping/pick.py](../grasping/pick.py).

- [Simulation](#simulation): launch files and commands for Gazebo.
- [Real robot](#real-robot): what exists so far.
- [Both](#both): mission options and how search behaves, in the sim and on the robot.

---

## Simulation

### Quick run

| Goal | Command |
| --- | --- |
| Full mission (sim included) | `ros2 launch /home/ws/launch/search.launch.py target:=pringles` |
| Search and park, no pick | Add `grasp:=false` |
| Search on a stack you started by hand | `python3 -m core.pipeline.mission_tree --target pringles --top-k 3` |

Set `DEEPSEEK_API_KEY` for objects the scene graph does not know. The scene graph must exist first ([root README, step 2](../../README.md#2-build-the-scene-graph)).

### Run the pieces by hand

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

---

## Real robot

Bring-up runbook and the record of what was measured on hardware:
[launch/realrobot/navigation.readme](../../launch/realrobot/navigation.readme).
The maths behind the localiser and the costmap rules:
[docs/realrobot_navigation.tex](../../docs/realrobot_navigation.tex).

The robot runs Toyota's own stack; this PC adds Nav2 on top of it. Both are on
`ROS_DOMAIN_ID=5` with `.devcontainer/cyclonedds_profile.xml`, so every robot
topic is already visible here. Vision transport RX carries only RGB-D; `/scan`,
`/tf` and the odometry arrive over plain DDS.

### Nav2 against the robot

```bash
ros2 launch /home/ws/launch/realrobot/nav2_real.launch.py
```

Starts map_server on `config/realrobot/map/lab_20260811.yaml`, the four Nav2
servers with [config/realrobot/nav2/nav2_params_real.yaml](../../config/realrobot/nav2/nav2_params_real.yaml),
and an `/initialpose` relay. **No AMCL:** the robot's `laser_2d_localizer` and
`pose_integrator` already publish `map->odom` at 100 Hz, and the saved map is in
that same frame -- zero shift against the robot's live `/static_obstacle_ros_map`,
91% of its walls within 10 cm. A second `map->odom` publisher would fight it.
Use `localization:=true` only with the robot's own localization stopped.

### Localize before the first goal

`laser_2d_localizer` is not seeded at boot: it reports the map origin, `/scan`
misses the walls by 35 cm, and every plan aborts with "Either of the start or
goal pose are an obstacle". Seed it once per boot; it tracks from then on, so
this is not a per-goal step. Three ways, cheapest first:

```bash
python3 realrobot/live/localize.py            # matches /scan to the map, publishes the pose
python3 realrobot/live/localize.py --near 2 -2   # same, but only around a guess
python3 realrobot/live/localize.py --dry-run  # report only
```

`localize.py` publishes on `/laser_2d_correct_pose` and `/initialpose` both, so
it needs nothing else running -- run it **before** Nav2. RViz's **2D Pose
Estimate** does the same job by hand, but it only speaks `/initialpose`, so it
needs the launch's relay and therefore Nav2 already up:

```bash
rviz2 -d /home/ws/ros2_ws/install/hsrb_rosnav_config/share/hsrb_rosnav_config/rviz/hsr_navigation2.rviz
```

Always check it took. `/laser_2d_localizer/score` reads about **-0.01** when the
pose is right and **-0.07** when it is wrong, and in RViz `/scan` sits on the
map walls. Measured at the right pose: median beam-to-wall 0 cm, 81% of beams
within 10 cm; at the map origin, 35 cm and 13%.

- **The map is five weeks older than the lab.** It was built from the
  2026-08-11 bag; the desks and chairs have moved since, so scan matching on
  clutter is ambiguous and only the walls are dependable. A plain
  nearest-wall-distance score ties poses 3 m apart; `localize.py` uses AMCL's
  likelihood field (`exp(-d^2)`) instead, which separates them. Trust
  `/laser_2d_localizer/score` over any of it: that runs on the robot's own map.
- For a repeatable start, park the robot at a known spot and seed from there.
- **Localize first, then start Nav2.** The global costmap marks what the laser
  sees but only clears within about 3 m of the robot, so anything painted in
  while the pose was wrong stays there as a phantom wall, and routes across the
  lab fail with "Could not generate path between the given poses". Re-localized
  with Nav2 already up, wipe it:

  ```bash
  ros2 service call /global_costmap/clear_entirely_global_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}"
  ```

### Send a goal

Plan first -- `/compute_path_to_pose` returns a path without moving anything:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: X, y: Y},
   orientation: {w: 1.0}}}}"
```

Then drive it. RViz's **2D Nav Goal** sends the same action:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: X, y: Y},
   orientation: {w: 1.0}}}}"
```

Pick X and Y from cells with **clearance**, not merely cost 0: a cost-0 cell
at the edge of the inflation halo flickers to 253 whenever someone walks past,
and the plan dies with "start or goal pose are an obstacle". Ask for half a
metre of cost-0 margin all round:

```python
from scipy.ndimage import distance_transform_edt
clear = distance_transform_edt(costmap == 0) * resolution   # metres of slack
```

Keep a hand on the wireless stop for the first goal, and stand clear of both
the robot and the goal: a person in the wrong place is enough to abort it.

| Wired to | Why |
| --- | --- |
| `cmd_vel` -> `/base_velocity` | The input of `safety_velocity_limiter`, so its obstacle slowdown and the bumpers stay between Nav2 and the wheels. Toyota's own `navigation_launch.py` remaps to `/omni_base_controller/cmd_vel`, which skips them. |
| `odom_topic: /switched_odom` | The robot publishes no `/odom`; `odometry_switcher` publishes this and broadcasts `odom->base_footprint`. |
| `use_sim_time: False` | No `/clock` on the robot. |

- **Never send a goal to Toyota's `/move_base` while this runs.** Its
  `base_path_follower` publishes to `/base_velocity` too, and the two fight.
- **Goals in a cluttered lab.** `robot_radius` 0.3 plus `inflation_radius` 0.35
  buries most of the floor between the desks in the inflation halo, and Theta*
  refuses a start or goal at cost 253. Pick goals from the free cells of
  `/global_costmap/costmap`, not from the map image.

### Mission

1. Build the graph from Boxer's boxes ([Boxer README](../../docker/boxer/README.md)).
2. Plot a piece before driving:

   ```bash
   python3 visualization/viewpoints.py --furniture <name> --graph config/realrobot/scene_graph/lab_20260811.json
   ```

3. Run the mission on the stack above:

   ```bash
   python3 -m core.pipeline.mission_tree --target laptop --graph config/realrobot/scene_graph/lab_20260811.json
   ```

   The graph was built in `config/realrobot/map/lab_20260811.yaml`, so Nav2 must
   be localized in that map.

- **Run one Nav2 and no more.** A second stack duplicates every node name, and
  then `bt_navigator`'s action client takes goal replies from the wrong server:
  "unknown goal response, ignoring..." followed by "BtActionNode::Tick: invalid
  status value" and an immediate abort, while the orphaned `controller_server`
  runs on until "Failed to make progress". Check with `ros2 node list | sort |
  uniq -d` before blaming anything else.
- **Tested on hardware so far:** the stack comes up, the map loads, the relay
  and `localize.py` seed the localizer, and the planner returns a path. Nothing
  has driven the base yet, and `launch/search.launch.py` remains
  simulation-only.
- **Footprints:** every graph build fits each furniture piece with the smallest turned rectangle seen from above, so turned pieces keep their yaw. Stray mask points stretch that rectangle and shift every viewpoint with it. Graphs built before this use axis-aligned boxes until rebuilt.

---

## Both

### Mission options

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

### What the mission does at startup

| Step | Details |
| --- | --- |
| **Home arm** | Lift, flex, roll, wrist flex, wrist roll → `[0.0, 0.0, -1.57, -1.57, 0.0]` in 3 s. A rejection, failure or 30 s timeout stops the mission. Logs `[HOME]`, then `[READY] arm home pose reached`. |
| **Ready** | Waits for active Nav2 nodes, nav and head actions, IK, fresh localization, synced RGB-D with TF, and one SAM3 reply (thrown away). Shared 180 s budget; `startup_timeout:=300` for slow machines. |
| Then | The base moves only after Ready passes. Ready does **not** check that your map registration is accurate. |

- **Logs** are labelled by process: `simulation`, `navigation`, `ik`, `move_group`, `mission_tree`. The mission ends with the tree and each step's status.
- **Exit codes:** `0` done, `1` not found, `2` could not park, `3` startup/service error, `4` pick failed, `130` Ctrl+C. The sim stays open afterwards.
- **Watch live:** `py-trees-tree-watcher` in another sourced terminal. It is a plain command, not `ros2 run`.
- **Detections update the graph file.** Use a separate `graph:=/path/to/scenario.json` per demo scenario. The launch never rebuilds it.

### Preview the viewpoints

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

### How search decides

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
| **Parking accuracy** | Nav2's usual tolerance is 25 cm; IK needs about 7.5 cm. The final approach tightens `general_goal_checker` to 0.06 m / 0.11 rad and restores it after. Parked further than 0.08 m → exit `2`. |
| **Arm collisions** | The IK solver checks the whole robot, arm included, against every scene-graph furniture box except the one the object is on or in (graph boxes are solid, so that one would swallow the hand). Real shapes are not modelled: a shelf's open front or a table's legroom count as solid. |

- Head aiming is tested in Gazebo only. Check it before using hardware.
- Viewing tolerances are 0.30 m / 0.30 rad, not grasp tolerances. Poses on cells Nav2's costmap blocks are dropped first. A failed plan tries the next pose; two failed drives move on to the next place.

### If it stops

| Problem | Next step |
| --- | --- |
| Goal acceptance times out | Stop Nav2 before retrying; the goal state is unknown |
| Cancellation cannot be confirmed | Resolve the active goal before restarting |
| Camera, SAM3 or IK error | Fix the service. An error does not mean the object is missing or unreachable. |
| Old map-to-base TF | Check localization and clocks; TF must be under 2 s old |
| No reachable view | Check localization, furniture bounds and the costmap |

### IK solver settings

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
