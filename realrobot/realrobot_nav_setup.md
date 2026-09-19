# Real-robot navigation: setup, parameters, failures

Everything needed to drive the HSR to a scene-graph target, what was changed to
make it work, and how each failure so far was diagnosed. Current as of
**2026-09-18**, lab map `lab_20260811`.

Companion documents:

- [README.md](README.md) — the `realrobot/` tools, and the same run sequence in brief.
- [launch/realrobot/navigation.readme](../launch/realrobot/navigation.readme) — bring-up runbook and hardware measurements.
- [core/navigation/README.md](../core/navigation/README.md) — simulation and real-robot navigation.
- [docker/boxer/README.md](../docker/boxer/README.md) — building the scene graph.

---

## 1. What publishes what

Two stacks share `ROS_DOMAIN_ID=5`. Knowing which owns which frame is the whole
game, because **a TF frame has exactly one parent**.

| Edge | Published by | Where |
| --- | --- | --- |
| `map -> odom` | **`pose_integrator`** (`tmc_pose_integrator`) | the robot |
| `odom -> base_footprint` | the base controller's odometry | the robot |
| `base_footprint -> ...` | `robot_state_publisher` | the robot |
| costmaps, planner, controller, `map_server` | `nav2_real.launch.py` | this PC |

**`laser_2d_localizer` does not broadcast TF.** It publishes a pose, a particle
cloud and `/laser_2d_localizer/score`, and `pose_integrator` turns the pose into
the transform. Read `tmc_pose_integrator/src/pose_integrator_node.cpp`,
`CallbackTfBroadcast_`.

Two consequences that cost time on 2026-09-18:

- **Stopping `laser_2d_localizer` does not stop `map -> odom`.** The broadcast
  is a timer gated on odometry updates, not on localizer messages, so it keeps
  running off the last correction it received — silently frozen, which is worse
  than absent.
- **Giving Nav2 its own global frame does not avoid the conflict.** An AMCL
  publishing `map_nav -> odom` beside `pose_integrator`'s `map -> odom` gives
  `odom` two parents, which is the same broken tree under a new name. To run
  AMCL, `pose_integrator` has to stop. There is no way around it.

`pose_integrator` is a plain `rclcpp::Node` — no lifecycle, so no
`ros2 lifecycle set`. Its global frame is hardcoded to `"map"`
(`kGlobalFrameId`); only `odom_tf_name` and `base_tf_name` are parameters, and
they are read once in `Init()`. So the only lever is stopping the process, on
the robot, through whatever supervisor started it.

Stopping it also removes the `PoseStamped` it publishes, which Toyota's own
tools (`hsrb_interface`'s `omni_base.get_pose()`, their `move_base`) read.

```bash
ros2 run tf2_ros tf2_monitor map odom   # names the broadcaster for that edge
ros2 node info /pose_integrator
```

### The velocity chain

Toyota's `navigation.py` also owns the path from a velocity command to the
wheels, so stopping that launch cuts it:

```text
/base_velocity  -> safety_velocity_limiter  -> command_velocity_autonomy
                -> velocity_switcher        -> omni_base_controller/cmd_vel -> wheels
```

`nav2_real.launch.py` publishes to `/base_velocity` by default, which is the
right place *while Toyota's stack runs*: the limiter keeps the bumper stop and
obstacle slowdown, and the switcher lets teleop override Nav2. With that launch
down, `/base_velocity` has **no subscriber** and Nav2 drives a dead topic —
clean plans, a running controller, a stationary robot, no error anywhere.

```bash
ros2 topic info /base_velocity     # 0 subscribers means exactly this
```

Publish to the base controller instead:

```bash
ros2 launch /home/ws/launch/realrobot/nav2_real.launch.py \
  cmd_vel_topic:=/omni_base_controller/cmd_vel
```

**Confirmed working on hardware, 2026-09-19.** It bypasses both nodes, so there
is no bumper stop, no obstacle slowdown, and no teleop override — the joystick
would fight Nav2 for the same topic. Keep the runstop in reach.

---

## 2. Run sequence

From the repo root, in the dev container, with ROS sourced:

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
```

### 2.1 Scene graph (only when the scan is new)

```bash
bash docker/boxer/build_scene_graph.sh          # GPU host; steps 1-3, ~20 min
```

Writes `config/realrobot/scene_graph/lab_20260811.json` in the map frame.
Details and knobs: [docker/boxer/README.md](../docker/boxer/README.md).

### 2.2 See what is in the graph

```bash
python3 realrobot/offline/plan_overlay.py --survey
```

Every furniture footprint and floor object on the map, labelled with the node ID
you pass to `goto.py`. No ROS, no robot. Read this before blaming navigation:
it shows whether a piece is where you expect and whether the laser ever mapped
the floor around it.

### 2.3 Check the route offline

```bash
python3 realrobot/offline/plan_overlay.py --furniture 107 --start 2.0 -1.5
```

Real `planner_server` and real Theta\* on the saved map with the graph stamped
in, no robot. `--start` is roughly where the robot is; omit it for a random free
cell. Writes `outputs/realrobot/lab_20260811/nav/chair_107.png`.

### 2.4 Seed the localizer — once per boot, before Nav2

```bash
python3 realrobot/live/localize.py
```

Search the whole map. **Do not reach for `--near` unless you are certain of the
guess**: it will happily seed a wrong pose, and a confident wrong pose is worse
than none — everything downstream then agrees with it. On 2026-09-18 a
`--near 2 -1.5` from a bad guess put the base 6.1 m from where it stood, and
every symptom after that was downstream of it.

Check all three:

- `/laser_2d_localizer/score` about **-0.01**, not -0.07.
- median beam-to-wall in single-figure cm, most beams within 10 cm.
- the line comparing the match with what the stack already believes reads
  centimetres, not metres.

### 2.5 Nav2

With Toyota's navigation stack up (the default):

```bash
ros2 launch /home/ws/launch/realrobot/nav2_real.launch.py
```

With it stopped — which is when AMCL is usable, and when `/base_velocity` has
nobody listening:

```bash
ros2 launch /home/ws/launch/realrobot/nav2_real.launch.py \
  localization:=true cmd_vel_topic:=/omni_base_controller/cmd_vel
```

`map_server` + the four servers. `localization:=true` starts AMCL; both
arguments are only right when `pose_integrator` and the velocity chain are down,
and both are wrong when they are up. See §1 for each.

### 2.6 Drive

```bash
python3 realrobot/live/goto.py --furniture 107 --dry-run   # poses and paths only
python3 realrobot/live/goto.py --furniture 107             # drives
```

`--furniture` takes a node ID, instance name or label and must match exactly
one piece. The script rings the piece with observation poses, drops the ones the
live costmap blocks, orders them nearest-first, and drives to the first one
`compute_path_to_pose` can reach — `mission_tree`'s navigation leg without the
reasoner or the grasp. `--look` turns the head to the piece on arrival.

Nothing here moves the arm, so Ctrl-C is safe: `drive_to` catches it, cancels
the Nav2 goal and waits for confirmation before re-raising.

---

## 3. Parameters changed, and why

All in [config/realrobot/nav2/nav2_params_real.yaml](../config/realrobot/nav2/nav2_params_real.yaml).
`git checkout` that file reverts every row.

| Parameter | Was | Now | Why | Status |
| --- | --- | --- | --- | --- |
| `progress_checker.movement_time_allowance` | 10.0 | **30.0** | `SimpleProgressChecker` aborts when the base moves under `required_movement_radius` (0.5 m) within the allowance. MPPI threading a corridor gap is slower than 0.5 m / 10 s, so the goal aborted mid-drive with the base wedged in the gap. | **Confirmed on hardware** 2026-09-18: the same drive completed with 30 s. |
| `cmd_vel_topic` (launch argument) | `/base_velocity` | **`/omni_base_controller/cmd_vel`**, when Toyota's navigation launch is down | `/base_velocity` feeds `safety_velocity_limiter` and `velocity_switcher`, both of which stop with that launch, leaving the topic with no subscriber. See §1. | **Confirmed on hardware** 2026-09-19: the base drives. Costs the bumper stop, obstacle slowdown and teleop override. |
| `GridBased.allow_unknown` | true | **false** | The map is 86% unknown and Theta\* treats unknown as traversable, so it planned straight out of the building across unmapped space. `realrobot/offline/lab_planner.yaml` has had it false since 2026-09-16; the live config never got it. | Applied. Goals in unmapped space now refuse honestly instead of returning a fictional path. |
| `FollowPath.ObstaclesCritic.inflation_radius` | unset → 0.55 (Nav2 default) | **0.35** | MPPI's obstacle critic inverts a cell's cost back into a distance using *its own* copy of the inflation parameters. They must match the inflation layer. | **Untested.** Needs a restart to take effect. |
| `FollowPath.ObstaclesCritic.cost_scaling_factor` | unset → 10.0 (Nav2 default) | **3.5** | Same. On the defaults, a cell at cost 100 reads as `ln(253/100)/10` ≈ 0.09 m of clearance when the costmap means `ln(253/100)/3.5` ≈ 0.27 m — the critic believes every obstacle is ~0.17 m closer than it is. With `critical_weight: 20`, a normal corridor gap then scores as near-collision on every sampled trajectory and MPPI commands zero. | **Untested.** Suspected cause of the base stopping dead in the corridor. |

MPPI critics are **not** dynamically reconfigurable — they read their parameters
at configure time. `ros2 param set` changes the value and not the behaviour, so
these two need a Nav2 restart. Confirm it took:

```bash
ros2 param get /controller_server FollowPath.ObstaclesCritic.inflation_radius   # want 0.35
```

`movement_time_allowance` *is* dynamic, so it can be tried live before being
committed:

```bash
ros2 param set /controller_server progress_checker.movement_time_allowance 30.0
```

---

## 4. Parameters that matter but were left alone

Reach for these next, in this order, if the drive still stalls.

| Parameter | Value | What it buys, what it costs |
| --- | --- | --- |
| `robot_radius` (both costmaps) | 0.3 | The HSR's real radius is smaller. Dropping it widens every gap, at the cost of real clearance. |
| `inflation_radius` (both costmaps) | 0.35 | Lower opens tight gaps. **If you change it, change `ObstaclesCritic` to match** or you reintroduce §3's bug. |
| `behavior_plugins` | `["spin", "wait"]` | **No backup**, deliberately: the laser covers 240°, so reverse is unsensed. A base wedged nose-first cannot self-extract; teleop it out. |
| `vx_min` | 0.0 | No reversing, same reason. |
| `progress_checker.required_movement_radius` | 0.5 | The other half of the abort rule. Lowering it is gentler than raising the allowance further. |
| `GridBased.allow_unknown` | false | Changed on 2026-09-19, see §3. Keep it false: at true, 86% of the map is fair game and the paths are fiction. |
| `global_costmap.track_unknown_space` | true | Unknown stays distinct from free. `base_placement.costmap_grid` treats unknown as blocked when filtering observation poses. |
| `amcl.*` | configured, unused | `OmniMotionModel`, `base_footprint`, `odom`, `scan_topic: scan`, `set_initial_pose: False`. Ready for `localization:=true` once `pose_integrator` is stopped. `localize.py` already publishes `/initialpose`, which is AMCL's seed topic. |

---

## 5. Failure modes seen, and how each was identified

### Every observation pose reports `no path`

Theta\* refuses a plan whose **start** is an obstacle, and then logs *"Either of
the start or goal pose are an obstacle!"* for every goal however clear the goal
is. The symptom looks like an unreachable target and is not.

`goto.py` checks the base's own cell against the live global costmap and says so
before listing poses, with the nearest free cell and its bearing.

Causes, in the order worth checking:

1. **A wrong pose.** A mis-localised base paints `/scan` onto the map where the
   robot is not, and those obstacles land exactly where the stack believes it is
   standing — the start cell is blocked by the robot's own laser. Clearing the
   costmap achieves nothing because the next sweep repaints it. Fix localization
   first, *then* clear.
2. **Stale marks.** With localization sound, the obstacle layer can still hold
   what `/scan` saw earlier:
   ```bash
   ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}
   ```
   Real obstacles come back within a sweep; an old pose's smear does not. Never
   mid-drive.
3. **Genuinely wedged.** Teleop out, then rerun.

A failed `drive_to` can leave the base a few centimetres deeper in than it
started, which is why a run that planned once can refuse every pose on the next
try.

### A view pose stares through a wall

`standoff.candidates` rings a piece geometrically and filters only against other
furniture boxes and free cells; `standoff.sees()` checks bearing and range, not
occlusion. So a pose on the far side of a wall passes both and the mission would
drive there to look at nothing. `plan_overlay.py` has tested this offline since
2026-09-16 (`--sight-line`, on by default); `core/` still does not.

`goto.py` now runs the same test against the **live costmap** before Nav2 is
asked for a path, where unknown counts as opaque:

```text
[sight] 5 of 8 poses look at desk through a wall or unknown space; dropped
```

`core.pipeline.actions.plan` remains unfixed, so `mission_tree` still has the
gap.

### It drove partway and stopped

Not a connection loss if `goto.py` printed a start pose and returned promptly:
`robot_xy()` rejects TF older than 2 s and raises, and `drive_to` waits the full
`--timeout` before giving up. Rule the link out with
`python3 realrobot/live/dds_check.py` and `ros2 topic hz /scan`.

The usual sequence is §3's first row: MPPI slows in a tight gap, the progress
checker fires, the BT spins and waits, the goal aborts, and the base is left in
the gap where its own cell is inside the inflation — so every following plan
fails too, and it reads as the previous failure mode.

`goto.py` reports where it stopped, how far it got, whether that cell is now
blocked and which way the nearest free one lies. The reason itself is only in
the `controller_server` log.

### Stopped and commanding nothing

```bash
ros2 topic echo /base_velocity --once
```

`/base_velocity` is Nav2's `cmd_vel` after the launch's remap — it keeps
`safety_velocity_limiter` and `velocity_switcher` between Nav2 and the wheels.
Near-zero twists with a goal still active is MPPI refusing to move, which is
what §3's `ObstaclesCritic` mismatch produces. Nothing published at all means
the goal already aborted.

### The costmaps pull apart

The local costmap is in `odom`, the global in `map`, joined only by
`map -> odom`. This edge is known to be unreliable on this robot: it wandered by
tens of metres during bag recording, which is why
[dataprep/tf_filter.py](dataprep/tf_filter.py) drops `map` from the bag's `/tf`
and the offline pipeline runs its own `slam_toolbox`.

```bash
python3 realrobot/live/localize.py --watch
```

Scores the live `/scan` against the map at the pose the stack believes, every
2 s, and prints `map -> odom` beside it. Searches nothing, publishes nothing,
safe to leave running during a drive. A median climbing as the base moves while
`map -> odom` never changes is the localizer not correcting.

Switching to AMCL is the structural fix. It is a launch argument and the
parameters are ready; the blocker is §1.

---

## 6. Status

| | |
| --- | --- |
| Confirmed on hardware | `movement_time_allowance: 30.0`; `localize.py` global search finding a good pose (5 cm median, 75% of beams within 10 cm); `goto.py` resolving a target, planning and driving; `plan_overlay --survey`; the offline route to chair (107), 16.3 m. |
| Changed, not yet tested | `ObstaclesCritic.inflation_radius` / `cost_scaling_factor`. |
| Open | Whether AMCL replaces the robot's localizer — needs `pose_integrator` stopped on the robot. `mission_tree.py:236` builds `Navigator()` with the default `use_sim_time=True`, which raises *"No fresh map-to-base TF"* on hardware; `goto.py` passes `False` explicitly, `mission_tree` still needs the same fix. |
