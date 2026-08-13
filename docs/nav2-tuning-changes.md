# Nav2 tuning changes

Change-and-reason log for
`ros2_ws/src/hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml`.

**This file exists because that config is untracked.** `.gitignore:19` ignores
`ros2_ws/src/*`, so `git log` on the params file returns nothing and there is
no diff to review. If you change nav2 tuning, add a row here or the change
becomes invisible the moment you forget it.

## 2026-08-13 — NavFn/DWB to Smac/MPPI

| Change | Reason |
|---|---|
| `planner_server.GridBased.plugin`: `nav2_navfn_planner/NavfnPlanner` → `nav2_smac_planner/SmacPlanner2D` | Real A*. NavFn's own `use_astar: true` branch produces kinked, visibly suboptimal paths — Smac is the maintained A* |
| Not `SmacPlannerHybrid` | Hybrid-A* models an Ackermann turning radius the HSR's omni base does not have |
| Added `cost_travel_multiplier: 2.0` | >1 prefers low-cost open cells over the shortest route, keeping approaches out of the inflation skirt |
| Added `use_final_approach_orientation: false` | `standoff.candidates()` sets a yaw that faces the furniture centroid; that yaw is the whole point, so honour the goal's own orientation |
| Added `max_planning_time: 2.0` | `reachable()` plans up to 12 candidates per furniture *before* driving, so a hung plan costs 12x what it looks like |
| Kept `tolerance: 0.5` (unchanged from NavFn) | Pre-existing caveat, now documented: a returned path may end up to 0.5 m short, so `reachable()` can green-light a pose that is only reachable to within half a metre |
| `controller_server.FollowPath.plugin`: `dwb_core::DWBLocalPlanner` → `nav2_mppi_controller::MPPIController` | DWB was configured `max_vel_y: 0.0` — a differential-drive controller on a holonomic base |
| Added `motion_model: "Omni"`, `vy_max: 0.2` | Uses the base as it actually is. Robot strafes onto a standoff pose instead of turn-drive-turn; also the mechanism the grasp reposition in `navigation-and-grasping.md` Part 3 needs |
| `min_y_velocity_threshold`: `0.5` → `0.001` | **Latent bug.** The old value was above `vy_max`, so every lateral command MPPI produced would be floored to zero. Omni would have looked broken rather than disabled |
| Omitted `PreferForwardCritic` | It penalises non-forward motion — exactly the strafing Omni was enabled for |
| Added `TwirlingCritic` | The omni counterpart: stops needless rotation while translating |
| `PathAngleCritic.forward_preference: false` | Same reason — any heading may approach a path point on an omni base |
| `ObstaclesCritic.inflation_radius/cost_scaling_factor` set to `0.5`/`3.0` | Must mirror `local_costmap`'s inflation layer, or the critic reasons about a different inflation than the costmap it reads |
| `vx_min: 0.0` (no reverse) | The base laser is not 360°, so backing up is blind. On an omni base strafing gives the same escapes with sensing in front of them. Flip to ~`-0.15` if MPPI gets stuck |
| `visualize: false`, `regenerate_noises: false` | Publishing candidate trajectories and resampling noise every iteration are both avoidable cost |

### Verified

YAML parses; both plugin lookup names resolve against the installed
`.xml` exports; `model_dt` equals `1 / controller_frequency`; critic inflation
matches the local costmap. Parameter names were read out of the installed
Humble libraries, not from memory — this build is a recent sync (it has
`CostCritic` and `regenerate_noises`), and its `PathAngleCritic` takes
`forward_preference`, not the older `mode`.

### Not verified

**Never run.** Needs the sim stack up. Watch whether MPPI holds 20 Hz: Humble
ships the xtensor implementation, not the later Eigen rewrite, and it shares a
container with Gazebo. Trim `batch_size` 2000 → 1000 first, then `time_steps`
56 → 40. Both cost path quality, so check the actual frequency before trimming
on suspicion.

## 2026-08-13 — goal tolerance, for the grasp reposition

| Change | Reason |
|---|---|
| `general_goal_checker.xy_goal_tolerance`: `0.25` → `0.10` | The arm's usable reach band is ~0.20 m wide (`core/navigation/reach.py`). Aiming mid-band leaves ~0.10 m of budget, so a controller permitted to stop 0.25 m out misses the band routinely and every grasp candidate is then rejected as out of reach. Observed: robot parked 0.67 m from a can it had just detected, `{'too_wide': 56, 'out_of_reach': 44, 'kept': 0}` |
| `general_goal_checker.yaw_goal_tolerance`: `0.25` → `0.10` | The `arm` group has five joints and a pose has six degrees of freedom; the missing one is base heading. Yaw error is not absorbed anywhere, it goes straight into an unreachable grasp |
| `nav2_client.GOAL_TOLERANCE`: `0.45` → `0.25` | The arrival guard has to stay looser than Nav2's own tolerance or it rejects honest arrivals, but 0.45 was loose enough to accept arrivals useless for grasping. Tracks the new 0.10 with margin |

**Retrying is not a substitute for this.** `reach.move_into_reach` now drives up
to three times, re-checking `reachable_from` after each. That helps, but once
the remaining travel is under the controller's tolerance Nav2 considers itself
arrived and stops moving — so with a 0.25 m tolerance the loop stalls outside a
0.20 m band no matter how many attempts it gets. `tests/test_reach.py`
asserts the relationship rather than leaving it as folklore.

