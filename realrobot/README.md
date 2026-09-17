# `realrobot/`

Everything run against the HSR lab bag of 2026-08-11.

Grouped by what each script talks to: the bag, the recording, or the robot.

| Folder | Talks to | What it does |
| --- | --- | --- |
| [`dataprep/`](dataprep/) | the bag | bag → Nav2 map → keyframes → Boxer boxes → scene graph. Steps 1–8 and their checks: [dataprep/realrobot_dataprep.md](dataprep/realrobot_dataprep.md) |
| [`offline/`](offline/) | the recording | the finished map, graph and keyframes, no robot needed: path overlays, footprint checks, next-best-view replay |
| [`live/`](live/) | the real HSR | seed its localizer so Nav2 can drive; check what a grasp needs before anything moves |

---

## `realrobot/live/`

| File | Does |
| --- | --- |
| `localize.py` | seeds the robot's localizer by matching the live `/scan` to the map, so Nav2 can plan. Run it **before** Nav2 |
| `grasp_preflight.py` | checks the robot, vision transport RX and the model servers answer, before a pick moves anything |

Bring-up, the measurements and the failure modes:
[launch/realrobot/navigation.readme](../launch/realrobot/navigation.readme).

---

## `realrobot/offline/`

`plan_overlay.py` and `footprint_check.py` read the two artefacts step 1 and step 7 leave behind,
`config/realrobot/map/lab_20260811.{pgm,yaml}` and
`config/realrobot/scene_graph/lab_20260811.json`, and need nothing else running.

| File | Does |
| --- | --- |
| `plan_overlay.py` | query a target in the scene graph, ask Nav2 for a path to it, draw the path on the map |
| `footprint_check.py` | measure the scene graph's footprints against the laser map |
| `planner.launch.py` | `map_server` + `planner_server` alone, no robot. `plan_overlay.py` starts it itself |
| `lab_planner.yaml` | their parameters, taken from `config/nav2/nav2_params.yaml` minus the obstacle layer |
| `nbv_replay.py` | replay recorded keyframes through the next-best-view policy and score what it would have chosen |
| `target_tsdf.py` | keyframes + a text prompt → a TSDF cube around the detected target |

`plan_overlay.py` needs ROS sourced; `footprint_check.py` does not.

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

### Commands run on 2026-09-16, and what each one wrote

Every figure below is reproducible from the command next to it. Each run takes
about 40 s, most of it Nav2 coming up.

| # | Command | Generates |
| --- | --- | --- |
| 1 | `python3 realrobot/offline/footprint_check.py` | stdout only |
| 2 | `python3 realrobot/offline/plan_overlay.py --target laptop --seed 2` | `outputs/realrobot/lab_20260811/nav/laptop.png` · `nav/lab_20260811_blockers.{pgm,yaml}` |
| 3 | `python3 realrobot/offline/plan_overlay.py --furniture 8 --seed 4 --no-blockers --output outputs/realrobot/lab_20260811/nav/desk_8_laser_only.png` | `outputs/realrobot/lab_20260811/nav/desk_8_laser_only.png` |
| 4 | `python3 realrobot/offline/plan_overlay.py --furniture 4 --start -0.45 -4.7 --no-blockers --output outputs/realrobot/lab_20260811/nav/desk_4_laser_only.png` | `outputs/realrobot/lab_20260811/nav/desk_4_laser_only.png` |
| 5 | `python3 realrobot/offline/plan_overlay.py --furniture 4 --start -0.45 -4.7 --output outputs/realrobot/lab_20260811/nav/desk_4_with_blockers.png` | `outputs/realrobot/lab_20260811/nav/desk_4_with_blockers.png` |
| 6 | `python3 realrobot/offline/plan_overlay.py --furniture 8 --seed 4` | nothing: prints why every pose around desk (8) is unusable |

- 4 and 5 are the same start and the same target on the two maps, so the paths can be compared. 3 and 6 are the same query, and only the laser-map run has anywhere to stand.
- `--seed` is reproducible for a fixed map, `--clearance` and `--blockers`; change any of them and the same seed picks a different start, because the set of free cells it draws from changes.
- `nav/lab_20260811_blockers.{pgm,yaml}` is rewritten by every `--blockers` run. It is a derived file under `outputs/`, not a map to drive on.

### Options

| Option | Default | Use it to |
| --- | --- | --- |
| `--target <object>` | — | query an object: `laptop`, `coffee maker`, `trash can` |
| `--furniture <name\|id>` | — | query a piece of furniture; the name has to be unique, so `8` rather than `desk` |
| `--start X Y` | random | fix the start instead of drawing one |
| `--seed N` | `0` | pick a different random start |
| `--min-distance M` | `4.0` | how far the random start must be from the target |
| `--attempts N` | `6` | random starts to try before giving up |
| `--clearance M` | `0.3` | free space a pose needs; the `global_costmap` `robot_radius` |
| `--no-blockers` | off | plan on the laser map alone, without the scene graph stamped in |
| `--no-sight-line` | off | keep poses with a wall between them and the target |
| `--full-map` | off | draw all 50 × 28 m instead of the area around the path |
| `--graph`, `--map`, `--output`, `--domain-id` | lab bag, `91` | point at another run |

### How it works

1. **Resolve the query.** `core.pipeline.actions.targets` turns `--target`/`--furniture` into a `Location`. An object already in the graph is answered from memory, so no `DEEPSEEK_API_KEY` is needed.
2. **Turn it into a goal.** `core.pipeline.actions.plan` rings the furniture with observation poses and orders them — the same call `mission_tree` drives. The first pose that survives the filters becomes the Nav2 goal, so the picture shows the leg the mission would ask for, not a straight line to the object.
3. **Plan.** `ComputePathToPose` with `use_start`, because there is no robot and no localisation. A path that stops more than 10 cm short of the goal counts as a refusal, the same check `core.navigation.nav2_client.reachable` makes.
4. **Draw.** The laser map underneath, the scene graph over it, the path on top.

`planner.launch.py` parks a static `map -> base_footprint` transform at the origin only because `Costmap2DROS` will not activate without one. Nothing reads that pose. The stack runs on `ROS_DOMAIN_ID=91` so a sim or a mission on the devcontainer's domain 5 cannot answer these goals.

### What the figures show

**Bounding boxes.** Run 1 measures every furniture footprint against the laser map:

```
11 furniture pieces, mean occupied 10%, free 80%, unknown 10%
walls within 6 m of the furniture run at 22° mod 90 (664 fitted cells);
8 of 11 footprint yaws sit within 10° of that, median 16°
```

- **Placement is good.** Ten of eleven pieces touch an occupied cell (`edge` 0.00); only chair (6) floats, 18 cm off. Two independent sensors — RGB-D at head height and a laser at 19 cm — agree on where things are.
- **Orientation is good.** The room is not square to the map; it is turned about 22°. The footprint fit recovered that from the point clouds alone, and 8 of 11 pieces land within 10° of it. The two chairs that do not are round-ish, where a fitted rectangle has no strong angle to find.
- **80% of the furniture area is free floor in the PGM.** That is the point of stamping them in: the laser sees a desk as four thin legs, so a base planning on the laser map alone drives through the middle of one.
- **Objects standing on the floor are a different story.** All three (trash can, bin, computer tower) span the 19 cm laser plane, and the map already has 35% of their area occupied. Stamping them in sharpens a box the laser saw one side of; it does not add something new.

**Where the scene graph changes the path.** Runs 4 and 5, same start and goal: 5.88 m on the laser map, 5.91 m with the graph stamped in. Around desk (4) the two maps barely differ, because the wall behind it already forces the same route. The graph earns its place on the poses it rules out, not on the metres it adds.

**A goal the geometry gets wrong.** Run 6, desk (8): `standoff.candidates` rings a piece at 0.8 m and filters only against other furniture boxes and free cells. Desk (8) stands against a wall and its fitted box overhangs it by about 0.3 m, so most of the ring lands in the corridor on the far side — free cells that Nav2 happily reaches, from which the robot would be looking at the desk through a wall. `plan_overlay.py` adds a sight-line test on the occupancy grid that `core/` does not have, which drops those poses. What is left:

| Map | Poses clear of the map | Of those, in sight of desk (8) | Result |
| --- | --- | --- | --- |
| laser only (run 3) | 8 | 1, at (1.68, −1.63) | robot enters the room and parks north of the desk |
| scene graph stamped in (run 6) | 7 | 0 | nothing to drive to |

The one good pose has 0.47 m of clearance on the laser map and 0.29 m once office chair (11) and desk (8) are stamped in — a centimetre under the 0.30 m the base needs. So with the graph as blockers, desk (8) is genuinely unreachable for viewing, and run 6 says so instead of parking in the corridor. Rooms would not have fixed this on their own: both the good pose and the bad ones are in the same room as the desk. Two things would: a sight-line or same-enclosure test (this script's version is the crude one), and a tighter box on desk (8).

### Known limits

- The sight line is a ray to one aim point on the target, not a visibility test over the whole surface, and it ignores the head's field of view and tilt.
- `SKIN = 0.15 m` around the target decides which occupied cells are the target's own laser return. Furniture whose box is off by more than that reads its own wall as an obstruction.
- Unknown cells count as blocked, matching `global_costmap`'s `track_unknown_space`. 86% of this map is unknown, so a target in an unscanned corner has nowhere to stand: bookshelf (40) fails this way, and the script prints the breakdown.
- `allow_unknown: false` in `lab_planner.yaml` differs from the simulation stack, which allows it. A path through unmapped space would not mean anything here.
- The stack has no obstacle layer, because there is no live `/scan`. The saved map is all it knows.
