# Bag → scene graph → mission

End-to-end run instructions. Worked example `lab_20260919_wheel` (bag `bags/lab_20260919`),
verified 2026-09-19 on the HSR. A new recording uses one name for both.

Diagnostics: [TROUBLESHOOTING.md](TROUBLESHOOTING.md) · Changes: [fixes/](fixes/)

## 0. Set the recording

```bash
cd /home/ws
export RECORDING=lab_20260919_wheel
printenv RECORDING            # NOT echo — echo prints unexported vars too
```

One variable moves every path ([recording.py](core/utils/recording.py)). **Export it in every
terminal**: a stale value silently reports "already done" for the wrong recording.

| Path | Step | Holds |
| --- | --- | --- |
| `bags/<rec>/` | 1 | raw bag, 2 GB parts + `metadata.yaml` |
| `config/realrobot/map/<rec>/map.{pgm,yaml}` | 2 | laser map (AMCL seed, `localize.py`) |
| `config/realrobot/map/<rec>/map_furniture.{pgm,yaml}` | 6 | **the map Nav2 serves**: laser map + fixed furniture |
| `config/realrobot/map/<rec>/scan_to_map.json` | 3 | keyframe→map (identity) |
| `config/realrobot/map/<rec>/boxer_to_map.json` | 5 | undoes Boxer re-centring |
| `config/realrobot/scene_graph/<rec>.json` | 6 | the graph |
| `outputs/realrobot/<rec>/slam/` | 2 | `tf_mapping/`, `posegraph.*`, `odometry` (`wheel`\|`laser`, read by step 3) |
| `outputs/realrobot/<rec>/keyframes/` | 3 | `color/ depth/ poses/ scene.ply` |
| `outputs/realrobot/<rec>/boxer/` | 5 | ScanNet seq + fused CSVs |
| `outputs/realrobot/<rec>/nav/` | 8 | plan + survey PNGs |

---

## 1. Record · on the robot

```bash
ssh administrator@10.7.3.185
tmux new -s scan                          # ssh drop kills the bag otherwise
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=5
cd /path/to/disk && df -h .               # ~1 GB/min at 30 Hz

ros2 bag record -o lab_20260919 \
  --max-bag-size 2000000000 --max-cache-size 67108864 \
  /scan /tf /tf_static \
  /head_rgbd_sensor/rgb/image_rect_color \
  /head_rgbd_sensor/depth_registered/image_rect_raw \
  /head_rgbd_sensor/rgb/camera_info \
  /omni_base_controller/wheel_odom /robot_description /imu/data
```

Detach `Ctrl-B d` · reattach `tmux attach -t scan`

| Topic | Needed | Note |
| --- | --- | --- |
| `/scan` `/tf` `/tf_static` | **yes** | slam_toolbox + keyframe poses. `/tf` carries `odom→base_footprint_wheel`, which `WHEEL_ODOM=1` needs |
| `rgb/image_rect_color` | **yes** | |
| `depth_registered/image_rect_raw` | **yes** | |
| `rgb/camera_info` | **yes** | first message only |
| `wheel_odom` `robot_description` `imu/data` | no | diagnostics |
| `depth_registered/rectified_points` | **never** | 80 of 87 GB, nothing reads it |

| Flag | Why |
| --- | --- |
| `--max-bag-size 2000000000` | 2 GB parts → resumable rsync |
| `--max-cache-size 67108864` | 64 MB ≈ 4 s of buffer |
| no compression | the reader opens `sqlite3` only, not zstd |

**Wait 5 s before moving** — `/tf_static` and `camera_info` are latched.

Confirm every topic attached — file growth alone will not tell you:

```bash
for t in /scan /tf /tf_static \
         /head_rgbd_sensor/rgb/image_rect_color \
         /head_rgbd_sensor/depth_registered/image_rect_raw \
         /head_rgbd_sensor/rgb/camera_info; do
  printf '%-52s %s\n' "$t" "$(ros2 topic info "$t" | grep -i 'Subscription count')"
done
```

All must read **1+**. Healthy growth ≈ **15 MB/s** (9 RGB + 6 depth).

### Driving rules

| Rule | Number | Why |
| --- | --- | --- |
| Speed | ≤ 0.3 m/s, feather yaw | fast rotation breaks a 2D scan matcher |
| Loop closure | re-enter within **3 m** | `loop_search_maximum_distance` |
| Dwell at revisits | 3–4 s | needs ≥ 3 scans |
| Corridors | middle, dwell at doors/alcoves/pillars | parallel walls give no longitudinal lock — build with `WHEEL_ODOM=1` (§2–3) |
| Return leg | retrace the same line | else no closure fires |
| Each room | 2–3 laps back to one anchor | keyframe poses are the **online** estimate |
| Objects | 0.8–3 m, two directions | fusion needs conf ≥ 0.55 **and seen ≥ 4×** |
| Beyond 4 m | invisible | TSDF `--max-depth` |
| Head sweep | base **stopped** | blur + pose coupling |
| People / doors | out / fixed | moving obstacles bake into the map |

### Verify, then pull

```bash
ros2 bag info ~/lab_20260919              # no metadata.yaml? ros2 bag reindex
# check: /tf_static ≥ 1, camera_info present, RGB≈depth within 1%

# workstation. /home/ws is a host bind mount — costs nothing in Docker size.
rsync -aPz administrator@10.7.3.185:/home/administrator/lab_20260919 /home/ws/bags/
du -sb bags/$RECORDING                    # must equal the robot's
ros2 bag info bags/$RECORDING             # counts must match
```

---

## 2–6 in one command · GPU **host**

```bash
cd ~/dora-ovmm                            # the same folder the container sees as /home/ws
export DEEPSEEK_API_KEY=<key>             # for --rooms
mkdir -p outputs/realrobot/$RECORDING/logs
WHEEL_ODOM=1 bash docker/boxer/build_scene_graph.sh \
  --bag bags/$RECORDING --rooms --nav-map $RECORDING \
  2>&1 | tee outputs/realrobot/$RECORDING/logs/build.log
```

| Step | Runs in | Needs |
| --- | --- | --- |
| 0 map + keyframes, gate % printed | dev container (`docker exec`) | ROS |
| 1 keyframes → Boxer layout | dev container | numpy |
| 2 detect → fuse | host, `docker run --rm` | Docker, GPU |
| 3 scene graph + rooms | dev container | networkx, `DEEPSEEK_API_KEY` |
| 4 `map_furniture.yaml` (`--nav-map`) | dev container | OpenCV |

The dev container must be running. It is found by VS Code's label, so rebuilds are fine:
`docker ps --filter "label=devcontainer.local_folder=$PWD" --format '{{.Names}}'` must print
one name; else `DEV_CONTAINER=<name>`. Nothing is stopped or removed: Boxer's container goes
with `--rm`, its image stays (long build). Finished steps are skipped on rerun.

Finished steps are skipped, so a rerun only does what is missing. To force one:

| Add | Redoes | Time (13 min bag) |
| --- | --- | --- |
| nothing | detect → fuse → graph → rooms → nav map. Map and keyframes kept | ~3 min |
| `--graph-only` | graph → rooms → nav map, from the fused boxes already there. No GPU | ~10 s |
| `--fuse-only` | fuse → graph → … from the saved detections | ~30 s |
| `--re-extract` (needs `--bag`) | keyframes and everything after. Map kept | ~8 min |
| `--re-map` (needs `--bag`) | **the map** and everything after: keyframe poses hang off the map's SLAM run | ~25 min |
| `--rate N` | replay speed for the map step, default `1` (40 Hz `/scan`) | |

**A rebuilt graph replaces `config/realrobot/scene_graph/$RECORDING.json`**: objects the robot
added live and DeepSeek's cached picks are gone. Keep a copy first:
`cp config/realrobot/scene_graph/$RECORDING.json{,.bak}`

Test the command without touching the map (rooms re-clustered and renamed, ~3 min):

```bash
cd ~/dora-ovmm && export RECORDING=lab_20260919_wheel DEEPSEEK_API_KEY=<key>
cp config/realrobot/scene_graph/$RECORDING.json{,.bak}
bash docker/boxer/build_scene_graph.sh --rooms --nav-map $RECORDING \
  2>&1 | tee outputs/realrobot/$RECORDING/logs/rebuild.log
# expect: "ROS and Python steps run in the dev container: <id>", Step 1 already done,
#         Step 2 detect, Step 3 "3 rooms … 39 furniture, 111 objects …", Step 4 "[map] fixed furniture stamped …"
# no "Step 0" line = the map was not rebuilt. Run on the host 2026-09-19: ~3 min, rooms kitchen / living room / office.
# Boxer writes as root: sudo chown -R "$(id -u):$(id -g)" outputs/realrobot/$RECORDING/boxer
```

Sections 2–6 below are the same steps by hand. Worked example only: its bag is
`bags/lab_20260919`, so pass `--bag bags/lab_20260919` wherever `bags/$RECORDING` appears.

---

## 2–3. Map + keyframes · container, no GPU

```bash
source /opt/ros/humble/setup.bash
mkdir -p outputs/realrobot/$RECORDING/logs
WHEEL_ODOM=1 bash realrobot/dataprep/prepare.sh --bag bags/$RECORDING --rate 1 $RECORDING \
  2>&1 | tee outputs/realrobot/$RECORDING/logs/prepare.log
cat outputs/realrobot/$RECORDING/slam/odometry      # must print: wheel
```

**`--rate 1` when `/scan` is 40 Hz.** Rate 2 = 80 Hz into slam_toolbox → dropped scans.

**`WHEEL_ODOM=1` always.** SLAM is fed distance from the wheels and heading from the robot's
laser odometry ([tf_filter.py](realrobot/dataprep/tf_filter.py) `wheel_odometry`). Neither SLAM
nor the robot's default laser odometry can see motion along a corridor, so without it the
corridor by the office mapped **2.4 m short** and AMCL sat 2–4 m off, in either direction.

| Built on | map ÷ wheel distance, corridor by office | elsewhere | gate (§4) |
| --- | --- | --- | --- |
| laser odom (default) | 0.60 · 0.86 · 0.74 | 1.00–1.02 | 75.2% |
| wheels only | 1.01 | 1.00–1.03 | 58.6% — wheel yaw drifts 70° over the run |
| **wheel distance + laser heading** | **1.01 · 1.00 · 1.03** | 0.99–1.03 | **77.6%** |

| Flag | Default | Does |
| --- | --- | --- |
| `--bag PATH` | `bags/playable_bag` | bag to read |
| `--rate N` | `2` | replay speed |
| `--re-map` | off | redo step 2 |
| `--re-extract` | off | redo step 3 |

Reruns skip finished steps. Time ≈ bag length at `--rate 1`, plus ~2 min / 22k frames.

`extract_rgbd.py` knobs, if called directly:

| Flag | Default | |
| --- | --- | --- |
| `--min-translation` | `0.10` m | keyframe gate, distance |
| `--min-rotation` | `5.0` ° | keyframe gate, angle |
| `--pair-tolerance` | `0.035` s | max RGB/depth stamp gap |
| `--max-depth` | `4.0` m | fused into TSDF |
| `--voxel` | `0.02` m | TSDF voxel |

---

## 4. Gate the map · before any GPU time

```bash
cd realrobot/dataprep && python3 visualize.py \
  --scene ../../outputs/realrobot/$RECORDING/keyframes \
  --map ../../config/realrobot/map/$RECORDING/map.yaml \
  --slam-tf ../../outputs/realrobot/$RECORDING/slam/tf_mapping \
  --bag ../../bags/$RECORDING --figures figures_$RECORDING
cd /home/ws
```

| Check | Pass |
| --- | --- |
| `map_trajectory.png` | no doubled or sheared corridor — **else re-record** |
| `checks.json` | ≥ 80% of laser-height points within 10 cm (`lab_20260919_wheel`: 77.6%, accepted) |

The one-command build prints this number and carries on; it does not stop on it.

---

## 5–6. Scene graph · GPU **host** (no docker in the container)

```bash
cd ~/dora-ovmm
export DEEPSEEK_API_KEY=<key>             # else rooms are numbered, not named
nvidia-smi                                # card busy? add --cpu
RECORDING=$RECORDING bash docker/boxer/build_scene_graph.sh --rooms --nav-map \
  2>&1 | tee outputs/realrobot/$RECORDING/logs/boxer.log
```

Weights once (1.2 GB): `bash docker/boxer/run_boxer.sh --download-ckpts`

| Flag | Does |
| --- | --- |
| `--stride N` | keep every Nth keyframe — **main speed lever** |
| `--cpu` | ~4 s/frame instead of ~0.12 |
| `--rooms` | cluster + name rooms |
| `--min-confidence X` | drop boxes below X |
| `--no-video` / `--no-compile` | skip mp4 / skip Triton build |
| `--fuse-only` / `--graph-only` | re-fuse after Ctrl-C / step 6 alone |
| `--re-prepare` | rebuild the ScanNet sequence |
| `--bag PATH` | run steps 2–3 first too |
| `--nav-map` | write `map_furniture.yaml` after the graph |

891 keyframes on an RTX 4080 SUPER: detect ~90 s, fuse ~15 s, graph ~1 s.

To make the map hold a word it does not yet know, add it under `object:` in [scannet200.yaml](config/scene_graph/scannet200.yaml) and rerun this step — no re-record, no re-SLAM. Details: [docker/boxer/README.md](docker/boxer/README.md#the-vocabulary-is-a-prompt-list-you-control).

The host has no ROS, OpenCV or networkx: the script runs those steps in the dev container
by itself (see the table above). Graph step alone, from the container:
`bash docker/boxer/build_scene_graph.sh --graph-only --rooms --nav-map $RECORDING`

Boxer writes as root: `sudo chown -R "$(id -u):$(id -g)" outputs/realrobot/$RECORDING/boxer`

---

## 7. Rooms

Rooms are **geometric**; DeepSeek only names them ([rooms.py](core/scene_graph/rooms.py)).

```bash
python3 -m docker.boxer.to_scene_graph \
  --boxes outputs/realrobot/$RECORDING/boxer/$RECORDING/boxer_3dbbs_fused.csv \
  --transform config/realrobot/map/$RECORDING/boxer_to_map.json \
  --output /dev/null --room-sweep \
  --map config/realrobot/map/$RECORDING/map.yaml
```

Prints cluster count vs spacing → pick one → re-emit with `--room-spacing <v> --rooms`.

| Flag | Does |
| --- | --- |
| `--room-sweep` | print counts vs spacing, stop |
| `--room-spacing M` | floor metres where two pieces stop being one room |
| `--rooms` + `--map` | divide and name |

`lab_20260919_wheel`: 150 nodes · 94 edges · office 75 / kitchen 44 / living room 14 / none 17.

**Ids change with every rebuild** (detection is not bit-repeatable: 176 fused boxes one run, 175
the next; the pieces stay put to the centimetre). Look them up, never remember them:

```bash
python3 -c "
import json, os; g = json.load(open(f'config/realrobot/scene_graph/{os.environ[\"RECORDING\"]}.json'))
[print(n['id'], n['label'], n.get('room'), [round(v, 1) for v in n['centroid'][:2]]) for n in g['nodes'] if not n['movable']]"
```

2026-09-19 build: kitchen desk **30**, office table **93**, couch **66**.

---

## 8. Inspect + offline nav

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash

python3 visualization/scene_graph.py \
  --graph config/realrobot/scene_graph/$RECORDING.json \
  --scene outputs/realrobot/$RECORDING/keyframes/scene.ply \
  --output outputs/realrobot/$RECORDING/scene_graph.rrd

python3 realrobot/offline/footprint_check.py        # boxes vs laser map, no ROS
python3 realrobot/offline/plan_overlay.py --survey  # every footprint, coloured by room
python3 realrobot/offline/plan_overlay.py --nav-map # writes map_furniture.yaml (rerun after a graph rebuild)
python3 realrobot/offline/plan_overlay.py --furniture 30 --start -1.0 0.5
```

| Flag | Default | Does |
| --- | --- | --- |
| `--survey` | — | whole graph, no planning, no ROS |
| `--nav-map` | — | laser map + fixed furniture → `map_furniture.yaml`. No chairs, no movable floor objects |
| `--furniture NAME\|ID` | — | drive-to target |
| `--target LABEL` | — | object → plans to its furniture |
| `--start X Y` | random | **use it**, else runs aren't comparable |
| `--clearance M` | `0.3` | free space a pose needs = `robot_radius` |
| `--no-blockers` | stamped | laser map only |
| `--no-sight-line` | on | keep poses looking through a wall |
| `--seed` `--attempts` `--min-distance` | `0` `6` `4.0` | random-start control |
| `--full-map` | off | draw all 50 × 28 m |
| `--graph` / `--map` | from `$RECORDING` | both or neither |
| `--output PNG` | `outputs/realrobot/<rec>/nav/<query>.png` | |

---

## 9. Real robot · bring-up

Six terminals. **Order matters twice:** Nav2 before `localize.py`, and everything before the
mission. Terminals 3–5 can start any time before terminal 6.

```bash
# --- terminal 1 · Nav2 --------------------------------------------------------
cd /home/ws && export RECORDING=lab_20260919_wheel
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch launch/realrobot/nav2_real.launch.py \
  map:=/home/ws/config/realrobot/map/$RECORDING/map_furniture.yaml \
  cmd_vel_topic:=/omni_base_controller/cmd_vel \
  localization:=true
# wait for: "AMCL cannot publish a pose ... Please set the initial pose"
# and for:  "[ODOM] Nav2 and AMCL are on wheel odometry"   <- do not drive without it
```

What the launch starts besides Nav2:

| Piece | Why | If missing |
| --- | --- | --- |
| [use_wheel_odom.py](realrobot/live/use_wheel_odom.py) | the robot boots on laser odometry, which logged 6.4 m of a 13.5 m drive | 2–4 m pose error along corridors |
| `scan_filter` → `/scan_filtered` ([scan_filter.yaml](config/realrobot/nav2/scan_filter.yaml)) | one beam straddling a door jamb reads as an obstacle in mid-air; both costmaps read the filtered scan, AMCL the raw one | a doorway stays closed however often the costmap is cleared |
| `map_furniture.yaml` | on the laser map a table is four legs; `sg.blockers` only filters goal poses, never Nav2's path | paths cross furniture, the hand hits the table |

**The robot's stack restarting puts it back on laser odometry.** With Nav2 already up:
`python3 realrobot/live/use_wheel_odom.py` (`… laser_odom` switches back, to compare the two).

```bash
# --- terminal 2 · seed AMCL, AFTER terminal 1 is up --------------------------
cd /home/ws && export RECORDING=lab_20260919_wheel
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 topic info /initialpose          # Subscription count must be 1 first
python3 realrobot/live/localize.py
python3 realrobot/live/localize.py --dry-run   # "… 0.06 m and 1 deg from this match" = good
ros2 run tf2_ros tf2_echo map odom    # must stream = AMCL is publishing
```

`--dry-run` publishes nothing and is the pose check at any time: **centimetres** is healthy,
**metres** means relocalize and clear both costmaps. `--watch` logs it while driving.

```bash
# --- terminal 3 · collision-aware IK -----------------------------------------
ros2 launch /home/ws/launch/ik_solver.launch.py use_sim_time:=false
```

```bash
# --- terminal 4 · move_group + MoveIt Task Constructor + vision transport RX --
ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py
# restart it after any change to launch/planning_model.py
```

```bash
# --- GPU host · SAM3 ----------------------------------------------------------
bash docker/sam3/run_sam3.sh
```

```bash
# --- terminal 5 · verify everything answers before the mission ---------------
python3 realrobot/live/grasp_preflight.py
```

| Launch arg | Default | Use |
| --- | --- | --- |
| `map` | `Paths().map` | automatic if `RECORDING` is **exported**; pass it to be sure |
| `cmd_vel_topic` | `/base_velocity` | **`/omni_base_controller/cmd_vel`** when Toyota's stack is down |
| `localization` | `false` | **`true`** — AMCL, correct with `pose_integrator` stopped |
| `ik_solver use_sim_time` | `true` | **`false`** — there is no `/clock` on the robot |

**Safety.** `/omni_base_controller/cmd_vel` bypasses Toyota's limiter: no bumper stop, no
obstacle slowdown, no teleop override, and the joystick fights Nav2 for the same topic.
**Keep the runstop in reach.**

---

## 10. Run the mission

```bash
# --- terminal 6 ---------------------------------------------------------------
cd /home/ws
export RECORDING=lab_20260919_wheel HSR_REAL_ROBOT=1
export DEEPSEEK_API_KEY=<key>
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash

python3 -m core.pipeline.mission_tree --target "pringles can" --grasp false
```

```
Mission
  Home arm · Ready · Home head · Find target · Park · Pause Nav2
```

Then grasping, independently, with Nav2 already inactive:

```bash
HSR_REAL_ROBOT=1 python3 -m core.grasping.pick "pringles can"
```

| Flag | Tree it builds |
| --- | --- |
| `--grasp false` | Home arm · Ready · Home head · Find target · Park · **Pause Nav2** |
| `--grasp true` (default) | ... · Pause Nav2 · **Pick** |
| `--navigate-only true` | Home arm · Ready · Home head · Choose location · Go there — no SAM3, no graspable pose |
| `--target` | object to find, a noun phrase: `"pringles can"` |
| `--natural-language true` | `--target` is a spoken request; DeepSeek names the object first. Default `false` |
| `--top-k` / `--bearings` | `3` / `12` — reasoner guesses / parking probe directions |
| `--mode` | `auto` · `pickup` · `grasp` |
| `--render` | draw the tree to `outputs/`, no ROS |

How `Find target` chooses where to look:

| Order | Source | Note |
| --- | --- | --- |
| 1 | remembered objects, nearest first | `can` finds `pringles can` and `tomato can`, never `trash can` (`whole_name_only` in [scannet200.yaml](config/scene_graph/scannet200.yaml)) |
| 2 | cached DeepSeek picks | cleared when a search ends `NOT_FOUND` |
| 3 | DeepSeek, `--top-k` picks (default 3) | DeepSeek orders the rooms (current room first). Inside a room the order is enforced in code ([query.py](core/reasoner/query.py) `_tables_first`): tables and desks → counters → the rest → closed furniture (cabinets…), which the robot cannot open |
| 4 | DeepSeek again, once | only if every pick failed: tried pieces excluded, told to prefer other rooms |

So at most **2 × `--top-k`** DeepSeek places per mission (6 by default), after the remembered ones.
At each place: up to 2 views (more if the surface needs them), giving up after 2 failed drives.

`Park` fails (`NOT_GRASPABLE`) if the head cannot aim at the object afterwards; it used to warn
and report success. `Pause Nav2` deactivates the planner, controller and costmaps for the pick:
the next mission and `goto.py` resume them; by hand,
`ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"`.

`--navigate-only` drives to an *observation* pose. A graspable pose needs `Park`, which
needs the object from `Find target`, which needs SAM3 — so use `--grasp false`.

**`HSR_REAL_ROBOT=1` is required on hardware.** Same switch
[camera_ros2.py](core/perception/camera_ros2.py) reads: it selects the robot's clock and
defaults `--graph` to `config/realrobot/scene_graph/$RECORDING.json`.

**`DEEPSEEK_API_KEY` is required** unless the object is already in the graph. Without it
`Find target` fails immediately.

### Ask in a sentence instead

```bash
python3 -m core.pipeline.mission_tree --target "fetch me my spectacles" \
  --natural-language true --grasp false
# [REQUEST] 'fetch me my spectacles' -> 'spectacles'
```

One extra DeepSeek call, before ROS starts, cuts the request down to the object it names.
From there the run is identical to `--target "spectacles"`: only the noun is SAM3's prompt,
the guess-cache key and the label saved in the scene graph. Leave the flag off and nothing
changes — no extra call, and `--target` is used exactly as typed.

- **Always needs `DEEPSEEK_API_KEY`**, even for an object already in the graph.
- **The noun must be words you said.** A synonym ("spectacles" → "glasses") or an invented
  object is refused and the mission stops before the robot moves, because the label is
  saved for good. The catch: "two apples" will not become "apple".
- Check the extraction alone, one API call, no ROS:
  `python3 -c "from core.reasoner import query; print(query.object_of('fetch me my spectacles'))"`

### Drive to one piece of furniture instead

Navigation leg only, no reasoner, no perception:

```bash
python3 realrobot/live/goto.py --furniture 30 --dry-run   # poses and paths, no motion
python3 realrobot/live/goto.py --furniture 30 --look      # kitchen desk; 93 = office table (ids: §7)
python3 realrobot/live/localize.py --dry-run              # pose error after the drive
```

### What you will see, and what it means

| Log line / symptom | Cause | Do |
| --- | --- | --- |
| `[HEAD] … goal acceptance timed out` | fixed: one kept head action client | if it returns, the link is dropping first goals |
| `[NAV] no path; clearing the global costmap and planning once more` | stale laser marks boxed the robot in; this is the recovery | nothing; a second `no path` is real |
| every pose `no path`, clearing changes nothing | Nav2 still paused from the last mission | resume (above), or just run `goto.py` / the mission |
| `Location could not be observed: navigation failed` | every observation pose was unreachable, or there were none | `plan_overlay.py --furniture <id>` shows which |
| scan sits beside the map walls in RViz, robot stops short / overshoots | pose error: laser odometry, or a map built without `WHEEL_ODOM=1` | `use_wheel_odom.py`, `localize.py`, clear costmaps |
| `Joint 'odom_x' … outside bounds` → `pregrasp: START_STATE_INVALID` | fixed: base joints widened to ±100 m in [planning_model.py](launch/planning_model.py) | restart terminal 4 |
| `[RECOVER] failed: no /whole_body_moveit/joint_states` | move_group is down | `recover_home.py --direct` works without it (no collision checking: watch it) |
| `[ODOM] … switch … FAILED` | robot stack not up yet | rerun `use_wheel_odom.py` before driving |

---

## One command per stage

```bash
export RECORDING=<recording>                                                # every terminal
# GPU host: bag -> map, keyframes, scene graph, rooms, map_furniture.yaml
WHEEL_ODOM=1 bash docker/boxer/build_scene_graph.sh --bag bags/$RECORDING --rooms --nav-map $RECORDING
# container
python3 realrobot/offline/plan_overlay.py --survey
ros2 launch launch/realrobot/nav2_real.launch.py \
  map:=/home/ws/config/realrobot/map/$RECORDING/map_furniture.yaml \
  cmd_vel_topic:=/omni_base_controller/cmd_vel localization:=true         # wait for "[ODOM] … wheel odometry"
python3 realrobot/live/localize.py && python3 realrobot/live/localize.py --dry-run
ros2 launch /home/ws/launch/ik_solver.launch.py use_sim_time:=false
ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py
bash docker/sam3/run_sam3.sh                                                # host
HSR_REAL_ROBOT=1 python3 -m core.pipeline.mission_tree --target "pringles can" --grasp false
HSR_REAL_ROBOT=1 python3 -m core.grasping.pick "pringles can"
```
