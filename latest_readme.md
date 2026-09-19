# Bag → scene graph → mission

End-to-end run instructions. Worked example `lab_20260919`, verified 2026-09-19 on the HSR.

Diagnostics: [TROUBLESHOOTING.md](TROUBLESHOOTING.md) · Changes: [fixes/](fixes/)

## 0. Set the recording

```bash
cd /home/ws
export RECORDING=lab_20260919
printenv RECORDING            # NOT echo — echo prints unexported vars too
```

One variable moves every path ([recording.py](core/utils/recording.py)).

| Path | Step | Holds |
| --- | --- | --- |
| `bags/<rec>/` | 1 | raw bag, 2 GB parts + `metadata.yaml` |
| `config/realrobot/map/<rec>/map.{pgm,yaml}` | 2 | Nav2 map |
| `config/realrobot/map/<rec>/scan_to_map.json` | 3 | keyframe→map (identity) |
| `config/realrobot/map/<rec>/boxer_to_map.json` | 5 | undoes Boxer re-centring |
| `config/realrobot/scene_graph/<rec>.json` | 6 | the graph |
| `outputs/realrobot/<rec>/slam/` | 2 | `tf_mapping/`, `posegraph.*` |
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
| `/scan` `/tf` `/tf_static` | **yes** | slam_toolbox + keyframe poses |
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
| Corridors | middle, dwell at doors/alcoves/pillars | parallel walls give no longitudinal lock |
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

## 2–3. Map + keyframes · container, no GPU

```bash
source /opt/ros/humble/setup.bash
mkdir -p outputs/realrobot/$RECORDING/logs
bash realrobot/dataprep/prepare.sh --bag bags/$RECORDING --rate 1 $RECORDING \
  2>&1 | tee outputs/realrobot/$RECORDING/logs/prepare.log
```

**`--rate 1` when `/scan` is 40 Hz.** Rate 2 = 80 Hz into slam_toolbox → dropped scans.

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
| `checks.json` | ≥ 80% of laser-height points within 10 cm |

---

## 5–6. Scene graph · GPU **host** (no docker in the container)

```bash
cd /home/sesettip1/dora-ovmm
export DEEPSEEK_API_KEY=<key>             # else rooms are numbered, not named
nvidia-smi                                # card busy? add --cpu
RECORDING=lab_20260919 bash docker/boxer/build_scene_graph.sh --rooms \
  2>&1 | tee outputs/realrobot/lab_20260919/logs/boxer.log
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

827 keyframes on an RTX 4080 SUPER: detect ~100 s, fuse ~20 s, graph ~1 s.

To make the map hold a word it does not yet know, add it under `object:` in [scannet200.yaml](config/scene_graph/scannet200.yaml) and rerun this step — no re-record, no re-SLAM. Details: [docker/boxer/README.md](docker/boxer/README.md#the-vocabulary-is-a-prompt-list-you-control).

If the host lacks numpy/scipy/networkx/yaml it stops early. Split:

```bash
# container
python3 realrobot/dataprep/make_boxer_scene.py \
  --source outputs/realrobot/$RECORDING/keyframes \
  --output outputs/realrobot/$RECORDING/boxer/scannet/$RECORDING \
  --registration config/realrobot/map/$RECORDING/boxer_to_map.json
# host
RECORDING=$RECORDING bash docker/boxer/build_scene_graph.sh
# container
python3 -m docker.boxer.to_scene_graph \
  --boxes outputs/realrobot/$RECORDING/boxer/$RECORDING/boxer_3dbbs_fused.csv \
  --transform config/realrobot/map/$RECORDING/boxer_to_map.json \
  --output config/realrobot/scene_graph/$RECORDING.json \
  --rooms --map config/realrobot/map/$RECORDING/map.yaml
```

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

`lab_20260919`: 148 nodes · 92 edges · office 75 / kitchen 41 / living room 13 / none 19.

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
python3 realrobot/offline/plan_overlay.py --furniture 92 --start -7.45 -0.95
```

| Flag | Default | Does |
| --- | --- | --- |
| `--survey` | — | whole graph, no planning, no ROS |
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
cd /home/ws && export RECORDING=lab_20260919
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch launch/realrobot/nav2_real.launch.py \
  map:=/home/ws/config/realrobot/map/lab_20260919/map.yaml \
  cmd_vel_topic:=/omni_base_controller/cmd_vel \
  localization:=true
# wait for: "AMCL cannot publish a pose ... Please set the initial pose"
```

```bash
# --- terminal 2 · seed AMCL, AFTER terminal 1 is up --------------------------
cd /home/ws && export RECORDING=lab_20260919
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 topic info /initialpose          # Subscription count must be 1 first
python3 realrobot/live/localize.py
ros2 run tf2_ros tf2_echo map odom    # must stream = AMCL is publishing
```

```bash
# --- terminal 3 · collision-aware IK -----------------------------------------
ros2 launch /home/ws/launch/ik_solver.launch.py use_sim_time:=false
```

```bash
# --- terminal 4 · move_group + MoveIt Task Constructor + vision transport RX --
ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py
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
export RECORDING=lab_20260919 HSR_REAL_ROBOT=1
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
| `--target` | object to find, free text |
| `--top-k` / `--bearings` | `3` / `12` — reasoner guesses / parking probe directions |
| `--mode` | `auto` · `pickup` · `grasp` |
| `--render` | draw the tree to `outputs/`, no ROS |

`--navigate-only` drives to an *observation* pose. A graspable pose needs `Park`, which
needs the object from `Find target`, which needs SAM3 — so use `--grasp false`.

**`HSR_REAL_ROBOT=1` is required on hardware.** Same switch
[camera_ros2.py](core/perception/camera_ros2.py) reads: it selects the robot's clock and
defaults `--graph` to `config/realrobot/scene_graph/$RECORDING.json`.

**`DEEPSEEK_API_KEY` is required** unless the object is already in the graph. Without it
`Find target` fails immediately.

### Drive to one piece of furniture instead

Navigation leg only, no reasoner, no perception:

```bash
python3 realrobot/live/goto.py --furniture 92 --dry-run   # poses and paths, no motion
python3 realrobot/live/goto.py --furniture 92 --look
```

---

## One command per stage

```bash
export RECORDING=lab_20260919
bash realrobot/dataprep/prepare.sh --bag bags/$RECORDING --rate 1 $RECORDING
RECORDING=$RECORDING bash docker/boxer/build_scene_graph.sh --rooms          # host
python3 realrobot/offline/plan_overlay.py --survey
ros2 launch launch/realrobot/nav2_real.launch.py \
  map:=/home/ws/config/realrobot/map/$RECORDING/map.yaml \
  cmd_vel_topic:=/omni_base_controller/cmd_vel localization:=true
python3 realrobot/live/localize.py                                          # after Nav2
ros2 launch /home/ws/launch/ik_solver.launch.py use_sim_time:=false
ros2 launch /home/ws/launch/realrobot/grasp_real.launch.py
bash docker/sam3/run_sam3.sh                                                # host
HSR_REAL_ROBOT=1 python3 -m core.pipeline.mission_tree --target "pringles can" --grasp false
HSR_REAL_ROBOT=1 python3 -m core.grasping.pick "pringles can"
```
