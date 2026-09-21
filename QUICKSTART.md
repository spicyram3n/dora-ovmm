# Quickstart: from a recording to a pick

The shortest working route. For the reasons behind each step, see [README.md](README.md).

**Safety first.** Keep the runstop within reach. Nav2 drives the base without
Toyota's bumper stop.

---

## Before you start

Three values are set for you in every new terminal inside the dev container. You
do not need to export anything.

| Value | What it is | Where it comes from |
| --- | --- | --- |
| `RECORDING` | The name of the room recording to use | [.devcontainer/runtime.env](.devcontainer/runtime.env) |
| `HSR_REAL_ROBOT=1` | Real-robot mode: the robot's clock and its camera topics | [.devcontainer/runtime.env](.devcontainer/runtime.env) |
| `DEEPSEEK_API_KEY` | The key for the reasoning model | `/home/ws/.env`, a file with one line: `DEEPSEEK_API_KEY=<key>` |

The recording name is what links everything together. Every tool uses it to find
the right map, scene graph and output folders.

Check it before you build or drive:

```bash
printenv RECORDING
```

If it shows the wrong room, the tools will work on that wrong room without any
warning.

**To switch rooms:** change `RECORDING` in `runtime.env`, then open new terminals.

**To run the simulator instead:** put `HSR_REAL_ROBOT= ` (empty) in front of the
command. With real-robot mode on, a simulator run waits for a clock and camera
topics that are not there.

---

## The example already in this repo: the lab

`RECORDING` is set to `lab_20260919_wheel`, a recording of the lab that is
already built. Its files:

| What | Where |
| --- | --- |
| The recording itself (33 GB, a 13 minute drive) | `bags/lab_20260919/` |
| Laser map, used to find the robot's position | `config/realrobot/map/lab_20260919_wheel/map.yaml` |
| Map the robot drives on (laser map plus furniture) | `config/realrobot/map/lab_20260919_wheel/map_furniture.yaml` |
| Scene graph: 38 furniture, 111 objects, 3 rooms | `config/realrobot/scene_graph/lab_20260919_wheel.json` |

To use the lab, **skip Part 1** and go straight to Part 2.

---

## Part 1. Add a new room

Do this once per room. `myroom` stands for the name you choose.

### Step 1. Record, on the robot

```bash
ssh administrator@10.7.3.185
tmux new -s scan
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=5
ros2 bag record -o myroom --max-bag-size 2000000000 \
  /scan /tf /tf_static /head_rgbd_sensor/rgb/camera_info \
  /head_rgbd_sensor/rgb/image_rect_color \
  /head_rgbd_sensor/depth_registered/image_rect_raw \
  /omni_base_controller/wheel_odom
```

Drive slowly past every piece of furniture, then press Ctrl+C.

### Step 2. Build the map and scene graph, on the GPU host

Run this from the repo folder on the host, not inside the container. The host does
not read `runtime.env` or your `.bashrc`, so set both values by hand here.

```bash
rsync -aPz administrator@10.7.3.185:/home/administrator/myroom bags/
export RECORDING=myroom
set -a; source .env; set +a
WHEEL_ODOM=1 bash docker/boxer/build_scene_graph.sh \
  --bag bags/$RECORDING --rooms --nav-map $RECORDING
```

This takes about 25 minutes for a 13 minute recording. It creates
`config/realrobot/map/myroom/` and `config/realrobot/scene_graph/myroom.json`.

The recording folder and the recording name can differ. The lab was built with
`--bag bags/lab_20260919` and `RECORDING=lab_20260919_wheel`.

### Step 3. Point the container at the new room

Set `RECORDING=myroom` in `.devcontainer/runtime.env`, then open new terminals.

---

## Part 2. Run the robot

### Step 1. Start the services, every session

```bash
# on the GPU host
bash docker/sam3/run_sam3.sh
bash docker/graspgenx/run_graspgenx.sh

# on the PC, from vision-transport-poorna/
docker compose -f deployment/compose.yaml up -d vision_rx
```

### Step 2. Choose how to give commands

**Option A: from the terminal**

```bash
# terminal 1
ros2 launch /home/ws/launch/realrobot/search_real.launch.py target:="pringles can"

# terminal 2, once terminal 1 prints "Please set the initial pose"
python3 realrobot/live/localize.py && python3 realrobot/live/localize.py --dry-run
```

The launch starts one mission, the `target` it was given. **To send the robot again
without restarting anything**, run a mission by itself from another terminal. The
robot stack keeps running:

```bash
python3 -m core.pipeline.mission_tree --target "snack can" --top-k 2 2>&1 | tee outputs/last_run.log
```

| To | Command |
| --- | --- |
| Search and pick | `python3 -m core.pipeline.mission_tree --target "snack can" --top-k 2` |
| Search one piece only, then pick | `... --target "pringles can" --furniture 29 --top-k 2` |
| Drive to one piece and stop | `... --target desk --furniture 29 --navigate-only true` |
| Search and park, no pick | `... --target "snack can" --top-k 2 --grasp false` |
| Close and hold, no lift | `... --target "snack can" --top-k 2 --mode grasp` |
| Ask in plain words | `... --target "bring me something to drink" --natural-language true --top-k 2` |

These are the switches from the table below, written as `--name value`. Three things
to know:

- **Add `--top-k 2`.** Run this way the default is 3 suggestions per round; the
  launch file is what sets 2.
- **One mission at a time.** Wait for the previous one to print `[RESULT]`. Two
  missions would both drive the base and the arm. Starting the launch with no
  `target` and sending every request this way avoids the question.
- **Ctrl+C in this terminal** stops only this mission. The stack stays up, and Nav2,
  which a pick leaves paused, is resumed by the next mission on its own.

**Option B: from the web page**

```bash
# terminal 1: start the robot stack with no target
ros2 launch /home/ws/launch/realrobot/search_real.launch.py

# terminal 2: find the robot's position, same command as in option A
python3 realrobot/live/localize.py && python3 realrobot/live/localize.py --dry-run

# terminal 3: start the page
python3 -m web.server --natural-language true --top-k 2 --startup-timeout 600
```

Open http://localhost:8090. If the port is busy, put `WEB_PORT=8091` in front of
the command and open that port instead.

What to type on the page:

| Fetch box | Furniture box | The robot will |
| --- | --- | --- |
| an object or a request | empty | Search for it, then pick it |
| an object | a furniture id | Search only that piece, then pick |
| empty | a furniture id | Drive to that piece and stop |

The buttons over the 3D view work at any time. If a mission is running, the page
asks once, stops the mission, then does what you clicked.

| Button | Use it to |
| --- | --- |
| Stop | End the running mission |
| Open hand / Close hand | Let go of, or hold, whatever is in the hand |
| Recover arm | Free the arm after a pick that stopped part-way. Safe: opens the hand, backs out, then folds the arm |
| Home arm | Fold the arm. No collision check: only when the hand is empty and clear |
| Pause Nav2 / Resume Nav2 | Stop or allow base driving. Localization is kept either way |

---

## Switches

In option A, add these to the launch line as `name:=value`, or to a mission run by
itself as `--name value` (dashes instead of underscores: `--navigate-only true`).
In option B, add them to the `web.server` line as `--name value` (for example
`--mode grasp`). They then apply to every request until you restart the server.

| Switch | What it does |
| --- | --- |
| `furniture:=<id>` | Search that one piece only. No remembered places, no reasoning model |
| `natural_language:=true` | Treat `target` as a request, e.g. `target:="bring me something to drink"` |
| `top_k:=N` | How many places the reasoning model suggests per round. Default 2 |
| `grasp:=false` | Search and park, then stop. The hand is not used |
| `mode:=...` | What the hand does at the object. See the next table |
| `navigate_only:=true` | Choose a place and drive there. No detector, no pick |
| `nav:=false` | Pick only, for an object already in view. No driving, map or scene graph |
| `active_perception:=true` | Look from several viewpoints before parking. Add `rerun:=true` for the live viewer |
| `use_nav_rviz:=true`, `use_rviz:=true` | Open the Nav2 or MoveIt RViz window. Launch line only |
| `shutdown_when_done:=true` | Close everything when the mission ends. Launch line only |

A switch that does not exist, such as `pick:=false`, is ignored without an error.

### Pick modes

These only matter while `grasp` is `true`, which is the default.

| Mode | Closes on the object | Lifts it | Reports |
| --- | --- | --- | --- |
| `auto` (default) | yes | Only cylinders, by 3 cm. Other shapes are held, not lifted | `picked` or `grasped` |
| `pickup` | yes | Always, and checks with the camera that the object rose | `picked`, or a failure |
| `grasp` | yes | Never. It closes and holds | `grasped` |

The hand always approaches from the front or the side, never from above.

After `mode:=grasp`, or after any pick that stops part-way, the hand is still
around the object. Click **Open hand**, then **Recover arm**. Do not use Home arm.

---

## Useful examples

| Goal | Command or input |
| --- | --- |
| Fetch the Pringles can from the kitchen desk | `target:="pringles can" furniture:=29` (29 is the kitchen desk in the lab graph) |
| Watch the robot explore | `target:="snack can"`. The lab graph does not know that name, so the reasoning model picks the places. The detector still recognises a Pringles can by it |
| Ask in plain words | `target:="bring me something to drink" natural_language:=true` |
| Hold but do not lift | `target:="pringles can" mode:=grasp` |

---

## See what the robot saw

Every run saves its pictures in `outputs/detections/<time>_<object>/`. The run's
first lines print the folder name (`[RUN] detections in ...`).

| File | Shows |
| --- | --- |
| `001_find-target_snack-can_0.96.jpg`, ... | Each detector call, in order: the camera frame with the mask drawn on it, the score, what was asked, and which mission step asked. `_none` means nothing was found |
| `grasps_<object>.png` | The point cloud the mask became, from the side and from above, with the 4 best grasps drawn as open hands |
| `grasps_<object>.npz` | The same cloud and grasps as numbers |
| `index.jsonl` | One line per detector call: time, step, query, scores, boxes, file |

Runs started from the web page use the page's run id, so the folder matches the run
in the page's list. `DETECTIONS=0` in front of a command switches the saving off.

---

## Reset the scene graph between runs

When the robot finds an object, it saves it in the scene graph, along with the
places the reasoning model suggested. That is what you want in normal use: the
next request for the same object goes straight to where it was last seen.

For a demo or a test it gets in the way. After one successful "snack can" run, the
robot no longer explores for it: it remembers the place and drives straight there.
To get the same behaviour every time, keep a copy of the graph and put it back
before each run.

Save a copy once, when the graph is in the state you want to start from:

```bash
cp config/realrobot/scene_graph/$RECORDING.json config/realrobot/scene_graph/$RECORDING.json.demo_bak
```

Put it back before each run, while no mission is running:

```bash
cp config/realrobot/scene_graph/$RECORDING.json.demo_bak config/realrobot/scene_graph/$RECORDING.json
```

The robot stack can stay up: each mission reads the graph when it starts.

Do not use `git checkout` on the graph for this. It also throws away any changes
you made to the graph yourself and have not committed.

---

## When it goes wrong

| What you see | What to do |
| --- | --- |
| Stuck at `[WAIT] arm_trajectory_controller active` | Release the runstop |
| Stuck at `[WAIT] controller_server answering` | The launch in terminal 1 is not running. Start it |
| Stuck at `[WAIT] fresh map-to-base localization` | Run `localize.py` in terminal 2 |
| `0/12 probe directions reachable` | There is no free floor within arm's reach of the object. Clear space or move the object |
| The pick stopped part-way | **Recover arm** on the page, or `python3 realrobot/live/recover_home.py`. Not Home arm |
| You need to stop a mission | Stop on the page, or `curl -X POST localhost:8090/stop`, or Ctrl+C in terminal 1 |
| The page says `Address already in use` | Start it with `WEB_PORT=8091` in front |
| The robot drives straight to an object instead of exploring | It remembers it from an earlier run. Reset the scene graph, see above |
| You want to check everything is up | `python3 realrobot/live/grasp_preflight.py` |
