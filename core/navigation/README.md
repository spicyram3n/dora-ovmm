# Navigation from scene-graph search

Ported from dora-ovmm: `standoff.py`, `nav2_client.py`, and the analytical
`tools/experiments/reach.py` helper. `base_placement.py` was moved from `core/placement` with its
existing edits preserved. Production navigation/placement stays here; the analytical reach experiment lives under `tools/experiments`.

`search_object.py` connects the new reasoner to Nav2 and SAM3. It uses map-frame
bounds, checks paths before driving, verifies position and yaw after arrival,
and cancels timed-out goals before trying another candidate. Cancellation that
cannot be confirmed stops the run. Watch for the explicit goal-acceptance timeout
error: if the server never acknowledges a goal, its state is unknown; stop Nav2
before retrying. Observation tolerances are 0.30 m/rad, slightly wider than the
current Nav2 0.25 m/rad thresholds. These are NOT precision grasp tolerances.

The search tries up to two reachable views per location. A camera/server error
stops the run; a successful SAM3 reply with no instance tries another view.
Successful detection updates the graph. Failed observations do not prove absence
or remove a remembered object. The current head pose must see the target surface;
automatic head scanning is not part of this port.

## Run in simulation

In each ROS terminal, from the repository root:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

Terminal 1:

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

Terminal 2 (only one Nav2 launch; do not also run launch_nav2.py):

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  params_file:=/home/ws/ros2_ws/src/hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml \
  use_sim_time:=true
```

This navigation launch reads the initial AMCL pose from its parameter YAML.
It does not declare `initial_orientation_xyzw`; the current YAML sets yaw to zero.
Verify localization by checking the laser against the map in RViz.

Terminal 3, once localization is correct, BEFORE moving the freshly spawned robot:

```bash
python3 core/build_scene_graph.py register \
  --world-base 5.0 6.6 0.0 --output config/map/world_to_map.json
python3 core/build_scene_graph.py --transform config/map/world_to_map.json
python3 core/search_object.py 'pringles' --dry-run
```

The world-base values above apply only to the default fresh apartment spawn,
with the base footprint at that XY and yaw. If it has moved, supply its CURRENT
Gazebo base-footprint X/Y/yaw. Registration combines that pose with current TF
map-to-base; it cannot fix incorrect AMCL localization. Check graph furniture
positions against the map. Rebuild the graph after changing the registration.

For LLM room assignment and unknown-object queries, set DEEPSEEK_API_KEY in this
terminal (do not put the key in source files), then:

```bash
python3 core/build_scene_graph.py --transform config/map/world_to_map.json --rooms
```

A direct navigation test needs neither DeepSeek nor SAM3:

```bash
python3 core/search_object.py --furniture high_table01
```

Start SAM3 on the host using its existing script (another terminal):

```bash
bash docker/sam3/run_sam3.sh
```

Then search, navigate and detect with one command:

```bash
python3 core/search_object.py 'pringles' --top-k 3
```

Known objects use memory first; unknown objects call DeepSeek immediately. If a
remembered location fails, the next location is requested from DeepSeek, so set
the API key for that fallback even when the initial lookup is known. A dry-run
also calls DeepSeek for an unknown object, but sends no ROS motion commands.

To demonstrate scenario A with an object physically present, copy the graph and
remove that object's nodes from the COPY, leaving the Gazebo world untouched:

```bash
PYTHONPATH="/home/ws/core:$PYTHONPATH" python3 - <<'PY'
from scene_graph import graph as sg
scene = sg.load('outputs/scene_graph/apartment.json')
ids = [n for n, d in sg.objects(scene).items() if 'pringles' in d['label']]
scene.remove_nodes_from(ids)
sg.save(scene, 'outputs/scene_graph/apartment_unknown.json')
PY
python3 core/search_object.py 'pringles' --graph outputs/scene_graph/apartment_unknown.json --top-k 3
```

A successful search records the detected object, so recreate the copy to repeat
scenario A. Multiple same-named instances are resolved approximately by label and
proximity; search success means a matching object, not guaranteed instance identity.

## Grasping boundary

The search stops at detection; it does not execute a grasp or claim the base can
reach one. `tools/experiments/reach.py` is an analytical prefilter, not full IK: it does not check
wrist limits, arm collisions or the approach path. It is not used to move the
robot automatically by the search command. Its optional `move_into_reach` helper
is experimental; do not interpret its radial check as grasp feasibility.

MoveIt can be launched with your existing command:

```bash
ros2 launch hsrb_moveit_config demo.py description_package:=hsrc_description \
  description_file:=hsrc1s.urdf.xacro use_sim_time:=true
```

After detection, the existing grasp-generation command remains:

```bash
python3 core/run_pipeline.py 'pringles'
```

It also requires the GraspGenX server (`bash docker/graspgenx/run_graspgenx.sh`
on the host). It generates grasp files; it does not itself execute them.
Final collision-aware base selection and grasp execution remain to be connected.
The moved IK helper can still be called as:

```bash
python3 core/navigation/base_placement.py pringles --costmap
```

This needs `ros2 launch grasp_execution ik_solver.launch.py` and generated grasps.
Its default grasp frame remains odom; navigation goals are map and must be
transformed explicitly when this helper is connected to the search coordinator.

## Navigation failure checks

Search retries other candidate poses when planning or driving fails. It reports
an inaccessible location separately from a completed observation with no detection.
Known targets are faced directly, and fallback detections update an existing
object instead of creating another copy. Nav2 requires map-to-base TF no older
than two seconds; align clocks and keep localization publishing on the real robot.
A missing server or unconfirmed cancellation stops the mission.

Base-placement CLI uses `--costmap` by default. `--map` retains offline trinary
map loading, with unknown pixels blocked and origin yaw preserved. Other map
modes are rejected explicitly. Saved grasp input must be in `odom`; the command
rejects other frames instead of silently treating them as odom. IK timeout is an
error, not evidence that a grasp is unreachable. IK candidates still require
Nav2 path checks and a populated collision environment before arm execution.

Offline regression checks (no robot or paid API calls):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tools/test_navigation.py
```

## Observation before grasp planning

Object search now commands the HSRC head trajectory controller before capture.
The look-at calculation uses the head pivot TF and an approximate camera offset;
it is not a visibility or collision-aware head planner. Known objects supply the
look-at point. Tables/desks/counters use a point above their top; other furniture
uses its centre. Head joint limits are from the HSRC description. Verify aiming
in Gazebo before real hardware use.

Capture accepts only image pairs stamped after capture starts. A detected mask
needs at least 100 valid depth points and 25 percent valid masked depth; otherwise
search tries another viewpoint. These are initial quality thresholds. The second
candidate is on a distant side of the furniture, subject to Nav2 reachability.

Successful detections update the object's map-frame bounds and position. The
on/in/near edge is inferred from observed geometry, with a 0.5 m near limit;
unassociated detections retain no furniture edge. This is a geometric estimate,
not proof of support or containment. Search saves the graph only after success.
No detection or bad depth leaves graph memory unchanged.
