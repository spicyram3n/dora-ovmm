# HSR object search

**One command:** home the arm → search scene-graph/LLM locations → navigate → detect with SAM3 → park within arm reach.

Grasp generation and arm execution will be integrated later.

## 1. Clone

```bash
git clone git@github.com:spicyram3n/dora-ovmm.git
cd dora-ovmm
```

## 2. Set up Docker and the workspace

On the **host**, install Docker, NVIDIA drivers + NVIDIA Container Toolkit, and VS Code with the **Dev Containers** extension. Docker must work for your user and have GPU access for the model servers.

Open the repository in VS Code → **Dev Containers: Reopen in Container**. First setup downloads ROS packages and builds the workspace. Wait for it to finish.

After changing ROS sources, rebuild inside the **dev container**:

```bash
ROS2_REBUILD=1 bash /home/ws/.devcontainer/postCreate.sh
```

For Dockerfile changes, use **Dev Containers: Rebuild Container**. Python, YAML, and launch-file edits normally only need a process restart.

Run this in **each new dev-container terminal**:

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

## 3. Prepare the scene graph — once per scene

**Container terminal 1 — simulation:**

```bash
ros2 launch hsrb_gazebo_launch hsrb_apartment_world.launch.py \
  use_sim_time:=true use_navigation:=false robot_name:=hsrc \
  description_package:=hsrc_description description_file:=hsrc1s.urdf.xacro
```

**Container terminal 2 — Nav2:**

```bash
ros2 launch hsrb_rosnav_config navigation_launch.py \
  map:=/home/ws/config/map/apartment_world_map.yaml \
  params_file:=/home/ws/config/nav2_params.yaml use_sim_time:=true
```

Check that the laser scan lines up with the map in RViz. Keep the freshly spawned robot stationary.

**Container terminal 3 — register and build:**

Set `DEEPSEEK_API_KEY` in this terminal for room assignment and LLM search.

```bash
python3 core/build_scene_graph.py register \
  --world-base 5.0 6.6 0.0 --output config/map/world_to_map.json
python3 core/build_scene_graph.py \
  --transform config/map/world_to_map.json --rooms
python3 core/search_object.py pringles --dry-run
```

The spawn coordinates above apply only to the default fresh apartment. Check graph alignment before using it. Output: `outputs/scene_graph/apartment.json`.

Stop simulation and Nav2 with **Ctrl+C** in terminals 1 and 2 before the full launch below.

For an unknown-object demo, use a graph copy with the target removed but furniture retained. Successful searches update the selected graph. See [scene graph instructions](core/scene_graph/README.md) for more details.

## 4. Prepare models and start SAM3

Follow the [GraspGenX README](docker/graspgenx/README.md) for hand export, registration, and model setup. Its server is not required by the current search-to-placement launch.

For SAM3, accept the `facebook/sam3` model license on Hugging Face and put your token in `docker/sam3/.env` as `HF_TOKEN=...`.

In a **host terminal**, from the repository root:

```bash
bash docker/sam3/run_sam3.sh
```

Leave it running. First startup builds the image and downloads model weights; wait for the server to report ready.

## 5. Run the pipeline

In a sourced **dev-container terminal**, with `DEEPSEEK_API_KEY` set for LLM predictions:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles" top_k:=3
```

The launch starts simulation, Nav2, and IK; waits for services, localization, RGB-D, and SAM3; then sends the arm home before searching. It stops after base placement.

Watch **`[WAIT]` → `[READY]` → `[HOME]` → `[SEARCH]` → `[APPROACH]` → `[RESULT]`**. Process logs are labelled individually. Simulation stays open after completion; **Ctrl+C** stops the launched processes.

Useful overrides:

| Argument | Purpose |
| --- | --- |
| `target:="red cup"` | Change the queried object |
| `graph:=/path/to/scenario.json` | Select prepared object memory |
| `top_k:=5` | Ask for more candidate locations |
| `startup_timeout:=300` | Allow slower startup |

If simulation, Nav2, and IK are **already running**, reuse them:

```bash
ros2 launch /home/ws/launch/search.launch.py target:="pringles" \
  start_simulation:=false start_navigation:=false start_ik:=false
```

## More details

- [Navigation and troubleshooting](core/navigation/README.md)
- [Real scans → Mask3D/OpenYOLO3D → scene graph](docker/openyolo3d/README.md)
- [Execute saved grasps separately](ros2_ws/src/grasp_execution/README.md)
- [Camera transport between machines](vision-transport-poorna/README.md)
