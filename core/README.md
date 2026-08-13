# core

All the Python for the search-and-fetch pipeline. Everything here speaks numpy
and zenoh; only `perception/camera_ros2.py` and `navigation/nav2_client.py`
need ROS 2, so the rest can be run and tested without a robot.

Run everything from this directory, so that `scene_graph`, `perception` and
the rest resolve as packages:

```bash
cd core
```

## Folders

| Folder | What it does |
|---|---|
| `scene_graph/` | Builds the 3D scene graph from Gazebo ground truth or (later) Mask3D |
| `spatial_reasoning/` | Asks an LLM which room is which, and where an object probably is |
| `navigation/` | Works out where to stand to look at furniture, and drives there with Nav2 |
| `perception/` | Camera frames, SAM3 segmentation, depth to point cloud |
| `grasping/` | GraspGenX grasp generation, saving and plotting |
| `utils/` | Shared zenoh plumbing for the two Docker servers |

## Entry points

| Script | Purpose |
|---|---|
| `build_scene_graph.py` | Gazebo world to a saved scene graph. Offline, no simulator needed |
| `search_object.py` | Look an object up (graph, then LLM) and drive to it |
| `run_pipeline.py` | Prompt to SAM3 mask to GraspGenX grasps, saved for MoveIt |

## Running

```bash
# 1. Build the scene graph once. --no-rooms skips the LLM (no API key needed).
python3 build_scene_graph.py --no-rooms
python3 build_scene_graph.py                 # with LLM room names

# 2. See where the robot would go, without touching the robot.
python3 search_object.py --furniture high_shelf01 --dry-run

# 3. Actually drive there (needs ROS 2 and Nav2 up).
python3 search_object.py --furniture high_shelf01
python3 search_object.py "pringles"          # full scenario B then A

# 4. Grasp what is in front of the camera (needs the Docker servers).
python3 run_pipeline.py "pringles can" --gripper hsrc_hand
```

Outputs land in `outputs/scene_graph/<name>/` and `config/targets/<slug>/`,
both gitignored and both safe to delete and regenerate.

## Environment

| Variable | Needed by | Purpose |
|---|---|---|
| `DEEPSEEK_API_KEY` | `spatial_reasoning/` | LLM room naming and location guesses |
| `ZENOH_CONNECT` | `utils/zenoh_rpc.py` | Where SAM3 and GraspGenX listen; defaults to localhost |

## Tests

```bash
python3 -m tests.test_gazebo                                   # also prints the graph
python3 -m pytest tests/test_gazebo.py -p no:anyio
```

`-p no:anyio` works around this image's pytest 6.2.5 being older than the
installed anyio plugin expects. It is unrelated to the tests themselves.
