# Scene graph

Furniture and objects in map coordinates. The mission reads it to decide where to look, and saves what it finds back into it.

| Setup | Graph file | How it is built |
| --- | --- | --- |
| Simulation, kitchen scene (mission default) | `config/scene_graph/kitchen_objects.json` | Not reproducible with `build.py` yet: it stops at `rubiks_cube`, which has no ScanNet200 class |
| Simulation, apartment | `outputs/scene_graph/apartment.json` | [Simulation](#simulation-gazebo) |
| Real robot, lab bag 2026-08-11 | `config/realrobot/scene_graph/lab_20260811.json` | [Real robot](#real-robot) |

---

## Simulation (Gazebo)

### Build it

Needs the sim and Nav2 running for `register` ([root README, step 2](../../README.md#2-build-the-scene-graph)).

| Command | Why |
| --- | --- |
| `python3 -m core.scene_graph.build register --world-base 5.0 6.6 0.0` | Measure the Gazebo-world → map transform. Writes `config/map/world_to_map.json` (`--output` to change). |
| `python3 -m core.scene_graph.build --transform config/map/world_to_map.json` | Build the graph from the world file |
| Add `--rooms` | Let DeepSeek name the rooms (needs `DEEPSEEK_API_KEY`) |
| Add `--output path.json` | Save somewhere else |
| Add `--world path.world` | Use another Gazebo world |
| `python3 visualization/viewpoints.py pringles` | Check the result: where would a query search first? |

- The builder reads the **initial** world file, not objects moved during the sim.
- Walls stay in Nav2's costmaps, not in the graph.
- Every model becomes a [ScanNet200 class](../../config/scene_graph/scannet200.yaml). An unknown model stops the build: add it under `gazebo:` there.
- Warning: `--world-base` is the robot's current Gazebo `base_footprint` x, y, yaw. `5.0 6.6 0.0` is the fresh-spawn value only.

### Registration

`config/map/world_to_map.json`: `source_frame: "gazebo_world"` plus `map_from_source`. It comes from one moment, one physical base frame:

```text
T_map_world = T_map_base @ inverse(T_world_base)
```

Use a trusted localized pose and keep the transform fixed; do not recompute it from noisy localization.

---

## Real robot

Lab bag 2026-08-11. Every step from the bag to the graph: [Boxer README](../../docker/boxer/README.md).

### Build it

```bash
python3 -m docker.boxer.to_scene_graph \
  --boxes outputs/realrobot/lab_20260811/boxer/lab_20260811/boxer_3dbbs_fused.csv \
  --transform config/realrobot/map/lab_20260811_boxer_to_map.json \
  --output config/realrobot/scene_graph/lab_20260811.json
```

Look at it before trusting it:

```bash
python3 visualization/scene_graph.py --graph config/realrobot/scene_graph/lab_20260811.json \
  --scene outputs/realrobot/lab_20260811/keyframes/scene.ply --output outputs/realrobot/lab_20260811/scene_graph.rrd
```

Any saved graph works with `--graph`, the Gazebo one included. `--save-only` writes
the `.rrd` without opening Rerun; `python3 -m rerun <file>.rrd` reopens it.

### Registration

- **Keyframes:** the robot's keyframes are already in `map`: the same robot, in the same SLAM run as `config/realrobot/map/lab_20260811.yaml`.
- **Boxer's boxes:** Boxer moves its origin to the first camera, so its boxes are not in `map`.
- **The fix:** `config/realrobot/map/lab_20260811_boxer_to_map.json` shifts them back (`source_frame: "boxer_lab_20260811"`, no rotation). `realrobot/make_boxer_scene.py --registration` writes it.

---

## Both

### Any other source (Python)

From your own segments (given `segments` and a measured `registered_transform`):

```python
from core.scene_graph.instance import Instance
from core.scene_graph import graph as sg

instances = [Instance(label, points, confidence=score, name=segment_id)
             for label, points, score, segment_id in segments]
scene = sg.build(instances, source_frame='ipad_scan', map_from_source=registered_transform)
sg.save(scene, 'scan_graph.json')
```

This module does not run segmentation or estimate registration.

### Ask it where to search

```python
from core.scene_graph import graph as sg
from core.reasoner.query import search_order

scene = sg.load('outputs/scene_graph/apartment.json')
locations = search_order(scene, 'pringles can', top_k=3, near=(robot_x, robot_y))
first = next(locations)
```

From the shell, without ROS, for any graph (prints the whole order; the graph is not saved):

```bash
python3 -m core.reasoner.query 'coffee mug' --graph config/realrobot/scene_graph/lab_20260811.json --near 0 0
```

- **Order:** remembered objects first, then DeepSeek.
- **Cache:** DeepSeek's picks per object are saved in the graph (`llm_guesses`); a repeat query reuses them. A search that finds nothing forgets them, so the next query asks again.
- **One at a time:** take the next location only after a look fails. `list(locations)` asks DeepSeek too early.
- **Not goals:** returned centroids are object or furniture positions, **not** places to drive to. Navigation picks the viewing pose.
- **Bad answers:** an invalid DeepSeek answer raises an error.

### Frame rules

| Rule | Why |
| --- | --- |
| Metres, one source frame per build | Mixed frames or units corrupt every position |
| Give `source_frame` and a measured rigid 4×4 `map_from_source` | Scale and shear are rejected |
| `source_frame: "map"` needs the identity | Points already in `map` must not move |
| `map`, `odom` and Gazebo world are different frames | The occupancy YAML's `origin` is **not** world-to-map registration |
| Check registration against the map in RViz | Matrix checks cannot prove the alignment is right |
| Transform camera observations with timestamped TF before storing | Stored positions must be in `map` |
| For grasping, transform separately to `odom` | The arm plans in `odom` |

### What a node holds

| Field | Meaning |
| --- | --- |
| `label`, `name` | ScanNet200 class; instance or Gazebo model name. Object search matches both |
| `movable` | `true` = object, `false` = furniture |
| `centroid`, `dimensions`, `bounds` | Map-frame box; `bounds` are axis-aligned |
| `room` | Room name (with `--rooms`) |
| `footprint` | Furniture only: the smallest turned rectangle seen from above (`centre`, `size`, `yaw`) |
| Edge `relation` | Object → furniture: `on`, `in` or `near` |

- **Footprints:** navigation uses the footprint, not the bounds. Read it with `graph.footprint(scene.nodes[node_id])`.
- **Relations are guesses:** `on`, `in` and `near` are geometric estimates. They do not prove support, visibility or reach.
- **IDs:** node IDs survive save/load but can change on rebuild.
- **Updating an object:** `record_object(..., frame_id="map")` takes a box centre and dimensions. Pass `node_id` to update; leave it out to create. Labels alone never merge objects.

### Code map

| File | Purpose |
| --- | --- |
| [build.py](build.py) | Simulation CLI: register the transform, build the apartment graph |
| [to_scene_graph.py](../../docker/boxer/to_scene_graph.py) | Real-robot CLI: build a graph from Boxer's fused boxes |
| [graph.py](graph.py) | Build, update, validate, save and load graphs |
| [instance.py](instance.py) | Labelled source-frame points; label lookup |
| [scannet200.yaml](../../config/scene_graph/scannet200.yaml) | Label dictionary: 198 classes by role, Gazebo and synonym aliases |
| [gazebo.py](gazebo.py) | Read the world's initial collision geometry |
| [relations.py](relations.py) | Estimate object–furniture relations |
| [query.py](../reasoner/query.py) | Choose search locations |
| [deepseek.py](../reasoner/deepseek.py) | DeepSeek calls and room names |

Dependencies: numpy, scipy, trimesh, networkx ≥ 3.4, pydantic ≥ 2, openai; pycollada for COLLADA meshes.
