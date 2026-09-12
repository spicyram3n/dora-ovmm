# Scene graph

Furniture and objects in map coordinates. The mission reads it to decide where to look, and saves what it finds back into it.

Default file: `outputs/scene_graph/apartment.json`.

## Build it (Gazebo apartment)

Needs the sim and Nav2 running for `register` ([root README, step 2](../../README.md#2-build-the-scene-graph)).

| Command | Why |
| --- | --- |
| `python3 -m core.scene_graph.build register --world-base 5.0 6.6 0.0` | Measure the Gazebo-world → map transform. Writes `config/map/world_to_map.json` (`--output` to change). |
| `python3 -m core.scene_graph.build --transform config/map/world_to_map.json` | Build the graph from the world file |
| Add `--rooms` | Let DeepSeek name the rooms (needs `DEEPSEEK_API_KEY`) |
| Add `--output path.json` | Save somewhere else |
| Add `--world path.world` | Use another Gazebo world |
| `python3 -m core.pipeline.search pringles --dry-run` | Check the result: where would a query search first? |

- The builder reads the **initial** world file, not objects moved during the sim.
- Walls stay in Nav2's costmaps, not in the graph.
- ⚠️ `--world-base` is the robot's current Gazebo `base_footprint` x, y, yaw. `5.0 6.6 0.0` is the fresh-spawn value only.

## Build it from a scan

For OpenYOLO3D output ([scan workflow](../../docker/openyolo3d/README.md)):

```bash
python3 -m core.scene_graph.openyolo3d \
  --instances docker/openyolo3d/output/room1/instances.json \
  --transform /absolute/path/to/room1_to_map.json \
  --output outputs/scene_graph/room1.json
```

From your own segments, in Python (given `segments` and a measured `registered_transform`):

```python
from core.scene_graph.instance import Instance
from core.scene_graph import graph as sg

instances = [Instance(label, points, confidence=score, name=segment_id)
             for label, points, score, segment_id in segments]
scene = sg.build(instances, source_frame='ipad_scan', map_from_source=registered_transform)
sg.save(scene, 'scan_graph.json')
```

This module does not run segmentation or estimate registration.

## Ask it where to search

```python
from core.scene_graph import graph as sg
from core.reasoner.query import search_order

scene = sg.load('outputs/scene_graph/apartment.json')
locations = search_order(scene, 'pringles can', top_k=3, near=(robot_x, robot_y))
first = next(locations)
```

- **Order:** remembered objects first, then DeepSeek.
- **One at a time:** take the next location only after a look fails. `list(locations)` asks DeepSeek too early.
- **Not goals:** returned centroids are object or furniture positions, **not** places to drive to. Navigation picks the viewing pose.
- **Bad answers:** an invalid DeepSeek answer raises an error.

## Frame rules

| Rule | Why |
| --- | --- |
| Metres, one source frame per build | Mixed frames or units corrupt every position |
| Give `source_frame` and a measured rigid 4×4 `map_from_source` | Scale and shear are rejected |
| `map`, `odom` and Gazebo world are different frames | The occupancy YAML's `origin` is **not** world-to-map registration |
| Check registration against the map in RViz | Matrix checks cannot prove the alignment is right |
| Transform camera observations with timestamped TF before storing | Stored positions must be in `map` |
| For grasping, transform separately to `odom` | The arm plans in `odom` |

Registration JSON: `source_frame: "gazebo_world"` plus `map_from_source`. It comes from one moment, one physical base frame:

```text
T_map_world = T_map_base @ inverse(T_world_base)
```

Use a trusted localized pose and keep the transform fixed; do not recompute it from noisy localization.

## What a node holds

| Field | Meaning |
| --- | --- |
| `label`, `name` | Class and instance name |
| `movable` | `true` = object, `false` = furniture |
| `centroid`, `dimensions`, `bounds` | Map-frame box; `bounds` are axis-aligned |
| `room` | Room name (with `--rooms`) |
| `footprint` | Furniture only: the smallest turned rectangle seen from above (`centre`, `size`, `yaw`) |
| Edge `relation` | Object → furniture: `on`, `in` or `near` |

- **Footprints:** navigation uses the footprint, not the bounds. Read it with `graph.footprint(scene.nodes[node_id])`.
- **Relations are guesses:** `on`, `in` and `near` are geometric estimates. They do not prove support, visibility or reach.
- **IDs:** node IDs survive save/load but can change on rebuild.
- **Updating an object:** `record_object(..., frame_id="map")` takes a box centre and dimensions. Pass `node_id` to update; leave it out to create. Labels alone never merge objects.

## Code map

| File | Purpose |
| --- | --- |
| [build.py](build.py) | CLI: register the transform, build the apartment graph |
| [openyolo3d.py](openyolo3d.py) | CLI: build a graph from OpenYOLO3D output |
| [graph.py](graph.py) | Build, update, validate, save and load graphs |
| [instance.py](instance.py) | Labelled source-frame points |
| [gazebo.py](gazebo.py) | Read the world's initial collision geometry |
| [relations.py](relations.py) | Estimate object–furniture relations |
| [query.py](../reasoner/query.py) | Choose search locations |
| [deepseek.py](../reasoner/deepseek.py) | DeepSeek calls and room names |

Dependencies: numpy, scipy, trimesh, networkx ≥ 3.4, pydantic ≥ 2, openai; pycollada for COLLADA meshes.
