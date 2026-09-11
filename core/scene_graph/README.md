# Scene graph

Store objects and furniture in map coordinates, then choose where to search.

## Build the apartment graph

From the repository root, with a measured world-to-map registration:

```bash
python3 -m core.scene_graph.build --transform /path/to/world_to_map.json
```

For registration commands, use the [navigation quick start](../navigation/README.md). Set `DEEPSEEK_API_KEY` and add `--rooms` to assign room names.

The builder reads initial Gazebo world geometry, not objects moved during simulation. Walls stay in navigation costmaps, not semantic nodes.

## Search from Python

With `core` on Python's import path and the robot's map position in `robot_x, robot_y`:

```python
from core.scene_graph import graph as sg
from core.reasoner.query import search_order

scene = sg.load('outputs/scene_graph/apartment.json')
locations = search_order(scene, 'pringles can', top_k=3, near=(robot_x, robot_y))
first = next(locations)
```

Known objects use memory first. Unknown objects and failed-location fallback use DeepSeek. Iterate only after an observation fails; stop after detection. Calling `list(locations)` triggers fallback too early.

Returned centroids are object or furniture positions, **not base navigation goals**. Navigation chooses a reachable viewing pose. Invalid API answers raise an error.

## Frame rules

- Use metres. All instances in one build must share a source frame.
- Supply `source_frame` and a measured rigid 4×4 `map_from_source` transform. Scale and shear are rejected.
- `map`, `odom`, and Gazebo world are different frames. The occupancy YAML's `origin` is not world-to-map registration.
- Verify registration against the map in RViz. Matrix validation cannot prove alignment is accurate.
- Transform camera observations with timestamped TF before storing them in map. Transform separately to odom for grasping.

Registration JSON contains `source_frame: "gazebo_world"` and `map_from_source`. At the same instant, using the same physical base frame:

```text
T_map_world = T_map_base @ inverse(T_world_base)
```

Use a trusted localized pose and account for any robot-origin/base-frame offset. Keep this scene transform fixed instead of recalculating it from noisy localization.

## Build from a scan

Run segmentation and register the scan first. Given `segments` and `registered_transform`:

```python
from core.scene_graph.instance import Instance
from core.scene_graph import graph as sg

instances = [Instance(label, points, confidence=score, name=segment_id)
             for label, points, score, segment_id in segments]
scene = sg.build(instances, source_frame='ipad_scan',
                 map_from_source=registered_transform)
sg.save(scene, 'scan_graph.json')
```

This module does not run Mask3D or estimate registration. For OpenYOLO3D input, see the [scan workflow](../../docker/openyolo3d/README.md).

## Update objects

`record_object(..., frame_id="map")` takes a box centre and dimensions. Supply `node_id` to update; omit it to create. Labels alone never merge objects.

IDs survive save/load but may change on rebuild. Nodes store centroids and separate axis-aligned bounds. `on`, `in`, and `near` are geometry estimates; they do not prove support, visibility, or reachability.

## Code map

| File | Purpose |
| --- | --- |
| [instance.py](instance.py) | Labelled source-frame points |
| [graph.py](graph.py) | Build, update, validate, and save graphs |
| [gazebo.py](gazebo.py) | Read initial world collision geometry |
| [relations.py](relations.py) | Estimate object–furniture relations |
| [query.py](../reasoner/query.py) | Choose search locations |
| [deepseek.py](../reasoner/deepseek.py) | API calls and room assignments |

Dependencies: numpy, scipy, trimesh, networkx ≥ 3.4, pydantic ≥ 2, openai; pycollada for COLLADA meshes.
