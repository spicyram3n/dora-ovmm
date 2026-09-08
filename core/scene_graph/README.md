# Scene graph and reasoner

Read the code in this order:

1. `instance.py`: a labelled set of points in the source frame, in metres.
2. `graph.py`: validate the transform, build map-frame nodes and edges, and save/load JSON.
3. `../reasoner/query.py`: known object first, otherwise ranked furniture.

`gazebo.py` supplies initial collision geometry from the world file.
`relations.py` estimates `on`, `in`, or `near` from geometry.
`../reasoner/deepseek.py` asks DeepSeek for a complete furniture-to-room assignment.
Both reasoner functions share `graph.furniture_listing()` and the small API helper.
No ranking cache is used, so changes to the scene are reflected in each request.

## Frames

All instances supplied to one `build` call must share a source frame.
The caller explicitly supplies `source_frame` and `map_from_source`:

    p_map = R_map_source @ p_source + t_map_source

The transform is applied to every point BEFORE computing bounds and relations.
The graph records `frame_id: map`, `units: m`, the source frame and the transform.
Each node has a centroid, map-axis-aligned dimensions and bounds. Bounds are
stored separately because a point-cloud mean need not be its bounding-box centre.
`record_object(..., frame_id="map")` creates a new object; supply `node_id` to
update an existing one. It takes a box centre and dimensions, and never merges
objects solely by label. Node IDs persist across save/load; rebuilding a graph can renumber them. Do not
reuse old IDs across rebuilds. `name` retains the source's instance identifier.

`map`, `odom`, and Gazebo world are distinct. The occupancy YAML `origin` places
image cells in map; it is not the transform from Gazebo world to map.
At a known simulation instant, a calibrated base pose in both frames gives:

    T_map_world = T_map_base @ inverse(T_world_base)

Use the same physical base frame in both terms, at the same instant. Gazebo's
robot model origin may differ from `base_link`; include that fixed offset.
Use a trusted localized pose, not merely AMCL's unverified startup guess.
Check transformed wall corners against the occupancy map in RViz before navigation.
Do not recompute the static scene transform continuously from noisy localization.

Create a JSON registration file with `source_frame: "gazebo_world"` and
`map_from_source: [[...], [...], [...], [0, 0, 0, 1]]` using that measured transform.
There is deliberately no guessed apartment registration checked in.

From the repository root:

```sh
python3 core/build_scene_graph.py --transform /path/to/world_to_map.json
# Once DEEPSEEK_API_KEY is set, add --rooms to assign room names.
```

The world parser supports this apartment's parent-relative Euler poses and
primitive/mesh collision geometry. It rejects unresolved models and unsupported
nested/relative-frame poses rather than quietly misplacing geometry. It reads
initial world-file geometry, not runtime model states after physics or manipulation.
Walls are omitted from semantic nodes; they still belong in navigation costmaps.

## Future iPad / Mask3D input

Mask3D inference is separate. Convert each segment into an `Instance`, keeping all
points in the original scan frame. Convert scan units to metres first. Register
that frame to the robot map, including floor alignment, then use the same builder:

```python
from scene_graph.instance import Instance
from scene_graph import graph as sg

instances = [Instance(label, points, confidence=score, name=segment_id)
             for label, points, score, segment_id in segments]
scene = sg.build(instances, source_frame='ipad_scan',
                 map_from_source=registered_transform)
sg.save(scene, 'scan_graph.json')
```

`segments` and `registered_transform` are inputs from segmentation and registration;
this module does not run Mask3D or estimate alignment. Scale/shear matrices are
rejected. A valid rigid matrix does not prove the registration is accurate.
Store persistent object positions in map. Transform fresh camera observations
using timestamped TF before recording them; transform to odom separately for MoveIt.

## Search

With `core` on Python's import path:

```python
from scene_graph import graph as sg
from reasoner.query import search_order

scene = sg.load('outputs/scene_graph/apartment.json')
locations = search_order(scene, 'pringles can', top_k=3, near=(robot_x, robot_y))
first = next(locations)
```

`near` is the robot's XY in map. A known result carries `object_id`, the object's
centroid and its furniture association. An LLM result carries a furniture centroid.
Neither centroid is a base navigation goal: the navigation module must choose an
accessible observation pose. Continue iteration only after an adequate observation
fails; stop when detection succeeds. Converting the generator to a list would
trigger fallback too early. Unknown objects start with the LLM immediately.

Predictions contain exactly min(K, eligible furniture count) distinct valid IDs.
Already searched furniture is excluded before the request. No furniture means an
empty result without an API call. Bad answers raise an error instead of becoming
navigation destinations. Room assignments are checked completely before mutation.

Names use lightweight word matching, not semantic recognition. Relations use
footprints and height bounds, so shelf interiors and partial scans remain geometric
hypotheses. Neither room labels nor relation edges prove visibility or reachability.

Dependencies already available in this workspace: numpy, scipy, trimesh,
networkx >= 3.4, pydantic >= 2, openai; COLLADA meshes additionally need pycollada.
The API helper uses DeepSeek JSON output and a configurable model argument,
defaulting to `deepseek-v4-flash` with thinking disabled for these short requests.
See https://api-docs.deepseek.com/guides/json_mode/ and
https://api-docs.deepseek.com/guides/thinking_mode/ for the API format.
Tests use fake API responses and do not spend API credits.

```sh
python3 -m pytest tests/test_scene_reasoner.py tests/test_scene_gazebo.py -q
```

See [navigation commands](../navigation/README.md) for registration, launch, and search.
