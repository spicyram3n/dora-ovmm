# Core cleanup

The active workflow is `build_scene_graph.py` followed by `search_object.py`.
Search currently ends after detection and a map-frame graph update; grasp execution
is not yet connected.

## Changed locations

| Previous location | Current location |
| --- | --- |
| `core/navigation/register_gazebo.py` | `core/build_scene_graph.py register` |
| `core/scene_graph/frames.py` | `core/scene_graph/graph.py` |
| `core/reasoner/rooms.py` | `core/reasoner/deepseek.py` |
| `core/grasping/gripper_frame.py` | `core/grasping/grasp_io.py` |
| `core/navigation/reach.py` | `tools/experiments/reach.py` |
| `core/grasping/visualization/` | `tools/visualization/` |

The camera now shares `utils.transforms.matrix_from_transform` instead of
implementing the conversion again. Short comments explain timestamp selection,
frame conversion, navigation cancellation, clearance and lazy LLM fallback.
Nav2 action handling and offline map loading are unchanged.

## Commands

```bash
python3 core/build_scene_graph.py register --world-base 5.0 6.6 0.0
python3 core/build_scene_graph.py --transform config/map/world_to_map.json
python3 core/search_object.py pringles --dry-run
python3 tools/visualization/view_target.py pringles_can
python3 tools/visualization/animate_grasp.py pringles_can
```

Use the current stationary Gazebo base pose for registration; the example applies
only to the corresponding fresh spawn. Existing validated alignment files do not
need recalibration just because files moved.

The LaTeX guide retains its original detailed explanations as a historical
reference and includes current source listings in its cleanup appendix. Old
line numbers apply to the old embedded listings, not current editor lines.
The existing PDF predates this cleanup; rebuild the LaTeX to include the appendix.
