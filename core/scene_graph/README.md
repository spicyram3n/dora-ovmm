# scene_graph

Turns a scene into a graph the robot can reason over: every object and piece
of furniture is a node, and every movable object has one edge to the furniture
it is **in**, **on** or **near**.

Pure geometry and networkx. No LLM, no ROS, no API keys, so all of it can be
run and tested on a laptop.

The graph is a forest rooted at the furniture, so "where is the apple" is one
hop. Node attributes stay JSON-native, which is why saving needs no encoder
and the LLM prompt is a slice of the same structure.

## Files

| File | Purpose |
|---|---|
| `instance.py` | The contract every source produces: a labelled blob of points. Also the furniture/structure vocabulary |
| `relations.py` | Decides in / on / near between an object and the furniture, from convex hull footprints |
| `graph.py` | Builds the graph, looks objects up, records new ones, saves and loads JSON |
| `gazebo.py` | Source: reads a Gazebo `.world` and its models into Instances |
| `test_gazebo.py` | Checks the apartment world against the placements the world file states |

A second source for Mask3D on real scans slots in beside `gazebo.py`: it only
has to return `list[Instance]`, and nothing downstream can tell the two apart.

## Running

Nothing here is a program except the test. Everything else is imported, most
often through `core/build_scene_graph.py`.

```bash
cd core
python3 -m tests.test_gazebo                                 # prints the graph, then asserts
python3 -m pytest tests/test_gazebo.py -p no:anyio
```

Typical use:

```python
from scene_graph import gazebo, graph as sg

scene = sg.build(gazebo.load_world("…/apartment.world"))
sg.save(scene, "graph.json")

node = sg.find_object(scene, "pringles", near=(0.0, 0.0))   # nearest of several
furniture_id, relation = sg.location_of(scene, node)
```

## Things worth knowing

- **Coordinates are in the robot's odom frame**, not Gazebo's world frame.
  `gazebo.load_world` subtracts the robot's spawn point (5.0, 6.6 in the
  apartment). Getting this wrong puts every goal about 8 m out.
- **Furniture is decided by name**, via the vocabulary in `instance.py`. A
  source that knows better should pass `movable=` explicitly instead.
- **Walls, floors and doors are dropped** by `build`, so the LLM cannot send
  the robot at a wall.
- **Meshes are scaled by their own COLLADA unit.** Some models are authored in
  decimetres and come out 10x too large otherwise.
