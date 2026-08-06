# navigation

Turns "the object is probably on that shelf" into the robot parked in front of
that shelf, facing it.

Driving to the furniture's own centroid drives into it, so what is wanted is a
pose a little way off. `standoff.py` works out where those poses are; Nav2
decides which of them can actually be reached.

Nav2 owns the base at all times. Nothing here plans arm motion, for the reason
in `docs/navigation-and-grasping.md`: MoveIt's `whole_body` groups command the
same base controller as Nav2 and the two fight over it.

## Files

| File | Purpose |
|---|---|
| `standoff.py` | Ring of candidate poses around a piece of furniture, each facing it. Pure numpy, no ROS |
| `nav2_client.py` | Asks Nav2 whether a pose is reachable, then drives to it |

## Running

Nothing here is a program; `core/search_object.py` is the entry point.

```bash
cd core
python3 search_object.py --furniture high_shelf01 --dry-run   # geometry only, no ROS
python3 search_object.py --furniture high_shelf01             # needs ROS 2 + Nav2 up
```

```python
from navigation import standoff
poses = standoff.candidates(centroid, dimensions, robot_xy=(0.0, 0.0))
```

## How a standoff pose is chosen

1. **How far out**: clear of the furniture's *footprint* on that bearing, plus
   `WORKING_DISTANCE` (0.8 m), floored at `MIN_STANDOFF`. The ring is offset
   from the footprint rather than being a circle round the centroid, which
   matters the moment a piece is not square: a 0.5 x 2.3 m table wants 1.05 m
   from its long side and 1.95 m from its end.
2. **Which side**: 12 candidates, ordered by how close that bearing lets the
   robot stand (in 10 cm bands), then by how far it has to drive.
3. **Which one is real**: each is offered to Nav2's `ComputePathToPose`, and
   the first that returns a path is driven to with `NavigateToPose`.

Step 2 is why the long side of a table wins: everything on it is then within
about a metre, instead of two metres away down the length of it. Step 3 is why
the side of a shelf facing a wall needs no special case — no path reaches it,
so it loses on its own.

stretch-compose instead sizes the standoff so the whole piece fits the camera
frame, and finds the viewpoint by Poisson-meshing a prescanned point cloud and
raycasting for occlusion. Fitting a long table in frame parks the robot 2 m
away, where a can on it is a handful of pixels; the head pans and tilts, so a
wide piece is a sweep rather than a reason to back off.

## Frames

Goals go in **map**, because `bt_navigator`'s `global_frame` is map. `amcl`
starts with its initial pose at the origin and odom starts where the robot
spawned, so map and odom coincide at startup — which is what lets an
odom-frame scene graph be used as a Nav2 goal directly. After a long run AMCL
will have corrected map against odom; if goals start landing slightly off,
that drift is the first thing to check.

## Config this depends on

From `ros2_ws/src/hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml`:

| Setting | Value | Why it matters here |
|---|---|---|
| `robot_radius` / `inflation_radius` | 0.3 / 0.5 | Sets `WORKING_DISTANCE`, so goals stay out of inflated space |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.25 m / 0.25 rad | How accurately the robot ends up facing the furniture |
| planner | NavFn, `use_astar: false` | Dijkstra; flip the flag for A* |
| controller | DWB, `max_vel_y: 0.0` | Differential-drive settings on an omni base, so the robot turns instead of strafing |
