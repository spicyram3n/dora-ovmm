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
| `reach.py` | Ring of base poses from which the *arm* can reach a hand pose, plus `move_into_reach` to drive onto one. Pure numpy, no ROS |
| `nav2_client.py` | Asks Nav2 whether a pose is reachable, then drives to it |

`move_into_reach` takes a navigator rather than importing one, which is what
keeps `reach.py` free of ROS and its tests free of a running stack.

Two things that used to live here and no longer do: `core/reach_report.py`
(offline: which furniture can be grasped from at all) is a runnable script, so
it sits with the other entry points, and the tests are in `core/tests/`.

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

```bash
python3 -m pytest tests/test_reach.py -p no:anyio
python3 reach_report.py                          # what is graspable, per furniture
```

## Seeing versus reaching

Two different rings, and they never overlap.

`standoff.py` answers *where can the head camera see this furniture from* —
0.8 m clear of the footprint, set by Nav2's inflation radius. `reach.py`
answers *where can the arm reach this hand pose from* — at most **0.492 m**
from the target, set by the arm. The standoff ring is always outside the reach
ring, so driving in closer once the object is found is a mandatory stage, not
a refinement -- that second drive is `reach.move_into_reach`.
`tests/test_reach.py` asserts the two do not overlap, so if that ever changes
it fails loudly rather than silently making a stage redundant.

`reach.py` is a numpy port of `hsrb_analytic_ik`'s `GetHsrcBasePositionRange`.
HSR's inverse reachability is closed-form — back the palm offset off the hand
pose to get the wrist centre, and the lift absorbs its height, leaving an
annulus the base can sit anywhere on — so the sampled inverse reachability map
`docs/navigation-and-grasping.md` originally planned is unnecessary.

Numbers worth knowing, all for **hsrc** (hsrb differs by millimetres that are
enough to miss a grasp — `PARAMETERS` carries both and nothing defaults
silently):

| | |
|---|---|
| Max horizontal reach | 0.492 m |
| Min (arm folded) | 0.166 m |
| Wrist-centre height envelope | 0.048 – 1.385 m |
| Top-down grasp ceiling | 1.230 m — above this needs a side grasp |
| Floor picking | works top-down; the palm offset lifts the wrist centre 0.155 m clear |

```python
from navigation import reach
annulus = reach.base_annulus(grasp_pose_4x4)        # None => unreachable, don't drive
if annulus and not reach.reachable_from(grasp_pose_4x4, robot_xy):
    poses = reach.base_poses(annulus, robot_xy)     # nearest first, yaw already set
```

## How a standoff pose is chosen

1. **How far out**: clear of the furniture's *footprint* on that bearing, plus
   `WORKING_DISTANCE` (0.8 m), floored at `MIN_STANDOFF`. The ring is offset
   from the footprint rather than being a circle round the centroid, which
   matters the moment a piece is not square: a 0.5 x 2.3 m table wants 1.05 m
   from its long side and 1.95 m from its end.
2. **Which side**: 12 candidates, ordered by how close that bearing lets the
   robot stand (in 10 cm bands), then by how far it has to drive.
3. **Which one is real**: candidates landing inside another piece's footprint
   are dropped outright (`blockers`) -- a coffee table's broadside wins step 2
   and is exactly where its sofas are, and Nav2 will happily path to such a
   pose and then stop against the sofa. The rest are offered to Nav2's
   `ComputePathToPose`, and the first that returns a path is driven to with
   `NavigateToPose`, then checked against `robot_xy()` to confirm the robot
   really arrived rather than stopping short inside the goal tolerance.

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
| planner | `SmacPlanner2D`, `tolerance: 0.5` | A*. The tolerance is a caveat for `reachable()` — a returned path may end up to 0.5 m short of the pose it accepted |

| controller | MPPI, `motion_model: Omni` | Uses the holonomic base, so the robot strafes onto a standoff pose instead of turn-drive-turn |
