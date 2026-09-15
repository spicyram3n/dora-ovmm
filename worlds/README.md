# Kitchen object navigation scene

`kitchen_objects.world` preserves the kitchen-table arrangement: Pringles,
orange soda can, apple, two cereal boxes, Rubik's cube and spray bottle.
Room doors and their stoppers are removed, as are the two sofas from the
existing demo. Cabinet doors remain part of their furniture models.

The HSR starts beside the high table in the other room: world `(5.6, 4.0, 0)`,
yaw pi. It must navigate to the requested object. `launch/apartment.launch.py`
sets that spawn and the model resource paths; `launch/search.launch.py` starts
that scene, Nav2, IK, MoveIt and the mission tree. The matching initial AMCL
pose is map `(0.6, -2.6, pi)`. World-to-map translation remains `(-5, -6.6)`.

```bash
ros2 launch /home/ws/launch/search.launch.py target:="spray bottle" mode:=grasp
```

Use `mode:=pickup` to request verified lifting, or `grasp:=false` for search and
parking only. A grasp/pickup is not guaranteed. The September 14 kitchen-start
trial found the bottle and completed parking/approach, but failed bilateral
closure; it was correctly reported as a failed grasp.

The six table-object model directories are bundled in `assets/models`.
The standard apartment walls and furniture come from `tmc_gazebo_worlds`,
installed by the existing workspace setup. Toyota asset licensing is retained
in `assets/models/TOYOTA_LICENSE.txt`; the world derives from that same package.
No absolute diagnostic-directory paths are needed.

`kitchen_objects.json` records object placements, removed models and spawn.
The world is a reproducible startup scene, not a serialized mid-grasp state.
`config/map/kitchen_objects.pgm` starts with the demo's sofa-free map, clears
removed door collision footprints at the laser plane, and preserves wall
collision cells. Live laser costmaps still handle obstacles. Scene-graph
object records use the same table layout in `config/scene_graph/kitchen_objects.json`.
