# nbv — next best view for the HSR-C

Where the robot should look, and stand, to see enough of a target object to
grasp it. Following [ethz-asl/active_grasp][1] (Breyer et al., IROS 2022), with
our own perception stack substituted for theirs:

| active_grasp | here |
|---|---|
| TSDF volume (`UniformTSDFVolume(0.3, 40)`) | occupancy grid, `nbv/occupancy.py` |
| VGN grasp network | GraspGenX |
| — | SAM3 for the target detection |
| Franka + wrist D435, ROS Noetic | HSR-C + head RGBD, ROS Humble |

[1]: https://github.com/ethz-asl/active_grasp

**Which one we follow.** The *structure* is [ActPerMoMa][2]'s, because that is
the one built for a mobile manipulator with a head camera: the decision
variable is where the base goes, and the utility trades information gain
against grasp reachability over a path. active_grasp assumes a wrist camera on
a fixed base, where views come free along the reach and the base is never in
question — neither holds for the HSR. The *mechanics* are active_grasp's,
because they are simpler and enough: an object-anchored volume of fixed size,
raycast forward from a candidate view, count the unknown cells it would
uncover. Both anchor to the target, never to the robot.

[2]: https://arxiv.org/abs/2310.00433

## What exists now

`build_octomap.py` — occupancy over **one sphere** of the workspace, from the
head RGBD stream. It subscribes to the depth image and CameraInfo and
deprojects them itself, rather than to the `rectified_points` cloud: a third of
the bytes over DDS for the same points, and the pixel stride then skips pixels
before they are ever deprojected.

```bash
ros2 run nbv build_octomap.py --ros-args \
  -p sphere_centre:="[2.0, 1.0, 0.7]" -p sphere_radius:=0.6
```

Publishes occupied cell centres on `/nbv/occupied` as a PointCloud2, to view in
RViz. Everything else — topic, 5 cm cells, 4-pixel stride, 2.5 m range — is a
constant at the top of the script.

### Why it is shaped this way

A dense array, not an octree. An octree earns its keep by compressing the empty
space outside the region of interest, and we crop that away before storing
anything; active_grasp's dense 40³ volume is the same call. A 0.6 m sphere at
5 cm is 24³ cells, so information gain is a numpy mask rather than a tree walk.
The update rule, clamps and 0.5 threshold are octomap's.

Three downsamples, cheapest first, so nothing expensive sees a full frame:
pixel stride on the depth image, then the range cut (done on the depth values
themselves, before any arithmetic, taking missing depth with it), then
deduplicating points into cells before raycasting. The last one is what matters — a frame puts hundreds of points in
one cell, all with the same ray behind them. ~7 ms per frame at 19k points.

If something downstream ever needs a real `octomap_msgs/Octomap`, don't write
one: `octomap_server` is already installed, and can be pointed at a cropped
cloud.

## Still to do

- [x] **Free space from empty rays.** A pixel that returns nothing carves its
      ray out to `MAX_RANGE` instead of being dropped; without it the air in
      front of the camera stayed unknown for ever and information gain would
      have kept proposing views of it.
- [ ] **Information gain.** `SphereGrid.unknown()` is the quantity to shrink.
      active_grasp raycasts each candidate view through the grid and counts
      unique unknown cells hit, stopping at the first surface — the same
      raycast as `integrate`, run forward and counting instead of updating.
- [ ] **Candidate views.** active_grasp samples a sphere around the target
      (`thetas = [15°, 30°]`, `phis = 8 × 45°`) and keeps the ones IK can
      reach. Ours must go through the HSR's full-body kinematics: head pan and
      tilt, arm lift and flex, and torso together.
- [ ] **Base placement.** The base is part of that chain, so the candidate set
      spans base poses too, scored against navigation cost and checked for
      collisions against the map. active_grasp has no mobile base and so no
      equivalent; this is the part we cannot copy.
- [ ] **Graspability.** Their utility is `gain/Σgain − cost/Σcost` with cost
      fixed at 1. Since the point of looking is grasping, weight views that
      would reveal graspable surface, via GraspGenX, rather than unknown volume
      alone.
- [ ] **Sphere placement** from the scene graph: the target instance's
      centroid, radius from its bounding box plus gripper clearance. Right now
      it is a launch parameter.
