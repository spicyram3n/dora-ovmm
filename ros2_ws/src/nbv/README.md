# nbv — next best view for the HSR-C

Where the robot should look, and stand, to see enough of a target object to
grasp it. Following [ethz-asl/active_grasp][1] (Breyer et al., IROS 2022), with
our own perception stack substituted for theirs:

| active_grasp | here |
|---|---|
| TSDF volume (`UniformTSDFVolume(0.3, 40)`) | OctoMap, through `octomap-python` |
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

`build_octomap.py` — an OctoMap of everything the head RGBD stream sees,
reported over **one sphere** around the target. It subscribes to the depth image and CameraInfo and
deprojects them itself, rather than to the `rectified_points` cloud: a third of
the bytes over DDS for the same points, and the pixel stride then skips pixels
before they are ever deprojected.

```bash
ros2 run nbv build_octomap.py --ros-args \
  -p sphere_centre:="[2.0, 1.0, 0.7]" -p sphere_radius:=0.6
```

Publishes occupied cell centres on `/nbv/occupied` as a PointCloud2 and the
whole map on `/nbv/octomap`, to view in RViz. Other processes query the map
through `/nbv/unknown_fraction` instead of copying it. Everything else — topic, 5 cm cells, 4-pixel stride, 2.5 m range — is a
constant at the top of the script.

### Why it is shaped this way

OctoMap, not our own grid. We had a dense numpy grid; on 40 real frames it
needed 6.7 GB at 2.5 m / 1 cm where OctoMap needed 154 MB, and it hid a
free-space bug. `octomap-python` matched C++ OctoMap cell for cell.
`octomap-python`, not `pyoctomap`: pyoctomap 1.2.2 returns occupied cells half
a cell off.

Every ray goes into the map, not only those ending in the sphere. A ray that
crosses the sphere to a wall behind it is what makes that stretch free, and
OctoMap's bounding-box mode would drop it.

Two downsamples before OctoMap sees a frame: pixel stride on the depth
image, then the range cut (done on the depth values themselves, before any
arithmetic, taking missing depth with it). OctoMap then updates each cell once
per frame however many points land in it. ~15 ms per frame at 5 cm, 19k rays.

If something downstream ever needs a real `octomap_msgs/Octomap`, don't write
one: `octomap_server` is already installed, and can be pointed at a cropped
cloud.

## Still to do

- [x] **Free space from empty rays.** A pixel that returns nothing carves its
      ray out to `MAX_RANGE` instead of being dropped; without it the air in
      front of the camera stayed unknown for ever and information gain would
      have kept proposing views of it.
- [ ] **Information gain.** Unknown cells in the sphere (`getLabels == -1`)
      are the quantity to shrink. active_grasp raycasts each candidate view
      through the grid and counts unique unknown cells hit, stopping at the
      first surface; OctoMap's `castRay` stops at the first occupied cell.
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
