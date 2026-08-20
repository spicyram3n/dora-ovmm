# Active Perception and Next Best View: Brainstorming

Notes toward an occlusion aware next best view planner for the HSR-C.

## 1. The problem

A target object is recorded in the scene graph. After that, someone changes the
scene without updating the graph. The object is now hidden behind something, or
gone.

So the robot has a stale guess about where the object is. It has to move its
camera to either confirm or rule out that guess, and it should prefer cheap
motions over expensive ones.

Two things follow from this:

1. We care about looking *behind* known surfaces, not about exploring open
   space. This is what "occlusion aware" means here.
2. We have a prior. The scene graph tells us where to look first, even if it is
   wrong.

## 2. What we already have

### Octomap is already running

We do not need to install `octomap_server2`. MoveIt's move_group already builds
an octree from the head camera.

The config is `sensors_xtion.yaml`, loaded by `move_group.py:78`:

| setting | value | line |
|---|---|---|
| `sensor_plugin` | `occupancy_map_monitor/PointCloudOctomapUpdater` | `sensors_xtion.yaml:5` |
| `point_cloud_topic` | `/head_rgbd_sensor/depth_registered/rectified_points` | `sensors_xtion.yaml:6` |
| `max_range` | 5.0 | `sensors_xtion.yaml:7` |
| `padding_offset` | 0.1 | `sensors_xtion.yaml:9` |
| `max_update_rate` | 1.0 | `sensors_xtion.yaml:12` |
| `filtered_cloud_topic` | `/head_rgbd_sensor/depth_registered/filtered_cloud` | `sensors_xtion.yaml:13` |

move_group also filters the robot's own body out of that cloud and republishes
it on `filtered_cloud_topic`. That is a useful input even if we do not use the
octree itself. See section 5.6 for why we probably should not use the octree.

### Nav2

`ComputePathToPose` on the planner server returns a path without driving
anywhere. That gives us a free motion cost estimate for any base pose.

The global costmap is already read and rebinned by `core/placement/base_placement.py`
in `costmap_grid`.

### Toyota's IK solver

`core/placement/base_placement.py` wraps `/ik_solver_node/solve_ik_with_collision`.
Give it a `hand_palm_link` pose and it returns every base position the arm can
reach it from, collision free, in well under a millisecond.

This matters more than it looks. See section 5.1.

### Pipeline pieces that carry over unchanged

| piece | what it gives us |
|---|---|
| `perception.camera_ros2.grab_rgbd` | one RGBD frame plus `base_from_camera` |
| `perception.pointcloud.deproject` | depth and mask to 3D points |
| `perception.sam3_client.detect` | the "did we find it yet" test |
| `placement.base_placement.costmap_grid` | base feasibility mask |

## 3. HSR-C viewpoint kinematics

The camera pose is set by five numbers. Every limit is already in the repo.

| DOF | range | max velocity | limit source | velocity source |
|---|---|---|---|---|
| `head_pan_joint` | -3.84 to 1.75 rad | 1.0 rad/s | `head.urdf.xacro:15` | `joint_limits.yaml:27` |
| `head_tilt_joint` | -1.57 to 0.52 rad | 1.0 rad/s | `head.urdf.xacro:43` | `joint_limits.yaml:32` |
| `arm_lift_joint` | 0 to 0.69 m | 0.2 m/s | `arm.urdf.xacro:11` | `joint_limits.yaml:12` |
| `odom_x` | planar | 0.2 m/s | n/a | `joint_limits.yaml:47` |
| `odom_y` | planar | 0.2 m/s | n/a | `joint_limits.yaml:52` |
| `odom_t` | planar | 1.0 rad/s | n/a | `joint_limits.yaml:57` |

Paths: `head.urdf.xacro` and `arm.urdf.xacro` are under
`hsrb_common/hsrc_description/urdf/`. `joint_limits.yaml` is under
`hsrb_moveit/hsrb_moveit_config/config/`. All relative to `ros2_ws/src/`.

### The lift moves the camera

The head is mounted on the torso, and the torso follows the arm lift.

1. `hsrc1s.urdf.xacro:39` attaches the head to `torso_lift_link`.
2. `hsrc1s.urdf.xacro:35` attaches the torso to `base_link` with
   `mimic_joint="arm_lift_joint"`.
3. `torso.urdf.xacro:14` sets that mimic multiplier to 0.5.
4. `torso.urdf.xacro:11` caps the torso travel at 0.345 m.

So 0.69 m of arm lift raises the camera by 0.345 m.

### The base is holonomic, but nav2 is set up as if it is not

This is the most useful thing found in the whole config sweep.

The hardware and MoveIt both treat the base as holonomic:

* `odom_x`, `odom_y` and `odom_t` all have velocity limits
  (`joint_limits.yaml:45-57`)
* all three are in the `base` group (`hsrb.srdf:28`)
* `omni_base_controller` accepts all three as a `FollowJointTrajectory` action
  (`hsrb_controllers.yaml:19-26`)

But nav2's local planner has sideways motion switched off:

* `max_vel_y: 0.0` (`nav2_params.yaml:134`)
* `acc_lim_y: 0.0` (`nav2_params.yaml:140`)

So nav2 will only rotate, drive, rotate. A sideways step is the single most
useful motion for seeing around an obstacle, and nav2 cannot do it. But
`omni_base_controller` can, through the same kind of action we already use for
the head.

## 4. How much each motion costs

### 4.1 Time

Measured in Gazebo, recorded to bags, not estimated. See section 4.5 for the
nav2 configuration these numbers belong to.

| action | time | source |
|---|---|---|
| pan or tilt, about 1 rad | about 1 s | 1.0 rad/s limit, not yet measured |
| sideways step, 0.4 m | about 4 s | 0.2 m/s commanded at 66% efficiency, plus turret swing |
| full lift, 0.69 m | 3.5 s | 0.2 m/s limit, not yet measured |
| nav2 goal, 1 m sideways | **10.1 s** | measured, `bags/mppi` |
| nav2 goal, 5.8 m corridor | **29.8 s** | measured, `bags/corridor_shim` |

The old estimate for a 1 m nav2 goal was about 15 s under DWB. The rebuilt
stack does it in 10.1 s, and long moves improved far more: the same 5.8 m
corridor took 58.1 s before the rotation shim and 29.8 s after.

**The base only delivers about two thirds of commanded velocity.** Over the
5.8 m corridor it was told 0.298 m/s and achieved 0.198 m/s, with the path 99%
straight, so this is not wandering. Before the shim it was worse, 0.101 m/s
actual against 0.217 m/s commanded, 47%. This is the steered twin drive, and it
caps what any controller can do. Assume 66% when costing any base motion,
including the tier 2 sidestep.

### 4.2 Two hidden costs of going through nav2

1. `xy_goal_tolerance: 0.25` and `yaw_goal_tolerance: 0.25`
   (`nav2_params.yaml:124-125`). You arrive up to 25 cm and 14 degrees away from
   the viewpoint you actually scored. Tighten both at runtime for a viewpoint
   goal, see the note above `general_goal_checker` in that file.
2. Every base move feeds AMCL correction into the frame we are accumulating
   depth into. At 2 cm voxels that is a voxel or two of blur.

The camera rotation part of this is now much smaller than feared. Over the 1 m
sideways move the yaw excursion was 6.9 degrees, against 57.2 degrees under the
old DWB config. So a nav2 reposition now translates the camera without swinging
it much. The AMCL correction remains.

Head moves and omni base steps still avoid both, because they stay on odometry.

### 4.3 Energy

Rough numbers. These are estimates. The URDF `effort` values are simulation
placeholders, not real specs, and we do not have measured power draw for the
HSR-C.

| source | energy |
|---|---|
| lift, about 12 kg over about 0.45 m equivalent | about 53 J mechanical, maybe 130 J electrical |
| driving 1 m, friction 0.02 to 0.08 on about 37 kg | 7 to 29 N, so tens of joules |
| platform baseline (PC, LiDAR, RGBD, drivers holding position) | 80 to 150 W |

Now compare. The measured 10.1 s nav2 move burns about 1200 J of baseline power
against about 30 J of actual drive work. A 3.5 second lift burns about 420 J of
baseline against about 130 J of actuator work.

In both cases the actuator effort is a few percent of the total.

**So energy is just duration.** The ranking is the same as time. There is no
reason to build a separate energy term. Keep the cost in seconds and it is
already an energy model. This holds even if the estimates above are two or three
times off.

### 4.4 Which motion actually beats occlusion

Think about parallax, which is how much the viewing angle to the target
changes.

| motion | parallax |
|---|---|
| pan and tilt | none |
| lift | 0.345 m, vertical only |
| sideways step | horizontal, and not really bounded |

Pan and tilt rotate the camera around its own centre. That changes what is in
frame. It never changes what is hidden. Pan and tilt solve "the object is
outside the field of view", which is a different problem from occlusion. Still
worth having, because it is almost free and a bad first framing is common.

For the lift, there is a clean condition. To see a target of height `h_t` behind
an obstacle of height `h_o`, with `d_o` from camera to obstacle and `d_t` from
obstacle to target, the camera has to reach:

```
h_c >= h_o + (h_o - h_t) * d_o / d_t
```

The `d_o / d_t` ratio is what kills it. Take an obstacle 0.3 m taller than the
target at 0.8 m range:

| target sits behind obstacle by | extra camera height needed | within our 0.345 m? |
|---|---|---|
| 0.5 m | 0.48 m | no, but close |
| 0.15 m | 1.6 m | hopeless |

So the lift works when the height difference is small, or when the target is
well clear of the obstacle. Household clutter is often the opposite case.

The useful part: we can check this inequality straight from the voxel grid,
since we know obstacle heights and where the region of interest is. So the
choice of which motion to use can be worked out directly instead of searched
for.

### 4.5 The nav2 stack these numbers came from

Rebuilt over several measured runs. All of it lives in
`hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml`.

| piece | was | now |
|---|---|---|
| global planner | NavFn, Dijkstra | ThetaStar, any angle paths |
| local controller | DWB | MPPI wrapped in RotationShimController |
| motion model | diff drive in practice | `Omni` |
| sideways velocity | disabled, `max_vel_y: 0.0` | 0.2 m/s |
| reversing | allowed | disabled, `vx_min: 0.0` |

Four findings worth keeping:

1. **DWB could not be tuned into this.** With `max_vel_x` 0.3 above `max_vel_y`
   0.2, turning to face the goal genuinely scores better under its progress
   critics, so it rotated 4.99 rad for a 1 m sideways move. MPPI does the same
   move with 0.55 rad.
2. **Enabling strafing was not one parameter.** `min_y_velocity_threshold` was
   0.5, above the 0.2 m/s the base can do sideways, so lateral motion was
   rounded to zero before the controller ever saw it. `min_vel_y` was 0.0, a
   lower bound, so only one direction was reachable.
3. **MPPI needs two critics removed for an omni base.** `PreferForwardCritic`
   penalises any non-forward motion. `PathAngleCritic` penalises heading that
   differs from path direction. Both are in every stock example.
   `TwirlingCritic` is added instead, to penalise pointless spinning.
4. **The laser decides which directions are safe.** `/scan` spans -2.1 to 2.1
   rad, so 240.6 degrees of coverage and a 119.4 degree blind wedge behind.
   Sideways travel at 90 degrees sits inside the fan with 30 degrees to spare,
   so strafing is sensed. Reversing is not, hence `vx_min: 0.0`. Across both
   corridor runs, 0% of samples had the travel direction inside the wedge.

## 5. What the research says

### 5.1 The closest published system is ActPerMoMa

[ActPerMoMa](https://arxiv.org/html/2310.00433) does active perception for
mobile manipulation on a TIAGo++. That robot has a holonomic 3 DOF base, a
torso, a 2 DOF head and an arm. Very close to the HSR.

What they do:

1. **5D view sampling.** Base x, y, theta plus head pan and tilt. They sample
   base poses near the target, then build camera poses at those base poses.
2. **They keep base and arm or head motion separate**, and say they do it "for
   simplicity". So the tiered approach is what the closest published system
   does, not a shortcut we invented.
3. **Rear side voxel information gain**, not occlusion aware. See 5.2.
4. **Grasp reachability from a precomputed reachability map**, scored as
   `J_exec = max_reachability / path_length`, and combined as
   `argmax[J_IG + J_exec]`.

Their results: 92.6 percent success against 86.8 percent for a naive baseline in
complex scenes, and 80 percent on real hardware.

The thing to notice is point 4. They had to precompute a reachability map. We
have a live one, through `ik_solver_node`. That is better, and it means the same
candidate poses can be scored both for information gain and for "could I
actually grasp it from here" in one pass.

One correction to an earlier claim in this doc: ActPerMoMa uses base and head
only. It does not use the torso as a viewpoint DOF at all. That is evidence
against leaning on the lift.

### 5.2 "Occlusion aware" is a specific metric, and it loses

This was the biggest surprise.

"Occlusion aware" is not a general description. It is a named metric from
[Isler et al., ICRA 2016](https://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf):

```
I_OA(v)  = P_v(v) * H(v)
P_v(v_n) = product over i = 0..n-1 of (1 - P_o(v_i))
H(v)     = -P_o ln P_o - (1 - P_o) ln(1 - P_o)
```

In words: weight each voxel's entropy by how likely it is to be visible, and
visibility is the product of "is empty" probabilities for everything in front of
it along the ray.

[Delmerico et al., Autonomous Robots 2018](https://link.springer.com/article/10.1007/s10514-017-9634-0)
then benchmarked six of these against each other. On their headline coverage
measure the top three were:

| metric | score |
|---|---|
| Proximity Count | 0.9446 |
| Rear Side Voxel | 0.9440 |
| Rear Side Entropy | 0.9439 |

Occlusion Aware and Average Entropy trailed behind these.

Caveat: the original PDF would not parse and the CORE mirror returned 403, so
those exact numbers come from secondary sources. The ranking is backed up by who
adopted what.

The winners are all in the "rear side" family. Those metrics count unknown
voxels sitting *behind* surfaces we have already seen. That is exactly the
behaviour we want, but it is not the metric literally called "occlusion aware".
Proximity Count is Rear Side Voxel with a weight that falls off with distance
from a known surface.

Backing this up: ActPerMoMa chose Rear Side Voxel. Zaenker chose Proximity
Count. Nobody building a targeted system chose Occlusion Aware.

The top three are within 0.001 of each other, so pick any of them and move on.

### 5.3 Use `IG - lambda * C`, not `IG / C`

The standard form in the field is subtraction:

```
next_view = argmax over v of [ U(v) - lambda * D(current, v) ]
```

* Zaenker uses `U = IG - alpha * C`.
* Others use `U = IG * exp(-lambda * d)`.
* ActPerMoMa uses a ratio, but squared and along a path, `IG_rear / d^2`.

A plain `IG / C` ratio is the least used, and it breaks badly at our numbers.

As `C` goes to zero the ratio blows up. Our cheapest action is about 1 second
and our most expensive is about 15 seconds. That is a 15 times spread. With a
plain ratio, a 1 second head twitch with almost no gain beats a 15 second base
move with large gain, every time.

The failure mode is subtle. The robot would spend five views wiggling its head
at an obstacle it can never see past, and nothing would look broken.

Use `IG - lambda * C`. Tune lambda so that one base move has to beat roughly N
head moves. That is one number with a meaning you can explain.

### 5.4 Targeting a region of interest is worth about 2x

This is the strongest experimental result found.

[Zaenker et al.](https://ar5iv.labs.arxiv.org/html/2011.00275) compared
targeting a known region of interest against plain frontier exploration:

| | region targeted | exploration only |
|---|---|---|
| objects detected | 23.0 +/- 2.9 | 17.4 +/- 3.9 |
| target volume covered | 0.68 +/- 0.10 | 0.34 +/- 0.08 |

Twice the coverage on the metric that matters. This is the argument for using
the stale scene graph position as a prior instead of just exploring.

How they find the targets: frontier voxels next to known region of interest
voxels, using a 6 neighbour check.

One more trick worth copying. They throw away any viewpoint whose ray to the
target passes through an occupied cell, before computing information gain at
all. That removes most candidates for almost no cost.

Their resolutions: 1 cm planning octree, 2 cm workspace octree.

### 5.5 Stale priors specifically

[IGV-RRT](https://arxiv.org/html/2603.21887v2) is aimed at exactly our problem,
where an object gets moved and the old scene knowledge goes stale.

Their stated principle is to treat the prior as "informative but potentially
biased", and not assume it stays accurate. They fuse it with live observation
that is allowed to override it. They report 64.91 percent success against 49.76
percent for VLFM on scenarios with relocated objects.

For us this means: do not weight the region of interest heavily forever. If we
look at the place the scene graph claims and see it is empty, drop that weight
and let the rear side term take over. Otherwise we keep scoring views of a spot
we already disproved.

### 5.6 Why not use octomap, when everyone else does

Everyone in the literature uses octomap. Isler, Delmerico, Zaenker, ActPerMoMa.
We are going against standard practice, so the reasoning has to hold up.

They use it because of scale. Whole plants, whole rooms, walking all the way
around an object, at 1 cm resolution.

The strongest evidence for our case is what Zaenker had to do. He needed one
extra value per voxel, a region of interest probability alongside occupancy.
Stock octomap cannot carry that, so he forked it
([roi_viewpoint_planner](https://github.com/Eruvae/roi_viewpoint_planner)). That
is exactly the change we would need, and it is a C++ fork, not a config option.

Our region of interest is a single box about 1.2 m across. A plain dense array
carries as many values per voxel as we want, for free.

There are also reasons not to reuse move_group's octree in particular:

1. `/clear_octomap` can be called by anything on the graph. We do not control
   its lifetime.
2. `max_update_rate: 1.0` throttles updates to MoveIt's schedule, not ours.
3. No per voxel weighting for our region of interest.
4. Unknown space is implicit. Octomap represents unknown as a missing node, so
   listing "all unknown voxels in this box" means walking the box and querying
   every cell anyway. That is the dense array we would have built. The operation
   we need most is the one an octree is worst at.
5. `padding_offset: 0.1` inflates obstacles. That is right for not hitting
   things and wrong for measuring what is visible.

So: leave move_group's octree alone doing collision checking for the arm. Run
our own grid off the same depth stream. Two independent readers, no shared
state, no race on `/clear_octomap`.

## 6. The design

### 6.1 Map

One dense numpy log odds grid over a box about 1.2 m across, centred on the
target's last known position from the scene graph. At 2 cm that is 60 x 60 x 60,
so 216 thousand voxels. We never need the whole apartment. We only need the
volume that could be hiding the target.

Because the grid is fixed, we can skip raycasting entirely and use projection
instead.

**Integrating a new frame.** Project every voxel centre into the depth image.
Then compare:

| voxel depth vs measured depth | meaning |
|---|---|
| less than measured minus tau | free |
| within tau | occupied |
| greater than measured plus tau | leave unknown, it is hidden |

**Predicting gain for a candidate view.** Same projection, then a depth buffer.
`np.minimum.at` over pixel index gives the nearest occupied depth per pixel. A
voxel is visible if its depth is less than that. For the soft Isler weight,
accumulate the running product of `(1 - P_o)` in depth order instead of using a
hard cutoff.

Both are vectorised. Neither needs a ray marching loop.

### 6.2 Information gain

Rear side family, weighted up inside the region of interest:

```
IG(view) = sum over visible v of  w(v) * P_v(v) * H(p_v)
```

| condition on voxel v | weight |
|---|---|
| unknown, and has an occupied 6 neighbour | 3.0 |
| unknown, and inside the scene graph region of interest | 2.0 |
| anything else | 0.1 |

The weights multiply, so a voxel in the target's shadow scores 6x. Open unknown
space is deliberately not rewarded.

### 6.3 Utility

```
U = IG - lambda * C          with C in seconds
```

### 6.4 Candidate tiers

Costs updated from the measurements in section 4.1.

| tier | motion | cost | what it is for |
|---|---|---|---|
| 1 | pan and tilt | about 1 s | fixes framing, no parallax |
| 2 | sideways step, 0.3 to 0.5 m | about 4 s | the main tool against occlusion |
| 3 | lift | 3.5 s | only when the section 4.4 inequality says it can work |
| 4 | nav2 | 10 s at 1 m, 30 s at 6 m | cross the room or route around obstacles |

Tier 2 is 4 s rather than the 2 to 3 s first assumed, because the base only
delivers about 66% of commanded velocity and the turret needs roughly 0.9 s to
swing 90 degrees before lateral motion starts.

Tier 4 is far cheaper than the original 15 s estimate, so it should be selected
more readily than the first draft of this document implied. Scale it as roughly
6 s of fixed overhead plus distance at 0.2 m/s.

Tiers 1 to 3 keep the base on odometry rather than AMCL, so the grid stays
sharp.

Before scoring any candidate, throw it out if the ray to the target hits an
occupied voxel. Score reachability alongside information gain using
`ik_solver_node`.

### 6.5 The loop

```
grab a frame
integrate it into the grid
run sam3_client.detect

if found and score above threshold:
    hand off to the existing grasp pipeline, done
else:
    score the candidates, execute the best one, repeat
    stop after about 6 views
```

Also stop when the best information gain falls below a threshold. That means
there is nothing left worth looking at, so the target really is not here.

### 6.6 How to execute each motion

| tier | how |
|---|---|
| head | `FollowJointTrajectory` on `/head_trajectory_controller/follow_joint_trajectory` (`hsrb_controllers.yaml:28`), or MoveIt's `head` group (`hsrb.srdf:24`) |
| sideways step | `FollowJointTrajectory` on `omni_base_controller` (`hsrb_controllers.yaml:19`), or MoveIt's `whole_body` group (`hsrb.srdf:33`) |
| nav2 | `NavigateToPose` |

### 6.7 Scope

New code under `core/nbv/`, roughly 400 lines.

| file | what | lines |
|---|---|---|
| `voxel_grid.py` | dense log odds grid, integrate, predicted visibility | about 110 |
| `viewpoints.py` | HSR candidate generation and batched forward kinematics | about 80 |
| `planner.py` | information gain, utility, selection | about 70 |
| `execute.py` | head action, omni base action, nav2 | about 80 |
| `run_nbv.py` | the loop | about 60 |

Nothing existing has to change. Nothing new has to be installed.

## 7. Build plan

Seven steps. Each one produces something checkable on its own, so a mistake in
step 2 does not surface as confusing behaviour in step 6. Nothing here is
written yet.

### Step 0. The test scene

Everything else needs a scene where we already know the right answer.

`tmc_gazebo_worlds` ships 99 models, including exactly what this needs:
`hsr_pringles`, `hsr_coke`, `hsr_soda`, `kitchen_table`, `high_table`,
`book_box`, `cosmetic_box`, `first_aid_box`, `towel_box`. `hsr_pringles` is
already the target the existing pipeline is tested against.

Build one world with three cases on a `kitchen_table`:

| case | layout | which tier should win |
|---|---|---|
| A, not occluded | pringles in the open | none, first look finds it |
| B, shallow occlusion | pringles behind a `cosmetic_box` slightly taller, target well back | lift, per the 4.4 inequality |
| C, deep occlusion | pringles tucked right behind a tall `book_box` | sidestep, lift cannot clear it |

Record the true target pose from Gazebo. That is the ground truth every later
step is graded against. Case A is the control: if the planner ever moves for
case A, something is wrong.

**Done when:** the world launches, the robot spawns facing the table, and the
true pose of the pringles can is written down for all three cases.

### Step 1. Record datasets, do not go live

Drive the head through a grid of pan, tilt and lift values and record RGBD,
`camera_info` and tf to a bag for each case. Perhaps 30 poses.

This is the single highest leverage step. With bags, steps 2 to 4 are offline
numpy work that runs in a second and is fully repeatable. Without them, every
change means relaunching Gazebo.

**Done when:** three bags exist, one per case, each with frames from known head
configurations.

### Step 2. The voxel grid

Dense log odds array over a 1.2 m box at 2 cm, centred on the scene graph's
claimed target position. Integration by projection, not raycasting: project
voxel centres into the depth image and compare depths, as in section 6.1.

**Done when:** replaying a bag builds a grid whose occupied voxels line up with
the table and the boxes, and whose unknown region sits behind the occluder
where we can see it should be. Check by exporting a `.ply` and opening it, the
same way `pointcloud.save_ply` already works.

**Watch for:** the depth is registered to the rgb frame, so rgb intrinsics
apply. `camera_ros2.py` already handles the 16UC1 millimetres versus 32FC1
metres difference between the real robot and Gazebo.

### Step 3. Rear side voxel information gain

Implement the metric from section 5.2 and 6.2: visibility weight
`P_v = product of (1 - P_o)` along the ray, times entropy, times the rear side
and region of interest weights.

**Done when:** scoring a set of hand-picked viewpoints against a grid from
step 2 ranks them the way a human would. For case C, a viewpoint to the side
must beat a viewpoint from straight on. If it does not, the metric is wrong,
and finding that out here costs minutes rather than debugging it on a moving
robot.

### Step 4. Viewpoints and forward kinematics

Generate candidates over pan, tilt and lift, and turn each into a camera pose.
Compose the transform in numpy from constants cached once from tf, as in
section 6.1.

**Done when:** for a configuration the robot actually held during step 1, the
computed camera pose matches the tf recorded in the bag to within a centimetre.
That is a direct check against ground truth, so do not skip it.

### Step 5. Utility and selection

`U = IG - lambda * C`, with `C` from the measured costs in section 6.4. Add the
prefilter from section 5.4, discarding any viewpoint whose ray to the target
passes through an occupied voxel, before computing information gain at all.
Score reachability alongside, using `base_placement.solve`.

**Done when:** given a grid and the current robot state, it returns a ranked
list of viewpoints with their information gain and cost broken out separately,
so a bad choice can be traced to whichever term caused it.

### Step 6. Execution and the closed loop

Wire up the three action clients from section 6.6, then close the loop from
section 6.5.

**Done when:** case A resolves without moving, case B resolves with a lift, and
case C resolves with a sidestep. Log which tier resolved each run, per open
question 4.

### What we are taking from where

| source | what we take | what we do not |
|---|---|---|
| [ActPerMoMa](https://github.com/pearl-robot-lab/ActPerMoMa) ([paper](https://arxiv.org/html/2310.00433)) | the overall shape: decoupled base and head tiers, rear side voxel information gain, scoring reachability next to information gain | its Isaac Sim and TIAGo stack, and its precomputed reachability map, since `ik_solver_node` gives us a live one |
| [rpg_ig_active_reconstruction](https://github.com/uzh-rpg/rpg_ig_active_reconstruction) | the reference implementations of the information gain metrics, for checking our rear side voxel maths against | the octomap dependency and the C++ pipeline |
| [roi_viewpoint_planner](https://github.com/Eruvae/roi_viewpoint_planner) | region of interest targeted sampling, the 6 neighbour frontier check, the ray prefilter | its forked octomap, which is exactly the thing we avoid by using a dense array |
| [IGV-RRT](https://arxiv.org/html/2603.21887v2) | treating the prior as biased, decaying the region of interest weight on disconfirming observation | its vision language model layer, we have SAM3 already |

ActPerMoMa is the one to read first. It is the same problem on a very similar
robot, and its code is the closest thing to a reference implementation, even
though we will not run it.

## 8. Open questions

1. Does the omni base hold heading well enough during a sideways step that the
   integrated depth does not smear? The controller exists
   (`hsrb_controllers.yaml:19`), so this is now a tuning question, not a
   feasibility one.
2. Does the scene graph reliably give a target position? If not, fall back to
   the surface the object was last seen on.
3. What should lambda be? Start from "one base move has to beat about 4 head
   moves" and adjust.
4. Log which tier resolved each trial. About two dozen runs gives the real
   distribution for our scenes, which beats any estimate in this document.

## 9. Config trace

Everything cited above, in one place. Paths relative to `ros2_ws/src/`.

| what | where |
|---|---|
| head pan limit, -3.84 to 1.75 rad | `hsrb_common/hsrc_description/urdf/head_v0/head.urdf.xacro:15` |
| head tilt limit, -1.57 to 0.52 rad | `head.urdf.xacro:43` |
| arm lift limit, 0 to 0.69 m | `hsrb_common/hsrc_description/urdf/arm_v0/arm.urdf.xacro:11` |
| torso travel cap, 0.345 m | `hsrb_common/hsrc_description/urdf/torso_v0/torso.urdf.xacro:11` |
| torso mimic multiplier, 0.5 | `torso.urdf.xacro:14` |
| torso attached to base_link | `hsrb_common/hsrc_description/robots/hsrc1s.urdf.xacro:35` |
| head attached to torso_lift_link | `hsrc1s.urdf.xacro:39` |
| joint velocity limits | `hsrb_moveit/hsrb_moveit_config/config/joint_limits.yaml:12,27,32,47,52,57` |
| `head` planning group | `hsrb_moveit/hsrb_moveit_config/config/hsrb.srdf:24` |
| `base` planning group, odom x y t | `hsrb.srdf:28` |
| `whole_body` planning group | `hsrb.srdf:33` |
| `omni_base_controller`, odom x y t | `hsrb_moveit/hsrb_moveit_config/config/hsrb_controllers.yaml:19` |
| `head_trajectory_controller` | `hsrb_controllers.yaml:28` |
| octomap updater config | `hsrb_moveit/hsrb_moveit_config/config/sensors_xtion.yaml:5-13` |
| where that config is loaded | `hsrb_moveit/hsrb_moveit_config/launch/move_group.py:78` |
| nav2 goal tolerances, 0.25 and 0.25 | `hsrb_rosnav/hsrb_rosnav_config/config/nav2_params.yaml:124-125` |
| nav2 local planner, DWB | `nav2_params.yaml:129` |
| `max_vel_x: 0.3` | `nav2_params.yaml:133` |
| `max_vel_y: 0.0`, sideways disabled | `nav2_params.yaml:134` |
| `max_vel_theta: 0.3` | `nav2_params.yaml:135` |
| `acc_lim_y: 0.0` | `nav2_params.yaml:140` |

## 10. Sources

* [Isler et al., An Information Gain Formulation for Active Volumetric 3D Reconstruction, ICRA 2016](https://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf)
* [Delmerico et al., A comparison of volumetric information gain metrics for active 3D object reconstruction, Autonomous Robots 2018](https://link.springer.com/article/10.1007/s10514-017-9634-0)
* [rpg_ig_active_reconstruction, the reference implementation](https://github.com/uzh-rpg/rpg_ig_active_reconstruction)
* [Zaenker et al., Viewpoint Planning for Fruit Size and Position Estimation](https://ar5iv.labs.arxiv.org/html/2011.00275)
* [roi_viewpoint_planner](https://github.com/Eruvae/roi_viewpoint_planner)
* [ActPerMoMa, Active Perceptive Motion Generation for Mobile Manipulation](https://arxiv.org/html/2310.00433)
* [IGV-RRT, Prior Real Time Observation Fusion for Active Object Search in Changing Environments](https://arxiv.org/html/2603.21887v2)
* [OA-NBV, Occlusion Aware Next Best View Planning for Human Centered Active Perception](https://arxiv.org/html/2603.11072)
* [Potthast and Sukhatme, A probabilistic framework for next best view estimation in a cluttered environment](https://robotics.usc.edu/~potthast/potthast_vua2014.pdf). Abstract only, the host certificate has expired.
* [Next best view problem overview, for the standard utility form](https://www.emergentmind.com/topics/next-best-view-problem)
