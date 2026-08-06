# Navigation and grasping plan

This is a design doc, not code yet. It captures what was worked out about how
the robot should get from "an LLM picked a piece of furniture" to "the object
is actually grasped," so this can be implemented later without re deriving it.

The LLM and scene graph part is out of scope here. This starts from "we have
a furniture location to go to" and ends at "the grasp executes."

## The problem in one paragraph

The LLM only gives a furniture level location, not an object pose, since the
object might not even be in the scene graph yet. So the robot has to travel
to the furniture, search for the object once it is close, then place its own
base somewhere that actually lets the arm reach the object. Getting that base
placement right is what decides whether the grasp works at all, more than
the grasp planning itself does.

## Important finding: MoveIt's `whole_body` group fights Nav2

This was actually hit on the real setup (base drifting, MoveIt and Nav2
fighting for control), so it is not theoretical.

There are two separate paths that both end up commanding the same base
controller (`omni_base_controller`, type `hsrb_base_controllers/OmniBaseController`):

1. Nav2's `controller_server` publishes velocity commands, which flow through
   `safety_velocity_limiter` and `tmc_velocity_switcher`, and land on
   `omni_base_controller/cmd_vel`. See
   `ros2_ws/src/hsrb_launch/hsrb_common_launch/launch/navigation.py`.
2. MoveIt's `whole_body`, `whole_body_weighted`, and `whole_body_light` groups
   include the `odom_x, odom_y, odom_t` virtual joints (see `hsrb.srdf`), and
   when a plan on one of those groups executes, it sends a
   `FollowJointTrajectory` goal straight to `omni_base_controller`'s action
   interface. See `hsrb_controllers.yaml`.

`tmc_velocity_switcher` only arbitrates between teleop and Nav2. It has no
idea MoveIt's trajectory action exists. Nothing in this workspace stops both
paths from writing to the base at the same time. That is the drifting.

**Decision: never use a `whole_body*` MoveIt group during manipulation.**
Nav2 owns the base at all times, full stop. MoveIt only ever plans on the
`arm` group (fixed base) once the robot has already stopped. This also means
positioning the base well before grasping starts is not optional, it is the
only mechanism left for getting the object in reach, since MoveIt itself is
no longer allowed to move the base.

## Part 1: getting close (Nav2, unchanged)

Nothing new here. Nav2 is already configured
(`ros2_ws/src/hsrb_rosnav/hsrb_rosnav_config`) and does what it already does:
global planning, costmaps, obstacle avoidance while driving. The furniture
location from the scene graph becomes a normal Nav2 goal, offset out from the
furniture's footprint so the robot ends up facing it, not inside it.

## Part 2: searching for the object

Once near the furniture, the object's exact position is still unknown.

1. From the current base pose, sweep the head (pan and tilt) across the
   furniture's known height and width range. Run SAM3 detection on each
   frame until the queried object is found, or the sweep runs out.
2. If nothing is found, the current viewpoint might be the wrong side of the
   furniture, or the object might be occluded from here. Send a short Nav2
   goal to another standoff point around the furniture (front, and maybe one
   or two angles if it is something like an island or table) and repeat.

This is a plain "move, look, detect, repeat if empty" loop. Nothing exotic,
just needs a bound on how many viewpoints to try before giving up.

## Part 3: positioning so the object is actually graspable

This is the part that replaces what `whole_body` used to be relied on for.

### Step 1: build an inverse reachability map, once, offline

This does not depend on which object is being grasped, only on the arm's own
kinematics, so it can and should be built ahead of time, not per query.

- Sample a dense grid of end effector target poses (position and approach
  direction) relative to the base frame.
- For each one, solve IK using `hsrb_analytic_ik` (the same analytic solver
  MoveIt's own `HSRBKinematicsPlugin` wraps, so this is fast, well under a
  millisecond per query, cheap enough to sample very densely).
- Store, for each relative pose bucket, whether it is solvable and a
  manipulability score.
- Invert it: given a target pose, look up which base offsets (dx, dy, dyaw)
  would make it reachable, ranked by score.

This is the standard "inverse reachability map" technique from mobile
manipulation (the ROS package "Reuleaux" is the well known reference
implementation, worth reusing rather than rederiving the math).

### Step 2: once the object is found, look up candidate base poses

GraspGenX already returns several ranked grasp poses per object (see
`core/run_pipeline.py`, saved as `grasps.yaml`). For each candidate grasp:

1. Query the inverse reachability map for base offsets that would make that
   grasp pose reachable.
2. Rank the results by manipulability score, times the grasp's own
   confidence score, times how close the offset is to the robot's current
   position (prefer not moving far), and drop anything that does not fit in
   the local costmap's free space.

### Step 3: validate the top candidates before actually driving there

The reachability map is kinematics only, it knows nothing about what is
actually in the room right now. Before committing to a Nav2 goal, validate
the top few candidates with `tmc_ik_solver_node`'s `SolveIkWithCollision`
service. This node was read directly (not just its `.srv` file) to get the
mechanics right, since a few things about it are not obvious from the
message definition alone:

- The service is registered as `~/solve_ik_with_collision`, a private name.
  The real full service name depends on how the node is launched (the
  example launch runs it unnamespaced as `ik_solver_node`, giving
  `/ik_solver_node/solve_ik_with_collision`). Confirm the real name before
  wiring anything to it.
- **It does not read `environment.octomap`.** The `.srv` type
  (`moveit_msgs/PlanningSceneWorld`) has an octomap field, but the node's
  callback only loops over `environment.collision_objects`. Do not bother
  building an octomap encoder for this call, it would be ignored.
- **It does no coordinate transforms at all.** The code comment says so
  directly: it assumes every pose passed in is already in the same frame.
  `origin_to_hand_goal`, `initial_origin_to_base`, and every collision
  object's pose all need to already be in one consistent frame (use `odom`,
  same as everything else in this pipeline) before the request is built.
- Field by field:
  - `origin_to_hand_goal`: the candidate grasp pose from GraspGenX.
  - `environment.collision_objects`: known objects only (the target's
    bounding box, furniture). Can reuse `moveit_msgs/CollisionObject`
    directly from MoveIt's own `/get_planning_scene` service.
  - `environment.octomap`: leave empty, it is ignored.
  - `obstacle_map`: the current Nav2 costmap (`nav_msgs/OccupancyGrid`),
    passed straight through. This is what actually checks base footprint
    collisions, separately from the arm level check.
  - `initial_joint_state`: latest `/joint_states`.
  - `initial_origin_to_base`: the **hypothetical** base pose being tested,
    not necessarily where the robot currently is. This is the whole point,
    it lets many candidate base poses be checked without physically moving
    to any of them.
- The example launch (`hsrb_ik_solver_node/example/example.launch.py`) uses
  the plugin `hsrb_ik_solver_node::HsrbIkSolverPluginRobustToBasePositionError`
  rather than the plain solver. That is a good fit here, since these are
  being tested before Nav2 has actually driven and settled at that pose, so
  some localization and stopping error is expected.

Send the winning, validated pose as a short Nav2 goal (this is a local
reposition, usually well under a meter, not a long drive).

### Step 4: re check after arriving

Nav2's stopping error and a slightly different viewpoint both mean the
earlier detection should not just be trusted blindly. Re run SAM3 and
GraspGenX once more from the final pose before handing off to grasp
execution.

## Part 4: the grasp itself, once positioned (arm only)

### Collision checking: use both the octomap and explicit objects, not one or the other

These are not alternatives, MoveIt runs them as two separate layers merged
into one planning scene, and real setups use both at once.

- **Octomap** (`sensors_xtion.yaml`, already configured, always on) is the
  catch all safety net. It does not know what anything is, it just marks
  "something is here" from whatever the depth camera currently sees. This is
  what protects against a person walking up, or clutter that was never
  labeled by SAM3. This should never be turned off.
- **Explicit `CollisionObject`s** exist to give something an identity, which
  the octomap cannot do. Only a named object can later be attached to the
  gripper, or have collision checking against it turned off for the last few
  centimeters of approach. Register one only for the object actually being
  grasped, not the whole scene.
- MoveIt reconciles the two on its own: the octomap updater excludes points
  that fall inside a known `CollisionObject`'s current geometry, so a
  registered object does not create a second, conflicting blocky region.

One concrete number worth remembering: `sensors_xtion.yaml` sets
`padding_offset: 0.1`, so every occupied octomap voxel is inflated by 10cm
before it counts as an obstacle. That is bigger than a normal pregrasp
standoff, which is exactly why planning straight against the raw octomap
does not work for close in grasping, and why the object needs its own
explicit, precise collision shape for the final approach.

### Two approaches already tried on this exact file, worth knowing before redoing this

Both are logged in `bugs/`, and are worth reading in full before starting:

- `bugs/moveit-grasp-collision-standoff.md`: the standoff ladder, currently
  the actual code in `move_to_grasp.cpp`. Works, but tries up to 7 full
  plan and execute cycles per grasp candidate, in sequence, which is slow
  and looks like the robot is stuck thinking. See "fix the candidate search"
  below for the better version of this.
- `bugs/moveit-acm-diff-replaces-not-merges.md`: describes a different,
  better approach that was in progress, allow the gripper and forearm links
  to touch the perceived object directly (via an Allowed Collision Matrix
  diff) and plan straight to the real GraspGenX contact pose, no backing off
  and no separate approach segment needed. **This code is not in the repo.**
  Checked git log, stash, and branches, it only exists as the description in
  that bug doc, not as a commit. If this approach is picked back up, it has
  to be rebuilt from that doc's description, it cannot be recovered from git.
  Two real bugs are already documented there and worth not repeating:
  1. `applyPlanningScene()` with `is_diff = true` replaces the whole allowed
     collision matrix, it does not merge into it. Fetch the current matrix
     from `/get_planning_scene` first, add to it, then push the whole thing
     back.
  2. Allowing only the `gripper` group links to touch the object is not
     enough, reaching in close also brings the wrist and forearm near the
     object. The link list needs to include `wrist_roll_link`,
     `wrist_flex_link`, `arm_roll_link`, and `arm_flex_link` too, not just
     the hand.

### Fix the candidate search: stop looping standoffs, filter candidates instead

The standoff ladder is solving the wrong axis. If a grasp pose fails at one
standoff distance along its approach axis, it will almost always fail at
other distances along that same axis too, the arm configuration needed does
not change much. What actually varies between GraspGenX's ranked candidates
is position and orientation, not how far back the arm stands.

Better version: pick one fixed, sensible standoff (something safely past the
10cm octomap padding, so 0.12 to 0.15m). For each candidate grasp, best score
first, run a cheap IK only feasibility check at that standoff, not a full
plan, not an execution. `hsrb_analytic_ik` is fast enough that checking all
of GraspGenX's candidates costs almost nothing. Take the first one that
passes, and only then run one real plan and execute. If that single
execution fails for a real world reason, fall back to the next IK feasible
candidate, still without a blind distance sweep.

### Gripper actuation

`hsrb_gripper_controller` provides real actions under the `gripper_controller`
node:

- `grasp` (`tmc_control_msgs/GripperApplyEffort`): closes until the fingers
  stall on contact. This is the one to use for actually picking something
  up, since `stalled = true` in the result is the closest thing to grasp
  confirmation available.
- `apply_force`: closes or opens until a target force is reached.
- `follow_joint_trajectory`: direct joint trajectory control.

After the gripper closes, attach the grasped object to `hand_palm_link` as an
attached collision object (with the gripper's own links as touch links) so
it moves with the arm and does not trigger false self collision during the
retreat.

## Things that exist in this workspace but are not being used yet, worth knowing about

- `tmc_robot_rrt_planner_node`, launched by `hsrb_planner.launch.py`, is a
  full alternative whole body planner (CBiRRT), served as ROS services
  (`PlanWithHandGoals`, `PlanWithHandLine`, `PlanWithJointGoals`,
  `PlanWithTsrConstraints`). `PlanWithHandLine` in particular plans a
  straight line motion to a hand goal natively, collision checked, which is
  close to what the grasp approach needs. Every request also takes a
  `base_movement_type` field (`NONE`, `PLANAR`, and others), so a single
  request can lock the base for a fine approach or let it move for a coarse
  reach. Not needed right now since MoveIt's `arm` group plus Nav2 covers
  the same ground, but worth remembering if the `arm` group approach ever
  turns out to be too limited.
- `tmc_safe_pose_changer`: a simple, MoveIt independent way to move to a
  known joint pose safely (self collision checked), good for stow or return
  to neutral poses without invoking a full planner.
- `hsrb_timeopt_ros`: time optimal trajectory smoothing, available as a
  filter node if planned trajectories end up jerky.
- `tmc_manipulation_msgs`'s `DetectGraspPose`, `ValidateGrasp`, and
  `GetGraspState` are defined message types with no server implementing them
  anywhere in this workspace. They look like leftover hooks for a proprietary
  grasp stack that was never ported here. SAM3 plus GraspGenX already covers
  what `DetectGraspPose` would have done, no need to chase these.

## Open items for when this gets implemented

- Build the inverse reachability map offline (one time cost, not per run).
- Write the search loop (head sweep plus SAM3, with a bounded number of
  fallback viewpoints).
- Write the candidate ranking and `SolveIkWithCollision` validation step
  that turns "found the object" into "here is a validated base pose."
- Decide between rebuilding the ACM approach from `bugs/moveit-acm-diff-replaces-not-merges.md`,
  or the simpler remove and reattach approach (drop the target `CollisionObject`
  from the world right before the final approach, then re add it as an
  attached object after the gripper closes). Both solve the same problem,
  the ACM one plans straight to the real contact pose in one shot, the
  remove and reattach one is simpler and uses a short Cartesian path for
  the final few centimeters.
- Replace the standoff ladder in `move_to_grasp.cpp` with the IK prefilter
  over GraspGenX's ranked candidates.
- Make sure nothing in `grasp_execution` ever targets a `whole_body*` group.
