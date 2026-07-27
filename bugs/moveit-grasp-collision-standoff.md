# MoveIt grasp execution: collision-aware planning to a GraspGenX pose

Not one bug, a chain of them found while getting `move_to_grasp` to actually
drive the arm to a saved grasp pose. Logged together since they are all part
of the same debugging session.

## 1. GraspGenX poses are contact poses, not planning goals

**Symptom:** every candidate grasp failed with `MoveItErrorCodes` -27
(`GOAL_STATE_INVALID`). Each failure took under a second. That is too fast
for a planner that is genuinely searching and giving up.

**Diagnosis.** Used direct service calls to bypass the full pipeline and
isolate the cause:
```python
# /compute_ik with avoid_collisions=True  -> -31 (NO_IK_SOLUTION)
# /compute_ik with avoid_collisions=False -> 1  (SUCCESS)
```
IK finds a solution fine. It is only rejected once collision checking is on.
Then `/check_state_validity` on that IK solution showed the whole arm
(`arm_flex_link`, `arm_roll_link`, the hand links, `wrist_roll_link`) in
contact with `<octomap>`.

**Root cause:** GraspGenX's output pose is the final, closed-gripper contact
pose. The gripper is supposed to be touching or enclosing the object there.
No collision-aware planner will ever plan straight into that.

**Fix:** plan to a "pregrasp" pose instead. Same orientation, offset back
along the grasp's own local +Z axis (its approach axis, per GraspGenX's own
convention). Computed in `move_to_grasp.cpp` with the closed-form third
column of the rotation matrix for a quaternion. No need for tf2 or Eigen
just for this:
```cpp
axis.x = 2.0 * (q.x * q.z + q.w * q.y);
axis.y = 2.0 * (q.y * q.z - q.w * q.x);
axis.z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
```

## 2. The pregrasp offset still failed, because of sensor padding

**Symptom:** a 0.10m pregrasp offset (a reasonable-looking guess) still
failed the same way, every time, even after ruling out the other causes.

**Root cause:** `hsrb_moveit_config/config/sensors_xtion.yaml` configures
the octomap updater with `padding_offset: 0.1`. Every perceived point gets a
10cm buffer added before it counts as an obstacle. A 0.10m standoff lands
right on that padded boundary, not clear of it.

**Fix:** stop hardcoding one standoff value. `move_to_grasp.cpp` now tries a
ladder of standoffs per candidate grasp, closest to farthest, and stops at
the first one that is both reachable and collision-free:
```cpp
const std::vector<double> kStandoffs = {0.05, 0.07, 0.10, 0.12, 0.15, 0.20, 0.25};
```

**A bug in this fix, caught before it shipped.** The first version tried the
ladder farthest to closest and stopped at the first success. That guarantees
landing on the largest working value, not the smallest. That is backwards
from the goal, which is to get as close as safely possible. Confirmed by
testing: it kept landing on 0.25m even though 0.10m later turned out to work
for the same grasp once the order was reversed to closest first.

## 3. Ruled out along the way

- **Frame mismatch, base_link vs odom.** This one was real, but not the
  cause of the -27 errors. Grasps were saved in `base_link`, while MoveIt's
  `whole_body` group plans in `odom` (confirmed since `robot_description.py`
  attaches the virtual base joints to `odom`). Fixed by changing
  `camera_ros2.py`'s `BASE_FRAME` to `"odom"`. Odom and base_link were 2.7
  meters and 177 degrees apart at the time, more than enough to make every
  grasp unreachable on its own. But fixing the frame alone did not resolve
  the collision failures, so it was a separate issue.
- **IK solver timeout too short.** `kinematics.yaml` gives `whole_body` only
  5ms times 3 attempts, which looked suspicious. Overriding it to 1s times
  20 attempts changed nothing. Timing was identical before and after. Dead
  end, left alone.
- **Missing self-filter, robot seeing its own arm.** Assumed at first, since
  the octomap contacts spanned the whole arm, not just the fingertips near
  the object. That pattern looked like self-collision. Turned out
  `sensors_xtion.yaml` already configures real-time self-filtering via
  `occupancy_map_monitor/PointCloudOctomapUpdater`, confirmed loaded by
  `move_group.py:78,118`. Tucking the arm to the `"go"` pose (see
  `hsr-arm-go-stow-pose.md`) and recapturing did not fix it, confirming this
  was not the cause either.

## Result

`move_to_grasp` now tries each candidate grasp best score first, and for
each one tries the standoff ladder closest first, executing the first
combination that is reachable and collision-free. Verified working end to
end: reached a real grasp candidate in Gazebo, confirmed via `/joint_states`
and the `odom -> base_link` TF actually changing, not just the RViz preview,
at a 0.10m standoff after this fix, down from 0.25m before it.
