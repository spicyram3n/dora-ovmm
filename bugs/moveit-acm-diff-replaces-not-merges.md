# MoveIt: PlanningScene ACM diffs replace, they do not merge

**Where:** `ros2_ws/src/grasp_execution/src/move_to_grasp.cpp`

Replaced the standoff-ladder approach (see `moveit-grasp-collision-standoff.md`)
with the real underlying mechanism: explicitly allow specific robot links to
touch `<octomap>` (the perceived-scene collision entity), then plan straight
to the actual GraspGenX contact pose instead of backing off from it. Two bugs
found while building that.

## 1. A fresh AllowedCollisionMatrix diff wipes out the existing one

**Symptom:** after adding the "allow gripper to touch the object" logic,
every single grasp attempt started failing instantly with `MoveItErrorCodes`
-26 (`START_STATE_INVALID`), even grasp 1, even before any goal pose was
evaluated. That means the robot's own current, resting pose was being
rejected.

**Diagnosis.** `/check_state_validity` on the current state (no goal
involved at all) showed the robot in self-collision across totally unrelated
links: `arm_lift_link` vs `arm_flex_link`, `base_link` vs its own wheels,
`head_pan_link` vs `head_tilt_link`, and more. These are all adjacent links
that touch by design and are normally listed in the SRDF's
`disable_collisions` section. Queried the live scene directly:
```python
# /get_planning_scene with ALLOWED_COLLISION_MATRIX component
# -> only 2 entry_names total (should be ~245 pairs from the SRDF)
```

**Root cause:** `moveit::planning_interface::PlanningSceneInterface::applyPlanningScene()`
with `is_diff = true` does not merge the `allowed_collision_matrix` field
into the existing one. It replaces the whole matrix with whatever is in the
message. The code was building a brand new, mostly-empty
`collision_detection::AllowedCollisionMatrix`, setting a couple of entries,
and pushing that as the "diff", which then became the entire matrix,
discarding the SRDF's ~245 existing pairs.

**Old code:**
```cpp
collision_detection::AllowedCollisionMatrix acm;
acm.setEntry("<octomap>", link, true);

moveit_msgs::msg::PlanningScene diff;
diff.is_diff = true;
acm.getMessage(diff.allowed_collision_matrix);
psi.applyPlanningScene(diff);
```

**Fixed code:** fetch the full current matrix first, add to it, then push
the whole thing back:
```cpp
auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
client->wait_for_service();
auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
auto current_scene = client->async_send_request(request).get()->scene;

collision_detection::AllowedCollisionMatrix acm(current_scene.allowed_collision_matrix);
for (const auto& link : kArmLinks) {
  acm.setEntry("<octomap>", link, true);
}
```

**Extra wrinkle:** this bug lives in the *live* `move_group` process's
memory, not in a file. Fixing the code was not enough on its own. The
already-running `move_group` had to be restarted to clear the corrupted
matrix before the fix could be verified, since `move_group` is a long-lived
process that keeps its planning scene state across repeated launches of our
own node.

## 2. Only the "gripper" group links were allowed to touch the object

**Symptom:** with bug 1 fixed, `START_STATE_INVALID` was gone, but every
grasp still failed the goal check with -27 (`GOAL_STATE_INVALID`) or -2
(`INVALID_MOTION_PLAN`).

**Diagnosis.** Same `/compute_ik` + `/check_state_validity` probe as before,
run again on the actual grasp pose:
```
IK with avoid_collisions=False -> 1 (SUCCESS)
IK with avoid_collisions=True  -> -31 (NO_IK_SOLUTION)
check_state_validity contacts: <octomap> <-> arm_flex_link
                                <octomap> <-> arm_roll_link
                                <octomap> <-> wrist_roll_link
```

**Root cause:** the allowed-links list only covered the SRDF `gripper`
group (the hand and fingers). Reaching in close to a small object also
brings the forearm and wrist near perceived points, not just the fingers.
Those links were never added to the allow-list, so they were still being
blocked.

**Fix:** extended the list to include the wrist and lower arm links, not
just the hand:
```cpp
const std::vector<std::string> kArmLinks = {
    "hand_palm_link", "hand_l_distal_link", "hand_r_distal_link",
    "hand_l_spring_proximal_link", "hand_r_spring_proximal_link",
    "wrist_roll_link", "wrist_flex_link", "arm_roll_link", "arm_flex_link"};
```

## Status

Fix for both bugs is in and builds clean. End-to-end verification through
the full `move_to_grasp.launch.py` run was still in progress when this was
written: `/compute_ik` confirms the pose itself is reachable and
collision-free with the updated link list, but a live run through the actual
planner was taking longer than a 120 second background test window and got
cut off before finishing, not confirmed successful or failed yet.
