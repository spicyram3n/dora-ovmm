# Reference: HSR's "go" arm pose (stow for driving)

**Where:** `ros2_ws/src/hsrb_moveit/hsrb_moveit_config/config/hsrb.srdf`,
`group_state name="go" group="arm"`.

Not a bug, just a useful discovery worth keeping around: HSR's SRDF already
defines a named pose for tucking the arm in close to the body, meant for
driving around without the arm sticking out.

**Joint values:**
```
arm_lift_joint:   0.0
arm_flex_joint:   0.0
arm_roll_joint:  -1.57
wrist_flex_joint: -1.57
wrist_roll_joint: 0.0
```

**How to send the arm there directly**, no MoveIt planning needed since it's
a known-safe pose (goes straight to the arm's own trajectory controller):

```bash
ros2 action send_goal /arm_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint],
    points: [{positions: [0.0, 0.0, -1.57, -1.57, 0.0], time_from_start: {sec: 3}}]
  }
}"
```

There is also a plain `"neutral"` pose for the `arm` group (all zeros, wrist
flexed to -1.57) in the same SRDF file, used elsewhere in
`hsrb_moveit_config`'s own examples (see `example/interfaces.hpp`).

Note: we originally reached for "go" thinking it would fix an octomap
collision by getting the arm out of the depth camera's view. We assumed the
self-filter was missing. That assumption was wrong. `sensors_xtion.yaml`
already configures real-time self-filtering via
`occupancy_map_monitor/PointCloudOctomapUpdater`, loaded by `move_group.py`.
Keeping this note anyway since the pose itself is still useful, just not for
that specific reason.
