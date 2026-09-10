# OVMM demo readiness review

The current Nav2 configuration is a reasonable simulation baseline, but the full
supervisor demo is incomplete. This review covers the current checkout; no live
Gazebo, Nav2, GPU inference, or arm-execution rehearsal was performed.

## Supervisor requirements

| Requirement | Current implementation | Remaining evidence or work |
| --- | --- | --- |
| Gazebo ground-truth graph | `core/build_scene_graph.py` reads the initial world file with explicit world-to-map registration | Validate geometry against the occupancy map. It is not a live snapshot of moved/spawned models. |
| LLM room clustering | `--rooms` assigns furniture rooms through DeepSeek | Rehearse API credentials and inspect spatially plausible assignments. |
| A: unknown object, top-k furniture | Lazy LLM fallback in `reasoner/query.py`; IDs validated against furniture | Prepare a graph without the target object while retaining furniture; keep the object in simulation. |
| B: remembered object | Remembered instances tried before LLM fallback | Demonstrate both successful memory lookup and failed remembered location followed by fallback. |
| Navigate and observe; retry misses | Bounded views, head aiming, SAM3 and map-frame RGB-D | Test unavailable services separately from genuine misses and unreachable locations. |
| Generate grasps and move arm | GraspGenX client and separate MoveIt executable exist | Search stops before this handoff; a single-command mission is missing. |
| Head control | `Navigator.look_at()` uses the head trajectory action | Demonstrate pan/tilt and target in camera view; defer advanced active perception. |
| Real scans and Mask3D graph | OpenYOLO3D wrapper uses Mask3D proposals plus YOLO-World labels | GPU inference is unverified; run the real scan and inspect masks and registration. |

## Nav2 assessment

Keep Omni AMCL and Omni MPPI as the starting point for the holonomic HSR base.
FollowPath now loads MPPI directly: the rotation shim was removed so lateral
approaches do not require an initial turn toward the path. PathAngleCritic still
encourages heading alignment during transit. Earlier YAML comments reported
faster long transit with the shim; that result has not been reproduced here,
so compare long transit and lateral final approaches in simulation.
The 0.3 m/s forward limit and 2.8 s prediction horizon are plausible starting
values, not validated tuning. A 4 m local map comfortably exceeds the nominal
0.84 m forward prediction distance. Measure controller timing with perception
servers running: 2,000 trajectories at 20 Hz must still meet the control budget.

The 0.30 m circular footprint must enclose the actual robot in its navigation
posture. A 0.35 m inflation radius leaves only 0.05 m beyond that nominal radius;
check clearance at furniture and doorways before changing weights. A planar
laser cannot represent table tops and all arm-height obstacles. Furniture-box
filters currently reject endpoints, not collisions along the entire path.

The 0.25 m / 0.25 rad navigation goal tolerance is for observation transit. Final
placement tightens it to 0.08 m / 0.15 rad. Neither a small residual nor a nearby
IK probe proves an actual grasp: recheck actual grasp poses at the measured
parked state with the environment populated. The IK scoring neighborhood is
not a mathematical certificate of reachability at arbitrary intervening poses.

`ViewpointPath` remains an inactive parameter template. The current mission uses
`FollowPath` for both transit and final placement. MPPI PreferForwardCritic
penalises negative longitudinal velocity, not lateral motion; with `vx_min=0`,
it does not prevent diagonal movement. Do not tune its weight to fix strafing.

The documented launch now selects `/home/ws/config/nav2_params.yaml`. Humble
uses `behavior_plugins`; the old `recovery_plugins` key was corrected. An explicit
NavigateToPose tree now uses bounded costmap-clearing/wait recovery without
BackUp. Both NavigateToPose and NavigateThroughPoses have explicit no-reverse
trees: Humble loads both during activation, so leaving the second at its default
would require the disabled backup action and prevent startup. Their absolute paths assume the
repository's documented `/home/ws` container mount.

## Priority before demo day

1. Integrate one mission command: search, reacquire synchronized RGB-D after
   parking, save object cloud and frame/timestamp metadata, generate actual
   grasps, transform them consistently, collision-check and execute an arm goal.
   Return success only on confirmed execution; report stage-specific failures.
2. Populate the IK/MoveIt environment with furniture and relevant obstacles.
   Check selected grasps at the actual base/joint configuration before motion.
3. Save independent scenario fixtures. Never rebuild the unknown-object fixture
   from complete ground truth immediately before scenario A: that restores the
   answer to memory. Use copies so successful detections do not change later runs.
4. Rehearse A, B, stale B fallback, no detection, unreachable furniture, service
   timeout, bad depth, failed final alignment, and failed grasp execution. Record
   selected locations, poses, observations, stage timings, and final status.
5. Run the real-scan GPU pipeline and save a visually inspected graph artifact.
   Keep active perception at head control and bounded viewpoints for this demo.

## Checks completed in this review

Two offline regressions cover a distant same-label detection preserving the old
instance and a small XY error with excessive yaw never reporting successful
placement. YAML/XML parsing and Python syntax checks are separate from runtime
validation. Existing unrelated edits and transport merge conflicts were left intact.

References: [Humble behavior configuration migration](https://github.com/ros-navigation/docs.nav2.org/blob/master/migration/Galactic.rst),
[Humble MPPI forward critic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/src/critics/prefer_forward_critic.cpp),
[Humble navigation tree](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml).
