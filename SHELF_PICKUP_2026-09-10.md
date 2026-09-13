# Live shelf-can generalization — 10 September 2026

The other Pringles can on the apartment shelf was physically lifted 2 cm and
withdrawn 20 cm. Fresh RGB-D verified the height change and stable hold.
This used fresh SAM3 and GraspGenX inference, not the saved table grasp.
The final clean single-command validation also succeeded, with no manual
intervention between detection and verified hold. The simulator is left holding
the shelf can after withdrawal.

## Final single-command result

`config/diagnostics/shelf_2026-09-10/final_flow/pipeline.log` ends with
`RGBD_PICKUP_VERIFIED` and `run completed in 119.39s; success=True` (exit 0).
The run started in a fresh scene with the stowed robot and empty gripper,
after positioning the head to view the can. It used the first fresh batch:
SAM3 0.94 s, GraspGenX 1.09 s, 100 saved candidates, 55 eligible aligned
candidates, selected candidate 12 with score approximately 0.92.

Visual refinement converged to 0.22 mm residual. Feedback closure stopped at
motor 0.360140 rad with spring deflections 0.098856 / 0.114374 rad. The lift
checked 101 states (15 shallow initial support-contact samples); withdrawal
checked 201 strictly collision-free states. The initial observed can top was
0.885499830 m; three held tops were 0.905503069, 0.905501275, and 0.905501358 m.
The observed rise was approximately **20.002 mm**, with less than 0.002 mm
variation across the three samples.

Verification data: `final_flow/targets/pringles_can/pickup/result.json` and
the adjacent `before.npz`, `hold_0.npz`, `hold_1.npz`, and `hold_2.npz`.
The camera comparison is `final_flow/verified_pickup.png`.

Validation: 20 regression tests passed (5 geometry, 3 perception, 12 execution).
Execution results are saved in `execution_tests.log`. The C++ executor rebuilt
successfully; Python compilation and `git diff --check` passed. One final clean
end-to-end success is not a measured reliability rate across many trials.

## Evidence already obtained

Artifacts are under `config/diagnostics/shelf_2026-09-10/`.

- First live detection: SAM3 score 0.98 in 0.80 s; GraspGenX produced 100
  candidates in 1.11 s. See `perception.log`.
- First grasp reached contact with a 5.7 mm pad-centering residual. A fresh
  detection and bounded checked correction reduced it to 0.34 mm. The first
  asymmetric closure stopped without lifting; it was reopened before correction.
- `extracted_pickup/result.json`: first verified extracted pickup. Three
  held-top observations were approximately 0.905474 m. A separate Gazebo
  read-only check confirmed physical can Z=0.791494 m after the 2 cm lift.
- A new proposal batch selected raw score rank **65**, score 0.7074; this
  candidate would have been discarded by the previous top-50 limit.
- `full_flow_100/swept_pickup/result.json`: another verified lift and extraction
  using deterministic feedback closure. Observed top before=0.885478 m;
  three held tops=0.905493 m, a 20.015 mm rise.

These development trials used staged recovery while integrating the flow.
Do not describe them alone as unattended single-command validation.

## Integrated behavior

`core/run_pipeline.py --execute` now performs fresh synchronized RGB-D capture,
SAM3 segmentation, GraspGenX generation, complete checked approach, fresh visual
centering, feedback closure, short lift, optional withdrawal, and RGB-D hold
verification. It requires one prompt and a depth-fitted upright cylinder.

All 100 usual proposals are retained for reachability checking. When a batch
has no reachable approach and no target has been separated, the CLI can acquire
a new observation and generate another batch, up to `--attempts` (default 3).
Failed detection never substitutes an old grasp file. After target separation,
failure stops rather than blindly retrying a new approach. Attempt artifacts
are kept in separate directories.

At contact, the measured can axis is compared with the reached palm FK and
calibrated pad center. Corrections are limited to 1 cm, checked at endpoints
and midpoints, and must converge within 2 mm. Fingers remain open throughout.

Closure uses a deterministic motor trajectory, with URDF mimic joints propagated
for each full-state collision check. Motor increments in validation stay below
0.005 rad; only the motor is commanded. Physical spring feedback is sampled
after each small contact step. Both springs must exceed 0.08 rad with bounded
asymmetry; excessive deflection or a closure limit stops the trial. The motor
endpoint is not the old table experiment's fixed 0.35 rad.

The 2 cm lift checks 101 full robot/attached-target samples. Its only exception
is the previously documented shallow initial target/support overlap: <=1.5 mm
penetration, <=5 mm lift, contact near the original cylinder bottom. Withdrawal
checks 201 full states with **no** contact exception. These operations attach
only the MoveIt collision model; no Gazebo weld or artificial grasp is used.

The initial top-only depth check was occluded by the shelf lip despite a real
lift. Withdrawing the can makes its top visible again. Verification requires
three fresh observations showing the requested rise (6 mm tolerance) and less
than 5 mm variation. Raw observations and `result.json` are saved under `pickup/`.

The old detached executor spin thread was replaced with cancellation and join
on exit. A persistent OMPL gripper-goal/state-validity discrepancy was observed
during development; the motor sweep uses explicit states and full collision
checks instead of that gripper goal sampler. Its underlying discrepancy has
not been conclusively diagnosed.

## Reproduce in the simulation

Start the containers as described in `START_GRASP_SIM.md`. In the simulation
container, launch a clean scene facing the shelf:

```bash
bash /home/ws/ros2_ws/launch/run_grasp_sim_nvidia.sh \
  spawn_x:=4.337 spawn_y:=12.35 spawn_yaw:=1.570796
```

Start MoveIt in another sourced container terminal:

```bash
ros2 launch /home/ws/ros2_ws/launch/hsrc_move_group_sim.launch.py
```

Wait for the automatic arm stow. Bring the entire lower can into view:

```bash
ros2 action send_goal /head_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [head_pan_joint, head_tilt_joint], points: [{positions: [0.0, -0.4], time_from_start: {sec: 3}}]}}'
```

From `/home/ws`, with ROS and the workspace sourced:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RGB_TOPIC=/head_rgbd_sensor/rgb/image_raw
export DEPTH_TOPIC=/head_rgbd_sensor/depth_registered/image
export ZENOH_CONNECT=tcp/127.0.0.1:7447,tcp/127.0.0.1:7448
python3 -u core/run_pipeline.py "pringles can" \
  --output-dir config/diagnostics/new_shelf_trial/targets \
  --execute --lift .02 --retreat .20
```

The withdrawal is along odom -X, appropriate to this initial facing direction.
It is a bounded local extraction, not navigation. Start a new pickup with an
empty gripper and a fresh planning scene; a completed run leaves the can held.

## Scope

This generalizes the demonstrated behavior across fresh can poses and GraspGenX
samples. It is still an upright-can experiment with calibrated HSRC geometry
and contact limits. It does not establish arbitrary-object manipulation,
placing, exclusion cleanup for repeated tasks, autonomous navigation, or
continuous video-rate grasp tracking. SAM3 and GraspGenX stay loaded; perception
runs freshly per attempt and again at contact. Inference takes about two seconds
in the measured runs; robot motion and feedback are substantially slower.
