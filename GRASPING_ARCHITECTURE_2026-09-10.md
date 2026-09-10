# Grasping boundary and kitchen diagnosis

The navigation/search teammate owns finding the requested instance and reaching
an observation/manipulation region. The grasping component should receive the
request plus a target instance/region, acquire synchronized RGB-D and TF, generate
SAM3 masks and GraspGenX proposals, validate complete motions, execute with
feedback, and report measured success or a specific recoverable failure.
A text prompt alone is ambiguous when several matching objects are visible.

## What is fixed

The upright-cylinder fitter selected the 15th–85th point-count percentiles in Z.
From above a low table, the flat lid contributes enough pixels to enter that band,
biasing the radius and causing rejection. It now selects the middle 60% of the
physical height between robust 1st/99th percentile endpoints, with at least 100
side points. The 1.5 mm circular-fit residual limit is unchanged.

The two rejected kitchen clouds now fit with approximately 0.05 mm residual and
32.89 mm radius. Saved shelf and desk clouds remain accepted. Six geometry tests
(including a lid-heavy view) and three perception tests pass. Rejection messages
now expose the underlying fitting error.

## What is not generalized yet

SAM3 and GraspGenX run on newly observed targets, but execution currently requires
an upright cylinder, uses calibrated side-pad alignment, and verifies a cylinder
with depth top-height change. This is not an arbitrary unseen-object executor.
A general executor needs target geometry reconstructed from observation, grasp
width/contact estimates, object-aware approach/lift collision policies, and
shape-independent hold verification. Unsupported targets must remain explicit
failures rather than being silently treated as cylinders.

## MTC decision

MoveIt Task Constructor is a suitable manipulation-planning framework, not a
replacement for SAM3, GraspGenX, navigation or physical grasp feedback. Its stages
can connect external grasp proposals to IK, approach, scene transitions, lift,
and alternative complete solutions. A Humble branch is available upstream.

Recommended direction: retain perception/model clients and registration; migrate
custom planning sequencing to MTC after validating the current failure fixes.
Keep real closure feedback and visual verification as explicit execution steps.
Collision checking depends on current measured scene geometry regardless of which
sequencer is used. A planning-scene attachment is not evidence of a physical hold.

References:
- https://github.com/moveit/moveit_task_constructor
- https://moveit.ai/moveit/ros/2020/02/25/mtc.html
- https://moveit.ai/deep%20learning/grasping/moveit/3d%20perception/2020/09/28/grasp-deep-learning.html

## Realtime and recovery

Measure capture/TF age, inference, planning, execution and verification separately.
Recent inference is about 2 seconds combined; this is online operation, not a
hard realtime guarantee or a video-rate replanning loop. Motor control remains
in controllers; expensive perception/planning should be asynchronous with stale
observation rejection and cancellation.

Recovery must distinguish no detection, invalid geometry, no reachable complete
plan, controller tracking error, service timeout, and unsuccessful physical hold.
A navigation/base adjustment requires a checked path and fresh observation after
movement. Do not mask a stalled collision service by commanding movement.

Kitchen live validation after the fit fix required a checked 15 cm backward,
10 cm sideways base adjustment. A complete approach then executed, but visual
centering rejected 2.7 cm error. Measured vs commanded palm transforms showed a
small planar tracking discrepancy. A fully checked 12-second base correction
(2.8 mm translation and 0.040 rad yaw) reduced visual error to 0.09 mm.
This recovery is diagnostic, not yet integrated as autonomous generic recovery.

## Verified kitchen outcome

`config/diagnostics/kitchen_height_fit_20260910/finish_recovery.log` ends with
`RGBD_PICKUP_VERIFIED`. The robot physically closed with bilateral spring feedback,
lifted 20 mm and withdrew 200 mm. Before top Z: 0.626501534 m; held top Z:
0.646056897, 0.646056314, 0.646055068 m. Measured rise: 19.55 mm with less than
0.002 mm spread. The result is saved in
`aligned/targets/pringles_can/pickup/result.json`.

This is a successful recovered pickup using fresh SAM3/GraspGenX, not an
unattended single-command kitchen success. The live simulator is left holding
the can. Release and clear the attached model before another pickup.

## Integrated local autonomy update

The CLI now forwards the identity of the actually reached aligned grasp to the
feedback executor. On a centering error above 1 cm, the executor can correct one
small planar tracking discrepancy (<=1 cm base translation, <=0.05 rad yaw,
<=2 mm vertical / 0.002 rad nonplanar mismatch), checking all 201 states and
verifying base feedback before fresh visual refinement. The original visual
centering and target-motion limits remain unchanged.

Before target separation, a failed first batch may trigger one local base
recovery selected from collision-aware IK solutions of that batch. It requires
an empty open gripper and stowed arm, limits translation to 25 cm and yaw to
0.15 rad, checks the complete base trajectory, and then acquires new RGB-D and
new SAM3/GraspGenX proposals. It does not run after separation, attachment,
service failures, or failed execution. This is bounded local positioning, not
whole-apartment search/navigation.

If the hand occludes the fitted can, contact refinement can try three bounded
head views. Every head path is collision checked; every fit uses fresh SAM3 and
depth. No reliable view means failure, not an assumed grasp success.

The first integrated trial reached contact but exposed that view-occlusion gap.
A staged retry with the new head-view recovery is development evidence only.

## Single-command autonomous validation

The final successful run is
`config/diagnostics/kitchen_autonomous_retry_20260910.log` (exit 0).
Starting from a fresh stowed robot and upright can on kitchen_lowtable, one
`run_pipeline.py --execute --lift .02 --retreat .20` invocation performed:

1. Fresh SAM3/GraspGenX; no reachable first batch.
2. Automatic checked base displacement (-0.04778, +0.13126) m in odom, yaw
   +0.00059 rad; fresh RGB-D and a new proposal batch afterwards.
3. Complete checked approach, selected aligned candidate 17/40.
4. Visual rejection of 17.7 mm centering error, then automatic checked planar
   tracking correction (+0.00127, -0.00182) m, yaw -0.03007 rad; fresh visual
   refinement before closure.
5. Bilateral spring-feedback closure, checked 20 mm commanded lift and 200 mm
   withdrawal, then three fresh depth hold observations.

Before top: 0.626441466 m. Held tops: 0.642311629, 0.642311640,
0.642311852 m. Measured rise 15.87 mm passes the current 14 mm minimum for a
20 mm command, with <0.001 mm spread. No intervention occurred between starting
this invocation and verified hold. The successful second batch including its
execution took 119.92 seconds; that number excludes the first batch and base
recovery, so is not the total command duration.

Result: `kitchen_autonomous_retry_20260910/targets/attempt_2/pringles_can/pickup/result.json`.
The simulator is left running with the can held. A preceding fresh-scene run
was interrupted by simulator shutdown (a signal followed by a controller cleanup
segfault); it is not counted as a success.

Validation: 8 geometry, 3 perception, and 18 execution tests pass (29 total).
The base and planar-tracking recovery were exercised in this successful live
run. Head-view recovery was exercised in the earlier development recovery.
This demonstrates autonomous local upright-can pickup, not arbitrary unseen
objects, whole-apartment search/navigation, guaranteed success, or hard realtime.

## Subsequent different-placement test

A later user-selected position executed approach, visual refinement, closure,
lift and withdrawal autonomously, but failed final hold verification. The can
ended substantially tilted. This remains a grip-stability/generalization
limitation; the branch does not claim reliable success at every table position.
Diagnostic paths in this document refer to local run artifacts, intentionally
excluded from Git.
