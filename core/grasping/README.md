# Grasping on the real HSR: what broke and what fixed it

Session of 2026-09-17 / 18, first end-to-end grasping on the real robot. The
target throughout was a Pringles can on a round table.

**Result:** one verified grasp on hardware —

```
[GRASP] reached candidate 0, contact width 68.1 mm
[CONTACT] position hold, springs 0.113, 0.164
[GRASPED] bilateral contact; holding without lift
```

The hardware baseline is that state plus one retry fix (§7). Later attempts
were reverted; §9 says what they were and why. The offline geometry follow-up
below is newer and has not yet been validated on hardware.

Line numbers are as of this revert. Related: [grasping_numbers.md](grasping_numbers.md)
for the hardware constants, [../../docs/realrobot_grasp_test.md](../../docs/realrobot_grasp_test.md)
for problems 1–10 from the previous session.

---

## Summary

| # | Problem | Where the fix lives | The number |
| --- | --- | --- | --- |
| 1 | Arm trajectory goals silently lost | `.devcontainer/cyclonedds_profile.xml` | fragments 8000 B → **1344 B** |
| 2 | "Head scan ... foreground-occluded" on an empty table | `pick.py:943` `foreground_occlusion` | occlusion **0.201 → 0.000** |
| 3 | Every joint read a fresh 5 s gamble | `pick.py:1107-1133` `joint_positions` | 20 reads in **1.57 s**, median 10 ms |
| 4 | 5 s lost per gripper step | `pick.py:1226-1234` `action_client` | **9 resends → 1** per run |
| 5 | Arm jerked when a cut-off run was repeated | `pick.py:814` `compact` | joint state stripped from scene diffs |
| 6 | (guard) plan starting away from the robot | `pick.py:1507-1532` `check_start` | refuses above **0.05 rad** |
| 7 | Recovery failed: "controller unavailable" | `pick.py:1236` `run_action` | 1 × 10 s wait → **3 × 10 s** |

---

## 1. The DDS fragment size — the big one

**Symptom.** `executing reach failed: MoveIt error -4`. move_group logged
`no goal response in 5 s ... resending` three times, then
`the controller never acknowledged the goal`.

**Cause.** This PC is `131.220.7.198`, the robot `10.7.3.185` — a *routed* path
whose MTU is 1500, even though `enp5s0` itself offers 9000. The profile set
`FragmentSize 8000B`, inherited from the simulation profile where everything is
loopback at MTU 65536. Every RTPS fragment became six IP fragments, and one lost
fragment discards the whole sample.

That is why the failures tracked **message size**:

| Message | Size | Behaviour |
| --- | --- | --- |
| gripper goal (1 joint, 1 point) | tiny | always worked |
| head goal (2 joints, 1 point) | small | lost on first send, fine on resend |
| **arm trajectory (5 joints, many points)** | **several kB** | **lost 3/3** |
| point cloud | ~600 kB | never arrived |

**Fix.** In `.devcontainer/cyclonedds_profile.xml` (real robot only; the
simulation profile keeps its jumbo values, and images travel over Zenoh here so
nothing needs them):

| Setting | Was | Now |
| --- | --- | --- |
| `FragmentSize` | 8000B | **1344B** |
| `MaxMessageSize` | 8500B | 14720B |
| `MaxAutoParticipantIndex` | 20 | 100 |

**Impact.** Arm trajectories began executing on the robot. This retroactively
explains problems 7 and 9 in the older write-up.

**What it does not fix.** The link is still mildly lossy — `/joint_states` at
100 Hz shows **2.0%** loss (1945 messages in 20 s, ~40 gaps). A *fresh* client's
first request is lost roughly half the time; a warm one answered **46 of 48**
round trips. Inbound fragmentation is set by the robot's own DDS config, which
we do not control. `realrobot/live/dds_check.py` tests this in a few seconds.

---

## 2. False "foreground occlusion" — `pick.py:943`

**Symptom.** `pick failed: Head scan remains clipped or contains
foreground-occluded geometry`, on a table holding nothing but the can.

**Cause.** `foreground_occlusion` compared each pixel around the silhouette
against *its nearest interior pixel*. The head looks down, so an upright can's
top face is genuinely nearer than the body just inside the mask — the can's own
lid scored as something in front of it. Measured **0.201** against a 0.08
threshold; all of the "blockers" were one band at the top rim.

**Fix.** Measure against the target's own near surface instead:

```python
near = float(np.percentile(inside, 2))
return float(np.mean(depth[valid] < near - .03))
```

**Impact.** 0.201 → **0.000** on the same scene. A real blocker still trips it:
anything nearer than 0.459 m when the can's face is at 0.489 m.

---

## 3. Joint reads — `pick.py:1107-1133`

**Symptom.** Runs dying with `no /whole_body_moveit/joint_states` for no reason.

**Cause.** `wait_for_message` **creates a new subscriber, waits, throws it
away** — and on this link a new subscriber's first message is often lost. Every
call was a 5 s gamble, and `close()` makes up to 100 of them.

**Fix.** One subscription per node, kept for the run, read for its latest
message. It still waits for a sample newer than the call so contact readings
stay current.

**Impact.** 20 consecutive reads in **1.57 s total, median 10 ms**. The single
1.15 s outlier is the first read building the subscription.

---

## 4. Action client reuse — `pick.py:1226-1234`

**Cause.** `run_action` built a new `ActionClient` per call and destroyed it in
`finally`. It is a *client's first goal* that the link loses, so every closure
step paid a 5 s resend.

**Fix.** `action_client()` caches per node and action name; the client is no
longer destroyed.

**Impact.** A closure run went from **nine** `no goal response in 5 s` lines
(~45 s of dead time) to **one**.

---

## 5. The jerk on a repeated run — `pick.py:814`

**Symptom.** After a run was cut off part way, the next run jerked the arm.

**Cause.** In `compact()`. The function already handled `not scene.is_diff`,
which was the clue: MTC solutions **do** contain full scene replacements, and a
full scene carries the robot state captured at *planning* time. Applying it
mid-execution pins the scene to where the robot was meant to be rather than
where it is, so the next motion starts from a pose the arm never reached.

**Fix.** Every sub-trajectory keeps its attached objects but loses its joint
state:

```python
scene.robot_state.is_diff = True
scene.robot_state.joint_state = JointState()
scene.robot_state.multi_dof_joint_state = MultiDOFJointState()
```

---

## 6. `check_start` — `pick.py:1507-1532`

A guard, not a fix: `execute()` compares the plan's first point against live
joint positions and refuses above **0.05 rad**.

```
the plan starts 0.412 rad away from the robot at arm_roll_joint (limit 0.050);
stow the arm with realrobot/live/recover_home.py and run again
```

§5 addresses the cause as understood; this stops the arm being driven to a
mismatched start whatever the cause turns out to be.

---

## 7. `wait_for_server` retry — `pick.py:1236`

**Symptom.** `[RECOVER] failed: /gripper_controller/follow_joint_trajectory
unavailable` — while the robot was up, reachable, and answering.

**Cause.** A single 10 s wait. Discovery over this link arrives late and in
part: a fresh client took **6.8 s** to see a server another client saw
instantly.

**Fix.** Three attempts of 10 s, each announced. Affects only how long we wait
for a server to appear; it cannot change any motion.

---

## 8. New tools

| File | What it does |
| --- | --- |
| `realrobot/live/dds_check.py` | Read-only round trip to all four controllers via `list_parameters`. Catches what `grasp_preflight.py` cannot: `wait_for_server` is graph-level and says nothing about whether a request would be answered. |
| `realrobot/live/recover_home.py` | Stows the arm after a failed pick. `pick()` has **no stow on failure** — it prints and returns. Opens the hand, drops the stale target box and octomap, retreats 10 cm along the palm axis, folds to the observation pose. `--direct` bypasses planning (no collision checking). |
| `config/realrobot/rviz/grasp_real.rviz` | MoveIt displays plus RX's RGB and depth. Both image displays request **best-effort** QoS — RViz defaults to Reliable, which never matches RX and shows an empty panel. Selected by `rviz_config:=`, added to `move_group.launch.py` and `grasp_real.launch.py`. |

---

## 9. Tried and reverted

All of these were removed to get back to the version that grasped. They are
recorded because the measurements behind them are real and the mistakes are
worth not repeating.

| Change | Why it was tried | Why it went |
| --- | --- | --- |
| Relative cylinder-fit gate (`residual ≤ .10 × radius`) | The can measured **2.70 mm** residual against a 1.5 mm limit calibrated in simulation, so it always fell to the front-section fallback, which centres the pads on the *visible* surface — **17.3 mm** in front of a 33.3 mm-radius axis, i.e. `r/2`. That is the "grasping at the edge, can slips away" symptom. Verified fix: offset **17.3 → 0.00 mm**, width exactly 66.6 mm. | Reverted with the rest. Note a box seen from two faces fits a circle to .118–.126 of its radius against a real can's .081, so any such gate also needs the box branch to win when both fit. |
| Wiring in `close_hand()` (the −0.3 Nm effort grasp) | `close()` only rests the pads against the object and holds position; `close_hand()` was **defined and never called**. A position-held grip on a smooth cylinder slipped on the first real lift. Needs `GRIP_CEILING` ≈ .60 as well, or `wait_for_hold` aborts a real grip as "excessive contact". | Reverted. |
| `BASE_SLACK` 0.1 → 0.02 m | The whole-body IK spends whatever window it is given — **93–98 mm** every time — and `omni_base_controller` aborted with `PATH_TOLERANCE_VIOLATED`. | **Made things worse.** At 20 mm *no* candidate solved, every grasp fell back to a sampled pose goal, and the reach stopped planning at all. Trading an execution failure for a planning failure is a regression: before, the arm at least reached pregrasp. **Lesson: run the change once before recommending it.** |
| `drop_small_base_motion` | Strip odom joints from a plan that barely moves the base, so no goal reaches `omni_base_controller`. | Reverted with the rest. |
| `[IK]` rejection logging | `seeded_ik` returns `None` through three gates and logged nothing, so an empty log looked identical whether the arm could not reach, the base window was exceeded, or the solution collided. | Reverted — but it is what produced §10's numbers, and worth re-adding when diagnosing reach failures. |
| `realrobot/live/park_for_grasp.py` | Asked Toyota's collision-aware IK solver where to park. Deleted during the revert; not in the tree. | — |

---

## 10. Still open

**The base cannot execute what it is given.** `omni_base_controller` aborts with
`PATH_TOLERANCE_VIOLATED` at 98 mm and at 27 mm alike, so the size was never the
issue. Robot-side; it will affect navigation too.

**The robot is parked too far from the can.** With logging in place, of 26
grasps only **1** was reachable inside the 100 mm window; 19 needed **131–276 mm**
of base travel, median **233 mm**. The arm cannot reach these grasps without the
base, so the reach becomes a drive-then-reach and the base is the part that
fails. Positioning belongs to [../navigation/](../navigation/) — "park where the
arm can reach it" — before the pick, not to MoveIt during it.

**The head is not re-aimed before the post-lift check.** `pick()` verifies the
lift with `observe()` from wherever the head is pointing. The head has twice
been found reset to `pan 0.0, tilt 0.0` — level, looking across the room — after
robot-side intervention, giving `SAM3 found no instance` on an object plainly
present. Do not read that message as a detection failure without checking the
head first.

**`pick()` does not stow on failure.** It prints and returns, leaving the arm
wherever it stopped. Use `recover_home.py`.

**Finger asymmetry.** The right spring leads the left by ~0.05 rad throughout a
closure. Harmless at the thresholds used (0.10 for `grasp`, 0.14 for `pickup`),
but if it grows, `max > .20` trips while `min` is still below the contact
threshold — which is the shape of the `Excessive or asymmetric finger contact`
failure seen once, from a run that started with the hand already closed on the
can (springs 0.757/0.811 rather than a relaxed ~0).

**`hand_motor_joint` sits out of bounds.** Measured **−0.865** against a URDF
limit of `[−0.798, 1.240]` — 0.067 rad over, inside the 0.1 adapter tolerance so
it passes today. Same trap as the finger springs in problem 6 of the older
write-up.


## 11. Geometry follow-up (offline validation)

`cylinder()` now accepts RMS residual up to `max(1.5 mm, 0.10 × radius)`.
This retains the original small-object tolerance while allowing the measured
2.70 mm noise on a can of roughly 33 mm radius. A resolved rectangular top
now takes priority over a cylinder fit in `contact_candidates()`, since two
box faces can also fit a circle. Cylinder contact remains centred on the
fitted axis, with aperture set to the fitted diameter.

Six offline regressions in `tests/test_grasp_geometry.py` cover clean/noisy
cylinders, poor fits, insufficient visible arc, calibrated pad placement, and
box precedence when both fits succeed. Run them with:

```bash
python3 -m pytest -q -p no:anyio tests/test_grasp_geometry.py
```

The local `/tmp/graspdiag/planned_grasps.npz` cloud, rejected by the original
fit threshold, is now accepted with fitted radius 37.74 mm. Its true radius
and hidden axis are not independently known; this demonstrates acceptance,
not real-world positioning accuracy. Synthetic 33.3 mm-radius half-cylinder
clouds with 2.7 mm radial noise recover the axis and radius within 2 mm.

This change has **not** been tested on hardware. Closure still uses bounded
position steps; effort holding and lift-slip prevention remain unresolved.
Because `auto` requests a lift for fitted cylinders, objects newly classified
as cylinders can now enter the lift path. Use `--mode grasp` for the first
hardware contact-placement check, then validate retention separately before
claiming a reliable pickup. The local `tests/` directory is git-ignored by
this repository.

## 12. Fragmented depth during the head scan

A run can stop at home with `target depth is fragmented; acquire another
view`, before grasp fitting or arm planning. Previously, that exception from
an extra head-scan view aborted the entire scan. Supplemental views with
fragmented or missing depth are now logged and skipped, like a view where
SAM3 finds no target. The head still returns to its original tilt, and the
scan still requires two valid overlapping views. Invalid initial depth still
aborts; this change does not relax depth connectivity or collision checks.

Offline regression checks cover either supplemental view failing, both
failing, head restoration, and rejection of disconnected depth. Hardware
confirmation is pending.

A subsequent read-only check found the head at pan/tilt approximately zero
and the can mask touching row 479 of a 480-row image. Scanning around that
level pose lost the can upward and did not yield two overlapping views.
`observe_geometry()` now re-aims at the initial detected cloud centre when
that view is clipped, acquires a fresh observation, and reads the new head
angles before scanning. Non-overlapping views now also log their rejection.
An offline regression verifies this ordering and the scan angles; the new
head motion still needs hardware validation.

The first hardware run through that path (02:01) failed before the head
moved: `pick failed: No fresh odom-to-head_pan_link transform`. The re-aim
built a fresh `RobotTransforms` on the spot, whose empty TF buffer had 3 s to
discover the transform publishers. That is the §3/§4 endpoint problem again:
on this link a new endpoint can take ~7 s to see what an older one already
sees. `observe_geometry()` and `perceive()` now take the run's long-lived
`poses` listener from `pick()`, the same object that already served
`reframe_target()` on the occlusion-recovery path and the post-reach pose
read. The regression asserts the re-aim reuses it and constructs no new
listener. The head motion itself is still unvalidated on hardware.

The next run (02:04) got through the re-aim, so that fix held, and then lost
both supplemental views:

```
[GEOMETRY] initial target clipped; re-aiming head before scan
[GEOMETRY] skipping head tilt -0.84: no valid depth inside the mask
[GEOMETRY] skipping head tilt -0.44: Head views do not overlap on the same static target
pick failed: Could not obtain complementary target depth views
```

The head was left at pan -0.27, tilt -0.64, so the scan tilts were ±0.20
around the re-aimed view. Read-only checks afterwards: `odom` to
`base_footprint` moved 4 mm in 25 s (no base drift), `/joint_states` has a
single publisher with steady values, and vision transport carries the
camera's original stamps (`image_codec.cpp`), so a delayed frame still gets
the right head pose. The depth stream is lossless PNG per frame. None of
that explains a SAM3 mask with no depth at one tilt and a non-overlapping
cloud at the other; the run logged nothing about either view, and neither
did the 01:49 bundle.

`observe()` now prints, for every view, the SAM3 score, mask bounds, the
fraction of the mask with valid depth, and the median range, before any
rejection. With `HSR_GRASP_DIAGNOSTICS` set it also saves each view's RGB
(`views/NN.jpg`) and depth, mask, intrinsics, camera pose and score
(`views/NN.npz`). `observe_geometry()` prints point count, z range, extent,
clipped flag and support height for the initial view and for every accepted
or rejected scan view, and the overlap rejection now states both overlap
fractions. Two offline regressions cover the saved view and the message.

Two things to check in the next bundle: whether the mask at the steeper
tilt sits on the can or on something else, and whether the median range is
near the Xtion's minimum (the can looked about 0.55 m from the camera in the
01:44 frame, and the good runs had 12-14k points against 3.3k at 01:49).

## 13. Truncated observation became a truncated collision object

The 01:49 near-collision bundle (`/tmp/grasp_incident_0149`) held a fitted
envelope of 69.86 × 69.86 × 85.59 mm spanning odom z 0.659–0.745 m: the top
86 mm of a roughly 235 mm can, with the lower ~150 mm absent. The can mask
touched row 479 of 480, so the view was clipped at the bottom of the frame
(§12). `model_target()` built the collision box straight from that observed
envelope, so the planning scene contained a stub floating above the table
and the arm could sweep through the unmodelled lower body on approach. The
same truncation also raised the cylinder contact height by ~66 mm, to 22 mm
below the rim, because contact sits at `(bottom+top)/2 + min(.03, .25*(top-bottom))`.

An object rests on its support surface, and `observe()` already locates that
surface for the occlusion test. `support_plane()` now also returns the plane
height in odom, `observe()` stores it as `support`, `merge_target_views()`
keeps the highest support any view measured (a clipped view may not see the
table at all), and `target_box()` extends the collision box from the observed
top down to that plane whenever the lowest observed point sits above it.
A gap over 10 mm is logged as `[GEOMETRY] lowest observed target point is
N mm above the support plane`. Over-covering only makes the planner more
conservative, so the change fails safe when the plane is misjudged; a missing
plane leaves the box exactly as before. `planned_grasps.npz` now also records
`support` (NaN when none was found) so a future bundle shows what the box
was built from.

Replaying the incident envelope through `target_box()` with the table
assumed 235 mm below the observed top gives a box of z 0.510–0.745 m instead
of 0.659–0.745 m. That table height is an assumption, not a measurement:
the raw cloud saved at 01:23 (`/tmp/graspdiag/planned_grasps.npz`, 9083
points, 237 mm tall) spans z 0.653–0.890 m, so if the can stood on the same
surface both times, the incident cloud was the top of the can displaced
about 145 mm downward rather than a stub sitting at the true top. The saved
per-view data described in §12 is what settles this; the support-plane box
is correct in either case because it reads the table from depth, not from
the target cloud. Four offline regressions cover the synthetic support height,
the box extension and its no-op cases, the merge rule, and the published
`CollisionObject` from a clipped stub. This change does **not** move the
grasp contact: pad height still comes from the observed cylinder extent, so
a clipped view still grasps near the rim. The §12 re-aim is what prevents
the clipped observation in the first place. Not yet validated on hardware.

## 14. The 02:29 run: perception fixed, the base still cannot follow

With the per-view logging of §12 in place the scan worked first time:

```
[GEOMETRY] initial view at pan 0.14 tilt -0.28: view 0, 9848 points, z 0.659-0.895 m, ... support 0.66
[GEOMETRY] accepted view at tilt -0.48: view 1, 11486 points, z 0.663-0.895 m
[GEOMETRY] accepted view at tilt -0.08: view 2, 9185 points, z 0.670-0.896 m, clipped True
[GEOMETRY] 3 head views, extent [ 39.2  75.  236.2] mm
pringles can: SAM3 0.75, 12915 points, cylinder, 17 aperture-compatible grasps
```

The table is at 0.66 m and the can top at 0.895 m, measured from depth. So the
01:49 stub (z 0.659–0.745) was not the top of the can: it sat at table height
and was 145 mm short at the top, which the §13 replay had assumed the other
way round. The support-plane box is unaffected; the per-view files would now
show what that view actually contained.

The run then failed exactly as 01:49 and 01:53 did: `executing reach failed:
MoveIt error -4`, arm finished, `omni_base_controller` aborted with
`PATH_TOLERANCE_VIOLATED`. Read-only checks afterwards: the palm stood at
z 0.821 m, 10 cm back along the approach axis of grasp 9, i.e. at its
pregrasp. Seeded IK
from the parked pose needed **100–322 mm** of base travel for every one of
the 17 pregrasps (median 224 mm) against the 100 mm window, so every
candidate was a drive-then-reach. The controller runs `open_loop_control:
true` with path tolerances of 0.05 m and 0.1 rad, and the whole-body
trajectory at 0.15 velocity scaling asked it for roughly 10 cm in 11 s.

`seeded_ik()` now logs the base travel and turn of every solution, and why a
candidate was rejected (no IK, outside the window, in collision), as §9
recommended. Nothing else changed in the reach. The robot was parked with
the can 0.63 m from the base centre; §10 and the earlier write-up put the
arm-only reach at about 0.45 m. Park closer before the next run, and expect
the base part of the plan to stay the failing piece until the reach stops
sending the base slow trajectories.

## 15. The base is the whole remaining problem, and the arm cannot avoid it

The 02:39 run repeated 02:29 with the robot parked slightly differently. The
scan again worked first time (3 views, 11 879 points, 242 mm extent, table at
0.66 m) and the new `[IK]` logging showed the plan was **not** starved of
candidates:

```
[IK] seeded solution moves the base 86 mm and turns +0.00 rad
[IK] seeded solution moves the base 99 mm and turns +0.00 rad
```

Two of 23 candidates solved inside the 100 mm window and were handed to MTC as
joint goals; the other 21 needed 144–312 mm. Execution then failed exactly as
before: `arm_trajectory_controller successfully finished`, then
`omni_base_controller failed with error PATH_TOLERANCE_VIOLATED`, 0.2 s later,
i.e. at the very end of the trajectory. Same shape at 02:29, where the abort
came 0.006 s after the arm finished.

So a candidate needing only 86 mm still fails. Together with §10's "98 mm and
27 mm alike", the size of the base motion is not the deciding factor.

**The base cannot be taken out of the reach.** Read-only IK against the live
move_group, on the 20 saved pregrasps of that run:

| group | solved |
| --- | --- |
| `arm` (base frozen) | 0 / 20 |
| `whole_body` | 20 / 20 |

Sweeping the same six pregrasps to simulated park distances of 0.40, 0.45,
0.50 and 0.55 m, `arm` solved **0/6 at every distance**. The HSR arm is 5-DOF
and uses the base to aim, so freezing the base, or stripping the odom joints
from the executed trajectory as §9's `drop_small_base_motion` proposed, cannot
work here: there is no arm-only solution to fall back on.

**Parking closer still helps a lot.** Whole-body base travel for those six
candidates, by simulated distance:

| park distance | base travel of the six candidates (mm) |
| --- | --- |
| 0.40 m | 57, 61, 97, 106, 129, 182 |
| 0.45 m | 28, 37, 98, 110, 117, 194 |
| 0.50 m | 22, 49, 109, 111, 122, 207 |
| 0.55 m | 33, 66, 105, 125, 136, 221 |
| 0.66 m (as run) | 86, 99, and 21 more at 144–312 |

At 0.45 m the best candidate needs 28 mm instead of 86 mm. Whether the
controller can track 28 mm is unproven: §10 reports a violation at 27 mm.

`pick()` now logs the parked base pose and the target's distance before the
reach, so this number appears in every future log rather than being
reconstructed afterwards.

**What is left.** `omni_base_controller` runs `open_loop_control: true` with
path tolerances of 0.05 m and 0.1 rad and unit p-gains. Whether it fails
because the caster must re-steer before the base translates, or because the
whole-body trajectory at 0.15 velocity scaling commands speeds it ignores, is
not established from the logs available. Settling that needs base-only motion
tests with `/omni_base_controller/state` recorded during the move, which is a
hardware experiment and separate from the pick.
