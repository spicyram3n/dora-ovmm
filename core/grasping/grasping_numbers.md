# Grasping numbers (HSR-C)

Every number the grasping stack depends on: joint limits, arm reach, gripper
opening, the constants in `pick.py`, and the commands that move the hand.

Robot: **HSR-C** (`hsrc1s`). Limits come from
`ros2_ws/src/hsrb_common/hsrc_description/` (the `*.urdf.xacro` files plus
`urdf/ros2_control.robot.xacro`). Reach and finger-gap values were computed by
forward kinematics on the flattened
`ros2_ws/src/tmc_manipulation_base/tmc_manipulation_tests/urdf/hsrc/hsrc1s.urdf`.

Units: metres, radians, seconds, and Nm unless a row says otherwise.

---

## 1. Gripper (hand)

### 1.1 Hand joints (`urdf/hand_v0/hand.urdf.xacro`)

| Joint | Type | Min | Max | Effort | Vel | Notes |
|---|---|---|---|---|---|---|
| `hand_motor_joint` | revolute | **-0.798** | **1.24** | 100 | 1.0 | The only commanded joint. Larger values open the hand. |
| `hand_{l,r}_proximal_joint` | revolute | -0.798 | 1.24 | 100 | 1.0 | Mimics `hand_motor_joint` (x1). |
| `hand_{l,r}_spring_proximal_joint` | revolute | 0.0 | 0.698 | 10 | 1.0 | Passive spring. Its deflection is the **contact sensor**. |
| `hand_{l,r}_mimic_distal_joint` | revolute | -0.698 | 0.0 | 100 | 1.0 | Mimics the spring joint (x -1). |
| `hand_{l,r}_distal_joint` | revolute | -1.24 | 0.798 | 100 | 1.0 | Mimics the motor (x -1, offset **-0.087**). |

xacro properties:

| Property | Value |
|---|---|
| `proximal_joint_angle_range_min` / `_max` | -0.798 / 1.24 |
| `spring_proximal_joint_angle_range_min` / `_max` | 0.0 / 0.698 |
| `distal_joint_angle_offset` | 0.087 |
| palm to proximal origin (x, y, z) | (-0.031, ∓0.0245, -0.0165) |
| proximal to distal (z) | 0.07 |
| distal to fingertip frame (x, y, z) | (0.0195, ±0.0226, 0.0409) |
| wrist_roll_link to hand_palm_link | z 0.155, rpy (0, 0, π) |
| hand camera origin in the palm frame | (0.026, 0, 0.0034) |

ros2_control, `hand_motor_joint` (`ros2_control.robot.xacro`):
position command min/max -0.798 / 1.24, motor_id 41, drive_mode 0,
velocity_limit 3.0, gear_ratio 0.005090909, torque_constant 0.0184,
control_type `Gripper`.

### 1.2 Opening width vs. motor angle

In the palm frame, **+z is the approach direction and the fingers close along +y**.

FK on the HSR-C URDF (fingertip-frame to fingertip-frame distance):

| `hand_motor_joint` | Fingertip gap | Fingertip z in front of palm |
|---|---|---|
| 1.24 (URDF max) | 0.129 | 0.045 |
| 1.20 (controller max) | 0.127 | 0.048 |
| **1.10 (`OPEN_HAND`)** | **0.122** | **0.054** |
| 0.60 | 0.076 | 0.080 |
| 0.10 (lowest step in `close()`) | 0.011 | 0.092 |
| 0.0 (config "close") | 0.003 | 0.092 |
| -0.5 (controller min) | 0.070 | 0.084 |
| -0.6 | 0.082 | 0.080 |
| -0.798 (URDF min) | 0.103 | 0.071 |

Below 0 the fingertips cross and move apart again. The gap alone therefore
cannot tell a closed hand from an open one: -0.60 and +0.60 give about the
same gap. The fingertips also move **forward** while closing (5.4 cm at open,
9.2 cm when shut). There is no fixed contact plane.

Measured at `OPEN_HAND = 1.1` (`bugs/grasp-pose-convention.md`):

| | z from palm | Separation |
|---|---|---|
| knuckle | 1.53 cm | 17.38 cm |
| fingertip | 5.40 cm | 12.16 cm |

The usable pad band is **1.5 to 5.4 cm** in front of the palm.

### 1.3 Calibrated closing profile

Source: `docker/graspgenx/x_grippers/hsrc_hand/closing_profile.json`. It has
21 samples, motor 0.10 to 1.10 in 0.05 steps. `gap[0]` is the pad gap, and
`center` is the pad centre in the GraspGenX gripper frame. Every sample has
`center` x = 3.85e-5 and y = 0.01276.

| motor | gap (m) | pad centre z (m) |
|---|---|---|
| 0.10 | 0.0112 | 0.0845 |
| 0.20 | 0.0250 | 0.0834 |
| 0.30 | 0.0386 | 0.0817 |
| 0.40 | 0.0517 | 0.0793 |
| 0.50 | 0.0643 | 0.0763 |
| 0.55 | 0.0704 | 0.0745 |
| 0.60 | 0.0763 | 0.0726 |
| 0.70 | 0.0874 | 0.0684 |
| 0.80 | 0.0976 | 0.0636 |
| 0.90 | 0.1069 | 0.0583 |
| 1.00 | 0.1150 | 0.0527 |
| 1.10 | 0.1220 | 0.0466 |

Constant for every sample: pad height `gap[1]` = 0.0350 and pad thickness
`gap[2]` = 0.0091.

**Accepted object width** (`pad_for_width`): from `gaps[0]` to `gaps[-1] - 0.012`,
which is **0.0112 to 0.1100 m**.

### 1.4 GraspGenX gripper config (`x_grippers/hsrc_hand/config.json`)

| Key | Value |
|---|---|
| `open.hand_motor_joint` | 1.1 |
| `close.hand_motor_joint` | 0.0 |
| `fingertip` | (3.85e-5, 0.01276, 0.04659) |
| `sweep_volume.extents` / `offset` | (0.1220, 0.0350, 0.0091) at z 0.0466 |
| `sweep_volume.extents2` / `offset2` | (0.0704, 0.0350, 0.0091) at z 0.0745 |
| `standoff` | [0.0, 0.00455] |
| `bbox` min / max | (-0.0982, -0.0435, -0.0519) / (0.0919, 0.0320, 0.0553) |
| `base_rotation` | 90° about z (palm frame to GraspGenX frame) |
| `type` / `symmetric` | `parallel_2f` / true |

`graspable.py` and `vgn_client.py` use other hand lengths:

| Constant | Value | File |
|---|---|---|
| `FINGER_LENGTH` | 0.094 | `core/navigation/graspable.py` |
| `APPROACH_DEPTH` (palm set back from grasp point) | 0.07 | `core/navigation/graspable.py` |
| `FINGER_DEPTH` (VGN fingertip reach past the origin) | 0.05 | `core/grasping/vgn_client.py` |

---

## 2. Arm, torso, head: joint limits

From `hsrc_description`. Effort and velocity are the xacro values. The
flattened test URDF lists effort 100 for the lift joints and wrist_flex
min -1.74; the xacro values below take precedence.

| Joint | Type | Min | Max | Effort | Vel | Parent → origin |
|---|---|---|---|---|---|---|
| `arm_lift_joint` | prismatic (+z) | **0.0** | **0.69** | 300 | 0.2 m/s | base_link, z 0.350 |
| `torso_lift_joint` | prismatic (+z) | 0.0 | 0.345 | 300 | 0.1 m/s | base_link, z 0.762; **mimics arm_lift x 0.5** |
| `arm_flex_joint` | revolute (-y) | **-2.62** | **0.0** | 100 | 1.2 | arm_lift_link, (0.141, 0.0785, 0) |
| `arm_roll_joint` | revolute (z) | -1.92 | 3.67 | 100 | 2.0 | arm_flex_link, (0.005, 0, 0.345) |
| `wrist_flex_joint` | revolute (-y) | -1.75 | 1.22 | 100 | 1.5 | arm_roll_link, (0, 0, 0) |
| `wrist_roll_joint` | revolute (z) | -1.92 | 3.67 | 100 | 1.5 | wrist_flex_link, (0, 0, 0) |
| `head_pan_joint` | revolute (z) | -3.84 | 1.75 | 100 | 1.0 | torso_lift_link |
| `head_tilt_joint` | revolute (-y) | -1.57 | 0.52 | 100 | 1.0 | head_pan_link, x 0.02 |

ros2_control (`ros2_control.robot.xacro`). `velocity_limit` is in driver units.

| Joint | motor_id | reduction | drive_mode | velocity_limit | gear_ratio | torque_const |
|---|---|---|---|---|---|---|
| arm_lift | 21 | 40.273862263 | 5 | 6.05 | 0.002407705 | 0.0174 |
| arm_flex | 22 | 1.0 | 5 | 1.58 | 0.00128576 | 0.0208 |
| arm_roll | 23 | 1.0 | 5 | 2.0 | 0.003333333 | 0.0184 |
| wrist_flex | 24 | 1.0 | 5 | 2.0 | 0.0035 | 0.0184 |
| wrist_roll | 25 | 1.0 | 7 | 2.0 | 0.0035 | 0.0184 |
| head_pan | 31 | 1.0 | 5 | 2.0 | 0.01 | 0.0184 |
| head_tilt | 32 | -1.0 | 5 | 2.0 | 0.0028 | 0.0184 |
| hand_motor | 41 | 1.0 | 0 | 3.0 | 0.005090909 | 0.0184 |

Base: `base_roll_joint` (vel 2.5, effort 34.46); drive wheels at
(-0.11, ±0.133, 0.04) (vel 20.8, effort 11.067). `base_footprint` →
`base_link` is identity.

---

## 3. Arm reach (how far it extends)

Link lengths along the chain:

- base_link → shoulder (`arm_flex` axis): z **0.35**, x 0.141, y 0.0785
- shoulder → wrist (`arm_flex` → `wrist_flex`): **0.345** (+0.005 x)
- wrist → palm (`hand_palm_link`): **0.155**
- **shoulder → palm: 0.50 m**; shoulder → fingertip at open hand: about 0.55 m
- Arm lift adds up to **0.69 m** of vertical travel. The torso and head rise by half of it (0.345 m).

FK results in `base_footprint` (x forward, z up), hand at 1.1:

| Pose | arm_lift | Shoulder z | Palm (x, y, z) | Fingertip (x, z) |
|---|---|---|---|---|
| Arm straight up | 0.0 | 0.35 | (0.146, 0.078, **0.850**) | (0.157, 0.904) |
| Arm straight up | 0.69 | 1.04 | (0.146, 0.078, **1.540**) | (0.157, 1.594) |
| Arm horizontal (flex -π/2) | 0.0 | 0.35 | (**0.641**, 0.078, 0.345) | (0.695, 0.333) |
| Arm horizontal (flex -π/2) | 0.69 | 1.04 | (**0.641**, 0.078, 1.035) | (0.695, 1.023) |
| Flex at lower limit -2.62 | 0.0 | 0.35 | (0.386, 0.078, -0.086) | (0.403, -0.139) |
| Home (0, 0, -1.57, -1.57, 0) | 0.0 | 0.35 | (0.146, -0.076, 0.695) | n/a |

Summary:

- **Horizontal reach from base centre:** palm about **0.64 m**, fingertips about **0.70 m**
  (about 0.50 m / 0.55 m ahead of the shoulder axis).
- **Palm height with a horizontal palm:** **0.345 to 1.035 m**.
- **Highest palm point:** **1.54 m** (fingertip about 1.59 m).
- **Lowest:** in kinematics the palm reaches below the floor (flex -2.62), so the floor and base collision limit the real minimum.
- Head RGB-D sensor height: 1.01 m at lift 0, 1.355 m at lift 0.69.
  `core/active_perception/views.py` uses `CAMERA_HEIGHT = 1.015`.
  `pick.reframe_target` uses 0.248 as the camera height above the head pan pivot.
- The base supplies the remaining degrees of freedom (`whole_body` group: `odom_x`, `odom_y`, `odom_t`).

---

## 4. Constants in `core/grasping/pick.py`

### 4.1 Hand / closure

| Name / literal | Value | Meaning |
|---|---|---|
| `OPEN_HAND` | **1.1** | Open position of `hand_motor_joint`. A wider opening breaks the distal finger limit. |
| `CLOSE_EFFORT` | **-0.3 Nm** | Effort for the grasp action. Negative closes the hand. |
| `HSR_GRASP_EFFORT_NM` env | allowed **-0.3 to -0.012** | Overrides the closing effort. |
| `EMPTY_GAP` | **0.01 m** | A fingertip gap below this means the hand closed on nothing. |
| open-hand settle check | motor within 0.05 of open, springs < 0.3, 5 s | `open_hand()` |
| open trajectory time | 1.5 s (goal timeout 15 s) | `open_hand()` |
| open-before-plan threshold | motor < `OPEN_HAND - 0.05` | `pick()` |
| MTC "open hand" trigger | abs(motor - OPEN_HAND) > 0.005 | `Pick.reach()` |
| `close()` step, before contact | **-0.02 rad** | Each step takes 0.5 s; goal timeout 10 s. |
| `close()` step, after contact | **-0.005 rad** | Used once min(spring) > 0.06. |
| `close()` minimum motor | **0.10** | Stop: "Minimum closure reached". |
| `close()` max iterations | 100 | |
| contact seen | min(spring) > **0.06** | |
| contact lost | min(spring) < **0.02** after contact | |
| contact confirm | 3 samples, 0.2 s apart, threshold 0.06 | |
| close threshold, lift requested | **0.14** | `close(threshold=...)` |
| close threshold, hold only | **0.10** | |
| `contact_state` threshold range | **0.06 to 0.18** | Values outside raise ValueError. |
| spring overload | max(spring) > **0.20** | Stops closure (excessive or asymmetric). |
| `wait_for_hold` | dwell 5 s, timeout 90 s, poll 0.2 s, threshold 0.06 | |
| grasp action goal timeout | 30 s | `close_hand()` |

### 4.2 Motion / arm

| Name / literal | Value | Meaning |
|---|---|---|
| `APPROACH` | **0.08 m** | Straight final approach along palm +z (pregrasp back-off). |
| `TEST_LIFT` | **0.03 m** | Verification lift. |
| `GRASP_LIFT_MAX` | **0.65 m** | Upper bound on `arm_lift_joint` during reach (constraint centre 0.325 ± 0.325). |
| `BASE_SLACK` | odom_x 0.1, odom_y 0.1, odom_t 0.2 | Allowed base drift around the parked pose. |
| `HSR_GRASP_INSERTION_M` env | 0 to **0.01** (default 0.01 for cylinders, 0 otherwise) | Capped at 0.2 × width. |
| Cartesian `step_size` | 0.005 | |
| velocity / accel scaling | **0.15 / 0.15** | Both OMPL and Cartesian. |
| overhead unfold pose | lift 0.35, flex -0.5, roll -1.57, wflex -1.07, wroll 0 | Used when grasp z-z < -0.9. |
| observation home pose | lift 0, flex 0, roll -1.57, wflex -1.57, wroll 0 | `clear_view()` |
| endpoint correction window | 4 to 10 mm and ≤ 0.04 rad | |
| refuse-closure tolerance | > **4 mm** or > **0.04 rad** | |
| candidate match score | distance + 0.1 × angle | |
| IK timeout | 1 s | `seeded_ik` |
| pregrasp MoveTo timeout | 3 s (15 s with an IK seed; 10 s for the side/overhead route) | |
| `side_waypoint` distance | 0.12 m (≤ half the distance to the base) | Overhead grasps (z-z ≤ -0.9). |
| MTC payload cap | 32 MiB | |
| execute timeout | 300 s | |

### 4.3 Perception / shape policy

| Name / literal | Value |
|---|---|
| `CANDIDATES` | 20 |
| `MAX_CLOUD_POINTS` | 8192 |
| GraspGenX `num_grasps` | 200 (client default) |
| `cloud()` | ≥ 100 points, extent ≤ **0.6 m** |
| `SHAPE_POLICIES` | cylinder: front; sphere/cube: top_down if top ≤ **0.95 m**; cuboid: front, top_down when wide |
| cylinder fit | radius **0.008 to 0.15**, residual ≤ max(1.5 mm, 0.10 × radius), height ≥ 0.04, middle 60 % band |
| cylinder contact | 0.03 or 0.25 × height (whichever is smaller) above centre; ≤ 0.10 m from proposal |
| top rectangle | edges 0.012 to 0.45, fill ≥ 0.9, face band ±3 mm |
| sphere fit | radius **0.015 to 0.055**, RMS ≤ 2.5 mm |
| box contact depth | min(0.025, 0.35 × width, 0.45 × depth) |
| cube vs cuboid | aspect < 1.15 |
| cuboid raised contact | +min(0.04, 0.2 × height) |
| front-section band | palm y 0.01276 ± 0.0175; depth ≥ 0.01; shift ≤ 0.06 |
| orientation filters | abs(approach z) ≤ 0.35; facing dot ≥ 0.5 (0.65 for sections) |
| head tilt scan offsets | ±0.20 rad (clipped to -1.57 to 0.52) |
| occlusion reject | > 0.08; foreground margin 0.03 m |
| head workspace | pan -3.84 to 1.75, tilt -1.57 to 0.52 |
| lift verification | expected ≥ 0.015; rise ≥ 0.6 × expected; error ≤ 0.01 to 0.012; coverage ≥ 0.7 |

### 4.4 Visual servo (currently disabled)

Max approach 0.08 m, approach speed ≤ 0.006 m/s, IBVS speed ≤ 0.012 m/s,
angular ≤ 0.08 rad/s, IBVS correction ≤ 35 mm / 0.06 rad, base drift ≤ 3 mm /
0.01 rad, convergence < 2 px × 3 frames, lateral corridor 4 mm / 0.04 rad.

---

## 5. Commands to control the gripper

Controller: `/gripper_controller`
(`hsrb_gripper_controller/HrhGripperController`, joint `hand_motor_joint`).
The same controller package runs the HSR-C hand.

| Action | Type | Use |
|---|---|---|
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | **Set a motor angle** (open, or any value). Goal tolerance 0.05 rad by default (0.2 in sim). |
| `/gripper_controller/grasp` | `tmc_control_msgs/action/GripperApplyEffort` | **Close with torque** (`effort` in Nm, negative closes). Succeeds on stall. Torque tolerance 1.0. **No position bound.** |
| `/gripper_controller/apply_force` | `tmc_control_msgs/action/GripperApplyEffort` | Close until the fingertip **force** (N) is reached. Not used by our code. |
| `/gripper_controller/set_distance` | `tmc_control_msgs/action/GripperSetDistance` | Go to a **fingertip distance** (m). Tolerance 0.005; motor clamped to -0.5 to 1.2. Not used by our code. |
| `/gripper_controller/follow_distance_trajectory` | `control_msgs/action/FollowJointTrajectory` | Distance trajectory. Tolerance 0.003; motor clamped to -0.5 to 1.2. |

Topics: `/gripper_controller/fingertip_distance` (`std_msgs/Float32`),
`/gripper_controller/controller_state` (`JointTrajectoryControllerState`),
`/joint_states`, `/whole_body_moveit/joint_states` (used by `pick.py`).

Controller defaults (from the source): set_distance P/I/D 2.0 / 0 / 2.5,
stall velocity 0.05, stall timeout 1.3 s. apply_force P/I/D 0.1 / 0.15 / 0.4,
force tolerance 0.1 N, stall 0.05 / 2.0 s, LPF 0.8, integral max 0.15.
Sim overrides (`hsrb_gazebo_bringup/config/gazebo_ros2_control.yaml`):
position_goal_tolerance 0.2, distance P/I/D 2.0 / 0 / 0, force P/I/D 0.3 / 0.05 / 0.4.

### CLI

```bash
# OPEN (what pick.py and the web UI do)
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [hand_motor_joint], points: [{positions: [1.1], time_from_start: {sec: 2}}]}}"

# SET A VALUE (any angle in -0.798..1.24; e.g. half open)
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [hand_motor_joint], points: [{positions: [0.6], time_from_start: {sec: 1}}]}}"

# CLOSE with torque (pick.py / web UI)
ros2 action send_goal /gripper_controller/grasp tmc_control_msgs/action/GripperApplyEffort "{effort: -0.3}"

# SET FINGERTIP DISTANCE (metres)
ros2 action send_goal /gripper_controller/set_distance tmc_control_msgs/action/GripperSetDistance "{distance: 0.05}"

# CLOSE to a fingertip force (N)
ros2 action send_goal /gripper_controller/apply_force tmc_control_msgs/action/GripperApplyEffort "{effort: 5.0}"

# READ
ros2 topic echo /gripper_controller/fingertip_distance
ros2 topic echo /joint_states   # hand_motor_joint, hand_{l,r}_spring_proximal_joint
```

### Python (as in `pick.py`)

```python
# set a position (open_hand / close() step)
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = ['hand_motor_joint']
goal.trajectory.points = [JointTrajectoryPoint(positions=[OPEN_HAND],
                                               time_from_start=Duration(sec=1, nanosec=500000000))]
run_action(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', goal, 15.)

# close with effort
run_action(node, GripperApplyEffort, '/gripper_controller/grasp', GripperApplyEffort.Goal(effort=-0.3), 30.)
```

MoveIt: the `gripper` group uses `setGoal({"hand_motor_joint": OPEN_HAND})`
(`Pick.reach`). In `hsrb_controllers.yaml`, `gripper_controller` is a
FollowJointTrajectory controller.

Web UI (`web/server.py`): `POST /gripper/open` sends `OPEN_HAND = 1.1` over
2 s. `POST /gripper/close` sends `CLOSE_EFFORT = -0.3` on `/grasp`.

### Caveats

- `grasp` with nothing in the hand drives the motor to the -0.798 stop and keeps
  applying the effort. Sending a position goal afterwards replaces the effort
  goal and releases the hand.
- A fingertip gap alone cannot detect an empty hand when the motor is negative
  (the fingers have crossed). Check that `hand_motor_joint` is greater than 0 as well.
- Real-robot preflight (`realrobot/live/grasp_preflight.py`) checks that `arm_lift_joint`,
  `hand_motor_joint` and `head_pan_joint` appear on `/joint_states`, that the
  arm, head, gripper and grasp actions exist, and that TF is no older than 0.5 s.

---

## 6. Related poses and constants elsewhere

| Where | Value |
|---|---|
| `core/pipeline/actions.py` `home_goal` | lift 0, flex 0, roll -1.57, wflex -1.57, wroll 0 (3 s) |
| `core/pipeline/actions.py` `MAX_SPAN` | 0.6 m (largest object the gripper can pick up) |
| `core/active_perception/explore.py` | `ARM_LIFT_MAX = 0.69`, torso = 2 × rise, HOME = [0, -1.57, -1.57, 0], 4 s |
| `core/navigation/base_placement.py` `NEUTRAL` | [0, 0, 0, -1.57, 0] |
| `core/navigation/graspable.py` | nav tolerance 0.25 / yaw 0.25; precise 0.08 / 0.15; approach timeout 180 s |
| `core/navigation/standoff.py` | working distance 0.8, robot radius 0.3, max range 1.5 |
