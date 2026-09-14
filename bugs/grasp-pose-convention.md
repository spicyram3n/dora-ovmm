# Host investigation: GraspGenX pose convention vs the HSRC hand

You are on the **host**, in the repo root, where Docker and the GPU live. The dev
container cannot run Docker, so the visual checks below have never been run.

## The job

`core/grasping/pick.py` picks objects with SAM3 + GraspGenX + MoveIt Task
Constructor. In simulation the fingers close on the *front surface* of the
object, or on nothing, and it slips. Several attempts to fix this by adjusting
offsets failed because one assumption underneath them was never verified. Verify
it first, then fix the code.

## The unverified assumption

`perceive()` in `core/grasping/pick.py` contains:

```python
# GraspGenX stops with the object at the fingertips. Slide each grasp along
# its approach axis (palm z) until the object's centre is between the pads.
grasps[:, :3, 3] += (to_centre - PAD_DEPTH)[:, None] * approach
```

Nobody has confirmed that first sentence. If GraspGenX already returns poses with
the object between the pads, this slide double-corrects and pushes the hand past
the object -- which matches the observed symptom. **Settle this before changing
any numbers.**

Run these (they build the image with the repo root as context; that was broken
until recently, so they may never have run):

```bash
cd docker/graspgenx
bash run_vis.sh hsrc_hand --show-sweep-volume     # the hand opening and closing
bash run_demo.sh --vis-top-grasp-meshes --num-top-grasp-meshes 10
```

`run_demo.sh` renders the hand mesh at the raw predicted grasps on sample clouds,
with **no** slide applied. Look at where the object sits relative to the fingers:

- object at the fingertips  -> the slide is right, keep it and calibrate it
- object already between the pads -> delete the slide entirely

## Gripper geometry already measured (do not redo)

From `docker/graspgenx/x_grippers/hsrc_hand/gripper.urdf`, running the finger
chain `proximal -> spring_proximal -> mimic_distal -> distal -> finger_tip_frame`:

- `base_link` -> `hand_palm_link` is identity, and `config.json`'s `base_rotation`
  is pure rotation. So the GraspGenX grasp origin **is** `hand_palm_link`.
- Fingers close along palm **y**; palm **z** is the approach.
- `hand_motor_joint` range is **-0.798 .. 1.24**. At `OPEN_HAND = 1.1`:

  | | z from palm | separation |
  |---|---|---|
  | knuckle | 1.53 cm | 17.38 cm |
  | fingertip | 5.40 cm | 12.16 cm |

  So the usable pad is the band **1.5-5.4 cm** in front of the palm. An object
  has to be inside that band before the fingers close.
- The fingertips travel **forward** as they close: 5.4 cm open, 8.0 cm at motor
  0.6 (7.6 cm apart), 9.2 cm shut. There is no fixed contact plane.
- Below motor 0, the fingertips **cross** and separate again: at -0.60 they read
  8.2 cm apart, indistinguishable by gap alone from a held Pringles can at +0.60.

## Three bugs confirmed by measurement, currently NOT fixed in the code

1. **`solid()` assumes depth == width in camera x.** A can lying on its side
   reads 20 cm across and 6 cm tall; calling it 20 cm deep puts the centroid
   -- which every grasp is positioned from -- **6.6 cm behind** the real object.
   Using `min(x_extent, y_extent)` as the depth was exact on standing 7.6x24,
   lying 20x6 and lying 30x9 synthetic clouds.

2. **The held check is gap-only and cannot detect an empty hand.** `pick()` tests
   `gap < EMPTY_GAP`. A real failure logged `fingertips 8.2 cm apart,
   hand_motor_joint -0.80` and reported "holding the pringles can" with nothing
   in the hand. Gate it on `hand_motor_joint` staying positive; the value is
   already read and printed, just unused.

3. **`GripperApplyEffort` has no position bound.** On an empty close the motor
   runs to the -0.798 stop and holds `CLOSE_EFFORT` there indefinitely. Consider
   commanding a position afterwards to release it.

## Testing without the full mission

`pick.py` needs only move_group, SAM3 and GraspGenX -- no Nav2, no scene graph:

```bash
ros2 launch /home/ws/launch/grasp.launch.py     # sim + move_group only
python3 -m core.grasping.pick "pringles can"    # re-run freely, no relaunch
```

Model servers: `docker compose -f docker/compose.yaml up -d --wait`. SAM3 needs
`HF_TOKEN=hf_...` in `docker/sam3/.env` (gated repo).

## Ground rules

- Resolve the pose-convention question **before** touching offsets. Repeated
  blind offset edits are what produced the mess this note exists to undo.
- Verify each change against a measurement or a log line, and say which.
- `git diff` against `9ee5db0` shows what is already modified; `pick.py` is
  currently HEAD plus a stow task and a per-task OMPL planner fix.

## Findings (2026-09-14, on the host)

**Verdict: raw GraspGenX poses leave the object *past* the open fingertips.
The slide is right and stays.** It was mis-sliding because of bug 1, not
because of a double correction.

Measured with the call `app.py` makes (`GraspGenXSampler.run_inference`,
`hsrc_hand`, diffusion only), object points expressed in each of the top 10
raw grasp frames (z = approach from `hand_palm_link`):

| cloud (visible surface sent) | true centre z | nearest object surface between the fingers, z |
|---|---|---|
| standing can 7.6 x 24 cm | 10.4-11.9 cm | 6.9-8.1 cm |
| lying can 20 x 6 cm | 9.9-11.8 cm | 7.1-8.5 cm |
| lying can 30 x 9 cm | 9.1-13.1 cm | 6.0-8.4 cm |
| GraspGenX sample clouds (3) | - | mostly 6-8 cm |

The open hand's collision mesh ends at z = 5.5 cm, so nothing is in the
1.5-5.4 cm pad band. The fingertips only reach the object as they travel
forward while closing (8.0 cm at motor 0.6), which pinches the front surface:
the reported symptom.

Why: the wizard registered `hsrc_hand` with sweep volumes that are 9 mm
slabs at the fingertips (open at z 4.66 cm, half-closed at 7.45 cm;
`config.json`, also printed by `run_vis.sh`). The model conditions on those
volumes, so it puts the object's surface at the tips, not between the pads.

Slide outcome, true centre z after the slide (PAD_DEPTH 7.5 cm):

| cloud | `solid()` before | `solid()` with min(x, y) depth |
|---|---|---|
| standing 7.6 x 24 | 7.5-7.6 | 7.5-7.6 |
| lying 20 x 6 | 1.0-6.5 (palm into the can) | 7.5-7.6 |
| lying 30 x 9 | -2.1 to 16.1 | 7.3-7.7 |

PAD_DEPTH stays at 7.5 cm: the fingertips are 7.6 cm apart at z 8.0 cm, so
they close across a 6-8 cm can just behind its widest point.

Tooling notes:
- `run_vis.sh ... --show-sweep-volume` fails: `vis_gripper_desc.py` has no
  such flag. Port 8080 is held by the `ggx_dev` container; use `--port 8090`.
- `run_demo.sh` defaults to `--planner graspmoe`, which adds rule-placed OBB
  candidates the server never returns. Use `--planner diffusion`.
