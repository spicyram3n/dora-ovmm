# GraspGenX setup for the HSRC gripper

This folder runs [GraspGenX](https://github.com/NVlabs/GraspGenX), a grasp pose
generator, in its own Docker container, and registers HSR's actual gripper
with it so grasp poses come out sized and shaped for the real hand instead of
a generic parallel gripper.

## Why GraspGenX, and why a separate container

The plain [GraspGen](https://github.com/NVlabs/GraspGen) model needs a
separate trained model per gripper. Getting HSRC-specific grasps out of it
would mean running its full training pipeline (simulate a grasp dataset,
train a generator, train a discriminator), which needs real GPU time and
simulation infrastructure we do not have set up.

GraspGenX solves this differently. It is one model, trained across many
gripper shapes, that generalizes to a new gripper just from its geometry, no
retraining needed. You describe the gripper once (a short interactive step)
and the same pretrained weights work for it.

This runs in its own Docker container, separate from the ROS 2 workspace and
from the SAM3 container, for the same reason those are already separate:
each one pins its own exact set of Python and CUDA library versions, and
mixing them in one environment causes conflicts. The only thing this
container needs to share with the rest of the system is the zenoh message
bus that `app.py` listens on.

## What is in this folder

```
Dockerfile              builds the image (clones GraspGenX, installs it with uv)
app.py                  the long-running inference server (zenoh queryable "graspgenx/generate")
run_wizard.sh           one-time step: register the HSRC gripper's shape with GraspGenX
run_vis.sh              open a browser view of a registered gripper (sanity check)
run_demo.sh             run real inference on bundled sample point clouds (sanity check)
run_graspgenx.sh        start the actual inference server for real use
checkpoints/            downloaded model weights, kept here so they survive across runs
x_grippers/             registered gripper configs (currently just hsrc_hand), same reason
gripper_export/         turns HSR's URDF into a file GraspGenX's setup wizard can read
```

`checkpoints/` and `x_grippers/` live inside this project folder on purpose,
not in `~/.cache`. Cache directories are meant to hold things that are safe
to delete at any time, and multi-gigabyte model weights that take real time
to redownload do not belong there, especially if you clear your cache
regularly.

## How the HSRC gripper URDF was made (`gripper_export/`)

GraspGenX's setup wizard needs a plain URDF file for the gripper, with real
mesh files it can load, and nothing ROS-specific in it. HSR's own robot
description does not ship a file like that, so we had to build one.

**The problem**: HSR's hand is defined in
`ros2_ws/src/hsrb_common/hsrc_description/urdf/hand_v0/hand.urdf.xacro`, but
that file only defines xacro macros. It never actually places them into a
robot. The macros get instantiated by the full-robot file,
`hsrc1s.urdf.xacro`, which attaches the hand macro to the end of the arm and
wrist. There is no file anywhere in the workspace that is just "the hand by
itself".

Also note the package name: HSR ships two robot description packages,
`hsrb_description` (an older/simulation variant) and `hsrc_description` (the
real HSRC hardware). We originally exported from `hsrb_description` by
mistake. The hand geometry is different between the two (different finger
offsets, different mesh files, confirmed by diffing the two `hand.urdf.xacro`
files), so this got corrected to pull from `hsrc_description` everywhere.

**The fix, in three small steps:**

1. `hsrc_hand_standalone.urdf.xacro` is a small wrapper file. It includes
   HSR's real hand macros and calls the `hsrb_hand` macro (that is its real
   name, even inside `hsrc_description`) on a plain throwaway `base_link`,
   instead of the real wrist. Running `xacro` on this file expands it into a
   normal, flat URDF containing just the gripper.

2. That flat URDF still has mesh references like
   `package://hsrc_description/meshes/hand_v0/palm.stl`. The `package://`
   scheme only resolves inside a running ROS process. `rewrite_mesh_paths.py`
   rewrites every one of those to a plain relative path like
   `meshes/palm.stl`, so any ordinary tool can load the file.

3. `export.sh` runs both of the above, then copies the actual mesh files
   (`.dae` and `.stl`) next to the new URDF, so the whole `hsrc_hand/` folder
   it produces is self-contained. It does not depend on the ROS workspace
   being mounted anywhere.

To run it (**inside the devcontainer**, with `ros2_ws` already built, since it
uses `xacro` and `ros2 pkg prefix` -- the host has no ROS, and neither does the
GraspGenX image):

```bash
bash docker/graspgenx/gripper_export/export.sh
```

This produces `gripper_export/hsrc_hand/hsrc_hand.urdf` and
`gripper_export/hsrc_hand/meshes/`. You can check the file is a valid,
sane URDF with:

```bash
source /opt/ros/humble/setup.bash
check_urdf docker/graspgenx/gripper_export/hsrc_hand/hsrc_hand.urdf
```

## Setup, in order

Run these from your normal host terminal, from the `docker/graspgenx` folder,
unless noted otherwise -- with one exception, step 1.

### 1. Export the gripper URDF

**This one runs inside the devcontainer, not the host**, since it is the only
step that needs ROS. Already covered above. Only needs to be rerun if HSR's
hand description changes.

```bash
bash docker/graspgenx/gripper_export/export.sh
```

### 2. Register the gripper with GraspGenX (the wizard)

```bash
bash run_wizard.sh
```

This builds the image (first build takes a while, later ones are cached),
then opens a small web app at `http://localhost:8080`. Open that in a
browser and go through its steps:

- Align the gripper's frame so that +Z points in the direction the gripper
  approaches an object (out of the fingertips), and +X points in the
  direction the fingers close. For HSR's hand, this works out to roughly:
  the URDF's own local Z stays the approach axis, and the URDF's own local Y
  becomes the closing axis. This was confirmed by computing real forward
  kinematics from the URDF rather than guessing, see the render script
  further down if you want to reproduce that check.
- Confirm the open and closed joint positions look right.
- Confirm the gripper type. HSR's hand is a `parallel_2f` gripper (two
  fingers, driven by one motor through a linkage), even though the fingers
  move by rotating rather than sliding straight.

When it finishes, it writes the gripper's config into
`x_grippers/hsrc_hand/`. This only needs to be done once, unless you want
to redo the alignment.

### 3. Fix the missing collision mesh (known gap in the wizard)

The wizard only ever writes a file called `vis_mesh.obj` (a preview mesh). It
never writes `coll_mesh.obj`, which is the file GraspGenX's own
visualization tools look for when drawing the gripper at a grasp pose. Until
that file exists, they draw a tiny 1cm placeholder cube instead, which is
easy to mistake for a rendering bug.

Fix it once by reusing the mesh the wizard did generate:

```bash
cp x_grippers/hsrc_hand/vis_mesh.obj x_grippers/hsrc_hand/coll_mesh.obj
```

If this fails with a permission error, it is because the file was created
by the Docker container running as root, so your normal user cannot write
next to it. Fix the ownership once and retry:

```bash
sudo chown -R $(whoami):$(whoami) checkpoints x_grippers
```

Note: this substitute mesh may not be perfectly centered on the gripper's
real grasp reference point, since it was generated for a different purpose
than the file it is standing in for. If a gripper mesh shown in the demo
viewer looks offset from where a grasp pose says it should be, this is the
likely reason. The grasp poses themselves are computed from the wizard's
numeric config, not from this mesh, so they are not affected by it.

### 4. Sanity check: look at the registered gripper

```bash
bash run_vis.sh
```

Opens a browser view animating the gripper opening and closing, using the
config from step 2. Good for confirming the gripper geometry and joint
limits look correct, on their own, before worrying about grasp poses at all.

Extra options:

```bash
bash run_vis.sh hsrc_hand --show-sweep-volume   # also show the sweep volume box
bash run_vis.sh some_other_gripper              # look at a different registered gripper
```

### 5. Sanity check: run real inference

```bash
bash run_demo.sh
```

Loads the real model weights (downloads them automatically the first time,
about 1 GB, and keeps them in `checkpoints/` afterward) and runs actual
grasp inference on point clouds that ship with GraspGenX
(`assets/sample_data/object_pc/`), using the `hsrc_hand` gripper. Opens a
browser viewer showing the object point cloud, all the candidate grasps as
small coordinate frames, and the gripper mesh at the best one.

Extra flags pass straight through to GraspGenX's own demo script, for
example:

```bash
bash run_demo.sh --vis-top-grasp-meshes --num-top-grasp-meshes 10
```

### 6. Start the real inference server

```bash
bash run_graspgenx.sh
```

This is the one meant for actual use, not just checking things look right.
It starts `app.py`, which loads the model once and then listens forever on
the zenoh key `graspgenx/generate`, waiting for point clouds to run
inference on. This has not been exercised end to end yet, since the piece
that would actually call it (turning a SAM3 mask and a depth image into a
point cloud, then querying it) has not been built yet. Steps 1 through 5 are
enough to confirm the gripper and the model both work correctly on their
own.

## How to query the running server

Once `run_graspgenx.sh` is running, any zenoh client can ask it for grasps:

```
key:        graspgenx/generate
payload:    a point cloud as raw float32 bytes, shape (N, 3)
parameters: gripper_name (optional, defaults to hsrc_hand)
            num_grasps   (optional, defaults to 200)

reply payload:    grasps.tobytes() + scores.tobytes()
                  grasps: (M, 4, 4) float32 poses, in the same frame the input points were in
                  scores: (M,) float32 confidence, 0 to 1
reply attachment: json, e.g. {"num_grasps": 12, "gripper_name": "hsrc_hand"}
```

See `app.py` for the exact implementation, and
`docker/sam3/app.py` / `sam3_trigger.py` for what the equivalent client-side
pattern looks like for the SAM3 container.

## Problems hit along the way, and their fixes

Kept here since they are not obvious and would be easy to reintroduce by
accident.

- **Plain `pip install -e .` failed**, backtracking through dozens of old,
  broken releases of `urdfpy` for several minutes before failing outright.
  GraspGenX's own dependency list leaves that package unpinned, and pip's
  resolver handles that badly. Fixed by installing with `uv` instead
  (GraspGenX's own README already recommends this for inference installs).

- **`gcc: No such file or directory`**, then **`Python.h: No such file or
  directory`**, while `uv sync` tried to compile the `sharedarray` C
  extension. The base image had no compiler and no Python headers at all.
  Fixed by adding `build-essential` and `python3.10-dev` to the Dockerfile.

- **`ModuleNotFoundError: No module named 'numpy'`** when running the wizard.
  `uv sync` installs everything into its own project-local `.venv`, not the
  system Python. Any command that skips `uv run` and calls `python3`
  directly misses that environment entirely. Fixed by always invoking
  scripts as `uv run python3 scripts/....py`, never bare `python3`.

- **Checkpoints silently never downloaded.** GraspGenX auto-downloads its
  model weights the first time the `graspgenx` package is imported, but only
  if `GRASPGENX_CHECKPOINT_DIR` is left unset. If that variable is set to a
  path, GraspGenX assumes you already put the weights there yourself and
  never downloads anything, even if the folder is empty. We had set it,
  pointing at an empty mounted folder, so nothing ever arrived, and
  inference failed later with `Generator config not found`. Fixed by not
  setting that variable at all, and instead mounting our storage folder
  directly onto GraspGenX's own default path
  (`/opt/graspgenx/ext/graspgenx_checkpoints`), so its normal
  download-once-and-keep behavior does the right thing.

- **`vis_gripper_desc.py` could not find `hsrc_hand`.** That script only
  looks inside the separate "gripper_descriptions" dataset by default, which
  is a different location from where custom, wizard-registered grippers are
  saved. Fixed by passing `--root /opt/graspgenx/assets/x_grippers`
  explicitly.

- **Gripper mesh missing or tiny during inference demos**, see step 3 above.

## What is still missing

The client that would actually use this in a real pick: something that
takes a SAM3 mask and a depth image, turns the masked region into an
object-centered point cloud, and queries `graspgenx/generate` on the running
server from `run_graspgenx.sh`. That has not been written yet.
