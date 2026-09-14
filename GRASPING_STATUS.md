# Grasping branch

This branch packages the pipeline-review grasping implementation. Run the
existing mission with `--mode grasp` to stop at bilateral finger contact,
`--mode pickup` to request a verified lift, or `--mode auto` to lift cylinders
and stop at contact for other shapes. Visual servoing is disabled by default.

All grasping implementation helpers (geometry, contact closure, pregrasp routes,
scene compaction and visual-servo utilities) are consolidated in
`core/grasping/pick.py`. Visual servoing remains disabled. The consolidation
passed its original 62-test suite. The current structure removes eight tests
exclusive to the deleted legacy helpers; no new live robot trial was run.

## Required simulator and MoveIt source changes

The vendor repositories under `ros2_ws/src` are separately cloned and ignored
by the main repository. Their tested changes are included as patches in
`.devcontainer/grasp_patches`; corresponding `.base` files record each base
commit. The MoveIt patch supplies the configured depth updater referenced by
the sensor configuration. The simulator patch corrects measured hand mimic
motion, including offsets and passive spring counter-rotation.

After obtaining those vendor revisions, apply each patch from its repository
root, before building. For example, from the project root:

```bash
git -C ros2_ws/src/hsrb_moveit apply --check ../../../.devcontainer/grasp_patches/hsrb_moveit.patch
git -C ros2_ws/src/hsrb_moveit apply ../../../.devcontainer/grasp_patches/hsrb_moveit.patch
git -C ros2_ws/src/hsrb_simulator apply --check ../../../.devcontainer/grasp_patches/hsrb_simulator.patch
git -C ros2_ws/src/hsrb_simulator apply ../../../.devcontainer/grasp_patches/hsrb_simulator.patch
```

Do not reapply patches to an already patched checkout. If an existing vendor
checkout has local edits, reconcile those edits first; the recorded bases
permit a clean reconstruction. Source ROS and rebuild with colcon, then restart
simulation and MoveIt to load the compiled changes. The stock postCreate script
does not apply these patches automatically.

## Validation and limitations

- Current structure: 60 tests pass (54 grasping and 6 pipeline integration) with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest core/grasping/test core/pipeline/test -q`.
  ROS launch argument inspection and the mission-tree CLI passed without motion.
- Simulator package built; native mimic tracking regression passed. Loaded
  finger linkage measurements agreed with the URDF within 6.1e-7 rad.
- Spray bottle grasp-only trial succeeded using unchanged grasp logic:
  50.2 mm observed contact width, spring feedback 0.121/0.101 rad, followed
  by independent bilateral-contact confirmation. No lift was attempted.
- Apple pickup and carry remain unsuccessful. Static finger contact must not
  be presented as a verified pickup.
- Earlier can pickup and cube/cereal grasp-only successes preceded the latest
  simulator correction and need retesting against this complete revision.

These results do not establish universal grasping. Large diagnostic recordings,
local test scenes and experimental pressure-maintenance scripts are excluded.

## Pipeline structure

The supported entry point is `launch/search.launch.py`, matching the
`feat/pipeline` layout: `core/pipeline/mission_tree.py` calls actions in
`core/pipeline/actions.py`, which dispatches `core/grasping/pick.py`.
Use `mode:=grasp` for a contact hold, `mode:=pickup` for a verified lift, or
`mode:=auto` for geometry-dependent behavior. The tree stops after one pick
attempt and never executes an unconditional home after a hold.

The old standalone pipeline, finish/recovery scripts, grasp_execution C++
package, and overlay copies have been removed. Their history remains in Git.
The simulator and MoveIt vendor patches above remain required. The live review
checkout is separate from this published source; do not assume it was updated.

## September 14 live test and saved world

Fixed a missing readiness camera import found by the live run, with a regression
test. The kitchen-start spray-bottle trial passed readiness, search, parking,
Nav2 pause and MTC approach, but failed closure: minimum motor position was
reached without bilateral contact (left 0.047, right 0.171 rad at the last saved
sample). It exited with code 4; no verified grasp or pickup is claimed.

The packaged world removes room doors/stoppers and keeps seven kitchen-table
objects. The new default robot spawn is in the high-table room. World loading,
asset paths and spawn are checked separately; cross-room navigation and pickup
in the door-free world have not yet been validated. See worlds/README.md.
