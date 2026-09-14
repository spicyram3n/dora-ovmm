# Grasping branch

This branch packages the pipeline-review grasping implementation. Run the
existing mission with `--mode grasp` to stop at bilateral finger contact,
`--mode pickup` to request a verified lift, or `--mode auto` to lift cylinders
and stop at contact for other shapes. Visual servoing is disabled by default.

All grasping implementation helpers (geometry, contact closure, pregrasp routes,
scene compaction and visual-servo utilities) are consolidated in
`core/grasping/pick.py`. Visual servoing remains disabled. The consolidation
passed the 62-test suite; no new live robot trial was run.

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

- Grasping Python suite: 62 tests pass in the sourced ROS environment with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest core/grasping/test -q`.
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

## Branch integration

The prior feat/grasp history is preserved by a merge. Older grasp entrypoints
and C++ execution sources remain available for reference, with their original
README in LEGACY_GRASP_README.md. The current supported path is
core.pipeline.mission and core.grasping.pick; the legacy overlay installer is
not invoked by the current postCreate script. Follow the vendor patch setup
above for this pipeline. The historical entrypoints have not been revalidated
against the reorganized pipeline.
