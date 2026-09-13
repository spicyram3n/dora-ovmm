# Grasp runtime overlay

These files preserve the tested changes in the ignored `hsrb_moveit` checkout:
configured RGB-D octomap updater, target exclusion lifecycle, depth relay,
kinematics, planning and robot-description configuration. `manifest.json`
records the tested vendor revision. Upstream copyright headers are retained.

After cloning vendor repositories and applying `bug_fixes.sh`, install with:

```bash
python3 /home/ws/.devcontainer/apply_grasp_overlay.py /home/ws/ros2_ws
```

Then rebuild `hsrb_moveit_config` and `grasp_execution`. `postCreate.sh` performs
these steps on setup. Applying the overlay replaces the listed vendor files;
keep local vendor edits separately if you need them. This branch is based on the
tested project commit 3a5c05c, before the newer navigation work on main.

The NVIDIA simulation launcher uses workstation-specific libraries under
`.runtime/nvidia-gl`, which are not distributed here. Use the standard launcher
or configure container GPU access for your host. Model weights, recordings,
simulator captures and diagnostic logs are also excluded.
