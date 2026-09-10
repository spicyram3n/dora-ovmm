#!/bin/bash
# Export a standalone URDF + meshes for the HSRC gripper (hand_v0). Run ON
# THE HOST, inside the ROS 2 devcontainer, with the workspace already built.
#
# Produces hsrc_hand/hsrc_hand.urdf and hsrc_hand/meshes/ - a self-contained
# bundle with no package:// references, ready to hand to
# GraspGenX's scripts/gripper_config_wizard.py.
set -e

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source /home/ws/ros2_ws/install/setup.bash

RAW_URDF=$(mktemp --suffix .urdf)
xacro hsrc_hand_standalone.urdf.xacro -o "$RAW_URDF"

OUT_DIR="hsrc_hand"
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"
# -L: colcon's default symlink-install means the install-space meshes are
# symlinks back into ros2_ws/src - dereference them so this bundle is a real,
# self-contained copy that still works once mounted into a container that
# doesn't have ros2_ws mounted.
cp -rL "$(ros2 pkg prefix hsrc_description)/share/hsrc_description/meshes/hand_v0" "$OUT_DIR/meshes"
python3 rewrite_mesh_paths.py "$RAW_URDF" "$OUT_DIR/hsrc_hand.urdf"
rm "$RAW_URDF"

echo "Wrote $OUT_DIR/hsrc_hand.urdf, $(ls "$OUT_DIR/meshes" | wc -l) mesh files in $OUT_DIR/meshes/"
