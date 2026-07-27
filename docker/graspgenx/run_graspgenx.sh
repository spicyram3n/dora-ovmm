#!/bin/bash
# Build and run the GraspGenX inference server. Run ON THE HOST.
# Needs the HSRC gripper registered first - see gripper_export/export.sh and
# run_wizard.sh.
set -e

cd "$(dirname "$0")"
docker build -t hrl/graspgenx:latest .
docker run --gpus all -it --rm \
    --net=host \
    -v "$(pwd)/checkpoints:/opt/graspgenx/ext/graspgenx_checkpoints" \
    -v "$(pwd)/x_grippers:/opt/graspgenx/assets/x_grippers" \
    -e GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx/assets \
    -e GRASPGENX_DEFAULT_GRIPPER=hsrc_hand \
    hrl/graspgenx:latest
# Listens on the zenoh key "graspgenx/generate" (see docker/graspgenx/app.py).
# --net=host is required so zenoh scouting finds the ROS 2 side.
