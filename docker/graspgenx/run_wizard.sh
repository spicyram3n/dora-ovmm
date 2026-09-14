#!/bin/bash
# One-time setup: register the HSRC gripper's geometry with GraspGenX. Run ON
# THE HOST, after gripper_export/export.sh has produced gripper_export/hsrc_hand/.
#
# Opens a web UI at http://localhost:8080 - open that in a browser and work
# through the wizard (align the base frame, confirm the open/closed joint
# states, confirm the gripper type). It writes the "hsrc_hand" config into the
# same x_grippers volume run_graspgenx.sh reads from, so nothing else needs
# to be told about it afterwards.
set -e

cd "$(dirname "$0")"
# Repo root as the build context: the Dockerfile COPYs docker/graspgenx/app.py
# and core/utils/zenoh_rpc.py, the same paths compose.yaml builds it with.
docker build -t hrl/graspgenx:latest -f Dockerfile ../..
docker run --gpus all -it --rm \
    --net=host \
    -v "$(pwd)/checkpoints:/opt/graspgenx/ext/graspgenx_checkpoints" \
    -v "$(pwd)/x_grippers:/opt/graspgenx/assets/x_grippers" \
    -v "$(pwd)/gripper_export/hsrc_hand:/gripper_export/hsrc_hand:ro" \
    -e GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx/assets \
    hrl/graspgenx:latest \
    uv run python3 scripts/gripper_config_wizard.py \
        --urdf /gripper_export/hsrc_hand/hsrc_hand.urdf \
        --name hsrc_hand \
        --output-dir /opt/graspgenx/assets/x_grippers \
        --port 8080
