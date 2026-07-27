#!/bin/bash
# Run real GraspGenX inference on bundled sample point clouds, for a
# registered gripper (default: hsrc_hand). Run ON THE HOST, after
# run_wizard.sh has registered the gripper. Opens a browser viewer.
#
# Usage:
#   ./run_demo.sh                        # inference on hsrc_hand, gripper mesh shown
#   ./run_demo.sh --vis-top-grasp-meshes --num-top-grasp-meshes 10   # more than 5
set -e

cd "$(dirname "$0")"
docker build -t hrl/graspgenx:latest .
docker run --gpus all -it --rm \
    --net=host \
    -v "$(pwd)/checkpoints:/opt/graspgenx/ext/graspgenx_checkpoints" \
    -v "$(pwd)/x_grippers:/opt/graspgenx/assets/x_grippers" \
    -e GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx/assets \
    hrl/graspgenx:latest \
    uv run python3 scripts/demo_object_pc.py \
        --sample_data_dir /opt/graspgenx/assets/sample_data/object_pc \
        --gripper_name hsrc_hand --plot_top_mesh "$@"
# Watch the terminal output for the viser URL (usually http://localhost:8080).
