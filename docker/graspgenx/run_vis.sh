#!/bin/bash
# Visualize a registered gripper (default: hsrc_hand) in the browser. Run ON
# THE HOST, after run_wizard.sh has registered the gripper.
#
# Usage:
#   ./run_vis.sh                              # visualize hsrc_hand
#   ./run_vis.sh hsrc_hand --port 8090        # when 8080 is taken (ggx_dev holds it)
#   ./run_vis.sh some_other_gripper            # a different registered gripper
set -e

GRIPPER="${1:-hsrc_hand}"
[ $# -gt 0 ] && shift

cd "$(dirname "$0")"
# Repo root as the build context: the Dockerfile COPYs docker/graspgenx/app.py
# and core/utils/zenoh_rpc.py, the same paths compose.yaml builds it with.
docker build -t hrl/graspgenx:latest -f Dockerfile ../..
docker run --gpus all -it --rm \
    --net=host \
    -v "$(pwd)/checkpoints:/opt/graspgenx/ext/graspgenx_checkpoints" \
    -v "$(pwd)/x_grippers:/opt/graspgenx/assets/x_grippers" \
    -e GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx/assets \
    hrl/graspgenx:latest \
    uv run python3 scripts/vis_gripper_desc.py --gripper "$GRIPPER" \
        --root /opt/graspgenx/assets/x_grippers --port 8080 "$@"
# Open http://localhost:8080 in a browser.
