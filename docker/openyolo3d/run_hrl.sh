#!/usr/bin/env bash
# Run on the GPU host from any directory. First prepare_hrl.py, then checkpoints.
set -euo pipefail
cd "$(dirname "$0")"
bash run_openyolo3d.sh \
    --scene /data/hrl --output /output/hrl \
    --source-frame hrl_scan --depth-scale 1000 \
    "$@"
