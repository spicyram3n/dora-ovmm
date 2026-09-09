#!/usr/bin/env bash
# Run on the GPU host from any directory. First prepare_hrl.py, then checkpoints.
set -euo pipefail
cd "$(dirname "$0")"
bash run_openyolo3d.sh \
    --scene /data/hrl --output /output/hrl \
    --source-frame hrl_scan --depth-scale 1000 \
    --labels chair table desk sofa armchair cabinet shelf bookshelf bed \
      door window monitor television laptop keyboard mouse printer \
      refrigerator microwave sink trashcan box bag book bottle cup bowl plate \
      plant lamp backpack pillow \
    "$@"
