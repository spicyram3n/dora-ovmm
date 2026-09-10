#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ $# -eq 0 || "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'USAGE'
Usage:
  bash docker/openyolo3d/run_openyolo3d.sh --download-checkpoints
  bash docker/openyolo3d/run_hrl.sh --check-only
  bash docker/openyolo3d/run_hrl.sh
  bash docker/openyolo3d/run_openyolo3d.sh \
    --scene /data/room1 --output /output/room1 --source-frame room1_scan \
    --labels chair table cup

Uses the existing image. Set OPENYOLO3D_REBUILD=1 to rebuild after code changes.
Input: docker/openyolo3d/data/; results: docker/openyolo3d/output/.
This runs an offline reconstructed-scene job, not a live camera service.
USAGE
    if [[ $# -eq 0 ]]; then exit 2; fi
    exit 0
fi
if ! command -v docker >/dev/null; then
    echo 'Docker is required. Run this script on the GPU host.' >&2
    exit 1
fi
mkdir -p checkpoints cache data output
if [[ "${OPENYOLO3D_REBUILD:-0}" == 1 ]] || ! docker image inspect hrl/openyolo3d:latest >/dev/null 2>&1; then
    docker build -t hrl/openyolo3d:latest .
else
    echo 'Using existing hrl/openyolo3d:latest image'
fi
docker run --rm --gpus all --shm-size=8g \
    -v "$PWD/checkpoints:/opt/openyolo3d/pretrained/checkpoints" \
    -v "$PWD/cache:/root/.cache" \
    -v "${OPENYOLO3D_DATA:-$PWD/data}:/data:ro" \
    -v "${OPENYOLO3D_OUTPUT:-$PWD/output}:/output" \
    hrl/openyolo3d:latest "$@"
