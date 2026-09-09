#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v docker >/dev/null; then
    echo 'Docker is required. Run this script on the GPU host.' >&2
    exit 1
fi
mkdir -p checkpoints cache data output
docker build -t hrl/openyolo3d:latest .
docker run --rm --gpus all --shm-size=8g \
    -v "$PWD/checkpoints:/opt/openyolo3d/pretrained/checkpoints" \
    -v "$PWD/cache:/root/.cache" \
    -v "${OPENYOLO3D_DATA:-$PWD/data}:/data:ro" \
    -v "${OPENYOLO3D_OUTPUT:-$PWD/output}:/output" \
    hrl/openyolo3d:latest "$@"
