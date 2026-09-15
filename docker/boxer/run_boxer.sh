#!/usr/bin/env bash
# Run on the GPU host. One offline job per invocation; nothing stays running.
set -euo pipefail
cd "$(dirname "$0")"
if [[ $# -eq 0 || "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'USAGE'
Usage:
  bash docker/boxer/run_boxer.sh --download-ckpts
  bash docker/boxer/run_boxer.sh --download-data [hohen_gen1 ...]
  bash docker/boxer/run_demo.sh
  bash docker/boxer/run_boxer.sh --input nym10_gen1 --max_n=90 --track
  bash docker/boxer/run_boxer.sh --input scene0707_00 --labels=chair,table --fuse

Anything else is passed straight to upstream run_boxer.py.
Uses the existing image. Set BOXER_REBUILD=1 to rebuild after changing this folder.
Set BOXER_CPU=1 to run without the GPU (shared card; fusing needs no GPU).
Sequences and results live in outputs/boxer/; a real-robot run sets
BOXER_DATA=$PWD/outputs/realrobot/<recording>/boxer (BOXER_OUTPUT overrides results only).
This lifts 2D detections to 3D boxes over a recorded sequence; it is not a live service.
USAGE
    if [[ $# -eq 0 ]]; then exit 2; fi
    exit 0
fi
if ! command -v docker >/dev/null; then
    echo 'Docker is required. Run this script on the GPU host.' >&2
    exit 1
fi
# Prepared sequences and results live with the repo's other generated files.
# A real-robot run passes BOXER_DATA=outputs/realrobot/<recording>/boxer.
data=${BOXER_DATA:-$(realpath ../..)/outputs/boxer}
output=${BOXER_OUTPUT:-$data}
mkdir -p ckpts cache "$data" "$output"
if [[ "${BOXER_REBUILD:-0}" == 1 ]] || ! docker image inspect hrl/boxer:latest >/dev/null 2>&1; then
    docker build -t hrl/boxer:latest .
else
    echo 'Using existing hrl/boxer:latest image'
fi

# --labels=scannet200 prompts with the scene graph's own dictionary, so every box
# lands on a class the graph knows. The catch-all names are left out: OWLv2 picks
# each box's best prompt, and "object" would outscore the specific class.
python3 - ../../config/scene_graph/scannet200.yaml cache/scannet200_classes.csv <<'PY'
import sys
import yaml
data = yaml.safe_load(open(sys.argv[1]))
names = [name for role in ("furniture", "object", "structure") for name in data[role]
         if name not in ("furniture", "object", "structure")]
open(sys.argv[2], "w").write("\n".join(names) + "\n")
PY

# BOXER_CPU=1 keeps the container off the GPU entirely: the card is shared, and
# fusing saved boxes (--cache3d --fuse) never needs it.
gpus=(--gpus all)
if [[ "${BOXER_CPU:-0}" == 1 ]]; then gpus=(); fi

# Fusion breaks label-vote ties by set order (utils/fuse_3d_boxes.py upstream),
# which follows Python's per-process hash seed; a fixed seed makes reruns agree.
run() {
    docker run --rm "${gpus[@]}" -e PYTHONHASHSEED=0 \
        -v "$PWD/ckpts:/opt/boxer/ckpts" \
        -v "$data:/opt/boxer/sample_data" \
        -v "$output:/opt/boxer/output" \
        -v "$PWD/cache:/root/.cache" \
        -v "$PWD/cache/scannet200_classes.csv:/opt/boxer/owl/scannet200_classes.csv:ro" \
        "$@"
}

# The two downloads are upstream shell scripts, not run_boxer.py flags, so they
# bypass the entrypoint. Both write into the mounts above and skip existing files.
case "${1:-}" in
    --download-ckpts)
        [[ $# -eq 1 ]] || { echo '--download-ckpts takes no arguments' >&2; exit 2; }
        run --entrypoint bash hrl/boxer:latest scripts/download_ckpts.sh
        exit 0
        ;;
    --download-data)
        shift
        run --entrypoint bash hrl/boxer:latest scripts/download_aria_data.sh "$@"
        exit 0
        ;;
esac

run hrl/boxer:latest "$@"
