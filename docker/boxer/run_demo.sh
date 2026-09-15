#!/usr/bin/env bash
# Upstream Demo #1 end to end: weights, one Aria sequence, then 90 frames.
# Run on the GPU host from any directory. Expect a few minutes on the downloads.
set -euo pipefail
cd "$(dirname "$0")"
bash run_boxer.sh --download-ckpts
bash run_boxer.sh --download-data nym10_gen1
bash run_boxer.sh --input nym10_gen1 --max_n=90 --track "$@"
echo 'Results: outputs/boxer/nym10_gen1/'
