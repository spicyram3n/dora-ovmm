#!/bin/bash
# Run on the host. Models remain running after this command exits.
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose -f compose.yaml up -d --build --wait --wait-timeout 900 vgn
