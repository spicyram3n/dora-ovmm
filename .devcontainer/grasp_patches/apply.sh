#!/bin/bash
# Apply this project's changes to the cloned HSR vendor repositories.
#
# Each <repo>.patch here was made against the commit recorded in <repo>.base.
# Checkouts that already carry a patch are skipped, so reruns are safe.
# Prints each repository it patched on stdout; progress goes to stderr.
#
#   bash .devcontainer/grasp_patches/apply.sh [path/to/ros2_ws/src]
set -euo pipefail

PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${1:-$PATCH_DIR/../../ros2_ws/src}"

for patch in "$PATCH_DIR"/*.patch; do
    repo=$(basename "$patch" .patch)
    if git -C "$SRC/$repo" apply --reverse --check "$patch" 2>/dev/null; then
        continue
    fi
    if ! git -C "$SRC/$repo" apply --check "$patch"; then
        echo "Cannot apply $patch; it was made against $repo $(cat "${patch%.patch}.base")." >&2
        echo "Stash or reset local edits in $SRC/$repo, then rerun." >&2
        exit 1
    fi
    echo "Patching $repo..." >&2
    git -C "$SRC/$repo" apply "$patch"
    echo "$repo"
done
