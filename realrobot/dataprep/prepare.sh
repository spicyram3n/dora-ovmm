#!/usr/bin/env bash
# Bag -> Nav2 map -> keyframes: data prep steps 1 and 2 in one go.
#
#   bash realrobot/dataprep/prepare.sh --bag bags/lab_newdata lab_newdata
#   bash realrobot/dataprep/prepare.sh                    # lab_20260811, its own bag
#   bash realrobot/dataprep/prepare.sh --rate 1           # slower replay, same result
#
# Needs ROS sourced and the container: step 1 replays the bag through
# slam_toolbox, step 2 reads the bag again to pose and gate the keyframes.
# Neither step touches the GPU. Afterwards, docker/boxer/build_scene_graph.sh
# turns the keyframes into a scene graph.
#
# Safe to rerun: each step is skipped once its output exists, so a failure part
# way through costs only the step that failed. --re-map and --re-extract redo one.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$root"

recording=${RECORDING:-lab_20260811}
bag=${BAG:-}
rate=2
remap=0
reextract=0

die() { echo "error: $*" >&2; exit 1; }
step() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

while [[ $# -gt 0 ]]; do
    case $1 in
        --bag) bag=$2; shift 2 ;;
        --rate) rate=$2; shift 2 ;;
        --re-map) remap=1; shift ;;
        --re-extract) reextract=1; shift ;;
        -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) die "unknown option $1" ;;
        *) recording=$1; shift ;;
    esac
done

bag=${bag:-bags/playable_bag}
map=config/realrobot/map/$recording/map.yaml
slam=outputs/realrobot/$recording/slam
keyframes=outputs/realrobot/$recording/keyframes
registration=config/realrobot/map/$recording/scan_to_map.json

# --- preflight, before a replay that takes minutes --------------------------

[[ -e $bag ]] || die "no bag at $bag (--bag to point elsewhere)"
command -v ros2 >/dev/null || die "ROS is not sourced: source /opt/ros/humble/setup.bash"

# --- step 1: bag -> Nav2 map + SLAM's map->odom -----------------------------

if [[ $remap == 1 ]]; then
    step "Removing the map and pose graph (--re-map)"
    rm -rf "$slam" "config/realrobot/map/$recording"
fi
if [[ -s $map && -d $slam/tf_mapping ]]; then
    step "Step 1 already done: $map"
else
    step "Step 1: $bag -> $map  (~6.5 min for a 13 min bag at rate $rate)"
    RECORDING=$recording BAG=$root/$bag bash realrobot/dataprep/slam_replay.sh "$rate"
fi

# --- step 2: bag + those poses -> keyframes ---------------------------------

if [[ $reextract == 1 ]]; then
    step "Removing the keyframes (--re-extract)"
    rm -rf "$keyframes"
fi
if [[ -d $keyframes/poses ]]; then
    step "Step 2 already done: $(ls "$keyframes/poses" | wc -l) keyframes at $keyframes"
else
    step "Step 2: keyframes and fused cloud"
    # extract_rgbd refuses to overwrite, which is why the check above exists.
    python3 realrobot/dataprep/extract_rgbd.py \
        --bag "$bag" --slam-tf "$slam/tf_mapping" \
        --output "$keyframes" --registration "$registration"
fi

cat <<EOF

Prepared: $(ls "$keyframes/poses" | wc -l) keyframes at $keyframes
          map at $map

Now build the scene graph (needs Docker and the GPU host):
  RECORDING=$recording bash docker/boxer/build_scene_graph.sh --rooms

Check the data prep first, if you want the figures:
  cd realrobot/dataprep && python3 visualize.py \\
    --scene ../../$keyframes --map ../../$map \\
    --slam-tf ../../$slam/tf_mapping --bag ../../$bag --figures figures
EOF
