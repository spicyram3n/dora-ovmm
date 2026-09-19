#!/usr/bin/env bash
# Keyframes -> 3D boxes -> scene graph: steps 1-3 of README.md in one go.
#
#   bash docker/boxer/build_scene_graph.sh                  # lab_20260811, GPU
#   bash docker/boxer/build_scene_graph.sh --cpu            # shared card busy
#   bash docker/boxer/build_scene_graph.sh --fuse-only      # after a Ctrl-C
#   bash docker/boxer/build_scene_graph.sh --graph-only     # step 3 alone
#   bash docker/boxer/build_scene_graph.sh --graph-only --rooms  # ... and name rooms
#   bash docker/boxer/build_scene_graph.sh other_recording  # another bag
#   export RECORDING=other_recording                     # ... and every other tool
#   bash docker/boxer/build_scene_graph.sh --bag bags/lab_newdata lab_newdata
#                                             # ... from the bag: data prep too
#   WHEEL_ODOM=1 bash docker/boxer/build_scene_graph.sh \
#       --bag bags/lab_newdata --rooms --nav-map lab_newdata
#                                             # bag -> robot-ready map and graph
#   ... --bag bags/lab_newdata --re-map ...   # rebuild the map, and all that hangs off it
#   ... --bag bags/lab_newdata --re-extract . # keep the map, redo keyframes and onward
#
# Run it on the GPU host: step 2 runs Docker, which the dev container has not got.
# Everything else needs what only the dev container has (ROS, OpenCV, networkx),
# so those steps are run inside it with `docker exec`. The repo is one bind mount
# seen by both, so the files are shared. The container is found by the label VS
# Code gives it; set DEV_CONTAINER=<name or id> to name it yourself.
#
# Nothing here stops or removes a container or an image. Boxer is `docker run
# --rm`, so its container goes when it exits; its image stays, being a long build.
#
# Safe to rerun: step 1 is skipped once the frames are linked, and step 2
# overwrites its own CSVs. Only --rooms asks DeepSeek anything (it needs
# DEEPSEEK_API_KEY); the graph step 3 writes is what core.reasoner.query reads.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$root"

# $RECORDING is the same switch core.utils.recording gives the Python tools,
# so one export moves the whole pipeline to another bag. A positional argument
# still wins, for building one recording without changing the shell's default.
recording=${RECORDING:-lab_20260811}
stride=1
min_confidence=
rooms=0
device=gpu
compile_mode=auto
video=1
only=
reprepare=0
bag=
nav_map=0
rate=1          # /scan is 40 Hz; rate 2 feeds slam_toolbox 80 Hz and it drops scans
redo=()

die() { echo "error: $*" >&2; exit 1; }
step() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

while [[ $# -gt 0 ]]; do
    case $1 in
        --cpu) device=cpu; shift ;;
        --no-compile) compile_mode=off; shift ;;
        --no-video) video=0; shift ;;
        --stride) stride=$2; shift 2 ;;
        --min-confidence) min_confidence=$2; shift 2 ;;
        --rooms) rooms=1; shift ;;
        --bag) bag=$2; shift 2 ;;
        --fuse-only) only=fuse; shift ;;
        --graph-only) only=graph; shift ;;
        --re-prepare) reprepare=1; shift ;;
        --nav-map) nav_map=1; shift ;;
        --rate) rate=$2; shift 2 ;;
        # Keyframe poses come from the map's SLAM run, and Boxer's frames from the
        # keyframes: redoing one without what hangs off it leaves stale poses behind.
        --re-map) redo=(--re-map --re-extract); reprepare=1; shift ;;
        --re-extract) redo=(--re-extract); reprepare=1; shift ;;
        -h|--help) sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) die "unknown option $1" ;;
        *) recording=$1; shift ;;
    esac
done

src=outputs/realrobot/$recording/keyframes
work=outputs/realrobot/$recording/boxer
seq=$work/scannet/$recording
res=$work/$recording
fused=$res/boxer_3dbbs_fused.csv
registration=config/realrobot/map/$recording/boxer_to_map.json
graph=config/realrobot/scene_graph/$recording.json

# --- preflight, before anything that takes half an hour ----------------------

# The steps around Boxer need ROS (data prep), OpenCV (the nav map) and numpy,
# scipy, networkx, yaml (the graph). A GPU host usually has none of them.
missing=$(python3 - <<'PY' 2>/dev/null || echo python3
import importlib.util
print(",".join(m for m in ("numpy", "scipy", "networkx", "yaml", "cv2")
                if not importlib.util.find_spec(m)))
PY
)
[[ -e /opt/ros ]] || missing=${missing:+$missing,}ros

# Where those steps run: here if this machine can, else in the dev container.
container=${DEV_CONTAINER:-}
if [[ -z $container && -n $missing ]]; then
    command -v docker >/dev/null || die "this machine lacks $missing and has no docker to reach the dev container"
    container=$(docker ps -q --filter "label=devcontainer.local_folder=$root" | head -n 1)
    [[ -n $container ]] || die "this machine lacks $missing and no dev container for $root is running; open the repo in VS Code, or set DEV_CONTAINER=<name>"
fi
[[ -n $container ]] && echo "ROS and Python steps run in the dev container: $container"

# dev <command...>: one step, run from the repo root with ROS sourced. The workspace
# overlay too: core/ imports Toyota's tmc_* messages, which live in ros2_ws, and a
# `docker exec` shell starts with neither.
ros_setup="source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash"
dev() {
    local line; line=$(printf '%q ' "$@")
    if [[ -z $container ]]; then
        bash -c "$ros_setup && $line"
    else
        docker exec -u hsr -w /home/ws -e RECORDING="$recording" -e WHEEL_ODOM="${WHEEL_ODOM:-}" \
            -e DEEPSEEK_API_KEY="${DEEPSEEK_API_KEY:-}" "$container" \
            bash -c "$ros_setup && $line"
    fi
}

# --- step 0: bag -> map + keyframes, when asked and not already done --------

# Data prep needs ROS and not the GPU, the other way round from step 2.
[[ ${#redo[@]} -gt 0 && -z $bag ]] && die "--re-map and --re-extract read the bag again: add --bag <path>"
if [[ -n $bag && $only == "" ]] && [[ ${#redo[@]} -gt 0 || ! -d $src/poses ]]; then
    step "Step 0: data prep for $recording from $bag"
    dev bash realrobot/dataprep/prepare.sh --bag "$bag" --rate "$rate" "${redo[@]}" "$recording"
    # The runbook's gate, before any GPU time. It reports; both lab recordings sit
    # just under its 80% (75% and 78%), so failing on it would refuse every build.
    step "Gate: keyframe cloud against the map (latest_readme.md section 4 wants >= 80% within 10 cm)"
    figures=realrobot/dataprep/figures_$recording
    dev python3 realrobot/dataprep/visualize.py --scene "$src" \
        --map "config/realrobot/map/$recording/map.yaml" \
        --slam-tf "outputs/realrobot/$recording/slam/tf_mapping" \
        --bag "$bag" --figures "$figures" >/dev/null
    dev python3 -c 'import json, sys
a = json.load(open(sys.argv[1]))["laser_height_alignment"]
print("    %.1f%% of the cloud within 10 cm of the map walls, median %.1f cm" % (a["within_10cm"] * 100, a["median_m"] * 100))' "$figures/checks.json"
fi

if [[ $only != graph ]]; then
    [[ -d $src/poses ]] || die "no keyframes at $src (run with --bag <path>, or see realrobot/dataprep/realrobot_dataprep.md)"
    command -v docker >/dev/null || die "step 2 needs Docker; run this on the GPU host"
    ls docker/boxer/ckpts/*.ckpt >/dev/null 2>&1 \
        || die "no weights in docker/boxer/ckpts (run: bash docker/boxer/run_boxer.sh --download-ckpts)"
fi

# --- step 1: keyframes -> Boxer layout ---------------------------------------

if [[ $only == "" ]]; then
    if [[ $reprepare == 1 ]]; then
        step "Removing the prepared sequence (--re-prepare)"
        rm -rf "$seq"
    fi
    if [[ -d $seq/frames ]]; then
        have=$(ls "$seq/frames/pose" 2>/dev/null | wc -l)
        total=$(ls "$src/poses" | wc -l)
        want=$(( (total + stride - 1) / stride ))
        [[ $have == "$want" ]] \
            || die "$seq holds $have frames, --stride $stride wants $want; rerun with --re-prepare"
        step "Step 1 already done: $have keyframes linked at $seq"
    else
        step "Step 1: keyframes -> Boxer layout"
        dev python3 realrobot/dataprep/make_boxer_scene.py \
            --source "$src" --output "$seq" \
            --registration "$registration" --stride "$stride"
    fi
fi

# --- step 2: detect -> 3D boxes -> fuse --------------------------------------

if [[ $only != graph ]]; then
    if [[ $device == gpu ]]; then
        # Triton JIT-builds a CUDA shim on the first GPU forward. Old images have
        # no compiler for it, so top the image up; if that still leaves none,
        # fall back to eager OWLv2 rather than dying in the warmup.
        if [[ $compile_mode == auto ]] && docker image inspect hrl/boxer:latest >/dev/null 2>&1 \
           && ! docker run --rm --entrypoint bash hrl/boxer:latest -c \
                'command -v gcc >/dev/null && test -e /usr/include/python3.12/Python.h' 2>/dev/null; then
            step "Step 2a: image has no C compiler for Triton, rebuilding that layer"
            BOXER_REBUILD=1 bash docker/boxer/run_boxer.sh --build \
                || die "rebuild failed; rerun with --no-compile"
            docker run --rm --entrypoint bash hrl/boxer:latest -c \
                'command -v gcc >/dev/null && test -e /usr/include/python3.12/Python.h' 2>/dev/null \
                || compile_mode=off
        fi
        if [[ $compile_mode == off ]]; then
            echo "    torch.compile disabled: OWLv2 runs eagerly, slower but no Triton build"
            export TORCH_COMPILE_DISABLE=1 TORCHDYNAMO_DISABLE=1
        fi
    fi

    boxer_flags=(--input "/opt/boxer/sample_data/scannet/$recording" --labels=scannet200 --fuse)
    [[ $only == fuse ]] && boxer_flags+=(--cache3d)      # reuse the boxes already saved
    [[ $video == 0 ]] && boxer_flags+=(--skip_viz)

    step "Step 2: detect -> 3D boxes -> fuse (${device^^})"
    BOXER_CPU=$([[ $device == cpu ]] && echo 1 || echo 0) \
    BOXER_DATA=$PWD/$work \
        bash docker/boxer/run_boxer.sh "${boxer_flags[@]}"

    [[ -s $fused ]] || die "$fused was not written"
    # The container writes as root unless the daemon remaps users.
    [[ -w $res ]] || echo "    note: results are root-owned; sudo chown -R \$USER $res $work"
fi

# --- step 3: boxes -> scene graph --------------------------------------------

[[ -s $fused ]] || die "no fused boxes at $fused; run step 2 first"

graph_cmd=(python3 -m docker.boxer.to_scene_graph
           --boxes "$fused" --transform "$registration" --output "$graph")
[[ -n $min_confidence ]] && graph_cmd+=(--min-confidence "$min_confidence")
# Needs DEEPSEEK_API_KEY; without it step 3 leaves every room null, as before.
# The map goes with it: the rooms are divided on the floor plan, and DeepSeek is
# asked only to name each one.
[[ $rooms == 1 ]] && graph_cmd+=(--rooms --map "config/realrobot/map/$recording/map.yaml")

step "Step 3: boxes -> scene graph"
dev "${graph_cmd[@]}"

if [[ $nav_map == 1 ]]; then
    step "Step 4: the map Nav2 serves, with the fixed furniture stamped in"
    dev python3 realrobot/offline/plan_overlay.py --nav-map \
        --graph "$graph" --map "config/realrobot/map/$recording/map.yaml"
fi

cat <<EOF

Scene graph: $graph  (map frame, the one $recording.yaml's map is in)

Look at it:
  python3 visualization/scene_graph.py --graph $graph \\
    --scene $src/scene.ply \\
    --output outputs/realrobot/$recording/scene_graph.rrd

Check a furniture goal, offline, no robot:
  python3 realrobot/offline/plan_overlay.py --furniture bookshelf --graph $graph
EOF
