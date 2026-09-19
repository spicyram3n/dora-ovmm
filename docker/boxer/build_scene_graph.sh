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
#
# Step 2 runs Docker, so this belongs on the GPU host. Steps 1 and 3 are plain
# Python (numpy, scipy, networkx, yaml); if the host lacks them the script stops
# before the long run and prints the command to finish in the dev container.
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
        -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
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

# Step 3 imports core/, so check its dependencies now rather than after the run.
graph_deps=$(python3 - <<'PY'
import importlib.util
print(",".join(m for m in ("numpy", "scipy", "networkx", "yaml")
                if not importlib.util.find_spec(m)))
PY
) || die "no working python3 on PATH"

# --- step 0: bag -> map + keyframes, when asked and not already done --------

# Data prep is a separate script because it needs ROS and not the GPU, which is
# the other way round from step 2. --bag runs it here so one command covers a
# new recording end to end on a host that has both.
if [[ -n $bag && $only == "" && ! -d $src/poses ]]; then
    step "Step 0: data prep for $recording from $bag"
    RECORDING=$recording bash realrobot/dataprep/prepare.sh --bag "$bag" "$recording"
fi

if [[ $only != graph ]]; then
    [[ -d $src/poses ]] || die "no keyframes at $src (run with --bag <path>, or see realrobot/dataprep/realrobot_dataprep.md)"
    command -v docker >/dev/null || die "step 2 needs Docker; run this on the GPU host"
    ls docker/boxer/ckpts/*.ckpt >/dev/null 2>&1 \
        || die "no weights in docker/boxer/ckpts (run: bash docker/boxer/run_boxer.sh --download-ckpts)"
fi
if [[ $only == graph && -n $graph_deps ]]; then
    die "step 3 needs $graph_deps; run it in the dev container"
fi
# Step 1 writes the registration with numpy; step 3 can wait for the container.
if [[ $only == "" && $graph_deps == *numpy* && ! -d $seq/frames ]]; then
    die "step 1 needs numpy; prepare the sequence in the dev container first"
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
        python3 realrobot/dataprep/make_boxer_scene.py \
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

if [[ -n $graph_deps ]]; then
    step "Step 3 needs $graph_deps, which this host has not got"
    echo "    Boxes are ready. Finish in the dev container, from the repo root:"
    printf '      %s\n' "${graph_cmd[*]}"
    exit 0
fi

step "Step 3: boxes -> scene graph"
"${graph_cmd[@]}"

cat <<EOF

Scene graph: $graph  (map frame, the one $recording.yaml's map is in)

Look at it:
  python3 visualization/scene_graph.py --graph $graph \\
    --scene $src/scene.ply \\
    --output outputs/realrobot/$recording/scene_graph.rrd

Check a furniture goal, offline, no robot:
  python3 realrobot/offline/plan_overlay.py --furniture bookshelf --graph $graph
EOF
