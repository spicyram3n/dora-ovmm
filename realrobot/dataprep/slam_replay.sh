#!/usr/bin/env bash
# Replay a bag through slam_toolbox (one mapping pass) and keep, for $RECORDING:
#   config/realrobot/map/<recording>/map.{pgm,yaml}   the Nav2 map
#   outputs/realrobot/<recording>/slam/tf_mapping/    bag of /tf with SLAM's map->odom
#   outputs/realrobot/<recording>/slam/posegraph.*    slam_toolbox's graph, for reuse
# Usage: bash realrobot/dataprep/slam_replay.sh [rate]     (rate 2 = 6.5 min for a 13 min bag)
#   RECORDING=<name> names the outputs; BAG=<path> is the bag to replay.
source /opt/ros/humble/setup.bash
set -euo pipefail
# Another ROS 2 system shares this machine; a private domain keeps its /tf out.
# (CYCLONEDDS_URI from the devcontainer already pins the loopback interface.)
export ROS_DOMAIN_ID=87
RATE=${1:-2}
ROOT=/home/ws
RECORDING=${RECORDING:-lab_20260811}
BAG=${BAG:-$ROOT/bags/playable_bag}
OUT=$ROOT/outputs/realrobot/$RECORDING/slam
MAP=$ROOT/config/realrobot/map/$RECORDING/map
[[ -e $BAG ]] || { echo "no bag at $BAG" >&2; exit 1; }
NODE=$(ros2 pkg prefix slam_toolbox)/lib/slam_toolbox/sync_slam_toolbox_node
mkdir -p "$OUT" "$(dirname "$MAP")"
rm -rf "$OUT/tf_mapping"
# extract_rgbd.py pairs SLAM's map->odom with the bag's odometry; it has to be this one.
[[ -n ${WHEEL_ODOM:-} ]] && echo wheel > "$OUT/odometry" || echo laser > "$OUT/odometry"

# A node left over from an earlier run would publish a second map->odom.
pkill -f "[s]lam_toolbox_node" && sleep 2 || true
# The bag's /tf goes to /tf_bag; tf_filter forwards it minus the old map->odom.
# WHEEL_ODOM=1 feeds SLAM the bag's wheel odometry instead; see tf_filter.py.
python3 "$ROOT/realrobot/dataprep/tf_filter.py" ${WHEEL_ODOM:+--wheel-odom} &
# The node binary is started directly: killing a `ros2 run` wrapper leaves its child alive.
"$NODE" --ros-args --params-file "$ROOT/realrobot/dataprep/slam_offline.yaml" > "$OUT/slam.log" 2>&1 &
ros2 bag record /tf /tf_static -o "$OUT/tf_mapping" > /dev/null 2>&1 &
RECORDER=$!
trap 'kill $(jobs -p) 2>/dev/null || true; wait 2>/dev/null || true' EXIT
sleep 5

START=$(date +%s)
ros2 bag play "$BAG" --clock 200 --rate "$RATE" \
  --topics /scan /tf /tf_static --remap /tf:=/tf_bag
echo "bag played in $(( $(date +%s) - START )) s at rate $RATE"

# slam_toolbox keeps publishing the final map after the bag ends.
# --free 0.196: the saver writes unknown as grey 205 (occupancy 0.1961), and its
# default 0.25 makes map_server load every unknown cell back as free.
ros2 run nav2_map_server map_saver_cli -f "$MAP" --free 0.196 \
  --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$OUT/posegraph'}"
kill $RECORDER; wait $RECORDER 2>/dev/null || true   # let the recorder flush
ls -la "$OUT" "$(dirname "$MAP")"
