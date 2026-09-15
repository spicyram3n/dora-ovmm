#!/usr/bin/env bash
# Replay bags/playable_bag through slam_toolbox (one mapping pass) and keep:
#   config/realrobot/map/lab_20260811.{pgm,yaml}       the Nav2 map
#   outputs/realrobot/lab_20260811/slam/tf_mapping/    bag of /tf with SLAM's map->odom
#   outputs/realrobot/lab_20260811/slam/posegraph.*    slam_toolbox's graph, for reuse
# Usage: bash realrobot/slam_replay.sh [rate]          (rate 2 = 6.5 min for the 13 min bag)
source /opt/ros/humble/setup.bash
set -euo pipefail
# Another ROS 2 system shares this machine; a private domain keeps its /tf out.
# (CYCLONEDDS_URI from the devcontainer already pins the loopback interface.)
export ROS_DOMAIN_ID=87
RATE=${1:-2}
ROOT=/home/ws
BAG=$ROOT/bags/playable_bag
OUT=$ROOT/outputs/realrobot/lab_20260811/slam
MAP=$ROOT/config/realrobot/map/lab_20260811
NODE=$(ros2 pkg prefix slam_toolbox)/lib/slam_toolbox/sync_slam_toolbox_node
mkdir -p "$OUT" "$(dirname "$MAP")"
rm -rf "$OUT/tf_mapping"

# A node left over from an earlier run would publish a second map->odom.
pkill -f "[s]lam_toolbox_node" && sleep 2 || true
# The bag's /tf goes to /tf_bag; tf_filter forwards it minus the old map->odom.
python3 "$ROOT/realrobot/tf_filter.py" &
# The node binary is started directly: killing a `ros2 run` wrapper leaves its child alive.
"$NODE" --ros-args --params-file "$ROOT/realrobot/slam_offline.yaml" > "$OUT/slam.log" 2>&1 &
ros2 bag record /tf /tf_static -o "$OUT/tf_mapping" > /dev/null 2>&1 &
RECORDER=$!
trap 'kill $(jobs -p) 2>/dev/null || true; wait 2>/dev/null || true' EXIT
sleep 5

START=$(date +%s)
ros2 bag play "$BAG" --clock 200 --rate "$RATE" \
  --topics /scan /tf /tf_static --remap /tf:=/tf_bag
echo "bag played in $(( $(date +%s) - START )) s at rate $RATE"

# slam_toolbox keeps publishing the final map after the bag ends.
ros2 run nav2_map_server map_saver_cli -f "$MAP" \
  --ros-args -p map_subscribe_transient_local:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$OUT/posegraph'}"
kill $RECORDER; wait $RECORDER 2>/dev/null || true   # let the recorder flush
ls -la "$OUT" "$(dirname "$MAP")"
