#!/bin/sh
set -eu
role=${1:-}
case "$role" in
    robot) domain=5 ;;
    pc) domain=9; : "${2:?Usage: sh run_bridge.sh pc ROBOT_IP}" ;;
    *) echo "Usage: sh run_bridge.sh robot | pc ROBOT_IP" >&2; exit 2 ;;
esac
bridge_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if ! command -v zenoh-bridge-ros2dds >/dev/null 2>&1; then
    echo "Install zenoh-bridge-ros2dds in this ROS container; see $bridge_dir/README.md" >&2
    exit 127
fi
if [ "$role" = pc ]; then
    set -- -e "tcp/$2:7449"
else
    set --
fi
# Run alongside ROS in the existing container. Ctrl+C stops the bridge.
export ROS_DOMAIN_ID="$domain"
export RUST_LOG="${RUST_LOG:-info}"
exec zenoh-bridge-ros2dds -c "$bridge_dir/$role.json5" "$@"
