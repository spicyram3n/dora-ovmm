#!/bin/sh
xhost +local:docker 2>/dev/null || true

# devcontainer.json bind-mounts this into /home/ws/bags/playable_bag, and
# docker refuses to start if the source is missing. On a machine that has the
# bag, make it a symlink to the real bag directory instead; docker follows it.
mkdir -p "$HOME/rosbags/playable_bag"
