#!/usr/bin/env bash
# Run inside the dora-ovmm dev container. Display workaround verified on
# this workstation: internal Gazebo renders worked while its Qt window
# was entirely black with the default graphics environment.
set -e
source /opt/ros/humble/setup.bash
source /home/ws/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_X11_NO_MITSHM=1
export LIBGL_DRI3_DISABLE=1
export LIBGL_ALWAYS_SOFTWARE=1
exec ros2 launch /home/ws/ros2_ws/launch/hsrc_apartment_near_shelf.launch.py \
  gui:=true rviz:=false use_navigation:=false use_teleop:=false "$@"
