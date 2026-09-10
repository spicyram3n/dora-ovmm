#!/usr/bin/env bash
# Workstation-specific graphics fallback: libraries copied from the host's
# NVIDIA 580.159.04 driver into .runtime/nvidia-gl. Refresh after driver updates.
set -e
source /opt/ros/humble/setup.bash
source /home/ws/ros2_ws/install/setup.bash
test -r /home/ws/.runtime/nvidia-gl/libGLX_nvidia.so.0
export LD_LIBRARY_PATH="/home/ws/.runtime/nvidia-gl:${LD_LIBRARY_PATH:-}"
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export QT_X11_NO_MITSHM=1
export QT_XCB_GL_INTEGRATION=xcb_glx
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset LIBGL_ALWAYS_SOFTWARE LIBGL_DRI3_DISABLE MESA_LOADER_DRIVER_OVERRIDE GALLIUM_DRIVER
exec ros2 launch /home/ws/ros2_ws/launch/hsrc_apartment_near_shelf.launch.py \
  gui:=true rviz:=false use_navigation:=false use_teleop:=false "$@"
