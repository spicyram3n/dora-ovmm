#!/usr/bin/env bash
# NVIDIA workstation launcher for the complete search pipeline.
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname -- "$SCRIPT_DIR")"
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
# Optional local driver-library fallback; version must match the host driver.
if test -r "$PROJECT_ROOT/.runtime/nvidia-gl/libGLX_nvidia.so.0"; then
    export LD_LIBRARY_PATH="$PROJECT_ROOT/.runtime/nvidia-gl:${LD_LIBRARY_PATH:-}"
fi
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export QT_X11_NO_MITSHM=1
export QT_XCB_GL_INTEGRATION=xcb_glx
unset LIBGL_ALWAYS_SOFTWARE LIBGL_DRI3_DISABLE MESA_LOADER_DRIVER_OVERRIDE GALLIUM_DRIVER QT_QUICK_BACKEND QSG_RENDER_LOOP
exec ros2 launch "$SCRIPT_DIR/search.launch.py" "$@"
