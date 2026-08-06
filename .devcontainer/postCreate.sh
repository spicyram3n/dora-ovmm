#!/bin/bash
# devcontainer postCreate: shell env, hsr_ros2 vendor sources, workspace build.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"  # resolved before any cd
ROS2_WS=/home/ws/ros2_ws

sudo chown -R "$(whoami)" /home/ws

# --- shell env ---------------------------------------------------------------

bashrc_line() {
    grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc
}

bashrc_line 'export PYTHONPATH=$PYTHONPATH:/home/ws'
bashrc_line 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'  # matches the real robot
bashrc_line "export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds_profile.xml"

# --- vendor sources ----------------------------------------------------------

mkdir -p "$ROS2_WS/src"

# package list per hsr-project/hsr_ros2_doc, humble, setup_sim_en.md, plus
# hsrb_moveit, which is where move_group comes from
if [ ! -d "$ROS2_WS/src/hsrb_common" ]; then
    echo "cloning hsr_ros2 vendor packages"
    cd "$ROS2_WS/src"
    for repo in \
        hsrb_controllers hsrb_common hsrb_drivers hsrb_launch hsrb_manipulation \
        hsrb_rosnav hsrb_simulator hsr_common hsrb_teleop tmc_gazebo tmc_teleop \
        tmc_common tmc_common_msgs tmc_drivers tmc_database tmc_manipulation \
        tmc_manipulation_base tmc_manipulation_planner tmc_point_cloud \
        tmc_realtime_control tmc_voice tmc_navigation hsrb_moveit; do
        git clone -b humble "https://github.com/hsr-project/${repo}.git"
    done
    rm -rf hsrb_launch/hsrb_robot_launch hsrb_simulator/hsrb_rviz_simulator tmc_drivers/tmc_pgr_camera
fi

# --- dependencies ------------------------------------------------------------

sudo apt-get update  # the Dockerfile clears /var/lib/apt/lists
rosdep update
rosdep install --from-paths "$ROS2_WS/src" --ignore-src -r -y

# --- build -------------------------------------------------------------------

# Expanding the HSR-C URDF fails here rather than at launch. Used both to decide
# whether to wipe and to confirm the rebuild: Docker/devcontainer.json churn
# continuously, so whether the existing install works is the signal, not a hash.
smoke_test() {
    source "$ROS2_WS/install/setup.bash" || return 1
    xacro "$(ros2 pkg prefix hsrc_description)/share/hsrc_description/robots/hsrc1s.urdf.xacro" > /dev/null 2>&1 || return 1
    test -f "$(ros2 pkg prefix hsrb_gazebo_launch)/share/hsrb_gazebo_launch/launch/hsrc_apartment_world.launch.py"
}

if [ -f "$ROS2_WS/install/setup.bash" ] && (smoke_test); then
    echo "ros2_ws healthy - incremental build"
else
    echo "ros2_ws missing or unhealthy - wiping build/install/log"
    rm -rf "$ROS2_WS/build" "$ROS2_WS/install" "$ROS2_WS/log"
fi

# unconditional: the patches are idempotent and a re-cloned src loses them
"$SCRIPT_DIR/bug_fixes.sh" "$ROS2_WS"

# no rmw_zenohd router here - this container speaks rmw_cyclonedds_cpp, and the
# zenoh RPC to SAM3/GraspGenX (core/utils/zenoh_rpc.py) scouts without a router
cd "$ROS2_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

if ! (smoke_test); then
    echo "postCreate.sh: HSR-C smoke test failed after colcon build" >&2
    exit 1
fi
echo "HSR-C sim ready: ros2 launch hsrb_gazebo_launch hsrc_apartment_world.launch.py"
