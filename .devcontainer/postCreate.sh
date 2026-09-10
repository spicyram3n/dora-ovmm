#!/bin/bash
set -e

# resolved before the cd below
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

sudo chown -R $(whoami) /home/ws

grep -qxF 'export PYTHONPATH=$PYTHONPATH:/home/ws' ~/.bashrc || \
    echo 'export PYTHONPATH=$PYTHONPATH:/home/ws' >> ~/.bashrc

# Was rmw_zenoh_cpp so this container could see SAM3/GraspGenX over zenoh
# scouting instantly instead of falling back to plain scouting. The real robot
# only ships rmw_cyclonedds_cpp, and rmw_zenoh_cpp/rmw_cyclonedds_cpp nodes
# can't discover each other at all (different transports, not just different
# QoS), so this container now matches the robot instead. The zenoh RPC path
# to SAM3/GraspGenX (core/utils/zenoh_rpc.py) doesn't go through the RMW at
# all - it opens its own bare zenoh session - so it's unaffected either way.
grep -qxF 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' ~/.bashrc || \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

ROS2_WS=/home/ws/ros2_ws
mkdir -p "$ROS2_WS/src"

# hsr_ros2 package list per hsr-project/hsr_ros2_doc, humble, setup_sim_en.md
# grasp_execution is tracked, so src is nonempty even on a fresh clone.
# Clone each missing vendor repository independently.
cd "$ROS2_WS/src"
for repo in \
    hsrb_controllers hsrb_common hsrb_drivers hsrb_launch hsrb_manipulation \
    hsrb_rosnav hsrb_simulator hsr_common hsrb_teleop tmc_gazebo tmc_teleop \
    tmc_common tmc_common_msgs tmc_drivers tmc_database tmc_manipulation \
    tmc_manipulation_base tmc_manipulation_planner tmc_point_cloud \
    tmc_realtime_control tmc_voice tmc_navigation hsrb_moveit; do
    if [ ! -d "$repo/.git" ]; then
        git clone -b humble "https://github.com/hsr-project/${repo}.git"
        case "$repo" in
            hsrb_launch) rm -rf hsrb_launch/hsrb_robot_launch ;;
            hsrb_simulator) rm -rf hsrb_simulator/hsrb_rviz_simulator ;;
            tmc_drivers) rm -rf tmc_drivers/tmc_pgr_camera ;;
        esac
    fi
done
"$SCRIPT_DIR/bug_fixes.sh" "$ROS2_WS"
python3 "$SCRIPT_DIR/apply_grasp_overlay.py" "$ROS2_WS"

# the Dockerfile clears /var/lib/apt/lists, so apt has no index until refreshed
sudo apt-get update
rosdep update
rosdep install --from-paths "$ROS2_WS/src" --ignore-src -r -y

# HSR-C smoke test: expanding the URDF fails loudly here rather than at launch.
# Used both as a pre-build health check (decides whether to wipe) and as the
# post-build confirmation - Docker/devcontainer.json churn continuously here,
# so whether the *existing* install actually works is the real signal, not a hash.
smoke_test() {
    source "$ROS2_WS/install/setup.bash" || return 1
    xacro "$(ros2 pkg prefix hsrc_description)/share/hsrc_description/robots/hsrc1s.urdf.xacro" > /dev/null 2>&1 || return 1
    test -f "$(ros2 pkg prefix hsrb_gazebo_launch)/share/hsrb_gazebo_launch/launch/hsrc_apartment_world.launch.py"
}

if [ -f "$ROS2_WS/install/setup.bash" ] && (smoke_test); then
    echo "ros2_ws already built and healthy - keeping build/install for an incremental colcon build"
else
    echo "ros2_ws missing or failed the health check - wiping build/install/log for a clean rebuild"
    rm -rf "$ROS2_WS/build" "$ROS2_WS/install" "$ROS2_WS/log"
fi

cd "$ROS2_WS"
source /opt/ros/humble/setup.bash

# rmw_zenohd router no longer started here: this container now runs
# rmw_cyclonedds_cpp to match the real robot, and the zenoh RPC path to
# SAM3/GraspGenX (core/utils/zenoh_rpc.py) doesn't need a router - confirmed
# it still works, router or not, over plain zenoh scouting.

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

if ! (smoke_test); then
    echo "postCreate.sh: HSR-C smoke test failed after colcon build" >&2
    exit 1
fi
echo "HSR-C sim ready: ros2 launch hsrb_gazebo_launch hsrc_apartment_world.launch.py"
