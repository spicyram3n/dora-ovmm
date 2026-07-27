#!/bin/bash
set -e

# resolved before the cd below
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

sudo chown -R $(whoami) /home/ws

grep -qxF 'export PYTHONPATH=$PYTHONPATH:/home/ws' ~/.bashrc || \
    echo 'export PYTHONPATH=$PYTHONPATH:/home/ws' >> ~/.bashrc

grep -qxF 'export ZENOH_SESSION_CONFIG_URI=/home/ws/.devcontainer/zenoh/session_shm.json5' ~/.bashrc || \
    echo 'export ZENOH_SESSION_CONFIG_URI=/home/ws/.devcontainer/zenoh/session_shm.json5' >> ~/.bashrc

# every ros2 node in this container uses zenoh instead of DDS by default now
grep -qxF 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' ~/.bashrc || \
    echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc

# postCreateCommand only runs once, at container creation - a plain container
# restart (as opposed to a rebuild) kills the router below without anything
# bringing it back, so also self-heal it from every new interactive shell
ZENOH_ROUTER_SELFHEAL='pgrep -f "rmw_zenoh_cpp/rmw_zenohd" > /dev/null || { nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/rmw_zenohd.log 2>&1 & disown; }'
grep -qxF "$ZENOH_ROUTER_SELFHEAL" ~/.bashrc || echo "$ZENOH_ROUTER_SELFHEAL" >> ~/.bashrc

ROS2_WS=/home/ws/ros2_ws
mkdir -p "$ROS2_WS/src"

# hsr_ros2 package list per hsr-project/hsr_ros2_doc, humble, setup_sim_en.md
if [ -z "$(ls -A "$ROS2_WS/src")" ]; then
    echo "ros2_ws/src is empty - cloning hsr_ros2 packages"
    cd "$ROS2_WS/src"
    for repo in \
        hsrb_controllers hsrb_common hsrb_drivers hsrb_launch hsrb_manipulation \
        hsrb_rosnav hsrb_simulator hsr_common hsrb_teleop tmc_gazebo tmc_teleop \
        tmc_common tmc_common_msgs tmc_drivers tmc_database tmc_manipulation \
        tmc_manipulation_base tmc_manipulation_planner tmc_point_cloud \
        tmc_realtime_control tmc_voice tmc_navigation; do
        git clone -b humble "https://github.com/hsr-project/${repo}.git"
    done
    rm -rf hsrb_launch/hsrb_robot_launch hsrb_simulator/hsrb_rviz_simulator tmc_drivers/tmc_pgr_camera

    # not in the doc's list; move_group comes from hsrb_moveit_config
    git clone -b humble "https://github.com/hsr-project/hsrb_moveit.git"

    "$SCRIPT_DIR/bug_fixes.sh" "$ROS2_WS"
fi

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

# zenoh router so other containers (SAM3, GraspNet, etc.) discover this one
# instantly instead of falling back to scouting-based peer discovery
if ! pgrep -f "rmw_zenoh_cpp/rmw_zenohd" > /dev/null; then
    echo "starting rmw_zenohd router (log: /tmp/rmw_zenohd.log)"
    nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/rmw_zenohd.log 2>&1 &
    disown
else
    echo "rmw_zenohd router already running"
fi

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

if ! (smoke_test); then
    echo "postCreate.sh: HSR-C smoke test failed after colcon build" >&2
    exit 1
fi
echo "HSR-C sim ready: ros2 launch hsrb_gazebo_launch hsrc_apartment_world.launch.py"
