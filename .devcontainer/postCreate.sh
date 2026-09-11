#!/bin/bash
set -euo pipefail

WS=/home/ws/ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


if [ ! -w /home/ws ]; then
    sudo chown "$(whoami):$(id -gn)" /home/ws
fi

mkdir -p "$WS/src"

# Bashrc setup
bashrc_line() {
    grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc
}

bashrc_line 'export PYTHONPATH=$PYTHONPATH:/home/ws'
bashrc_line 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
bashrc_line "export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds_sim.xml"
bashrc_line '[ -f /home/ws/.devcontainer/runtime.env ] && { set -a; source /home/ws/.devcontainer/runtime.env; set +a; }'

# Clone HSR repositories
cd "$WS/src"

sources_added=false

repos=(
    hsrb_controllers
    hsrb_common
    hsrb_drivers
    hsrb_launch
    hsrb_manipulation
    hsrb_rosnav
    hsrb_simulator
    hsr_common
    hsrb_teleop
    tmc_gazebo
    tmc_teleop
    tmc_common
    tmc_common_msgs
    tmc_drivers
    tmc_database
    tmc_manipulation
    tmc_manipulation_base
    tmc_manipulation_planner
    tmc_point_cloud
    tmc_realtime_control
    tmc_voice
    tmc_navigation
    hsrb_moveit
)

for repo in "${repos[@]}"; do
    if [ ! -d "$repo/.git" ]; then
        echo "Cloning $repo..."

        clone_dir=$(mktemp -d "$WS/src/.${repo}.XXXXXX")

        if git clone \
            --quiet \
            --depth 1 \
            --branch humble \
            "https://github.com/hsr-project/$repo.git" \
            "$clone_dir"; then

            rm -rf "$repo"
            mv "$clone_dir" "$repo"
            sources_added=true
        else
            rm -rf "$clone_dir"
            echo "Failed to clone $repo"
            exit 1
        fi
    fi
done

# Remove unsupported/unwanted packages
rm -rf \
    hsrb_launch/hsrb_robot_launch \
    hsrb_simulator/hsrb_rviz_simulator \
    tmc_drivers/tmc_pgr_camera


# Patch hsrb_moveit
CMAKE_FILE="hsrb_moveit/hsrb_moveit_config/CMakeLists.txt"

if ! grep -q 'find_package(geometric_shapes REQUIRED)' "$CMAKE_FILE"; then
    echo "Patching hsrb_moveit_config..."

    sed -i \
        -e '/^find_package(Eigen3 REQUIRED)$/a\
find_package(Boost REQUIRED COMPONENTS random)\
find_package(geometric_shapes REQUIRED)\
add_link_options("-Wl,--no-as-needed")' \
        -e '/^ament_target_dependencies/ s/rclcpp)$/rclcpp geometric_shapes)/' \
        "$CMAKE_FILE"
fi

# Dependencies
# ROS setup scripts reference optional unset variables.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "Updating apt package lists..."
sudo apt-get update

echo "Updating rosdep..."
rosdep update

echo "Installing ROS dependencies..."
rosdep install \
    --from-paths "$WS/src" \
    --ignore-src \
    --rosdistro humble \
    -y


# Dependencies belong to the container; install them even when the mounted
# workspace already contains a completed build.
# Reuse an already completed build
SIM_LAUNCH="$WS/install/hsrb_gazebo_launch/share/hsrb_gazebo_launch/launch/hsrc_apartment_world.launch.py"
BUILD_MARKER="$WS/install/.postcreate-build-complete"

if [ "$sources_added" = false ] \
    && [ "${ROS2_REBUILD:-0}" != "1" ] \
    && [ -f "$WS/install/setup.bash" ] \
    && [ -f "$SIM_LAUNCH" ] \
    && [ -f "$BUILD_MARKER" ]; then

    echo "ros2_ws already built."
    echo "Set ROS2_REBUILD=1 to force an incremental rebuild."
    exit 0
fi

rm -f "$BUILD_MARKER"

# Build
cd "$WS"

echo "Building ros2_ws..."

colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17

touch "$BUILD_MARKER"

echo
echo "HSR ROS 2 workspace build complete."