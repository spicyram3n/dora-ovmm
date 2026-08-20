#!/bin/bash
set -e
sudo chown -R $(whoami) /home/ws

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Appends a line to ~/.bashrc unless it's already there.
bashrc_line() {
    grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc
}

bashrc_line 'export PYTHONPATH=$PYTHONPATH:/home/ws'
bashrc_line 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'  # matches the real robot
bashrc_line "export CYCLONEDDS_URI=file://$SCRIPT_DIR/cyclonedds_profile.xml"

WS=/home/ws/ros2_ws
mkdir -p $WS/src

# hsr_ros2 package list per hsr-project/hsr_ros2_doc, humble, setup_sim_en.md
# (plus hsrb_moveit, which is not in that list but is where move_group lives)
if [ -z "$(ls -A $WS/src)" ]; then
    cd $WS/src
    for repo in \
        hsrb_controllers hsrb_common hsrb_drivers hsrb_launch hsrb_manipulation \
        hsrb_rosnav hsrb_simulator hsr_common hsrb_teleop tmc_gazebo tmc_teleop \
        tmc_common tmc_common_msgs tmc_drivers tmc_database tmc_manipulation \
        tmc_manipulation_base tmc_manipulation_planner tmc_point_cloud \
        tmc_realtime_control tmc_voice tmc_navigation hsrb_moveit; do
        git clone -q --depth 1 -b humble https://github.com/hsr-project/$repo.git
    done
    rm -rf hsrb_launch/hsrb_robot_launch hsrb_simulator/hsrb_rviz_simulator tmc_drivers/tmc_pgr_camera

    # hsrb_moveit_config's demo executables use geometric_shapes and Boost::random
    # without declaring either, so they fail to link
    sed -i -e '/^find_package(Eigen3 REQUIRED)$/a\
        find_package(Boost REQUIRED COMPONENTS random)\
        find_package(geometric_shapes REQUIRED)\
        add_link_options("-Wl,--no-as-needed")' \
        -e '/^ament_target_dependencies/ s/rclcpp)$/rclcpp geometric_shapes)/' \
        hsrb_moveit/hsrb_moveit_config/CMakeLists.txt
fi

# apt-get update is required: the Dockerfile clears /var/lib/apt/lists, so apt
# has no package index until refreshed
sudo apt-get update
rosdep update
rosdep install --from-paths $WS/src --ignore-src -r -y

# Reuse build/ and install/ only if the last build actually finished.
SIM_LAUNCH=$WS/install/hsrb_gazebo_launch/share/hsrb_gazebo_launch/launch/hsrc_apartment_world.launch.py
if [ -f $WS/install/setup.bash ] && [ -f $SIM_LAUNCH ]; then
    echo "ros2_ws looks built - incremental colcon build"
else
    echo "ros2_ws missing or half-built - wiping build/install/log"
    rm -rf $WS/build $WS/install $WS/log
fi

cd $WS
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
