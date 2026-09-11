cmake_minimum_required(VERSION 3.10.2)
project(nbv)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

# C++ nodes.
add_executable(go_to_pose src/go_to_pose.cpp)
ament_target_dependencies(go_to_pose geometry_msgs nav2_msgs rclcpp rclcpp_action)
install(TARGETS go_to_pose RUNTIME DESTINATION lib/${PROJECT_NAME})

# Python. The first line installs nbv/ as an importable module, the second
# installs the runnable scripts so `ros2 run` can find them.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/where_am_i.py scripts/build_octomap.py scripts/save_occupied.py
        scripts/plot_octomap.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
