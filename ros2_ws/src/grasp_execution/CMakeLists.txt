cmake_minimum_required(VERSION 3.10.2)
project(grasp_execution)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

add_executable(move_to_grasp src/move_to_grasp.cpp)
ament_target_dependencies(move_to_grasp moveit_ros_planning_interface rclcpp rclcpp_action tmc_control_msgs octomap_msgs)

install(TARGETS move_to_grasp RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
