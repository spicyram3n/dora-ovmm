cmake_minimum_required(VERSION 3.10.2)
project(grasp_execution)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_to_grasp src/move_to_grasp.cpp)
ament_target_dependencies(move_to_grasp moveit_ros_planning_interface rclcpp)

install(TARGETS move_to_grasp RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
