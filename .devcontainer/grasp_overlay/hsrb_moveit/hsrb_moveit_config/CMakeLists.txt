cmake_minimum_required(VERSION 3.10.2)
project(hsrb_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GLUT REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)

add_library(hsr_configured_depth_updater SHARED src/configured_depth_updater.cpp)
ament_target_dependencies(hsr_configured_depth_updater moveit_ros_perception pluginlib rclcpp moveit_msgs)
find_library(DEPTH_UPDATER_CORE moveit_depth_image_octomap_updater_core
             HINTS "${moveit_ros_perception_DIR}/../../../lib" REQUIRED)
target_link_libraries(hsr_configured_depth_updater ${DEPTH_UPDATER_CORE})
pluginlib_export_plugin_description_file(moveit_ros_occupancy_map_monitor configured_depth_updater.xml)
install(TARGETS hsr_configured_depth_updater LIBRARY DESTINATION lib)

find_package(Eigen3 REQUIRED)
        find_package(Boost REQUIRED COMPONENTS random)
        find_package(geometric_shapes REQUIRED)
        add_link_options("-Wl,--no-as-needed")

add_executable(moveit_constraints_demo example/moveit_constraints_demo.cpp)
target_include_directories(moveit_constraints_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_constraints_demo moveit_ros_planning_interface rclcpp geometric_shapes)

add_executable(moveit_fk_demo example/moveit_fk_demo.cpp)
target_include_directories(moveit_fk_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_fk_demo moveit_ros_planning_interface rclcpp geometric_shapes)

add_executable(moveit_gripper_demo example/moveit_gripper_demo.cpp)
target_include_directories(moveit_gripper_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_gripper_demo moveit_ros_planning_interface rclcpp geometric_shapes)

add_executable(moveit_ik_demo example/moveit_ik_demo.cpp)
target_include_directories(moveit_ik_demo PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(moveit_ik_demo moveit_ros_planning_interface rclcpp geometric_shapes)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_joint_states_publisher.py scripts/depth_camera_relay.py DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS moveit_constraints_demo moveit_fk_demo moveit_gripper_demo moveit_ik_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_package()
