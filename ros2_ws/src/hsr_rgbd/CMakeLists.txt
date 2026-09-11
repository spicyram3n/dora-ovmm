cmake_minimum_required(VERSION 3.10.2)
project(hsr_rgbd)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/depth_camera_relay.py DESTINATION lib/${PROJECT_NAME})

ament_package()
