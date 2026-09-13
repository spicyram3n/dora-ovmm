cmake_minimum_required(VERSION 3.10.2)
project(nbv)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UnknownFraction.srv" 
  "srv/NextView.srv" 
  DEPENDENCIES geometry_msgs)


install(PROGRAMS 
  scripts/build_octomap.py 
  scripts/view_gain.py scripts/next_view.py    
  visualization/plot_octomap.py 
  visualization/save_occupied.py
DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
