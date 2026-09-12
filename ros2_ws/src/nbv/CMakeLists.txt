cmake_minimum_required(VERSION 3.10.2)
project(nbv)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# The map node's query service. Its generated Python module is named `nbv`,
# which is why this package has no Python module of its own under that name.
rosidl_generate_interfaces(${PROJECT_NAME} "srv/UnknownFraction.srv" DEPENDENCIES geometry_msgs)

# The runnable scripts, so `ros2 run` can find them.
install(PROGRAMS scripts/build_octomap.py
        visualization/plot_octomap.py visualization/save_occupied.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
