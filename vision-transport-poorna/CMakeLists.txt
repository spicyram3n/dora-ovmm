cmake_minimum_required(VERSION 3.16)
project(vision_transport)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(zenohc REQUIRED)

find_path(COMPRESSED_DEPTH_IMAGE_TRANSPORT_INCLUDE_DIR codec.h
  PATHS /opt/ros/humble/include/compressed_depth_image_transport
  REQUIRED
)
find_library(COMPRESSED_DEPTH_IMAGE_TRANSPORT_LIBRARY compressed_depth_image_transport
  PATHS /opt/ros/humble/lib
  REQUIRED
)

add_library(${PROJECT_NAME}_core
  src/image_codec.cpp
  src/payload_envelope.cpp
  src/rate_limiter.cpp
  src/stream_config.cpp
  src/stream_metadata.cpp
  src/zenoh_session.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${COMPRESSED_DEPTH_IMAGE_TRANSPORT_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBS}
  ${COMPRESSED_DEPTH_IMAGE_TRANSPORT_LIBRARY}
  yaml-cpp
  nlohmann_json::nlohmann_json
  zenohc::lib
)
ament_target_dependencies(${PROJECT_NAME}_core
  compressed_depth_image_transport
  cv_bridge
  point_cloud_transport
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(vision_transport_tx src/tx_node.cpp)
target_link_libraries(vision_transport_tx ${PROJECT_NAME}_core)
ament_target_dependencies(vision_transport_tx rclcpp sensor_msgs std_msgs cv_bridge compressed_depth_image_transport point_cloud_transport pluginlib)

add_executable(vision_transport_rx src/rx_node.cpp)
target_link_libraries(vision_transport_rx ${PROJECT_NAME}_core)
ament_target_dependencies(vision_transport_rx rclcpp sensor_msgs std_msgs cv_bridge compressed_depth_image_transport point_cloud_transport pluginlib)

add_executable(vision_transport_codec_benchmark src/codec_benchmark.cpp)
target_link_libraries(vision_transport_codec_benchmark ${PROJECT_NAME}_core)
ament_target_dependencies(vision_transport_codec_benchmark rclcpp sensor_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_core vision_transport_tx vision_transport_rx vision_transport_codec_benchmark
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter ${PROJECT_NAME}_core)

  ament_add_gtest(test_stream_config test/test_stream_config.cpp)
  target_link_libraries(test_stream_config ${PROJECT_NAME}_core)

  ament_add_gtest(test_stream_metadata test/test_stream_metadata.cpp)
  target_link_libraries(test_stream_metadata ${PROJECT_NAME}_core)

  ament_add_gtest(test_payload_envelope test/test_payload_envelope.cpp)
  target_link_libraries(test_payload_envelope ${PROJECT_NAME}_core)

  ament_add_gtest(test_image_codec test/test_image_codec.cpp)
  target_link_libraries(test_image_codec ${PROJECT_NAME}_core)

  ament_add_gtest(test_zenoh_session test/test_zenoh_session.cpp)
  target_link_libraries(test_zenoh_session ${PROJECT_NAME}_core)

  ament_add_gtest(test_gateway_integration test/test_gateway_integration.cpp)
  target_link_libraries(test_gateway_integration ${PROJECT_NAME}_core)
  ament_target_dependencies(test_gateway_integration rclcpp sensor_msgs)

  add_test(
    NAME test_executable_gateway
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/executable_gateway_test.py
      $<TARGET_FILE:vision_transport_tx>
      $<TARGET_FILE:vision_transport_rx>
  )
  set_tests_properties(test_executable_gateway PROPERTIES TIMEOUT 20)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(compressed_depth_image_transport cv_bridge point_cloud_transport pluginlib rclcpp sensor_msgs std_msgs)
ament_package()
