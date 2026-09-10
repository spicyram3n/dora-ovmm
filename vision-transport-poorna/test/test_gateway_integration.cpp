#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vision_transport/image_codec.hpp"
#include "vision_transport/payload_envelope.hpp"
#include "vision_transport/stream_config.hpp"
#include "vision_transport/zenoh_session.hpp"

namespace
{

sensor_msgs::msg::Image make_test_image()
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 21;
  image.header.stamp.nanosec = 22;
  image.header.frame_id = "integration_camera";
  image.width = 2;
  image.height = 2;
  image.encoding = "rgb8";
  image.is_bigendian = false;
  image.step = 6;
  image.data = {
    255, 0, 0, 0, 255, 0,
    0, 0, 255, 255, 255, 255,
  };
  return image;
}

vision_transport::StreamConfig make_stream()
{
  vision_transport::StreamConfig stream;
  stream.name = "integration_rgb";
  stream.type = vision_transport::StreamType::Image;
  stream.input_topic = "/vision_transport/test/input";
  stream.output_topic = "/vision_transport/test/output";
  stream.zenoh_key = "vision_transport/tests/integration/rgb";
  stream.queue_depth = 10;
  stream.codec.type = "png";
  stream.codec.png_level = 1;
  return stream;
}

}  // namespace

TEST(GatewayIntegration, ReconstructsImageAcrossRosZenohRosLoopback)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  const auto stream = make_stream();
  vision_transport::ZenohSession zenoh;

  auto source_node = std::make_shared<rclcpp::Node>("gateway_integration_source");
  auto tx_node = std::make_shared<rclcpp::Node>("gateway_integration_tx");
  auto rx_node = std::make_shared<rclcpp::Node>("gateway_integration_rx");
  auto sink_node = std::make_shared<rclcpp::Node>("gateway_integration_sink");

  auto source_pub = source_node->create_publisher<sensor_msgs::msg::Image>(
    stream.input_topic, rclcpp::QoS(10));
  auto output_pub = rx_node->create_publisher<sensor_msgs::msg::Image>(
    stream.output_topic, rclcpp::QoS(10));

  bool received = false;
  sensor_msgs::msg::Image received_image;
  auto sink_sub = sink_node->create_subscription<sensor_msgs::msg::Image>(
    stream.output_topic,
    rclcpp::QoS(10),
    [&](const sensor_msgs::msg::Image::ConstSharedPtr image) {
      received_image = *image;
      received = true;
    });

  auto tx_sub = tx_node->create_subscription<sensor_msgs::msg::Image>(
    stream.input_topic,
    rclcpp::QoS(10),
    [&](const sensor_msgs::msg::Image::ConstSharedPtr image) {
      const auto encoded = vision_transport::encode_image(*image, stream, 1);
      const auto envelope = vision_transport::pack_envelope({encoded.metadata, encoded.payload});
      zenoh.put_cbor(stream.zenoh_key, envelope);
    });

  zenoh.subscribe(stream.zenoh_key, [&](const std::vector<std::uint8_t> & bytes) {
    const auto envelope = vision_transport::unpack_envelope(bytes);
    const auto image = vision_transport::decode_image(envelope.metadata, envelope.payload);
    output_pub->publish(image);
  });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(source_node);
  executor.add_node(tx_node);
  executor.add_node(rx_node);
  executor.add_node(sink_node);

  const auto expected = make_test_image();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    source_pub->publish(expected);
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(received);
  if (received) {
    EXPECT_EQ(received_image.header.frame_id, expected.header.frame_id);
    EXPECT_EQ(received_image.encoding, expected.encoding);
    EXPECT_EQ(received_image.width, expected.width);
    EXPECT_EQ(received_image.height, expected.height);
    EXPECT_EQ(received_image.step, expected.step);
    EXPECT_EQ(received_image.data, expected.data);
  }

  executor.remove_node(source_node);
  executor.remove_node(tx_node);
  executor.remove_node(rx_node);
  executor.remove_node(sink_node);
  rclcpp::shutdown();
}
