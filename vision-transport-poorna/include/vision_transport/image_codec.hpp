#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "vision_transport/stream_config.hpp"
#include "vision_transport/stream_metadata.hpp"

namespace vision_transport
{

struct EncodedImage
{
  StreamMetadata metadata;
  std::vector<std::uint8_t> payload;
};

EncodedImage encode_image(
  const sensor_msgs::msg::Image & image,
  const StreamConfig & stream,
  std::uint64_t sequence);

sensor_msgs::msg::Image decode_image(
  const StreamMetadata & metadata,
  const std::vector<std::uint8_t> & payload);

EncodedImage encode_depth_image(
  const sensor_msgs::msg::Image & image,
  const StreamConfig & stream,
  std::uint64_t sequence);

sensor_msgs::msg::Image decode_depth_image(
  const StreamMetadata & metadata,
  const std::vector<std::uint8_t> & payload);

EncodedImage encode_point_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const StreamConfig & stream,
  std::uint64_t sequence);

sensor_msgs::msg::PointCloud2 decode_point_cloud(
  const StreamMetadata & metadata,
  const std::vector<std::uint8_t> & payload);

}  // namespace vision_transport
