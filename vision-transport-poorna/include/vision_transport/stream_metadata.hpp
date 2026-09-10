#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace vision_transport
{

struct ImageMetadata
{
  std::string encoding;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
};

struct PointCloudMetadata
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  bool is_bigendian{false};
  std::uint32_t point_step{0};
  std::uint32_t row_step{0};
  bool is_dense{false};
  std::vector<sensor_msgs::msg::PointField> fields;
};

struct StreamMetadata
{
  std::uint32_t schema_version{1};
  std::string stream_id;
  std::string stream_type;
  std::string codec;
  std::uint64_t sequence{0};
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string frame_id;
  ImageMetadata image;
  PointCloudMetadata pointcloud;
};

std::vector<std::uint8_t> encode_metadata_cbor(const StreamMetadata & metadata);
StreamMetadata decode_metadata_cbor(const std::vector<std::uint8_t> & bytes);

}  // namespace vision_transport
