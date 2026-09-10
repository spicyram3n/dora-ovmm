#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vision_transport
{

enum class StreamType
{
  Image,
  Depth,
  PointCloud
};

struct CodecConfig
{
  std::string type;
  int quality{80};
  int png_level{3};
  int zstd_level{3};
  double cloudini_resolution{0.001};
};

struct StreamConfig
{
  std::string name;
  StreamType type{StreamType::Image};
  std::string input_topic;
  std::string output_topic;
  std::string zenoh_key;
  double max_rate_hz{0.0};
  std::uint32_t queue_depth{1};
  std::optional<double> resize_scale;
  std::optional<double> voxel_size;
  CodecConfig codec;
};

std::vector<StreamConfig> load_stream_config(const std::string & path);
std::vector<StreamConfig> load_tx_stream_config(const std::string & path);
std::vector<StreamConfig> load_rx_stream_config(const std::string & path);
std::string to_string(StreamType type);
StreamType stream_type_from_string(const std::string & value);

}  // namespace vision_transport
