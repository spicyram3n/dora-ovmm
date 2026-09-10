#include "vision_transport/stream_config.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace vision_transport
{

namespace
{

enum class ConfigRole
{
  Tx,
  Rx,
  Full
};

template<typename T>
T required(const YAML::Node & node, const std::string & key, const std::string & stream_name)
{
  if (!node[key]) {
    throw std::runtime_error("stream '" + stream_name + "' is missing required key '" + key + "'");
  }
  return node[key].as<T>();
}

void validate_common(const StreamConfig & stream, ConfigRole role)
{
  if (stream.name.empty()) {
    throw std::runtime_error("stream name must not be empty");
  }
  if ((role == ConfigRole::Tx || role == ConfigRole::Full) && stream.input_topic.empty()) {
    throw std::runtime_error("stream '" + stream.name + "' input_topic must not be empty");
  }
  if ((role == ConfigRole::Rx || role == ConfigRole::Full) && stream.output_topic.empty()) {
    throw std::runtime_error("stream '" + stream.name + "' output_topic must not be empty");
  }
  if (stream.zenoh_key.empty()) {
    throw std::runtime_error("stream '" + stream.name + "' zenoh_key must not be empty");
  }
  if (stream.queue_depth == 0) {
    throw std::runtime_error("stream '" + stream.name + "' queue_depth must be greater than zero");
  }
  if (stream.resize_scale && *stream.resize_scale <= 0.0) {
    throw std::runtime_error("stream '" + stream.name + "' resize_scale must be greater than zero");
  }
  if (stream.voxel_size && *stream.voxel_size <= 0.0) {
    throw std::runtime_error("stream '" + stream.name + "' voxel_size must be greater than zero");
  }
  if ((role == ConfigRole::Tx || role == ConfigRole::Full) && stream.codec.type.empty()) {
    throw std::runtime_error("stream '" + stream.name + "' codec.type must not be empty");
  }
}

CodecConfig parse_codec(const YAML::Node & stream_node, const std::string & stream_name)
{
  const auto codec_node = stream_node["codec"];
  if (!codec_node || !codec_node.IsMap()) {
    throw std::runtime_error("stream '" + stream_name + "' requires codec map");
  }

  CodecConfig codec;
  codec.type = required<std::string>(codec_node, "type", stream_name);
  if (codec_node["quality"]) {
    codec.quality = codec_node["quality"].as<int>();
  }
  if (codec_node["png_level"]) {
    codec.png_level = codec_node["png_level"].as<int>();
  }
  if (codec_node["zstd_level"]) {
    codec.zstd_level = codec_node["zstd_level"].as<int>();
  }
  if (codec_node["cloudini_resolution"]) {
    codec.cloudini_resolution = codec_node["cloudini_resolution"].as<double>();
  }
  return codec;
}

std::vector<StreamConfig> load_stream_config_for_role(const std::string & path, ConfigRole role)
{
  const auto root = YAML::LoadFile(path);
  const auto streams_node = root["streams"];
  if (!streams_node || !streams_node.IsMap()) {
    throw std::runtime_error("config requires top-level 'streams' map");
  }

  std::vector<StreamConfig> streams;
  for (const auto & item : streams_node) {
    StreamConfig stream;
    stream.name = item.first.as<std::string>();
    const auto stream_node = item.second;
    if (!stream_node || !stream_node.IsMap()) {
      throw std::runtime_error("stream '" + stream.name + "' must be a map");
    }

    stream.type = stream_type_from_string(required<std::string>(stream_node, "type", stream.name));
    if (stream_node["input_topic"]) {
      stream.input_topic = stream_node["input_topic"].as<std::string>();
    }
    if (stream_node["output_topic"]) {
      stream.output_topic = stream_node["output_topic"].as<std::string>();
    }
    stream.zenoh_key = required<std::string>(stream_node, "zenoh_key", stream.name);
    if (stream_node["max_rate_hz"]) {
      stream.max_rate_hz = stream_node["max_rate_hz"].as<double>();
    }
    if (stream_node["queue_depth"]) {
      stream.queue_depth = stream_node["queue_depth"].as<std::uint32_t>();
    }
    if (stream_node["resize_scale"]) {
      stream.resize_scale = stream_node["resize_scale"].as<double>();
    }
    if (stream_node["voxel_size"]) {
      stream.voxel_size = stream_node["voxel_size"].as<double>();
    }
    if (stream_node["codec"]) {
      stream.codec = parse_codec(stream_node, stream.name);
    }
    validate_common(stream, role);
    streams.push_back(stream);
  }

  if (streams.empty()) {
    throw std::runtime_error("config contains no streams");
  }
  return streams;
}

}  // namespace

std::string to_string(StreamType type)
{
  switch (type) {
    case StreamType::Image:
      return "image";
    case StreamType::Depth:
      return "depth";
    case StreamType::PointCloud:
      return "pointcloud";
  }
  throw std::runtime_error("unknown stream type");
}

StreamType stream_type_from_string(const std::string & value)
{
  if (value == "image") {
    return StreamType::Image;
  }
  if (value == "depth") {
    return StreamType::Depth;
  }
  if (value == "pointcloud") {
    return StreamType::PointCloud;
  }
  throw std::runtime_error("unsupported stream type '" + value + "'");
}

std::vector<StreamConfig> load_stream_config(const std::string & path)
{
  return load_stream_config_for_role(path, ConfigRole::Full);
}

std::vector<StreamConfig> load_tx_stream_config(const std::string & path)
{
  return load_stream_config_for_role(path, ConfigRole::Tx);
}

std::vector<StreamConfig> load_rx_stream_config(const std::string & path)
{
  return load_stream_config_for_role(path, ConfigRole::Rx);
}

}  // namespace vision_transport
