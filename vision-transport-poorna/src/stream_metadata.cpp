#include "vision_transport/stream_metadata.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace vision_transport
{

namespace
{

nlohmann::json point_field_to_json(const sensor_msgs::msg::PointField & field)
{
  return nlohmann::json{
    {"name", field.name},
    {"offset", field.offset},
    {"datatype", field.datatype},
    {"count", field.count},
  };
}

sensor_msgs::msg::PointField point_field_from_json(const nlohmann::json & json)
{
  sensor_msgs::msg::PointField field;
  field.name = json.at("name").get<std::string>();
  field.offset = json.at("offset").get<std::uint32_t>();
  field.datatype = json.at("datatype").get<std::uint8_t>();
  field.count = json.at("count").get<std::uint32_t>();
  return field;
}

}  // namespace

std::vector<std::uint8_t> encode_metadata_cbor(const StreamMetadata & metadata)
{
  nlohmann::json fields = nlohmann::json::array();
  for (const auto & field : metadata.pointcloud.fields) {
    fields.push_back(point_field_to_json(field));
  }

  const nlohmann::json json{
    {"schema_version", metadata.schema_version},
    {"stream_id", metadata.stream_id},
    {"stream_type", metadata.stream_type},
    {"codec", metadata.codec},
    {"sequence", metadata.sequence},
    {"stamp", {{"sec", metadata.stamp_sec}, {"nanosec", metadata.stamp_nanosec}}},
    {"frame_id", metadata.frame_id},
    {"image", {
      {"encoding", metadata.image.encoding},
      {"width", metadata.image.width},
      {"height", metadata.image.height},
      {"step", metadata.image.step},
    }},
    {"pointcloud", {
      {"width", metadata.pointcloud.width},
      {"height", metadata.pointcloud.height},
      {"is_bigendian", metadata.pointcloud.is_bigendian},
      {"point_step", metadata.pointcloud.point_step},
      {"row_step", metadata.pointcloud.row_step},
      {"is_dense", metadata.pointcloud.is_dense},
      {"fields", fields},
    }},
  };
  return nlohmann::json::to_cbor(json);
}

StreamMetadata decode_metadata_cbor(const std::vector<std::uint8_t> & bytes)
{
  const auto json = nlohmann::json::from_cbor(bytes);

  StreamMetadata metadata;
  metadata.schema_version = json.at("schema_version").get<std::uint32_t>();
  metadata.stream_id = json.at("stream_id").get<std::string>();
  metadata.stream_type = json.at("stream_type").get<std::string>();
  metadata.codec = json.at("codec").get<std::string>();
  metadata.sequence = json.at("sequence").get<std::uint64_t>();
  metadata.stamp_sec = json.at("stamp").at("sec").get<std::int32_t>();
  metadata.stamp_nanosec = json.at("stamp").at("nanosec").get<std::uint32_t>();
  metadata.frame_id = json.at("frame_id").get<std::string>();

  const auto image = json.at("image");
  metadata.image.encoding = image.at("encoding").get<std::string>();
  metadata.image.width = image.at("width").get<std::uint32_t>();
  metadata.image.height = image.at("height").get<std::uint32_t>();
  metadata.image.step = image.at("step").get<std::uint32_t>();

  const auto pointcloud = json.at("pointcloud");
  metadata.pointcloud.width = pointcloud.at("width").get<std::uint32_t>();
  metadata.pointcloud.height = pointcloud.at("height").get<std::uint32_t>();
  metadata.pointcloud.is_bigendian = pointcloud.at("is_bigendian").get<bool>();
  metadata.pointcloud.point_step = pointcloud.at("point_step").get<std::uint32_t>();
  metadata.pointcloud.row_step = pointcloud.at("row_step").get<std::uint32_t>();
  metadata.pointcloud.is_dense = pointcloud.at("is_dense").get<bool>();
  for (const auto & field : pointcloud.at("fields")) {
    metadata.pointcloud.fields.push_back(point_field_from_json(field));
  }

  return metadata;
}

}  // namespace vision_transport
