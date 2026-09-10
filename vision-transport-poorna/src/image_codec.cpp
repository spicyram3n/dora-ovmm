#include "vision_transport/image_codec.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <codec.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <point_cloud_transport/point_cloud_codec.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace vision_transport
{

namespace
{

bool is_mono8(const std::string & encoding)
{
  return encoding == "mono8" || encoding == "8UC1";
}

bool is_color8(const std::string & encoding)
{
  return encoding == "bgr8" || encoding == "rgb8";
}

bool is_depth_encoding(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "32FC1";
}

bool is_packed_color_field(const sensor_msgs::msg::PointField & field)
{
  return field.count == 1 &&
    (field.name == "rgb" || field.name == "rgba") &&
    field.datatype == sensor_msgs::msg::PointField::FLOAT32;
}

sensor_msgs::msg::PointCloud2 cloud_for_transport(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const StreamConfig & stream)
{
  if (stream.codec.type != "cloudini") {
    return cloud;
  }

  auto adjusted = cloud;
  for (auto & field : adjusted.fields) {
    if (is_packed_color_field(field)) {
      field.datatype = sensor_msgs::msg::PointField::UINT32;
    }
  }
  return adjusted;
}

std::string extension_for_codec(const std::string & codec)
{
  if (codec == "jpeg" || codec == "jpg") {
    return ".jpg";
  }
  if (codec == "png") {
    return ".png";
  }
  throw std::runtime_error("unsupported image codec '" + codec + "'");
}

std::vector<int> encode_params(const CodecConfig & codec)
{
  if (codec.type == "jpeg" || codec.type == "jpg") {
    return {cv::IMWRITE_JPEG_QUALITY, std::clamp(codec.quality, 1, 100)};
  }
  if (codec.type == "png") {
    return {cv::IMWRITE_PNG_COMPRESSION, std::clamp(codec.png_level, 0, 9)};
  }
  throw std::runtime_error("unsupported image codec '" + codec.type + "'");
}

cv::Mat mat_for_imencode(const sensor_msgs::msg::Image & image)
{
  if (is_mono8(image.encoding)) {
    return cv_bridge::toCvShare(image, std::shared_ptr<void const>(), image.encoding)->image;
  }
  if (!is_color8(image.encoding)) {
    throw std::runtime_error("unsupported image encoding '" + image.encoding + "'");
  }

  const auto source = cv_bridge::toCvShare(image, std::shared_ptr<void const>(), image.encoding)->image;
  if (image.encoding == "bgr8") {
    return source;
  }

  cv::Mat converted;
  cv::cvtColor(source, converted, cv::COLOR_RGB2BGR);
  return converted;
}

cv::Mat maybe_resize(const cv::Mat & source, const StreamConfig & stream)
{
  if (!stream.resize_scale || *stream.resize_scale == 1.0) {
    return source;
  }

  cv::Mat resized;
  cv::resize(source, resized, cv::Size(), *stream.resize_scale, *stream.resize_scale, cv::INTER_AREA);
  return resized;
}

sensor_msgs::msg::Image image_from_mat(
  const sensor_msgs::msg::Image & source,
  const cv::Mat & matrix)
{
  cv::Mat continuous = matrix;
  if (!continuous.isContinuous()) {
    continuous = continuous.clone();
  }

  sensor_msgs::msg::Image output;
  output.header = source.header;
  output.height = static_cast<std::uint32_t>(continuous.rows);
  output.width = static_cast<std::uint32_t>(continuous.cols);
  output.encoding = source.encoding;
  output.is_bigendian = source.is_bigendian;
  output.step = static_cast<std::uint32_t>(continuous.cols * continuous.elemSize());
  output.data.assign(continuous.datastart, continuous.dataend);
  return output;
}

sensor_msgs::msg::Image maybe_resize_image_raw(
  const sensor_msgs::msg::Image & image,
  const StreamConfig & stream)
{
  if (!stream.resize_scale || *stream.resize_scale == 1.0) {
    return image;
  }
  if (!is_mono8(image.encoding) && !is_color8(image.encoding)) {
    throw std::runtime_error("unsupported image encoding '" + image.encoding + "'");
  }

  const auto source = cv_bridge::toCvShare(image, std::shared_ptr<void const>(), image.encoding)->image;
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(), *stream.resize_scale, *stream.resize_scale, cv::INTER_AREA);
  return image_from_mat(image, resized);
}

sensor_msgs::msg::Image maybe_resize_depth_image(
  const sensor_msgs::msg::Image & image,
  const StreamConfig & stream)
{
  if (!stream.resize_scale || *stream.resize_scale == 1.0) {
    return image;
  }
  if (!is_depth_encoding(image.encoding)) {
    throw std::runtime_error("unsupported depth image encoding '" + image.encoding + "'");
  }

  const auto source = cv_bridge::toCvShare(image, std::shared_ptr<void const>(), image.encoding)->image;
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(), *stream.resize_scale, *stream.resize_scale, cv::INTER_NEAREST);
  return image_from_mat(image, resized);
}

int imread_mode_for_metadata(const StreamMetadata & metadata)
{
  if (is_mono8(metadata.image.encoding)) {
    return cv::IMREAD_GRAYSCALE;
  }
  return cv::IMREAD_COLOR;
}

cv::Mat mat_for_ros_encoding(const cv::Mat & decoded, const std::string & encoding)
{
  if (is_mono8(encoding)) {
    return decoded;
  }
  if (encoding == "bgr8") {
    return decoded;
  }
  if (encoding == "rgb8") {
    cv::Mat converted;
    cv::cvtColor(decoded, converted, cv::COLOR_BGR2RGB);
    return converted;
  }
  throw std::runtime_error("unsupported decoded image encoding '" + encoding + "'");
}

}  // namespace

EncodedImage encode_image(
  const sensor_msgs::msg::Image & image,
  const StreamConfig & stream,
  std::uint64_t sequence)
{
  if (!is_mono8(image.encoding) && !is_color8(image.encoding)) {
    throw std::runtime_error("unsupported image encoding '" + image.encoding + "'");
  }

  sensor_msgs::msg::Image raw_image;
  if (stream.codec.type == "raw" || stream.codec.type == "none") {
    raw_image = maybe_resize_image_raw(image, stream);
  } else {
    const auto matrix = maybe_resize(mat_for_imencode(image), stream);
    raw_image = image_from_mat(image, matrix);
  }

  std::vector<std::uint8_t> payload;
  if (stream.codec.type == "raw" || stream.codec.type == "none") {
    payload = raw_image.data;
  } else if (!cv::imencode(extension_for_codec(stream.codec.type), cv_bridge::toCvShare(
      raw_image, std::shared_ptr<void const>(), raw_image.encoding)->image, payload, encode_params(stream.codec)))
  {
    throw std::runtime_error("OpenCV image encoding failed for codec '" + stream.codec.type + "'");
  }

  EncodedImage encoded;
  encoded.payload = std::move(payload);
  encoded.metadata.schema_version = 1;
  encoded.metadata.stream_id = stream.name;
  encoded.metadata.stream_type = to_string(stream.type);
  encoded.metadata.codec = stream.codec.type == "none" ? "raw" : stream.codec.type;
  encoded.metadata.sequence = sequence;
  encoded.metadata.stamp_sec = raw_image.header.stamp.sec;
  encoded.metadata.stamp_nanosec = raw_image.header.stamp.nanosec;
  encoded.metadata.frame_id = raw_image.header.frame_id;
  encoded.metadata.image.encoding = raw_image.encoding;
  encoded.metadata.image.width = raw_image.width;
  encoded.metadata.image.height = raw_image.height;
  encoded.metadata.image.step = raw_image.step;
  return encoded;
}

sensor_msgs::msg::Image decode_image(
  const StreamMetadata & metadata,
  const std::vector<std::uint8_t> & payload)
{
  if (metadata.codec == "raw" || metadata.codec == "none") {
    sensor_msgs::msg::Image image;
    image.header.stamp.sec = metadata.stamp_sec;
    image.header.stamp.nanosec = metadata.stamp_nanosec;
    image.header.frame_id = metadata.frame_id;
    image.height = metadata.image.height;
    image.width = metadata.image.width;
    image.encoding = metadata.image.encoding;
    image.is_bigendian = false;
    image.step = metadata.image.step;
    image.data = payload;
    return image;
  }

  const auto encoded = cv::Mat(1, static_cast<int>(payload.size()), CV_8UC1, const_cast<std::uint8_t *>(payload.data()));
  const auto decoded = cv::imdecode(encoded, imread_mode_for_metadata(metadata));
  if (decoded.empty()) {
    throw std::runtime_error("OpenCV image decoding failed for codec '" + metadata.codec + "'");
  }

  const auto ros_matrix = mat_for_ros_encoding(decoded, metadata.image.encoding);
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = metadata.stamp_sec;
  image.header.stamp.nanosec = metadata.stamp_nanosec;
  image.header.frame_id = metadata.frame_id;
  image.height = static_cast<std::uint32_t>(ros_matrix.rows);
  image.width = static_cast<std::uint32_t>(ros_matrix.cols);
  image.encoding = metadata.image.encoding;
  image.is_bigendian = false;
  image.step = static_cast<std::uint32_t>(ros_matrix.cols * ros_matrix.elemSize());
  image.data.assign(ros_matrix.datastart, ros_matrix.dataend);
  return image;
}

EncodedImage encode_depth_image(
  const sensor_msgs::msg::Image & image,
  const StreamConfig & stream,
  std::uint64_t sequence)
{
  const auto resized_image = maybe_resize_depth_image(image, stream);
  std::vector<std::uint8_t> payload;
  if (stream.codec.type == "raw" || stream.codec.type == "none") {
    payload = resized_image.data;
  } else {
    auto compressed = compressed_depth_image_transport::encodeCompressedDepthImage(
      resized_image, 10.0, 100.0, stream.codec.png_level);
    if (!compressed) {
      throw std::runtime_error("compressed_depth_image_transport failed to encode depth image");
    }
    payload = std::move(compressed->data);
  }

  EncodedImage encoded;
  encoded.payload = std::move(payload);
  encoded.metadata.schema_version = 1;
  encoded.metadata.stream_id = stream.name;
  encoded.metadata.stream_type = to_string(stream.type);
  encoded.metadata.codec = stream.codec.type == "none" ? "raw" : stream.codec.type;
  encoded.metadata.sequence = sequence;
  encoded.metadata.stamp_sec = resized_image.header.stamp.sec;
  encoded.metadata.stamp_nanosec = resized_image.header.stamp.nanosec;
  encoded.metadata.frame_id = resized_image.header.frame_id;
  encoded.metadata.image.encoding = resized_image.encoding;
  encoded.metadata.image.width = resized_image.width;
  encoded.metadata.image.height = resized_image.height;
  encoded.metadata.image.step = resized_image.step;
  return encoded;
}

sensor_msgs::msg::Image decode_depth_image(
  const StreamMetadata & metadata,
  const std::vector<std::uint8_t> & payload)
{
  if (metadata.codec == "raw" || metadata.codec == "none") {
    return decode_image(metadata, payload);
  }

  sensor_msgs::msg::CompressedImage compressed;
  compressed.header.stamp.sec = metadata.stamp_sec;
  compressed.header.stamp.nanosec = metadata.stamp_nanosec;
  compressed.header.frame_id = metadata.frame_id;
  compressed.format = metadata.image.encoding + "; compressedDepth";
  compressed.data = payload;

  auto decoded = compressed_depth_image_transport::decodeCompressedDepthImage(compressed);
  if (!decoded) {
    throw std::runtime_error("compressed_depth_image_transport failed to decode depth image");
  }
  return *decoded;
}

EncodedImage encode_point_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const StreamConfig & stream,
  std::uint64_t sequence)
{
  const auto transport_cloud = cloud_for_transport(cloud, stream);
  point_cloud_transport::PointCloudCodec codec;
  rclcpp::SerializedMessage serialized;
  if (!codec.encode(stream.codec.type, transport_cloud, serialized)) {
    throw std::runtime_error("point_cloud_transport failed to encode with codec '" + stream.codec.type + "'");
  }

  const auto & rcl_serialized = serialized.get_rcl_serialized_message();
  EncodedImage encoded;
  encoded.payload.assign(
    rcl_serialized.buffer,
    rcl_serialized.buffer + rcl_serialized.buffer_length);
  encoded.metadata.schema_version = 1;
  encoded.metadata.stream_id = stream.name;
  encoded.metadata.stream_type = to_string(stream.type);
  encoded.metadata.codec = stream.codec.type;
  encoded.metadata.sequence = sequence;
  encoded.metadata.stamp_sec = cloud.header.stamp.sec;
  encoded.metadata.stamp_nanosec = cloud.header.stamp.nanosec;
  encoded.metadata.frame_id = cloud.header.frame_id;
  encoded.metadata.pointcloud.width = cloud.width;
  encoded.metadata.pointcloud.height = cloud.height;
  encoded.metadata.pointcloud.is_bigendian = cloud.is_bigendian;
  encoded.metadata.pointcloud.point_step = cloud.point_step;
  encoded.metadata.pointcloud.row_step = cloud.row_step;
  encoded.metadata.pointcloud.is_dense = cloud.is_dense;
  encoded.metadata.pointcloud.fields = cloud.fields;
  return encoded;
}

sensor_msgs::msg::PointCloud2 decode_point_cloud(
  const StreamMetadata & metadata,
  const std::vector<std::uint8_t> & payload)
{
  rclcpp::SerializedMessage serialized(payload.size());
  auto & rcl_serialized = serialized.get_rcl_serialized_message();
  if (payload.size() > rcl_serialized.buffer_capacity) {
    throw std::runtime_error("serialized point cloud payload exceeds allocated capacity");
  }
  std::memcpy(rcl_serialized.buffer, payload.data(), payload.size());
  rcl_serialized.buffer_length = payload.size();

  sensor_msgs::msg::PointCloud2 cloud;
  point_cloud_transport::PointCloudCodec codec;
  if (!codec.decode(metadata.codec, serialized, cloud)) {
    throw std::runtime_error("point_cloud_transport failed to decode with codec '" + metadata.codec + "'");
  }
  if (metadata.codec == "cloudini" &&
    cloud.point_step == metadata.pointcloud.point_step &&
    cloud.data.size() == static_cast<std::size_t>(cloud.row_step) * cloud.height)
  {
    cloud.fields = metadata.pointcloud.fields;
  }
  return cloud;
}

}  // namespace vision_transport
