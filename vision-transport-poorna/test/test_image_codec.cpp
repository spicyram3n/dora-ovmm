#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <point_cloud_transport/point_cloud_codec.hpp>

#include "vision_transport/image_codec.hpp"

namespace
{

vision_transport::StreamConfig png_stream()
{
  vision_transport::StreamConfig stream;
  stream.name = "front";
  stream.type = vision_transport::StreamType::Image;
  stream.codec.type = "png";
  stream.codec.png_level = 1;
  return stream;
}

vision_transport::StreamConfig raw_image_stream()
{
  vision_transport::StreamConfig stream;
  stream.name = "front";
  stream.type = vision_transport::StreamType::Image;
  stream.codec.type = "raw";
  return stream;
}

sensor_msgs::msg::Image make_image(
  const std::string & encoding,
  std::uint32_t width,
  std::uint32_t height,
  std::vector<std::uint8_t> data)
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 7;
  image.header.stamp.nanosec = 8;
  image.header.frame_id = "camera";
  image.width = width;
  image.height = height;
  image.encoding = encoding;
  image.is_bigendian = false;
  image.step = static_cast<std::uint32_t>(data.size() / height);
  image.data = std::move(data);
  return image;
}

sensor_msgs::msg::PointCloud2 make_xyz_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 13;
  cloud.header.stamp.nanosec = 14;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = 24;
  cloud.is_dense = true;

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  cloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  cloud.fields.push_back(field);
  cloud.data = {
    0, 0, 0x80, 0x3f, 0, 0, 0, 0x40, 0, 0, 0x40, 0x40,
    0, 0, 0x80, 0x40, 0, 0, 0xa0, 0x40, 0, 0, 0xc0, 0x40,
  };
  return cloud;
}

void append_point_xyzrgb(
  std::vector<std::uint8_t> & data,
  float x,
  float y,
  float z,
  std::uint32_t rgb)
{
  const float coordinates[3] = {x, y, z};
  const auto * coordinate_bytes = reinterpret_cast<const std::uint8_t *>(coordinates);
  data.insert(data.end(), coordinate_bytes, coordinate_bytes + sizeof(coordinates));
  const auto * color_bytes = reinterpret_cast<const std::uint8_t *>(&rgb);
  data.insert(data.end(), color_bytes, color_bytes + sizeof(rgb));
}

sensor_msgs::msg::PointCloud2 make_xyzrgb_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 15;
  cloud.header.stamp.nanosec = 16;
  cloud.header.frame_id = "rgb_lidar";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_bigendian = false;
  cloud.point_step = 16;
  cloud.row_step = 32;
  cloud.is_dense = true;

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  cloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  cloud.fields.push_back(field);
  field.name = "rgb";
  field.offset = 12;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields.push_back(field);

  append_point_xyzrgb(cloud.data, 1.0F, 2.0F, 3.0F, 0x00ff0000U);
  append_point_xyzrgb(cloud.data, 4.0F, 5.0F, 6.0F, 0x0000ff00U);
  return cloud;
}

void append_float32(std::vector<std::uint8_t> & data, float value)
{
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&value);
  data.insert(data.end(), bytes, bytes + sizeof(float));
}

float read_float32(const std::vector<std::uint8_t> & data, std::size_t index)
{
  float value{};
  std::memcpy(&value, data.data() + index * sizeof(float), sizeof(float));
  return value;
}

std::uint32_t read_uint32_at(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  std::uint32_t value{};
  std::memcpy(&value, data.data() + offset, sizeof(value));
  return value;
}

}  // namespace

TEST(ImageCodec, PngRoundTripsRgb8)
{
  const auto image = make_image(
    "rgb8",
    2,
    2,
    {
      255, 0, 0, 0, 255, 0,
      0, 0, 255, 255, 255, 255,
    });

  const auto encoded = vision_transport::encode_image(image, png_stream(), 3);
  const auto decoded = vision_transport::decode_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.sequence, 3U);
  EXPECT_EQ(encoded.metadata.image.encoding, "rgb8");
  EXPECT_EQ(decoded.encoding, "rgb8");
  EXPECT_EQ(decoded.width, image.width);
  EXPECT_EQ(decoded.height, image.height);
  EXPECT_EQ(decoded.step, image.step);
  EXPECT_EQ(decoded.data, image.data);
}

TEST(ImageCodec, PngRoundTripsMono8)
{
  const auto image = make_image("mono8", 3, 2, {0, 32, 64, 96, 128, 255});

  const auto encoded = vision_transport::encode_image(image, png_stream(), 4);
  const auto decoded = vision_transport::decode_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(decoded.encoding, "mono8");
  EXPECT_EQ(decoded.width, image.width);
  EXPECT_EQ(decoded.height, image.height);
  EXPECT_EQ(decoded.step, image.step);
  EXPECT_EQ(decoded.data, image.data);
}

TEST(ImageCodec, RawRoundTripsRgb8)
{
  const auto image = make_image(
    "rgb8",
    2,
    2,
    {
      255, 0, 0, 0, 255, 0,
      0, 0, 255, 255, 255, 255,
    });

  const auto encoded = vision_transport::encode_image(image, raw_image_stream(), 11);
  const auto decoded = vision_transport::decode_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.codec, "raw");
  EXPECT_EQ(encoded.payload, image.data);
  EXPECT_EQ(decoded.encoding, "rgb8");
  EXPECT_EQ(decoded.width, image.width);
  EXPECT_EQ(decoded.height, image.height);
  EXPECT_EQ(decoded.step, image.step);
  EXPECT_EQ(decoded.data, image.data);
}

TEST(ImageCodec, AppliesResizeScaleBeforeEncoding)
{
  auto stream = png_stream();
  stream.resize_scale = 0.5;
  const auto image = make_image(
    "mono8",
    4,
    4,
    {
      0, 16, 32, 48,
      64, 80, 96, 112,
      128, 144, 160, 176,
      192, 208, 224, 240,
    });

  const auto encoded = vision_transport::encode_image(image, stream, 5);
  const auto decoded = vision_transport::decode_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.image.width, 2U);
  EXPECT_EQ(encoded.metadata.image.height, 2U);
  EXPECT_EQ(decoded.width, 2U);
  EXPECT_EQ(decoded.height, 2U);
  EXPECT_EQ(decoded.step, 2U);
}

TEST(ImageCodec, CompressedDepthRoundTrips16UC1)
{
  vision_transport::StreamConfig stream;
  stream.name = "depth";
  stream.type = vision_transport::StreamType::Depth;
  stream.codec.type = "compressed_depth";
  stream.codec.png_level = 1;

  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 11;
  image.header.stamp.nanosec = 12;
  image.header.frame_id = "depth_camera";
  image.width = 2;
  image.height = 2;
  image.encoding = "16UC1";
  image.is_bigendian = false;
  image.step = 4;
  image.data = {
    0xe8, 0x03, 0xd0, 0x07,
    0xb8, 0x0b, 0xa0, 0x0f,
  };

  const auto encoded = vision_transport::encode_depth_image(image, stream, 6);
  const auto decoded = vision_transport::decode_depth_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.stream_type, "depth");
  EXPECT_EQ(decoded.header.frame_id, "depth_camera");
  EXPECT_EQ(decoded.encoding, "16UC1");
  EXPECT_EQ(decoded.width, image.width);
  EXPECT_EQ(decoded.height, image.height);
  EXPECT_EQ(decoded.step, image.step);
  EXPECT_EQ(decoded.data, image.data);
}

TEST(ImageCodec, CompressedDepthRoundTrips32FC1)
{
  vision_transport::StreamConfig stream;
  stream.name = "depth";
  stream.type = vision_transport::StreamType::Depth;
  stream.codec.type = "compressed_depth";
  stream.codec.png_level = 1;

  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 17;
  image.header.stamp.nanosec = 18;
  image.header.frame_id = "depth_camera";
  image.width = 2;
  image.height = 2;
  image.encoding = "32FC1";
  image.is_bigendian = false;
  image.step = 8;
  append_float32(image.data, 1.0F);
  append_float32(image.data, 2.5F);
  append_float32(image.data, 0.5F);
  append_float32(image.data, 9.0F);

  const auto encoded = vision_transport::encode_depth_image(image, stream, 10);
  const auto decoded = vision_transport::decode_depth_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.stream_type, "depth");
  EXPECT_EQ(decoded.header.frame_id, "depth_camera");
  EXPECT_EQ(decoded.encoding, "32FC1");
  EXPECT_EQ(decoded.width, image.width);
  EXPECT_EQ(decoded.height, image.height);
  EXPECT_EQ(decoded.step, image.step);
  ASSERT_EQ(decoded.data.size(), image.data.size());
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(read_float32(decoded.data, i), read_float32(image.data, i), 0.02F);
  }
}

TEST(ImageCodec, RawDepthRoundTrips16UC1)
{
  vision_transport::StreamConfig stream;
  stream.name = "depth";
  stream.type = vision_transport::StreamType::Depth;
  stream.codec.type = "raw";

  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 19;
  image.header.stamp.nanosec = 20;
  image.header.frame_id = "depth_camera";
  image.width = 2;
  image.height = 2;
  image.encoding = "16UC1";
  image.is_bigendian = false;
  image.step = 4;
  image.data = {
    0xe8, 0x03, 0xd0, 0x07,
    0xb8, 0x0b, 0xa0, 0x0f,
  };

  const auto encoded = vision_transport::encode_depth_image(image, stream, 12);
  const auto decoded = vision_transport::decode_depth_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.codec, "raw");
  EXPECT_EQ(encoded.payload, image.data);
  EXPECT_EQ(decoded.header.frame_id, "depth_camera");
  EXPECT_EQ(decoded.encoding, "16UC1");
  EXPECT_EQ(decoded.width, image.width);
  EXPECT_EQ(decoded.height, image.height);
  EXPECT_EQ(decoded.step, image.step);
  EXPECT_EQ(decoded.data, image.data);
}

TEST(ImageCodec, ResizesCompressedDepthBeforeEncoding)
{
  vision_transport::StreamConfig stream;
  stream.name = "depth";
  stream.type = vision_transport::StreamType::Depth;
  stream.codec.type = "compressed_depth";
  stream.codec.png_level = 1;
  stream.resize_scale = 0.5;

  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 21;
  image.header.stamp.nanosec = 22;
  image.header.frame_id = "depth_camera";
  image.width = 4;
  image.height = 4;
  image.encoding = "16UC1";
  image.is_bigendian = false;
  image.step = 8;
  for (std::uint16_t value : {
      100, 200, 300, 400,
      500, 600, 700, 800,
      900, 1000, 1100, 1200,
      1300, 1400, 1500, 1600,
    })
  {
    image.data.push_back(static_cast<std::uint8_t>(value & 0xff));
    image.data.push_back(static_cast<std::uint8_t>(value >> 8));
  }

  const auto encoded = vision_transport::encode_depth_image(image, stream, 9);
  const auto decoded = vision_transport::decode_depth_image(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.sequence, 9U);
  EXPECT_EQ(encoded.metadata.image.width, 2U);
  EXPECT_EQ(encoded.metadata.image.height, 2U);
  EXPECT_EQ(encoded.metadata.image.step, 4U);
  EXPECT_EQ(decoded.header.frame_id, "depth_camera");
  EXPECT_EQ(decoded.encoding, "16UC1");
  EXPECT_EQ(decoded.width, 2U);
  EXPECT_EQ(decoded.height, 2U);
  EXPECT_EQ(decoded.step, 4U);
  EXPECT_EQ(decoded.data.size(), 8U);
}

TEST(ImageCodec, RejectsUnsupportedEncoding)
{
  const auto image = make_image("16UC1", 2, 1, {0, 1, 2, 3});
  EXPECT_THROW(vision_transport::encode_image(image, png_stream(), 1), std::runtime_error);
}

TEST(PointCloudCodec, RawRoundTripsPointCloud2)
{
  vision_transport::StreamConfig stream;
  stream.name = "cloud";
  stream.type = vision_transport::StreamType::PointCloud;
  stream.codec.type = "raw";

  const auto cloud = make_xyz_cloud();

  const auto encoded = vision_transport::encode_point_cloud(cloud, stream, 7);
  const auto decoded = vision_transport::decode_point_cloud(encoded.metadata, encoded.payload);

  EXPECT_EQ(encoded.metadata.stream_type, "pointcloud");
  EXPECT_EQ(decoded.header.frame_id, "lidar");
  EXPECT_EQ(decoded.width, cloud.width);
  EXPECT_EQ(decoded.height, cloud.height);
  EXPECT_EQ(decoded.fields.size(), cloud.fields.size());
  EXPECT_EQ(decoded.point_step, cloud.point_step);
  EXPECT_EQ(decoded.row_step, cloud.row_step);
  EXPECT_EQ(decoded.data, cloud.data);
}

TEST(PointCloudCodec, RawPreservesPackedRgbField)
{
  vision_transport::StreamConfig stream;
  stream.name = "cloud";
  stream.type = vision_transport::StreamType::PointCloud;
  stream.codec.type = "raw";

  const auto cloud = make_xyzrgb_cloud();

  const auto encoded = vision_transport::encode_point_cloud(cloud, stream, 13);
  const auto decoded = vision_transport::decode_point_cloud(encoded.metadata, encoded.payload);

  ASSERT_EQ(decoded.fields.size(), cloud.fields.size());
  EXPECT_EQ(decoded.fields[3].name, "rgb");
  EXPECT_EQ(decoded.fields[3].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(read_uint32_at(decoded.data, 12), 0x00ff0000U);
  EXPECT_EQ(read_uint32_at(decoded.data, 28), 0x0000ff00U);
}

TEST(PointCloudCodec, CloudiniPreservesPackedRgbField)
{
  vision_transport::StreamConfig stream;
  stream.name = "cloudini";
  stream.type = vision_transport::StreamType::PointCloud;
  stream.codec.type = "cloudini";

  const auto cloud = make_xyzrgb_cloud();

  const auto encoded = vision_transport::encode_point_cloud(cloud, stream, 14);
  const auto decoded = vision_transport::decode_point_cloud(encoded.metadata, encoded.payload);

  ASSERT_EQ(decoded.fields.size(), cloud.fields.size());
  EXPECT_EQ(decoded.fields[3].name, "rgb");
  EXPECT_EQ(decoded.fields[3].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(decoded.point_step, cloud.point_step);
  EXPECT_EQ(read_uint32_at(decoded.data, 12), 0x00ff0000U);
  EXPECT_EQ(read_uint32_at(decoded.data, 28), 0x0000ff00U);
}

TEST(PointCloudCodec, DracoPreservesPackedRgbField)
{
  vision_transport::StreamConfig stream;
  stream.name = "draco";
  stream.type = vision_transport::StreamType::PointCloud;
  stream.codec.type = "draco";

  const auto cloud = make_xyzrgb_cloud();

  const auto encoded = vision_transport::encode_point_cloud(cloud, stream, 15);
  const auto decoded = vision_transport::decode_point_cloud(encoded.metadata, encoded.payload);

  ASSERT_EQ(decoded.fields.size(), cloud.fields.size());
  EXPECT_EQ(decoded.fields[3].name, "rgb");
  EXPECT_EQ(decoded.fields[3].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(decoded.point_step, cloud.point_step);
  EXPECT_EQ(read_uint32_at(decoded.data, 12), 0x00ff0000U);
  EXPECT_EQ(read_uint32_at(decoded.data, 28), 0x0000ff00U);
}

TEST(PointCloudCodec, DracoAndCloudiniRoundTripStructure)
{
  const auto cloud = make_xyz_cloud();
  for (const auto & codec_name : {"draco", "cloudini"}) {
    vision_transport::StreamConfig stream;
    stream.name = codec_name;
    stream.type = vision_transport::StreamType::PointCloud;
    stream.codec.type = codec_name;

    const auto encoded = vision_transport::encode_point_cloud(cloud, stream, 8);
    const auto decoded = vision_transport::decode_point_cloud(encoded.metadata, encoded.payload);

    EXPECT_EQ(encoded.metadata.codec, codec_name);
    EXPECT_EQ(decoded.header.frame_id, cloud.header.frame_id);
    EXPECT_EQ(decoded.width, cloud.width);
    EXPECT_EQ(decoded.height, cloud.height);
    EXPECT_EQ(decoded.fields.size(), cloud.fields.size());
    EXPECT_EQ(decoded.point_step, cloud.point_step);
    EXPECT_EQ(decoded.row_step, cloud.row_step);
  }
}

TEST(PointCloudCodec, LoadsConfiguredCompressionCandidates)
{
  point_cloud_transport::PointCloudCodec codec;
  std::vector<std::string> transports;
  std::vector<std::string> names;
  codec.getLoadableTransports(transports, names);

  EXPECT_NE(std::find(names.begin(), names.end(), "draco"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "cloudini"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "zstd"), names.end());
}
