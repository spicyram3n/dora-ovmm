#include <chrono>
#include <cstring>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "vision_transport/image_codec.hpp"
#include "vision_transport/payload_envelope.hpp"
#include "vision_transport/stream_config.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

sensor_msgs::msg::Image make_rgb_image(std::uint32_t width, std::uint32_t height)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "benchmark_camera";
  image.width = width;
  image.height = height;
  image.encoding = "rgb8";
  image.is_bigendian = false;
  image.step = width * 3;
  image.data.resize(static_cast<std::size_t>(height) * image.step);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto offset = static_cast<std::size_t>(y) * image.step + x * 3;
      image.data[offset + 0] = static_cast<std::uint8_t>(x % 256);
      image.data[offset + 1] = static_cast<std::uint8_t>(y % 256);
      image.data[offset + 2] = static_cast<std::uint8_t>((x + y) % 256);
    }
  }
  return image;
}

sensor_msgs::msg::Image make_depth_image(std::uint32_t width, std::uint32_t height)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "benchmark_depth";
  image.width = width;
  image.height = height;
  image.encoding = "16UC1";
  image.is_bigendian = false;
  image.step = width * 2;
  image.data.resize(static_cast<std::size_t>(height) * image.step);
  for (std::uint32_t i = 0; i < width * height; ++i) {
    const auto value = static_cast<std::uint16_t>(500 + (i % 5000));
    image.data[i * 2] = static_cast<std::uint8_t>(value & 0xff);
    image.data[i * 2 + 1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
  }
  return image;
}

sensor_msgs::msg::PointCloud2 make_cloud(std::uint32_t points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "benchmark_lidar";
  cloud.height = 1;
  cloud.width = points;
  cloud.is_bigendian = false;
  cloud.point_step = 12;
  cloud.row_step = points * cloud.point_step;
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

  cloud.data.resize(static_cast<std::size_t>(points) * cloud.point_step);
  for (std::uint32_t i = 0; i < points; ++i) {
    const float values[3] = {
      static_cast<float>(i % 1024) * 0.01F,
      static_cast<float>((i / 1024) % 1024) * 0.01F,
      static_cast<float>(i % 128) * 0.02F,
    };
    std::memcpy(cloud.data.data() + static_cast<std::size_t>(i) * cloud.point_step, values, sizeof(values));
  }
  return cloud;
}

vision_transport::StreamConfig stream(
  const std::string & name,
  vision_transport::StreamType type,
  const std::string & codec)
{
  vision_transport::StreamConfig config;
  config.name = name;
  config.type = type;
  config.codec.type = codec;
  config.codec.quality = 80;
  config.codec.png_level = 3;
  return config;
}

template<typename Encode, typename Decode>
void benchmark_case(
  const std::string & label,
  std::size_t raw_bytes,
  int iterations,
  Encode encode,
  Decode decode)
{
  std::size_t payload_bytes = 0;
  std::size_t envelope_bytes = 0;

  const auto encode_start = Clock::now();
  for (int i = 0; i < iterations; ++i) {
    const auto encoded = encode();
    payload_bytes = encoded.payload.size();
    envelope_bytes = vision_transport::pack_envelope({encoded.metadata, encoded.payload}).size();
  }
  const auto encode_end = Clock::now();

  const auto encoded = encode();
  const auto decode_start = Clock::now();
  for (int i = 0; i < iterations; ++i) {
    decode(encoded);
  }
  const auto decode_end = Clock::now();

  const auto encode_us = std::chrono::duration_cast<std::chrono::microseconds>(
    encode_end - encode_start).count() / static_cast<double>(iterations);
  const auto decode_us = std::chrono::duration_cast<std::chrono::microseconds>(
    decode_end - decode_start).count() / static_cast<double>(iterations);
  const auto ratio = raw_bytes == 0 ? 0.0 : static_cast<double>(payload_bytes) / raw_bytes;

  std::cout << label << ','
            << raw_bytes << ','
            << payload_bytes << ','
            << envelope_bytes << ','
            << ratio << ','
            << encode_us << ','
            << decode_us << '\n';
}

}  // namespace

int main()
{
  constexpr int kIterations = 20;
  const auto rgb = make_rgb_image(640, 480);
  const auto depth = make_depth_image(320, 240);
  const auto cloud = make_cloud(4096);

  std::cout << "case,raw_bytes,payload_bytes,envelope_bytes,payload_to_raw,encode_us,decode_us\n";

  try {
    const auto raw_rgb = stream("rgb_raw", vision_transport::StreamType::Image, "raw");
    benchmark_case(
      "rgb_raw",
      rgb.data.size(),
      kIterations,
      [&] { return vision_transport::encode_image(rgb, raw_rgb, 1); },
      [&](const vision_transport::EncodedImage & encoded) {
        (void)vision_transport::decode_image(encoded.metadata, encoded.payload);
      });

    const auto jpeg = stream("rgb_jpeg", vision_transport::StreamType::Image, "jpeg");
    benchmark_case(
      "rgb_jpeg_q80",
      rgb.data.size(),
      kIterations,
      [&] { return vision_transport::encode_image(rgb, jpeg, 1); },
      [&](const vision_transport::EncodedImage & encoded) {
        (void)vision_transport::decode_image(encoded.metadata, encoded.payload);
      });

    const auto png = stream("rgb_png", vision_transport::StreamType::Image, "png");
    benchmark_case(
      "rgb_png_l3",
      rgb.data.size(),
      kIterations,
      [&] { return vision_transport::encode_image(rgb, png, 1); },
      [&](const vision_transport::EncodedImage & encoded) {
        (void)vision_transport::decode_image(encoded.metadata, encoded.payload);
      });

    const auto raw_depth = stream("depth_raw", vision_transport::StreamType::Depth, "raw");
    benchmark_case(
      "depth_raw",
      depth.data.size(),
      kIterations,
      [&] { return vision_transport::encode_depth_image(depth, raw_depth, 1); },
      [&](const vision_transport::EncodedImage & encoded) {
        (void)vision_transport::decode_depth_image(encoded.metadata, encoded.payload);
      });

    const auto compressed_depth = stream(
      "depth_compressed", vision_transport::StreamType::Depth, "compressed_depth");
    benchmark_case(
      "depth_compressed_png_l3",
      depth.data.size(),
      kIterations,
      [&] { return vision_transport::encode_depth_image(depth, compressed_depth, 1); },
      [&](const vision_transport::EncodedImage & encoded) {
        (void)vision_transport::decode_depth_image(encoded.metadata, encoded.payload);
      });

    for (const auto & codec : {"raw", "draco", "cloudini", "zstd"}) {
      const auto cloud_stream = stream(
        std::string("cloud_") + codec, vision_transport::StreamType::PointCloud, codec);
      benchmark_case(
        std::string("cloud_") + codec,
        cloud.data.size(),
        kIterations,
        [&] { return vision_transport::encode_point_cloud(cloud, cloud_stream, 1); },
        [&](const vision_transport::EncodedImage & encoded) {
          (void)vision_transport::decode_point_cloud(encoded.metadata, encoded.payload);
        });
    }
  } catch (const std::exception & error) {
    std::cerr << "benchmark failed: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
