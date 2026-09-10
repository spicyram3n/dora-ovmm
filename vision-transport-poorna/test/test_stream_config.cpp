#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "vision_transport/stream_config.hpp"

namespace
{

std::string write_config(const std::string & contents)
{
  const auto path = testing::TempDir() + "/streams.yaml";
  std::ofstream file(path);
  file << contents;
  return path;
}

}  // namespace

TEST(StreamConfig, LoadsConfiguredStreams)
{
  const auto path = write_config(R"(
streams:
  rgb:
    type: image
    input_topic: /camera/color/image_raw
    output_topic: /remote/camera/color/image_raw
    zenoh_key: robot/camera/rgb
    max_rate_hz: 10.0
    queue_depth: 1
    resize_scale: 0.5
    codec:
      type: jpeg
      quality: 80
  cloud:
    type: pointcloud
    input_topic: /lidar/points
    output_topic: /remote/lidar/points
    zenoh_key: robot/lidar/points
    voxel_size: 0.02
    codec:
      type: cloudini
      cloudini_resolution: 0.001
)");

  const auto streams = vision_transport::load_stream_config(path);

  ASSERT_EQ(streams.size(), 2u);
  EXPECT_EQ(streams[0].name, "rgb");
  EXPECT_EQ(streams[0].type, vision_transport::StreamType::Image);
  ASSERT_TRUE(streams[0].resize_scale.has_value());
  EXPECT_DOUBLE_EQ(*streams[0].resize_scale, 0.5);
  EXPECT_EQ(streams[1].codec.type, "cloudini");
  ASSERT_TRUE(streams[1].voxel_size.has_value());
}

TEST(StreamConfig, LoadsMinimalRxStreams)
{
  const auto path = write_config(R"(
streams:
  rgb:
    type: image
    output_topic: /remote/camera/color/image_raw
    zenoh_key: robot/camera/rgb
  cloud:
    type: pointcloud
    output_topic: /remote/lidar/points
    zenoh_key: robot/lidar/points
    queue_depth: 2
)");

  const auto streams = vision_transport::load_rx_stream_config(path);

  ASSERT_EQ(streams.size(), 2u);
  EXPECT_EQ(streams[0].name, "rgb");
  EXPECT_EQ(streams[0].type, vision_transport::StreamType::Image);
  EXPECT_EQ(streams[0].output_topic, "/remote/camera/color/image_raw");
  EXPECT_TRUE(streams[0].input_topic.empty());
  EXPECT_TRUE(streams[0].codec.type.empty());
  EXPECT_EQ(streams[1].queue_depth, 2U);
}

TEST(StreamConfig, TxRejectsMissingInputTopic)
{
  const auto path = write_config(R"(
streams:
  rgb:
    type: image
    output_topic: /remote/camera/color/image_raw
    zenoh_key: robot/camera/rgb
    codec:
      type: jpeg
)");

  EXPECT_THROW(vision_transport::load_tx_stream_config(path), std::runtime_error);
}

TEST(StreamConfig, RejectsMissingCodec)
{
  const auto path = write_config(R"(
streams:
  rgb:
    type: image
    input_topic: /camera/color/image_raw
    output_topic: /remote/camera/color/image_raw
    zenoh_key: robot/camera/rgb
)");

  EXPECT_THROW(vision_transport::load_stream_config(path), std::runtime_error);
}
