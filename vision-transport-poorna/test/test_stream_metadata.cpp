#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "vision_transport/stream_metadata.hpp"

TEST(StreamMetadata, RoundTripsThroughCbor)
{
  vision_transport::StreamMetadata metadata;
  metadata.stream_id = "front_rgb";
  metadata.stream_type = "image";
  metadata.codec = "jpeg";
  metadata.sequence = 42;
  metadata.stamp_sec = 12;
  metadata.stamp_nanosec = 34;
  metadata.frame_id = "camera_link";
  metadata.image.encoding = "rgb8";
  metadata.image.width = 640;
  metadata.image.height = 480;
  metadata.image.step = 1920;

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  metadata.pointcloud.fields.push_back(field);

  const auto bytes = vision_transport::encode_metadata_cbor(metadata);
  const auto decoded = vision_transport::decode_metadata_cbor(bytes);

  EXPECT_EQ(decoded.schema_version, 1u);
  EXPECT_EQ(decoded.stream_id, "front_rgb");
  EXPECT_EQ(decoded.codec, "jpeg");
  EXPECT_EQ(decoded.sequence, 42u);
  EXPECT_EQ(decoded.frame_id, "camera_link");
  EXPECT_EQ(decoded.image.encoding, "rgb8");
  ASSERT_EQ(decoded.pointcloud.fields.size(), 1u);
  EXPECT_EQ(decoded.pointcloud.fields[0].name, "x");
}
