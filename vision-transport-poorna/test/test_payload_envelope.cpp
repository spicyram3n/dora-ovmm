#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "vision_transport/payload_envelope.hpp"

TEST(PayloadEnvelope, RoundTripsMetadataAndPayload)
{
  vision_transport::PayloadEnvelope envelope;
  envelope.metadata.stream_id = "front";
  envelope.metadata.stream_type = "image";
  envelope.metadata.codec = "png";
  envelope.metadata.sequence = 42;
  envelope.metadata.stamp_sec = 12;
  envelope.metadata.stamp_nanosec = 34;
  envelope.metadata.frame_id = "camera";
  envelope.metadata.image.encoding = "rgb8";
  envelope.metadata.image.width = 2;
  envelope.metadata.image.height = 1;
  envelope.metadata.image.step = 6;
  envelope.payload = {1, 2, 3, 4, 5};

  const auto packed = vision_transport::pack_envelope(envelope);
  const auto unpacked = vision_transport::unpack_envelope(packed);

  EXPECT_EQ(unpacked.metadata.stream_id, "front");
  EXPECT_EQ(unpacked.metadata.stream_type, "image");
  EXPECT_EQ(unpacked.metadata.codec, "png");
  EXPECT_EQ(unpacked.metadata.sequence, 42U);
  EXPECT_EQ(unpacked.metadata.frame_id, "camera");
  EXPECT_EQ(unpacked.metadata.image.encoding, "rgb8");
  EXPECT_EQ(unpacked.metadata.image.width, 2U);
  EXPECT_EQ(unpacked.metadata.image.height, 1U);
  EXPECT_EQ(unpacked.metadata.image.step, 6U);
  EXPECT_EQ(unpacked.payload, envelope.payload);
}

TEST(PayloadEnvelope, RejectsTruncatedHeader)
{
  EXPECT_THROW(vision_transport::unpack_envelope({1, 2, 3}), std::runtime_error);
}

TEST(PayloadEnvelope, RejectsInvalidMetadataLength)
{
  EXPECT_THROW(vision_transport::unpack_envelope({0, 0, 0, 8, 1, 2}), std::runtime_error);
}
