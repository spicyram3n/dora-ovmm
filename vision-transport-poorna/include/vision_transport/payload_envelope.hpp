#pragma once

#include <cstdint>
#include <vector>

#include "vision_transport/stream_metadata.hpp"

namespace vision_transport
{

struct PayloadEnvelope
{
  StreamMetadata metadata;
  std::vector<std::uint8_t> payload;
};

std::vector<std::uint8_t> pack_envelope(const PayloadEnvelope & envelope);
PayloadEnvelope unpack_envelope(const std::vector<std::uint8_t> & bytes);

}  // namespace vision_transport
