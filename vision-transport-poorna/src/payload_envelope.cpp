#include "vision_transport/payload_envelope.hpp"

#include <stdexcept>

namespace vision_transport
{

namespace
{

constexpr std::size_t kHeaderSize = 4;

void append_u32(std::vector<std::uint8_t> & bytes, std::uint32_t value)
{
  bytes.push_back(static_cast<std::uint8_t>((value >> 24) & 0xff));
  bytes.push_back(static_cast<std::uint8_t>((value >> 16) & 0xff));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
  bytes.push_back(static_cast<std::uint8_t>(value & 0xff));
}

std::uint32_t read_u32(const std::vector<std::uint8_t> & bytes)
{
  return (static_cast<std::uint32_t>(bytes[0]) << 24) |
         (static_cast<std::uint32_t>(bytes[1]) << 16) |
         (static_cast<std::uint32_t>(bytes[2]) << 8) |
         static_cast<std::uint32_t>(bytes[3]);
}

}  // namespace

std::vector<std::uint8_t> pack_envelope(const PayloadEnvelope & envelope)
{
  const auto metadata = encode_metadata_cbor(envelope.metadata);
  if (metadata.size() > UINT32_MAX) {
    throw std::runtime_error("metadata CBOR is too large for envelope header");
  }

  std::vector<std::uint8_t> bytes;
  bytes.reserve(kHeaderSize + metadata.size() + envelope.payload.size());
  append_u32(bytes, static_cast<std::uint32_t>(metadata.size()));
  bytes.insert(bytes.end(), metadata.begin(), metadata.end());
  bytes.insert(bytes.end(), envelope.payload.begin(), envelope.payload.end());
  return bytes;
}

PayloadEnvelope unpack_envelope(const std::vector<std::uint8_t> & bytes)
{
  if (bytes.size() < kHeaderSize) {
    throw std::runtime_error("payload envelope is shorter than header");
  }

  const auto metadata_size = read_u32(bytes);
  if (metadata_size > bytes.size() - kHeaderSize) {
    throw std::runtime_error("payload envelope metadata length exceeds payload size");
  }

  PayloadEnvelope envelope;
  const auto metadata_begin = bytes.begin() + static_cast<std::ptrdiff_t>(kHeaderSize);
  const auto metadata_end = metadata_begin + static_cast<std::ptrdiff_t>(metadata_size);
  envelope.metadata = decode_metadata_cbor({metadata_begin, metadata_end});
  envelope.payload.assign(metadata_end, bytes.end());
  return envelope;
}

}  // namespace vision_transport
