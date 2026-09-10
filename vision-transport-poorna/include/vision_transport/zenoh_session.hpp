#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace vision_transport
{

class ZenohSession
{
public:
  using Callback = std::function<void(const std::vector<std::uint8_t> &)>;

  explicit ZenohSession(const std::string & config_path = "");
  ~ZenohSession();

  ZenohSession(const ZenohSession &) = delete;
  ZenohSession & operator=(const ZenohSession &) = delete;

  ZenohSession(ZenohSession &&) noexcept;
  ZenohSession & operator=(ZenohSession &&) noexcept;

  void put_cbor(const std::string & key, const std::vector<std::uint8_t> & payload);
  void subscribe(const std::string & key, Callback callback);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vision_transport
