#include "vision_transport/zenoh_session.hpp"

#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <zenoh.h>

namespace vision_transport
{

namespace
{

std::runtime_error zenoh_error(const std::string & operation, z_result_t result)
{
  return std::runtime_error(operation + " failed with Zenoh error " + std::to_string(result));
}

std::vector<std::uint8_t> copy_bytes(const z_loaned_bytes_t * bytes)
{
  std::vector<std::uint8_t> data(z_bytes_len(bytes));
  auto reader = z_bytes_get_reader(bytes);
  const auto read = z_bytes_reader_read(&reader, data.data(), data.size());
  if (read != data.size()) {
    throw std::runtime_error("Zenoh sample payload ended before expected byte count");
  }
  return data;
}

std::string read_config(const std::string & path)
{
  if (path.empty()) {
    return R"({
      mode: "peer",
      listen: { endpoints: [] },
      connect: { endpoints: [] },
      scouting: { multicast: { enabled: false } },
      transport: { shared_memory: { enabled: false } }
    })";
  }

  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open Zenoh config '" + path + "'");
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace

struct ZenohSession::Impl
{
  struct Subscription
  {
    z_owned_keyexpr_t keyexpr;
    z_owned_closure_sample_t closure;
    z_owned_subscriber_t subscriber;
    Callback callback;
  };

  explicit Impl(const std::string & config_path)
  {
    z_owned_config_t config;
    const auto config_text = read_config(config_path);
    if (zc_config_from_str(&config, config_text.c_str()) < 0) {
      throw std::runtime_error("failed to construct Zenoh config from '" + config_path + "'");
    }

    z_open_options_t options;
    z_open_options_default(&options);
    const auto result = z_open(&session, z_move(config), &options);
    if (result < 0) {
      throw zenoh_error("z_open", result);
    }
  }

  ~Impl()
  {
    subscriptions.clear();
    z_drop(z_move(session));
  }

  void put_cbor(const std::string & key, const std::vector<std::uint8_t> & payload)
  {
    std::lock_guard<std::mutex> lock(mutex);
    z_owned_keyexpr_t keyexpr;
    auto result = z_keyexpr_from_str_autocanonize(&keyexpr, key.c_str());
    if (result < 0) {
      throw zenoh_error("z_keyexpr_from_str_autocanonize", result);
    }

    z_owned_bytes_t bytes;
    result = z_bytes_copy_from_buf(&bytes, payload.data(), payload.size());
    if (result < 0) {
      z_drop(z_move(keyexpr));
      throw zenoh_error("z_bytes_copy_from_buf", result);
    }

    z_put_options_t options;
    z_put_options_default(&options);
    z_owned_encoding_t encoding;
    z_encoding_clone(&encoding, z_encoding_application_cbor());
    options.encoding = z_move(encoding);

    result = z_put(z_session_loan(&session), z_keyexpr_loan(&keyexpr), z_move(bytes), &options);
    z_drop(z_move(keyexpr));
    if (result < 0) {
      throw zenoh_error("z_put", result);
    }
  }

  void subscribe(const std::string & key, Callback callback)
  {
    auto subscription = std::make_unique<Subscription>();
    subscription->callback = std::move(callback);

    auto result = z_keyexpr_from_str_autocanonize(&subscription->keyexpr, key.c_str());
    if (result < 0) {
      throw zenoh_error("z_keyexpr_from_str_autocanonize", result);
    }

    z_closure_sample(
      &subscription->closure,
      [](z_loaned_sample_t * sample, void * context) {
        auto * owned = static_cast<Subscription *>(context);
        owned->callback(copy_bytes(z_sample_payload(sample)));
      },
      nullptr,
      subscription.get());

    z_subscriber_options_t options;
    z_subscriber_options_default(&options);
    result = z_declare_subscriber(
      z_session_loan(&session),
      &subscription->subscriber,
      z_keyexpr_loan(&subscription->keyexpr),
      z_move(subscription->closure),
      &options);
    if (result < 0) {
      z_drop(z_move(subscription->keyexpr));
      throw zenoh_error("z_declare_subscriber", result);
    }

    std::lock_guard<std::mutex> lock(mutex);
    subscriptions.push_back(std::move(subscription));
  }

  z_owned_session_t session;
  std::mutex mutex;
  std::vector<std::unique_ptr<Subscription>> subscriptions;
};

ZenohSession::ZenohSession(const std::string & config_path)
: impl_(std::make_unique<Impl>(config_path))
{
}

ZenohSession::~ZenohSession() = default;

ZenohSession::ZenohSession(ZenohSession &&) noexcept = default;

ZenohSession & ZenohSession::operator=(ZenohSession &&) noexcept = default;

void ZenohSession::put_cbor(const std::string & key, const std::vector<std::uint8_t> & payload)
{
  impl_->put_cbor(key, payload);
}

void ZenohSession::subscribe(const std::string & key, Callback callback)
{
  impl_->subscribe(key, std::move(callback));
}

}  // namespace vision_transport
