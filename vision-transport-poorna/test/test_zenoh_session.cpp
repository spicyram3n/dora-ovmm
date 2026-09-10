#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "vision_transport/zenoh_session.hpp"

TEST(ZenohSession, PublishesAndReceivesLoopbackPayload)
{
  vision_transport::ZenohSession session;
  const std::string key = "vision_transport/tests/loopback";
  const std::vector<std::uint8_t> expected{9, 8, 7, 6};

  std::mutex mutex;
  std::condition_variable condition;
  std::vector<std::uint8_t> received;
  bool has_received = false;

  session.subscribe(key, [&](const std::vector<std::uint8_t> & payload) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      received = payload;
      has_received = true;
    }
    condition.notify_one();
  });

  session.put_cbor(key, expected);

  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(3), [&] { return has_received; }));
  EXPECT_EQ(received, expected);
}
