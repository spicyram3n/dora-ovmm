#include <chrono>

#include <gtest/gtest.h>

#include "vision_transport/rate_limiter.hpp"

TEST(RateLimiter, AcceptsEverythingWhenDisabled)
{
  vision_transport::RateLimiter limiter(0.0);
  const auto start = std::chrono::steady_clock::now();

  EXPECT_TRUE(limiter.should_accept(start));
  EXPECT_TRUE(limiter.should_accept(start));
}

TEST(RateLimiter, DropsFramesInsidePeriod)
{
  vision_transport::RateLimiter limiter(10.0);
  const auto start = std::chrono::steady_clock::now();

  EXPECT_TRUE(limiter.should_accept(start));
  EXPECT_FALSE(limiter.should_accept(start + std::chrono::milliseconds(50)));
  EXPECT_TRUE(limiter.should_accept(start + std::chrono::milliseconds(100)));
}
