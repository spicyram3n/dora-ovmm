#pragma once

#include <chrono>

namespace vision_transport
{

class RateLimiter
{
public:
  explicit RateLimiter(double max_rate_hz);

  bool should_accept(std::chrono::steady_clock::time_point now);
  double max_rate_hz() const;

private:
  double max_rate_hz_{0.0};
  std::chrono::steady_clock::duration min_period_{std::chrono::steady_clock::duration::zero()};
  std::chrono::steady_clock::time_point last_accept_{};
  bool has_last_accept_{false};
};

}  // namespace vision_transport
