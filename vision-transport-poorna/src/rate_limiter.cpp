#include "vision_transport/rate_limiter.hpp"

#include <chrono>

namespace vision_transport
{

RateLimiter::RateLimiter(double max_rate_hz)
: max_rate_hz_(max_rate_hz)
{
  if (max_rate_hz_ > 0.0) {
    const auto period_seconds = 1.0 / max_rate_hz_;
    min_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period_seconds));
  }
}

bool RateLimiter::should_accept(std::chrono::steady_clock::time_point now)
{
  if (max_rate_hz_ <= 0.0) {
    return true;
  }
  if (!has_last_accept_) {
    last_accept_ = now;
    has_last_accept_ = true;
    return true;
  }
  if (now - last_accept_ < min_period_) {
    return false;
  }
  last_accept_ = now;
  return true;
}

double RateLimiter::max_rate_hz() const
{
  return max_rate_hz_;
}

}  // namespace vision_transport
