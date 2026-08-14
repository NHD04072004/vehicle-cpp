#include "utils/time_utils.h"

#include <chrono>
#include <cstdio>
#include <ctime>

namespace vehicle {
namespace utils {

double monotonicSeconds() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double epochSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

std::string utcIso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t secs = std::chrono::system_clock::to_time_t(now);
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch()).count() % 1000000;
  std::tm tm_buf{};
  gmtime_r(&secs, &tm_buf);
  char stamp[64];
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tm_buf);
  char out[80];
  std::snprintf(out, sizeof(out), "%s.%06ld+00:00", stamp, static_cast<long>(micros));
  return std::string(out);
}

}  // namespace utils
}  // namespace vehicle
