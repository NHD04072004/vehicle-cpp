#include "utils/latency.h"

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "common/logging.h"
#include "utils/time_utils.h"

namespace vehicle {
namespace utils {
namespace {

bool envTruthy(const char* value) {
  if (value == nullptr || value[0] == '\0') return false;
  if (std::strcmp(value, "1") == 0) return true;
  if (::strcasecmp(value, "true") == 0) return true;
  if (::strcasecmp(value, "yes") == 0) return true;
  if (::strcasecmp(value, "on") == 0) return true;
  return false;
}

}  // namespace

bool latencyEnabled() {
  static const bool enabled = envTruthy(std::getenv("VEHICLE_LATENCY"));
  return enabled;
}

void latencyLog(const char* fmt, ...) {
  if (!latencyEnabled()) return;
  char buf[768];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  LOG_DEBUG("latency: %s", buf);
}

double msSince(double start_s) {
  if (start_s <= 0.0) return 0.0;
  return (monotonicSeconds() - start_s) * 1000.0;
}

}  // namespace utils
}  // namespace vehicle
