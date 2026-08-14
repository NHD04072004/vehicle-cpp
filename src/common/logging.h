// Logger tối giản dùng chung cho toàn app (stdout, có timestamp + level).
#pragma once

#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>

namespace vehicle {
namespace log {

enum class Level { kDebug = 0, kInfo, kWarn, kError };

// Level tối thiểu được in ra; đổi bằng env VEHICLE_LOG_LEVEL=debug|info|warn|error.
Level minLevel();

void write(Level level, const char* file, int line, const std::string& msg);

std::string format(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

}  // namespace log
}  // namespace vehicle

#define VLOG(level, ...)                                                     \
  do {                                                                       \
    if ((level) >= ::vehicle::log::minLevel())                               \
      ::vehicle::log::write((level), __FILE__, __LINE__,                     \
                            ::vehicle::log::format(__VA_ARGS__));            \
  } while (0)

#define LOG_DEBUG(...) VLOG(::vehicle::log::Level::kDebug, __VA_ARGS__)
#define LOG_INFO(...) VLOG(::vehicle::log::Level::kInfo, __VA_ARGS__)
#define LOG_WARN(...) VLOG(::vehicle::log::Level::kWarn, __VA_ARGS__)
#define LOG_ERROR(...) VLOG(::vehicle::log::Level::kError, __VA_ARGS__)
