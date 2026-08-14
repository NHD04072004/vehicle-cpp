#include "common/logging.h"

#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace vehicle {
namespace log {
namespace {

std::mutex g_mutex;

const char* levelName(Level level) {
  switch (level) {
    case Level::kDebug: return "DEBUG";
    case Level::kInfo: return "INFO ";
    case Level::kWarn: return "WARN ";
    case Level::kError: return "ERROR";
  }
  return "?????";
}

Level parseLevel(const char* text) {
  if (text == nullptr) return Level::kInfo;
  std::string value(text);
  for (char& c : value) c = static_cast<char>(::tolower(c));
  if (value == "debug") return Level::kDebug;
  if (value == "warn" || value == "warning") return Level::kWarn;
  if (value == "error") return Level::kError;
  return Level::kInfo;
}

const char* baseName(const char* path) {
  const char* slash = std::strrchr(path, '/');
  return slash ? slash + 1 : path;
}

}  // namespace

Level minLevel() {
  static const Level level = parseLevel(std::getenv("VEHICLE_LOG_LEVEL"));
  return level;
}

void write(Level level, const char* file, int line, const std::string& msg) {
  std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm_buf);

  std::lock_guard<std::mutex> lock(g_mutex);
  std::fprintf(stdout, "[%s] %s %s:%d | %s\n", stamp, levelName(level),
               baseName(file), line, msg.c_str());
  std::fflush(stdout);
}

std::string format(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  va_list copy;
  va_copy(copy, args);
  const int needed = std::vsnprintf(nullptr, 0, fmt, copy);
  va_end(copy);
  if (needed <= 0) {
    va_end(args);
    return std::string();
  }
  std::vector<char> buffer(static_cast<size_t>(needed) + 1);
  std::vsnprintf(buffer.data(), buffer.size(), fmt, args);
  va_end(args);
  return std::string(buffer.data(), static_cast<size_t>(needed));
}

}  // namespace log
}  // namespace vehicle
