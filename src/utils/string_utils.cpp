#include "utils/string_utils.h"

#include <algorithm>
#include <cctype>

namespace vehicle {
namespace utils {

std::string toUpper(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return text;
}

std::string trim(const std::string& text) {
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return std::string();
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string trimTrailingSlash(std::string text) {
  while (text.size() > 1 && text.back() == '/') text.pop_back();
  return text;
}

std::string replaceAll(std::string text, const std::string& from, const std::string& to) {
  if (from.empty()) return text;
  size_t pos = 0;
  while ((pos = text.find(from, pos)) != std::string::npos) {
    text.replace(pos, from.size(), to);
    pos += to.size();
  }
  return text;
}

bool endsWith(const std::string& text, const std::string& suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isAllDigit(const std::string& text) {
  if (text.empty()) return false;
  return std::all_of(text.begin(), text.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; });
}

bool isAllAlpha(const std::string& text) {
  if (text.empty()) return false;
  return std::all_of(text.begin(), text.end(),
                     [](unsigned char c) { return std::isalpha(c) != 0; });
}

}  // namespace utils
}  // namespace vehicle
