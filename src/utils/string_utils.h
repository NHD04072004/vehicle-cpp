#pragma once

#include <string>

namespace vehicle {
namespace utils {

std::string toUpper(std::string text);
std::string trim(const std::string& text);
std::string trimTrailingSlash(std::string text);
std::string replaceAll(std::string text, const std::string& from, const std::string& to);
bool endsWith(const std::string& text, const std::string& suffix);
bool isAllDigit(const std::string& text);
bool isAllAlpha(const std::string& text);

}  // namespace utils
}  // namespace vehicle
