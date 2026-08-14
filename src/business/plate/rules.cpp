#include "business/plate/rules.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

#include "utils/string_utils.h"

namespace vehicle {
namespace business {
namespace plate {
namespace {

// Rule 2 ký tự đầu: prefix số ∈ DIGIT_CAR → invalid; prefix chữ phải ∈ ALPHA_ARMY.
bool checkAlphaDigit(const std::string& plate) {
  if (plate.size() < 2) return true;
  const std::string prefix = utils::toUpper(plate.substr(0, 2));
  if (utils::isAllDigit(prefix)) return digitCarPrefixes().count(prefix) == 0;
  if (utils::isAllAlpha(prefix)) return alphaArmyPrefixes().count(prefix) != 0;
  return true;
}

struct CharStat {
  std::string text;
  int votes = 0;
  double total_conf = 0.0;
};

struct ScoredGroup {
  bool valid = false;
  size_t count = 0;
  double mean_conf = 0.0;
  std::string fused;
};

}  // namespace

const std::set<std::string>& digitCarPrefixes() {
  static const std::set<std::string> kPrefixes = {
      "00", "01", "02", "03", "04", "05", "06", "07", "08",
      "09", "10", "44", "45", "46", "87", "91", "96",
  };
  return kPrefixes;
}

const std::set<std::string>& alphaArmyPrefixes() {
  static const std::set<std::string> kPrefixes = {
      "AA", "AB", "AC", "AD", "AM", "AN", "AT", "AV", "BB", "BC", "BH", "BK",
      "BL", "BP", "BT", "CA", "CB", "CC", "CD", "CH", "CM", "CN", "CP", "CT",
      "CV", "HA", "HB", "HC", "HD", "HE", "HH", "HN", "HQ", "HT", "KA", "KB",
      "KC", "KD", "KK", "KP", "KT", "KV", "PA", "PG", "PK", "PM", "PQ", "PX",
      "QA", "QB", "QC", "QH", "QM", "TA", "TC", "TH", "TK", "TM", "TN", "TT",
      "VT",
  };
  return kPrefixes;
}

const std::vector<std::string>& defaultPlateStyles() {
  static const std::vector<std::string> kStyles = {
      "NNCNNNNN", "NNCNNNNNN", "CCNNNN",    "NNCNNNN",   "NNCCNNNN",
      "NNNNNCC",  "NNNNNCN",   "NNNNNCCNN", "NNCCNNNNN", "NNCCNNNNNN",
  };
  return kStyles;
}

bool matchesPlateStyle(const std::string& plate, const std::string& style) {
  if (plate.size() != style.size()) return false;
  for (size_t i = 0; i < plate.size(); ++i) {
    const unsigned char ch = static_cast<unsigned char>(plate[i]);
    if (style[i] == 'N' && std::isdigit(ch) == 0) return false;
    if (style[i] == 'C' && std::isalpha(ch) == 0) return false;
  }
  return true;
}

bool isPlateValid(const std::string& plate, const std::vector<std::string>& styles) {
  const std::string text = utils::toUpper(plate);
  if (text.empty()) return false;
  bool matched = false;
  for (const std::string& style : styles) {
    if (matchesPlateStyle(text, style)) {
      matched = true;
      break;
    }
  }
  if (!matched) return false;
  return checkAlphaDigit(text);
}

bool isPlateValid(const std::string& plate) {
  return isPlateValid(plate, defaultPlateStyles());
}

std::string fuseChars(const std::vector<CharSequence>& readings) {
  if (readings.empty()) return std::string();
  const size_t length = readings.front().size();
  for (const CharSequence& reading : readings) {
    if (reading.size() != length)
      throw std::invalid_argument("fuseChars requires all readings same length");
  }

  std::string fused;
  fused.reserve(length);
  for (size_t i = 0; i < length; ++i) {
    // Giữ thứ tự xuất hiện để hoà phiếu → chọn ký tự thấy trước (khớp Python dict).
    std::vector<CharStat> stats;
    for (const CharSequence& reading : readings) {
      const std::string text = utils::toUpper(reading[i].text);
      auto it = std::find_if(stats.begin(), stats.end(),
                             [&text](const CharStat& s) { return s.text == text; });
      if (it == stats.end()) {
        stats.push_back({text, 1, reading[i].confidence});
      } else {
        it->votes += 1;
        it->total_conf += reading[i].confidence;
      }
    }
    const CharStat* best = &stats.front();
    for (const CharStat& stat : stats) {
      if (stat.votes > best->votes ||
          (stat.votes == best->votes && stat.total_conf > best->total_conf)) {
        best = &stat;
      }
    }
    fused += best->text;
  }
  return fused;
}

std::string selectBestPlate(const std::vector<CharSequence>& list_plate_chars,
                            const PlateValidator& is_valid) {
  // Nhóm theo độ dài, giữ thứ tự nhóm xuất hiện lần đầu.
  std::vector<std::pair<size_t, std::vector<CharSequence>>> groups;
  for (const CharSequence& chars : list_plate_chars) {
    if (chars.empty()) continue;
    auto it = std::find_if(groups.begin(), groups.end(),
                           [&chars](const auto& g) { return g.first == chars.size(); });
    if (it == groups.end())
      groups.push_back({chars.size(), {chars}});
    else
      it->second.push_back(chars);
  }
  if (groups.empty()) return std::string();

  std::vector<ScoredGroup> scored;
  scored.reserve(groups.size());
  for (const auto& group : groups) {
    const std::string fused = fuseChars(group.second);
    double total_conf = 0.0;
    size_t total_char = 0;
    for (const CharSequence& chars : group.second) {
      for (const CharReading& ch : chars) total_conf += ch.confidence;
      total_char += chars.size();
    }
    const double mean_conf = total_char > 0 ? total_conf / static_cast<double>(total_char) : 0.0;
    scored.push_back({is_valid ? is_valid(fused) : false, group.second.size(), mean_conf, fused});
  }

  std::stable_sort(scored.begin(), scored.end(), [](const ScoredGroup& a, const ScoredGroup& b) {
    if (a.valid != b.valid) return a.valid > b.valid;
    if (a.count != b.count) return a.count > b.count;
    return a.mean_conf > b.mean_conf;
  });
  return scored.front().fused;
}

std::string normalizePlateForEmit(const std::string& plate,
                                  const std::vector<std::string>& styles) {
  const std::string text = utils::toUpper(utils::trim(plate));
  if (text.empty() || text == kEmptyPlate) return kUnknownPlate;
  if (!isPlateValid(text, styles)) return text + kUnkSuffix;
  return text;
}

std::string normalizePlateForEmit(const std::string& plate) {
  return normalizePlateForEmit(plate, defaultPlateStyles());
}

bool shouldEmitPlate(const std::string& plate, int send_mode) {
  if (send_mode == 2) return true;
  if (send_mode == 1) {
    if (plate == kUnknownPlate) return false;
    if (utils::endsWith(plate, kUnkSuffix)) return false;
    return !plate.empty();
  }
  return false;
}

}  // namespace plate
}  // namespace business
}  // namespace vehicle
