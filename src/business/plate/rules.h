// Pure plate rules — khớp plate_rules/{constants,validate,fuse,normalize,send_mode}.py.
#pragma once

#include <functional>
#include <set>
#include <string>
#include <vector>

#include "common/types.h"

namespace vehicle {
namespace business {
namespace plate {

const std::set<std::string>& digitCarPrefixes();
const std::set<std::string>& alphaArmyPrefixes();

// Mặc định lấy từ resources/config/config.yaml → plate.plate_style.
const std::vector<std::string>& defaultPlateStyles();

constexpr int kDefaultMaxRecognizeTimes = 5;
constexpr int kDefaultSendMode = 2;

constexpr const char* kEmptyPlate = "EMPTY";
constexpr const char* kUnknownPlate = "UNKOWN";  // typo cố định (tương thích VMS)
constexpr const char* kUnkSuffix = "_unk";

// Ngưỡng miss / cleanup (giây) — PIPELINE.md §5.1.
constexpr double kMissTrackIdleS = 1.0;
constexpr double kForceDeleteIdleS = 10.0;
constexpr double kForceDeleteAgeS = 120.0;

constexpr size_t kDedupCacheSize = 50;

// N = digit, C = alpha; độ dài phải khớp.
bool matchesPlateStyle(const std::string& plate, const std::string& style);

// Khớp ít nhất 1 style VÀ pass rule 2 ký tự đầu (DIGIT_CAR / ALPHA_ARMY).
bool isPlateValid(const std::string& plate, const std::vector<std::string>& styles);
bool isPlateValid(const std::string& plate);  // dùng defaultPlateStyles()

using CharSequence = std::vector<CharReading>;
using PlateValidator = std::function<bool(const std::string&)>;

// Vote ký tự theo từng vị trí giữa các reading cùng độ dài.
// Ưu tiên số phiếu; hoà → tổng confidence. Ném std::invalid_argument nếu
// các reading khác độ dài.
std::string fuseChars(const std::vector<CharSequence>& readings);

// Gom reading theo độ dài, fuse từng nhóm, xếp hạng: valid → số reading → mean conf.
std::string selectBestPlate(const std::vector<CharSequence>& list_plate_chars,
                            const PlateValidator& is_valid);

// rỗng/EMPTY → "UNKOWN"; sai style → "{text}_unk"; hợp lệ → text viết hoa.
std::string normalizePlateForEmit(const std::string& plate,
                                  const std::vector<std::string>& styles);
std::string normalizePlateForEmit(const std::string& plate);

// send_mode 1: chỉ biển tốt; 2: bắn tất cả (kể cả UNKOWN/_unk); khác: không bắn.
bool shouldEmitPlate(const std::string& plate, int send_mode = kDefaultSendMode);

}  // namespace plate
}  // namespace business
}  // namespace vehicle
