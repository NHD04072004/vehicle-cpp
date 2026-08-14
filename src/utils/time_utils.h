#pragma once

#include <string>

namespace vehicle {
namespace utils {

// Giây (float) từ đồng hồ đơn điệu — dùng cho lifecycle track.
double monotonicSeconds();

// Epoch giây (float) — dùng cho field `timestamp` của bbox payload.
double epochSeconds();

// Thời điểm hiện tại dạng UTC ISO 8601 (vd. 2026-08-03T04:05:06.123456+00:00).
std::string utcIso8601();

}  // namespace utils
}  // namespace vehicle
