// Nghiệp vụ độc lập trên cùng 1 track: định danh kind, bitmask, vòng đời.
//
// Tách khỏi business/plate/rules.h vì đây là khái niệm VI PHẠM, không phải luật
// biển số. plate/rules.h re-export lại các tên này để mã cũ không phải đổi.
#pragma once

#include <cstddef>
#include <cstdint>

namespace vehicle {
namespace business {
namespace violation {

// Mỗi kind có vòng đời riêng: bộ đếm riêng, cờ đã-bắn riêng, timeout riêng.
// Điều kiện CHUNG duy nhất còn lại là hasFinalPlate() — vi phạm không có biển
// số thì không xử phạt được.
//
// Giá trị dày đặc 0..kCount-1 để dùng trực tiếp làm index mảng.
enum class EventKind : uint8_t {
  kPlate = 0,   // event phương tiện (không phải vi phạm)
  kNoHelmet,
  kWrongLane,
  kWrongWay,
  kCount        // sentinel — KHÔNG phải kind hợp lệ
};

constexpr size_t kEventKindCount = static_cast<size_t>(EventKind::kCount);
constexpr size_t kindIndex(EventKind k) { return static_cast<size_t>(k); }

// Tập kind, 1 bit mỗi kind. Dùng cho "kind nào sẵn sàng bắn", "kind nào đã
// publish xong" — không cấp phát, so sánh/hợp/giao bằng 1 lệnh.
using EventKindMask = uint8_t;
constexpr EventKindMask kindBit(EventKind k) {
  return static_cast<EventKindMask>(1u << kindIndex(k));
}
constexpr bool maskHas(EventKindMask m, EventKind k) { return (m & kindBit(k)) != 0; }

// Mã vi phạm VMS; kPlate trả nullptr (không phải vi phạm).
const char* eventKindCode(EventKind k);
// Tên đọc được cho log.
const char* eventKindName(EventKind k);

// Vòng đời 1 nghiệp vụ trên 1 track. POD, không std::string, không cấp phát —
// nằm inline trong TrackPlateState dưới dạng std::array<ViolationState, 4>.
struct ViolationState {
  int hits = 0;                    // số lần ghi nhận (frame vi phạm)
  int detail = 0;                  // theo kind: kNoHelmet = số người không mũ
  double first_hit_at_s = 0.0;     // mốc hit đầu tiên (0 = chưa)
  double paired_at_s = 0.0;        // mốc đủ cả biển lẫn vi phạm (0 = chưa)
  double last_attempt_s = 0.0;     // lần thử emit gần nhất — retry throttle riêng
  bool needs_snapshot = false;     // đang chờ probe chụp ảnh bằng chứng
  bool has_snapshot = false;       // đã có ảnh đúng lúc vi phạm
  bool pushed = false;             // đã dựng job, đang bay tới worker
  bool posted = false;             // đã publish xong / bị chặn vĩnh viễn

  bool active() const { return hits > 0; }
};

}  // namespace violation
}  // namespace business
}  // namespace vehicle
