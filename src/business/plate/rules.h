// Pure plate rules — khớp plate_rules/{constants,validate,fuse,normalize,send_mode}.py.
#pragma once

#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "common/types.h"

namespace vehicle {
namespace business {
namespace plate {

// --- Nghiệp vụ độc lập trên cùng 1 track ------------------------------------
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
  int detail = 0;                  // theo kind: kNoHelmet = số người không mũ tối đa
  double first_hit_at_s = 0.0;     // mốc hit đầu tiên (0 = chưa)
  double paired_at_s = 0.0;        // mốc đủ cả biển lẫn vi phạm (0 = chưa)
  double last_attempt_s = 0.0;     // lần thử emit gần nhất — retry throttle riêng
  bool needs_snapshot = false;     // đang chờ probe chụp ảnh bằng chứng
  bool has_snapshot = false;       // đã có ảnh đúng lúc vi phạm
  bool pushed = false;             // đã dựng job, đang bay tới worker
  bool posted = false;             // đã publish xong / bị chặn vĩnh viễn

  bool active() const { return hits > 0; }
};

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

// Ngưỡng dịch chuyển anchor (px) giữa 2 frame để coi là xe ĐANG di chuyển.
// Dưới ngưỡng = jitter bbox của xe đứng yên → không tính cắt vạch.
constexpr double kMinWrongWayMovePx = 2.0;

// Số anchor giữ lại để tính vector hướng chuyển động (3-4 bbox là đủ mượt).
constexpr size_t kMotionHistoryLen = 4;
// Cần ít nhất ngần này anchor mới dám kết luận hướng.
constexpr size_t kMinMotionHistoryLen = 3;
// Tổng quãng đường (px) của cả history phải vượt ngưỡng này thì hướng mới đáng tin.
constexpr double kMinMotionLenPx = 6.0;
// Xe ĐỨNG YÊN: mọi anchor trong history nằm gọn trong bán kính này quanh tâm.
// Bbox xe đỗ vẫn nhấp nháy vài px mỗi frame nhưng không trôi đi đâu, nên tán xạ
// quanh tâm mới là dấu hiệu phân biệt với xe đang bò chậm.
constexpr double kStationaryRadiusPx = 4.0;
// Xe đứng yên: quãng đường tịnh (đầu → cuối history) không vượt ngưỡng này.
constexpr double kStationaryNetPx = 5.0;

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
