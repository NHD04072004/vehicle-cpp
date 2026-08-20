// WRONG_WAY — đi ngược chiều.
//
// Luật: line tên REVERSE_DIRECTION kèm direction_vector (chiều CẤM — mũi tên vẽ
// trên vạch). Xe vi phạm khi đoạn di chuyển giữa 2 frame CẮT QUA vạch và hướng
// đi lệch <= `max_angle_deg` so với mũi tên đó.
//
// Không gate theo zone PLATE: anchor phải cập nhật ở MỌI frame thì mới có vị trí
// frame trước để so; bỏ frame là đoạn di chuyển bị đứt và lần cắt vạch không
// được ghi nhận. Line cũng có thể nằm ngoài zone PLATE.
#pragma once

#include <json/json.h>

#include <string>
#include <vector>

#include "business/violation/zone_geometry.h"
#include "common/config.h"
#include "common/types.h"

namespace vehicle {
namespace business {
namespace violation {

// Line REVERSE_DIRECTION đã quy về pixel của frame hiện tại.
struct WrongWayLine {
  Point a;
  Point b;
  Point direction;  // chiều CẤM (mũi tên): đi cùng chiều này = vi phạm
  std::string name;
};

// Key ảnh bằng chứng trong kho snapshot của probe (xem wrong_lane.h).
constexpr const char* kWrongWaySnapshotKey = "__WRONG_WAY__";

// --- Ngưỡng chuyển động -----------------------------------------------------
// Ngưỡng dịch chuyển anchor (px) giữa 2 frame để coi là xe ĐANG di chuyển.
// Dưới ngưỡng = jitter bbox của xe đứng yên → không tính cắt vạch.
constexpr double kMinMovePx = 2.0;
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

// Lịch sử anchor của 1 track (cũ → mới, tối đa kMotionHistoryLen).
// Vector cố định trần 4 phần tử — hướng chuyển động mượt hơn hiệu 2 frame liên
// tiếp, vốn quá nhạy với jitter bbox.
class MotionHistory {
 public:
  void push(const Point& p);

  bool hasLast() const { return has_last_; }
  const Point& last() const { return last_; }

  // Hướng chuyển động hiện tại; {0,0} nếu chưa đủ điểm hoặc xe đứng yên.
  Point direction() const;
  // Xe đang đứng yên: anchor chỉ dao động quanh 1 tâm (sai số detection), không
  // trôi theo hướng nào. Chưa đủ history cũng coi là chưa đủ cơ sở → true.
  bool isStationary() const;
  // Đoạn di chuyển frame-trước → frame-này có đủ dài để xét cắt vạch không.
  static bool movedEnough(const Point& prev, const Point& now);

 private:
  std::vector<Point> anchors_;
  Point last_;
  bool has_last_ = false;
};

// Kết quả xét 1 frame.
struct CrossResult {
  const WrongWayLine* line = nullptr;  // vạch bị cắt ngược chiều; nullptr = không vi phạm
  double angle_deg = 0.0;              // góc lệch so với mũi tên cấm (log)
  // Có cắt vạch nhưng góc quá lớn → không phải vi phạm; giữ lại để log rõ lý do.
  const WrongWayLine* rejected_line = nullptr;
  double rejected_angle_deg = 0.0;
};

// Line nào là line REVERSE_DIRECTION hợp lệ (đúng tên, có direction, >= 2 điểm).
bool isWrongWayLine(const Line& line);

// Lọc + quy các line REVERSE_DIRECTION của camera về pixel của frame.
// `out_missing_direction`: tên line đầu tiên đúng tên nhưng thiếu direction —
// caller cảnh báo 1 lần/version thay vì mỗi frame.
std::vector<WrongWayLine> wrongWayLines(const std::vector<Line>& lines,
                                        const FrameScale& scale,
                                        std::string* out_missing_direction = nullptr);

// Xét đoạn `prev`→`now` với lịch sử `history` (đã push `now`).
// Trả line bị cắt ngược chiều, hoặc rejected_line nếu cắt nhưng góc quá lớn.
CrossResult detectCrossing(const Point& prev, const Point& now, const MotionHistory& history,
                           const std::vector<WrongWayLine>& lines, double max_angle_deg);

// Đủ ngưỡng để bắn event chưa (chưa xét cấu hình VMS).
bool wrongWayMeetsThreshold(const WrongWayViolationConfig& config, int hits);

// `violation_evidence` của payload pub_event.
Json::Value buildWrongWayEvidence(const std::string& line_name);

}  // namespace violation
}  // namespace business
}  // namespace vehicle
