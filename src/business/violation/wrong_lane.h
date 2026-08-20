// WRONG_LANE — đi sai làn.
//
// Luật: zone tên `<CLASS>_..._LANE` liệt kê loại xe ĐƯỢC PHÉP. Xe có anchor rơi
// vào một zone LANE mà loại của nó không nằm trong danh sách cho phép → vi phạm.
// Áp dụng cho mọi loại xe; chỉ xét xe đã (hoặc đang) ở trong zone PLATE.
#pragma once

#include <json/json.h>

#include <set>
#include <string>
#include <vector>

#include "business/violation/zone_geometry.h"
#include "common/config.h"
#include "common/types.h"

namespace vehicle {
namespace business {
namespace violation {

// Zone LANE đã quy về pixel của frame hiện tại.
struct LanePolygon {
  std::vector<Point> polygon;
  std::set<int> allowed_classes;  // loại xe được phép ở làn này
  std::string zone_name;
};

// Key ảnh bằng chứng trong kho snapshot của probe. Ảnh phải là frame ĐANG vi
// phạm nên không tái tạo được ở frame sau → giữ riêng, không bị dọn theo biển.
constexpr const char* kWrongLaneSnapshotKey = "__WRONG_LANE__";

// Zone nào là zone LANE hợp lệ (tên khớp `*_LANE` và liệt kê ít nhất 1 loại xe).
bool isLaneZone(const Zone& zone);

// Lọc + quy các zone LANE của camera về pixel của frame.
std::vector<LanePolygon> laneZones(const std::vector<Zone>& zones, const FrameScale& scale);

// Làn mà xe loại `vehicle_cls` đứng sai; nullptr nếu không vi phạm.
// Trả con trỏ vào `lanes` — caller giữ `lanes` sống trong lúc dùng.
const LanePolygon* findViolatedLane(const Point& anchor, int vehicle_cls,
                                    const std::vector<LanePolygon>& lanes);

// Xe có đủ điều kiện xét WRONG_LANE ở frame này không (cổng zone PLATE).
bool shouldEvaluateWrongLane(const LaneViolationConfig& config, bool in_plate_zone,
                             bool ever_entered_plate_zone);

// Đủ ngưỡng để bắn event chưa (chưa xét cấu hình VMS).
bool wrongLaneMeetsThreshold(const LaneViolationConfig& config, int hits);

// `violation_evidence` của payload pub_event.
Json::Value buildWrongLaneEvidence(const std::string& lane_zone);

}  // namespace violation
}  // namespace business
}  // namespace vehicle
