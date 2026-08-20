// Quy toạ độ ROI của VMS về không gian pixel của frame đang xử lý.
//
// VMS gửi 2 dạng: normalized (0..1) và pixel theo độ phân giải NGUỒN. Cùng một
// phép quy đổi này trước đây lặp 3 lần trong plate_probe (zone PLATE, zone LANE,
// line REVERSE_DIRECTION) — gom về đây để mọi nghiệp vụ dùng chung một luật.
#pragma once

#include <vector>

#include "common/types.h"

namespace vehicle {
namespace business {
namespace violation {

// Kích thước frame đích + độ phân giải nguồn của toạ độ pixel chưa chuẩn hoá.
struct FrameScale {
  double frame_w = 0.0;
  double frame_h = 0.0;
  double source_w = 0.0;
  double source_h = 0.0;

  // Hệ số quy pixel-nguồn → pixel-frame; 1.0 khi thiếu thông tin nguồn.
  double scaleX() const { return (source_w > 0.0) ? frame_w / source_w : 1.0; }
  double scaleY() const { return (source_h > 0.0) ? frame_h / source_h : 1.0; }
};

// Một điểm ROI → pixel của frame.
Point scalePoint(const Point& p, bool normalized, const FrameScale& scale);

// Polygon ROI → pixel của frame.
std::vector<Point> scalePolygon(const std::vector<Point>& points, bool normalized,
                                const FrameScale& scale);

}  // namespace violation
}  // namespace business
}  // namespace vehicle
