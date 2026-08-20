#include "business/violation/zone_geometry.h"

namespace vehicle {
namespace business {
namespace violation {

Point scalePoint(const Point& p, bool normalized, const FrameScale& scale) {
  if (normalized) return {p.x * scale.frame_w, p.y * scale.frame_h};
  return {p.x * scale.scaleX(), p.y * scale.scaleY()};
}

std::vector<Point> scalePolygon(const std::vector<Point>& points, bool normalized,
                                const FrameScale& scale) {
  // Hệ số tính 1 lần ngoài vòng lặp — scalePoint tự tra lại mỗi điểm sẽ chia
  // thừa 2 phép cho mỗi đỉnh, mà hàm này chạy cho mọi zone ở mọi frame.
  const double sx = normalized ? scale.frame_w : scale.scaleX();
  const double sy = normalized ? scale.frame_h : scale.scaleY();
  std::vector<Point> out;
  out.reserve(points.size());
  for (const Point& p : points) out.push_back({p.x * sx, p.y * sy});
  return out;
}

}  // namespace violation
}  // namespace business
}  // namespace vehicle
