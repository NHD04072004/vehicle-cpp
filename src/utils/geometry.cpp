#include "utils/geometry.h"

#include <algorithm>
#include <cmath>

namespace vehicle {
namespace utils {

Point anchorPoint(const BoundingBox& box, double bottom_ratio) {
  Point p;
  p.x = (box.x1 + box.x2) / 2.0;
  p.y = box.y2 - box.height() * bottom_ratio;
  return p;
}

bool pointInPolygon(const Point& p, const std::vector<Point>& polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Point& a = polygon[i];
    const Point& b = polygon[j];
    const bool straddles = (a.y > p.y) != (b.y > p.y);
    if (!straddles) continue;
    const double dy = b.y - a.y;
    if (dy == 0.0) continue;
    const double x_cross = (b.x - a.x) * (p.y - a.y) / dy + a.x;
    if (p.x < x_cross) inside = !inside;
  }
  return inside;
}

BoundingBox clampBox(const BoundingBox& box, double max_w, double max_h) {
  BoundingBox out;
  out.x1 = std::max(0.0, std::min(box.x1, max_w));
  out.y1 = std::max(0.0, std::min(box.y1, max_h));
  out.x2 = std::max(0.0, std::min(box.x2, max_w));
  out.y2 = std::max(0.0, std::min(box.y2, max_h));
  return out;
}

BoundingBox expandBox(const BoundingBox& box, double ratio, double max_w, double max_h) {
  const double dx = box.width() * ratio / 2.0;
  const double dy = box.height() * ratio / 2.0;
  BoundingBox out;
  out.x1 = box.x1 - dx;
  out.y1 = box.y1 - dy;
  out.x2 = box.x2 + dx;
  out.y2 = box.y2 + dy;
  return clampBox(out, max_w, max_h);
}

double iou(const BoundingBox& a, const BoundingBox& b) {
  const double x1 = std::max(a.x1, b.x1);
  const double y1 = std::max(a.y1, b.y1);
  const double x2 = std::min(a.x2, b.x2);
  const double y2 = std::min(a.y2, b.y2);
  const double inter_w = x2 - x1;
  const double inter_h = y2 - y1;
  if (inter_w <= 0.0 || inter_h <= 0.0) return 0.0;
  const double inter = inter_w * inter_h;
  const double area_a = std::max(0.0, a.width()) * std::max(0.0, a.height());
  const double area_b = std::max(0.0, b.width()) * std::max(0.0, b.height());
  const double denom = area_a + area_b - inter;
  if (denom <= 0.0) return 0.0;
  return inter / denom;
}

BoundingBox normalizeBox(const BoundingBox& box, double frame_w, double frame_h) {
  BoundingBox out;
  if (frame_w <= 0.0 || frame_h <= 0.0) return out;
  out.x1 = box.x1 / frame_w;
  out.y1 = box.y1 / frame_h;
  out.x2 = box.x2 / frame_w;
  out.y2 = box.y2 / frame_h;
  return out;
}

namespace {

// >0 trái, <0 phải, 0 thẳng hàng.
double cross(const Point& o, const Point& a, const Point& b) {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool onSegment(const Point& o, const Point& a, const Point& p) {
  return std::min(o.x, a.x) <= p.x && p.x <= std::max(o.x, a.x) &&
         std::min(o.y, a.y) <= p.y && p.y <= std::max(o.y, a.y);
}

}  // namespace

bool segmentsIntersect(const Point& p1, const Point& p2, const Point& a, const Point& b) {
  const double d1 = cross(a, b, p1);
  const double d2 = cross(a, b, p2);
  const double d3 = cross(p1, p2, a);
  const double d4 = cross(p1, p2, b);

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
      ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
    return true;

  // Trường hợp suy biến: điểm nằm ngay trên đoạn kia.
  if (d1 == 0.0 && onSegment(a, b, p1)) return true;
  if (d2 == 0.0 && onSegment(a, b, p2)) return true;
  if (d3 == 0.0 && onSegment(p1, p2, a)) return true;
  if (d4 == 0.0 && onSegment(p1, p2, b)) return true;
  return false;
}

double angleBetweenDeg(const Point& v1, const Point& v2) {
  const double n1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
  const double n2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
  if (n1 <= 1e-9 || n2 <= 1e-9) return -1.0;
  double cos_a = (v1.x * v2.x + v1.y * v2.y) / (n1 * n2);
  cos_a = std::max(-1.0, std::min(1.0, cos_a));  // chặn sai số làm NaN acos
  return std::acos(cos_a) * 180.0 / M_PI;
}

Point motionVector(const std::vector<Point>& track) {
  const size_t n = track.size();
  if (n < 2) return Point{0.0, 0.0};
  const double mean_i = (n - 1) / 2.0;
  double mean_x = 0.0, mean_y = 0.0;
  for (const Point& p : track) {
    mean_x += p.x;
    mean_y += p.y;
  }
  mean_x /= static_cast<double>(n);
  mean_y /= static_cast<double>(n);

  double sxx = 0.0, sx = 0.0, sy = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double di = static_cast<double>(i) - mean_i;
    sxx += di * di;
    sx += di * (track[i].x - mean_x);
    sy += di * (track[i].y - mean_y);
  }
  if (sxx <= 0.0) return Point{0.0, 0.0};
  // Nhân (n-1) để vector mang độ dài tổng quãng đường xấp xỉ, không phải mỗi frame.
  const double scale = static_cast<double>(n - 1);
  return Point{sx / sxx * scale, sy / sxx * scale};
}

}  // namespace utils
}  // namespace vehicle
