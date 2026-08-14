#include "utils/geometry.h"

#include <algorithm>

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

std::vector<Point> scaleZone(const std::vector<Point>& points, double frame_w, double frame_h) {
  std::vector<Point> scaled;
  scaled.reserve(points.size());
  for (const Point& p : points) scaled.push_back({p.x * frame_w, p.y * frame_h});
  return scaled;
}

}  // namespace utils
}  // namespace vehicle
