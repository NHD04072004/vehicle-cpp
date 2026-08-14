// Geometry helper cho probe: anchor, point-in-polygon, expand ROI, IoU.
#pragma once

#include <vector>

#include "common/types.h"

namespace vehicle {
namespace utils {

// Điểm test của bbox xe: đáy-giữa, nhích lên `bottom_ratio` * chiều cao.
Point anchorPoint(const BoundingBox& box, double bottom_ratio);

// Ray casting; polygon < 3 điểm → false.
bool pointInPolygon(const Point& p, const std::vector<Point>& polygon);

// Nới bbox thêm `ratio` tổng (chia đều 2 cạnh), clamp trong [0,0,max_w,max_h].
BoundingBox expandBox(const BoundingBox& box, double ratio, double max_w, double max_h);

BoundingBox clampBox(const BoundingBox& box, double max_w, double max_h);

double iou(const BoundingBox& a, const BoundingBox& b);

// Chuẩn hoá bbox pixel → 0..1 theo kích thước frame.
BoundingBox normalizeBox(const BoundingBox& box, double frame_w, double frame_h);

// Zone lưu toạ độ 0..1 → scale sang pixel.
std::vector<Point> scaleZone(const std::vector<Point>& points, double frame_w, double frame_h);

}  // namespace utils
}  // namespace vehicle
