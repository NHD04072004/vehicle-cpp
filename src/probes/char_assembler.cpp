#include "probes/char_assembler.h"

#include <algorithm>
#include <cmath>

#include "utils/geometry.h"

namespace vehicle {
namespace probes {
namespace {

double centerY(const BoundingBox& box) { return (box.y1 + box.y2) / 2.0; }
double centerX(const BoundingBox& box) { return (box.x1 + box.x2) / 2.0; }

struct TrendLine {
  double slope = 0.0;
  double intercept = 0.0;

  double residual(const BoundingBox& box) const {
    return centerY(box) - (slope * centerX(box) + intercept);
  }
};

TrendLine fitTrend(const std::vector<CharBox>& boxes) {
  TrendLine trend;
  const double n = static_cast<double>(boxes.size());
  if (n < 2.0) {
    if (n == 1.0) trend.intercept = centerY(boxes.front().box);
    return trend;
  }

  double sum_x = 0.0, sum_y = 0.0;
  for (const CharBox& item : boxes) {
    sum_x += centerX(item.box);
    sum_y += centerY(item.box);
  }
  const double mean_x = sum_x / n;
  const double mean_y = sum_y / n;

  double cov = 0.0, var_x = 0.0;
  for (const CharBox& item : boxes) {
    const double dx = centerX(item.box) - mean_x;
    cov += dx * (centerY(item.box) - mean_y);
    var_x += dx * dx;
  }
  // Ký tự xếp gần như thẳng đứng → không ước lượng được độ nghiêng.
  if (var_x > 1e-6) trend.slope = cov / var_x;
  trend.intercept = mean_y - trend.slope * mean_x;
  return trend;
}

void appendSortedByX(const std::vector<CharBox>& line, PlateReading* out) {
  std::vector<CharBox> sorted = line;
  std::stable_sort(sorted.begin(), sorted.end(), [](const CharBox& a, const CharBox& b) {
    return centerX(a.box) < centerX(b.box);
  });
  for (const CharBox& item : sorted) {
    out->chars.push_back({item.text, item.confidence});
    out->raw += item.text;
  }
}

}  // namespace

PlateReading assemblePlateText(std::vector<CharBox> boxes, const AssembleOptions& options) {
  PlateReading reading;
  if (boxes.empty()) return reading;

  // Dedup: duyệt theo confidence giảm dần, bỏ box chồng IoU > ngưỡng.
  std::stable_sort(boxes.begin(), boxes.end(), [](const CharBox& a, const CharBox& b) {
    return a.confidence > b.confidence;
  });
  std::vector<CharBox> kept;
  for (const CharBox& candidate : boxes) {
    const bool overlaps = std::any_of(
        kept.begin(), kept.end(), [&candidate, &options](const CharBox& other) {
          return utils::iou(candidate.box, other.box) > options.iou_dedup;
        });
    if (!overlaps) kept.push_back(candidate);
  }
  if (kept.size() <= 1) {
    appendSortedByX(kept, &reading);
    return reading;
  }

  double sum_h = 0.0;
  for (const CharBox& item : kept) sum_h += item.box.height();
  const double avg_h = sum_h / static_cast<double>(kept.size());

  // So độ trải theo trục y SAU khi khử nghiêng: biển 1 dòng bị chụp xiên vẫn
  // bám sát 1 đường thẳng, còn biển vuông 2 dòng thì lệch hẳn khỏi đường đó.
  const TrendLine trend = fitTrend(kept);
  double min_residual = trend.residual(kept.front().box);
  double max_residual = min_residual;
  for (const CharBox& item : kept) {
    const double residual = trend.residual(item.box);
    min_residual = std::min(min_residual, residual);
    max_residual = std::max(max_residual, residual);
  }
  const double spread = max_residual - min_residual;
  const bool two_lines = avg_h > 0.0 && (spread / avg_h) > options.square_ratio;

  if (!two_lines) {
    appendSortedByX(kept, &reading);
    return reading;
  }

  const double split = (min_residual + max_residual) / 2.0;
  std::vector<CharBox> top;
  std::vector<CharBox> bottom;
  for (const CharBox& item : kept) {
    if (trend.residual(item.box) <= split)
      top.push_back(item);
    else
      bottom.push_back(item);
  }
  appendSortedByX(top, &reading);
  appendSortedByX(bottom, &reading);
  return reading;
}

}  // namespace probes
}  // namespace vehicle
