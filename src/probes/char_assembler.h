// Ghép ký tự SGIE2 thành chuỗi biển (PIPELINE.md §4.3).
#pragma once

#include <string>
#include <vector>

#include "common/types.h"

namespace vehicle {
namespace probes {

struct CharBox {
  std::string text;
  double confidence = 0.0;
  BoundingBox box;
};

struct AssembleOptions {
  double iou_dedup = 0.40;
  double square_ratio = 0.50;
};

// Dedup box chồng, tách 1/2 dòng, ghép trái→phải (và trên→dưới nếu 2 dòng).
// Biển chụp xiên: khử nghiêng bằng đường hồi quy qua tâm ký tự trước khi xét dòng.
PlateReading assemblePlateText(std::vector<CharBox> boxes, const AssembleOptions& options);

}  // namespace probes
}  // namespace vehicle
