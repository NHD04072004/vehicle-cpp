#include "business/violation/wrong_lane.h"

#include <utility>

#include "utils/geometry.h"

namespace vehicle {
namespace business {
namespace violation {

bool isLaneZone(const Zone& zone) { return !laneAllowedClasses(zone.name).empty(); }

std::vector<LanePolygon> laneZones(const std::vector<Zone>& zones, const FrameScale& scale) {
  std::vector<LanePolygon> lanes;
  for (const Zone& zone : zones) {
    std::set<int> allowed = laneAllowedClasses(zone.name);
    if (allowed.empty()) continue;  // không phải zone *_LANE hợp lệ

    LanePolygon lane;
    lane.allowed_classes = std::move(allowed);
    lane.zone_name = zone.name;
    lane.polygon = scalePolygon(zone.points, zone.normalized, scale);
    lanes.push_back(std::move(lane));
  }
  return lanes;
}

const LanePolygon* findViolatedLane(const Point& anchor, int vehicle_cls,
                                    const std::vector<LanePolygon>& lanes) {
  for (const LanePolygon& lane : lanes) {
    if (lane.allowed_classes.count(vehicle_cls) > 0) continue;  // đúng làn
    if (utils::pointInPolygon(anchor, lane.polygon)) return &lane;
  }
  return nullptr;
}

bool shouldEvaluateWrongLane(const LaneViolationConfig& config, bool in_plate_zone,
                             bool ever_entered_plate_zone) {
  return config.enabled && (in_plate_zone || ever_entered_plate_zone);
}

bool wrongLaneMeetsThreshold(const LaneViolationConfig& config, int hits) {
  return config.enabled && hits >= config.min_hits;
}

Json::Value buildWrongLaneEvidence(const std::string& lane_zone) {
  Json::Value evidence(Json::objectValue);
  evidence["lane_zone"] = lane_zone;
  return evidence;
}

}  // namespace violation
}  // namespace business
}  // namespace vehicle
