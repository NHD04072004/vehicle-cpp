#include "business/violation/no_helmet.h"

namespace vehicle {
namespace business {
namespace violation {

bool shouldEvaluateNoHelmet(const HelmetViolationConfig& config, int vehicle_cls,
                            bool in_plate_zone) {
  return config.enabled && in_plate_zone && vehicle_cls == kClassMotorbike;
}

bool isNoHelmetClass(const HelmetViolationConfig& config, int class_id) {
  return class_id == config.no_helmet_class_id;
}

bool noHelmetMeetsThreshold(const HelmetViolationConfig& config, int vehicle_cls, int hits) {
  if (!config.enabled) return false;
  if (vehicle_cls != kClassMotorbike) return false;
  return hits >= config.min_hits;
}

Json::Value buildNoHelmetEvidence() {
  Json::Value evidence(Json::objectValue);
  evidence["road_type"] = "highway";
  return evidence;
}

}  // namespace violation
}  // namespace business
}  // namespace vehicle
