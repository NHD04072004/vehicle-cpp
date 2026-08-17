// Map loại xe + build payload pub_event — khớp plate_rules/event.py.
#pragma once

#include <json/json.h>

#include <string>
#include <vector>

#include "common/types.h"

namespace vehicle {
namespace business {
namespace plate {

bool mapVehicleType(int cls, std::string* out_text);

struct EventParams {
  std::string camera_id;
  uint64_t track_id = 0;
  std::string plate_text;
  int vehicle_cls = -1;
  std::string snapshot_url;        // object_key ảnh full-frame
  std::string snapshot_plate_key;  // object_key ảnh crop biển
  std::string event_time;          // UTC ISO
  std::string ai_modules = "PLATE";
  std::string plate_color;         // rỗng → null
  std::string direction;           // rỗng → null; violation mặc định "IN"
};

Json::Value buildVehicleEvent(const EventParams& params);

Json::Value buildViolationEvent(const EventParams& params,
                                const std::string& violation_type_code,
                                const Json::Value& evidence);

// Payload `pub_bbox` realtime.
Json::Value buildBboxPayload(const std::string& camera_code, const std::string& ai_modules,
                             double timestamp, const std::vector<Detection>& detections);

std::string toCompactJson(const Json::Value& value);

}  // namespace plate
}  // namespace business
}  // namespace vehicle
