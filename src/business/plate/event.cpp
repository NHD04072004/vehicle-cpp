#include "business/plate/event.h"

#include <memory>
#include <sstream>

namespace vehicle {
namespace business {
namespace plate {

bool mapVehicleType(int cls, std::string* out_text) {
  switch (cls) {
    case kClassCar: *out_text = "Ô TÔ"; return true;
    case kClassMotorbike: *out_text = "XE MÁY"; return true;
    case kClassTruck: *out_text = "XE TẢI"; return true;
    case kClassBus: *out_text = "XE KHÁCH"; return true;
    default: out_text->clear(); return false;
  }
}

Json::Value buildVehicleEvent(const EventParams& params) {
  Json::Value license_plate(Json::objectValue);
  license_plate["text"] =
      params.plate_text.empty() ? Json::Value() : Json::Value(params.plate_text);
  license_plate["status"] = params.plate_text.empty() ? "NOT_DETECTED" : "DETECTED";
  license_plate["plate_color"] =
      params.plate_color.empty() ? Json::Value() : Json::Value(params.plate_color);

  std::string vehicle_type;
  Json::Value vehicle_node(Json::objectValue);
  vehicle_node["license_plate"] = license_plate;
  vehicle_node["vehicle_type"] =
      mapVehicleType(params.vehicle_cls, &vehicle_type) ? Json::Value(vehicle_type)
                                                        : Json::Value();
  vehicle_node["car_type"] = Json::Value();
  vehicle_node["manufacturer"] = Json::Value();
  vehicle_node["color"] = Json::Value();
  vehicle_node["total_number_of_seats"] = Json::Value();

  Json::Value payload(Json::objectValue);
  payload["direction"] = Json::Value();  // luôn null — không có line crossing
  payload["vehicle"] = vehicle_node;

  Json::Value event(Json::objectValue);
  event["ai_modules"] = params.ai_modules;
  event["camera_id"] = params.camera_id;
  event["event_time"] = params.event_time;
  event["entity_type"] = "VEHICLE";
  event["entity_id"] = "vehicle_" + std::to_string(params.track_id);
  event["payload"] = payload;
  event["snapshot_url"] = params.snapshot_url;
  event["snapshot_base64"] = params.snapshot_plate_key;
  return event;
}

Json::Value buildBboxPayload(const std::string& camera_code, const std::string& ai_modules,
                             double timestamp, const std::vector<Detection>& detections) {
  Json::Value items(Json::arrayValue);
  for (const Detection& det : detections) {
    Json::Value bbox(Json::arrayValue);
    bbox.append(det.bbox_norm.x1);
    bbox.append(det.bbox_norm.y1);
    bbox.append(det.bbox_norm.x2);
    bbox.append(det.bbox_norm.y2);

    Json::Value item(Json::objectValue);
    item["id"] = det.id;
    item["class"] = det.cls;
    item["confidence"] = det.confidence;
    item["bbox"] = bbox;
    item["label"] = det.label;
    item["color"] = det.color;
    items.append(item);
  }

  Json::Value payload(Json::objectValue);
  payload["camera_code"] = camera_code;
  payload["ai_modules"] = ai_modules;
  payload["timestamp"] = timestamp;
  payload["detections"] = items;
  return payload;
}

std::string toCompactJson(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  builder["emitUTF8"] = true;
  std::ostringstream out;
  const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(value, &out);
  return out.str();
}

}  // namespace plate
}  // namespace business
}  // namespace vehicle
