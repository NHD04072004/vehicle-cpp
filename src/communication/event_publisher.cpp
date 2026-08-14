#include "communication/event_publisher.h"

#include "common/logging.h"
#include "utils/latency.h"
#include "utils/time_utils.h"

namespace vehicle {
namespace comm {

EventPublisher::EventPublisher(const Config& config, MqttClient* mqtt, HttpUploader* uploader)
    : config_(config), mqtt_(mqtt), uploader_(uploader) {}

void EventPublisher::publishBbox(const std::string& camera_code,
                                 const std::vector<Detection>& detections) {
  if (mqtt_ == nullptr || !config_.pipeline().probe.publish_bbox) return;
  const Json::Value payload = business::plate::buildBboxPayload(
      camera_code, config_.aiModule(), utils::epochSeconds(), detections);
  const std::string json = business::plate::toCompactJson(payload);
  if (dry_run_) {
    LOG_DEBUG("dry-run bbox[%s]: %s", camera_code.c_str(), json.c_str());
    return;
  }
  mqtt_->publish(config_.bboxTopic(camera_code), json);
}

bool EventPublisher::publishPlateEvent(const Camera& camera, const PlateEmit& emit) {
  if (mqtt_ == nullptr || uploader_ == nullptr) return false;

  const double pub_start_s = utils::monotonicSeconds();
  const std::string base = "vehicle_" + std::to_string(emit.track_id);
  std::string full_key;
  std::string plate_key;
  if (dry_run_) {
    full_key = emit.full_frame.empty() ? "" : "dry-run/" + base + ".jpg";
    plate_key = emit.plate_crop.empty() ? "" : "dry-run/" + base + "_plate.jpg";
  }
  if (!dry_run_ && !emit.full_frame.empty())
    full_key = uploader_->upload(camera.id, "vehicle", emit.full_frame.data, base + ".jpg");
  if (!dry_run_ && !emit.plate_crop.empty())
    plate_key = uploader_->upload(camera.id, "plate", emit.plate_crop.data, base + "_plate.jpg");

  if (full_key.empty() && plate_key.empty()) {
    LOG_WARN("track %lu: upload snapshot thất bại — hoãn publish event",
             static_cast<unsigned long>(emit.track_id));
    utils::latencyLog("track %lu publish FAIL after upload %.0fms",
                      static_cast<unsigned long>(emit.track_id),
                      utils::msSince(pub_start_s));
    return false;
  }

  business::plate::EventParams params;
  params.camera_id = camera.id;
  params.track_id = emit.track_id;
  params.plate_text = emit.plate;
  params.vehicle_cls = emit.vehicle_cls;
  params.snapshot_url = full_key;
  params.snapshot_plate_key = plate_key;
  params.event_time = utils::utcIso8601();
  params.ai_modules = config_.aiModule();

  const Json::Value event = business::plate::buildVehicleEvent(params);
  const std::string topic = config_.eventTopic();
  const std::string json = business::plate::toCompactJson(event);
  if (dry_run_) {
    LOG_INFO("dry-run event → %s (full=%zuB plate=%zuB)\n%s", topic.c_str(),
             emit.full_frame.data.size(), emit.plate_crop.data.size(), json.c_str());
    return true;
  }
  const double mqtt_start_s = utils::monotonicSeconds();
  if (!mqtt_->publish(topic, json)) {
    utils::latencyLog("track %lu mqtt FAIL after %.0fms",
                      static_cast<unsigned long>(emit.track_id),
                      utils::msSince(mqtt_start_s));
    return false;
  }
  utils::latencyLog("track %lu mqtt_publish: %.0fms",
                    static_cast<unsigned long>(emit.track_id),
                    utils::msSince(mqtt_start_s));

  LOG_INFO("event: camera=%s plate=%s", camera.code.c_str(), emit.plate.c_str());

  // Tổng hợp E2E: zone → event.
  const double zone_to_event =
      (emit.created_at_s > 0.0) ? (utils::monotonicSeconds() - emit.created_at_s) * 1000.0 : 0.0;
  const double vote_ms =
      (emit.final_at_s > 0.0 && emit.created_at_s > 0.0)
          ? (emit.final_at_s - emit.created_at_s) * 1000.0
          : 0.0;
  const double final_to_event =
      (emit.final_at_s > 0.0) ? (utils::monotonicSeconds() - emit.final_at_s) * 1000.0 : 0.0;
  utils::latencyLog(
      "track %lu E2E camera=%s plate='%s': zone→event=%.0fms | vote(zone→final)=%.0fms "
      "(%d readings) | final→event=%.0fms | publish=%.0fms",
      static_cast<unsigned long>(emit.track_id), camera.code.c_str(), emit.plate.c_str(),
      zone_to_event, vote_ms, emit.recognize_count, final_to_event,
      utils::msSince(pub_start_s));
  return true;
}

}  // namespace comm
}  // namespace vehicle
