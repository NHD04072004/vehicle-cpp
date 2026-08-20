#include "communication/event_publisher.h"

#include "business/violation/constants.h"
#include "common/logging.h"
#include "utils/latency.h"
#include "utils/time_utils.h"

namespace vehicle {
namespace comm {

EventPublisher::EventPublisher(const Config& config, MqttClient* mqtt, HttpUploader* uploader,
                               const business::violation::ConfigStore* violations)
    : config_(config), mqtt_(mqtt), uploader_(uploader), violations_(violations) {}

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

business::plate::EventKindMask EventPublisher::publishPlateEvent(const Camera& camera,
                                                                 const PlateEmit& emit) {
  using business::plate::EventKind;
  using business::plate::EventKindMask;
  using business::plate::kindBit;

  if (mqtt_ == nullptr || uploader_ == nullptr) return 0;

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
    // Không có ảnh nào → không nghiệp vụ nào bắn được. Mask rỗng = retry tất cả
    // ở lần sau (track không còn bị xoá sớm nên retry thực sự có hiệu lực).
    LOG_WARN("track %lu: upload snapshot thất bại — hoãn publish event",
             static_cast<unsigned long>(emit.track_id));
    utils::latencyLog("track %lu publish FAIL after upload %.0fms",
                      static_cast<unsigned long>(emit.track_id),
                      utils::msSince(pub_start_s));
    return 0;
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

  EventKindMask done = publishViolations(camera, emit, params);

  // Event phương tiện chỉ bắn khi track KHÔNG có vi phạm nào — giữ nguyên hành
  // vi cũ (event vi phạm đã mang đủ thông tin xe).
  const bool has_violation = canPublishNoHelmet(camera, emit) ||
                             canPublishWrongLane(camera, emit) ||
                             canPublishWrongWay(camera, emit);
  if (has_violation) {
    // PLATE coi như xong: thông tin xe đã nằm trong event vi phạm.
    done |= kindBit(EventKind::kPlate);
  } else {
    const Json::Value event = business::plate::buildVehicleEvent(params);
    const std::string json = business::plate::toCompactJson(event);
    const std::string topic = config_.eventTopic();
    if (dry_run_) {
      LOG_INFO("dry-run event → %s (full=%zuB plate=%zuB)\n%s", topic.c_str(),
               emit.full_frame.data.size(), emit.plate_crop.data.size(), json.c_str());
      done |= kindBit(EventKind::kPlate);
    } else {
      const double mqtt_start_s = utils::monotonicSeconds();
      if (!mqtt_->publish(topic, json)) {
        utils::latencyLog("track %lu mqtt FAIL after %.0fms",
                          static_cast<unsigned long>(emit.track_id),
                          utils::msSince(mqtt_start_s));
        return done;  // PLATE lỗi tạm → retry riêng nó, vi phạm đã xong vẫn giữ
      }
      utils::latencyLog("track %lu mqtt_publish: %.0fms",
                        static_cast<unsigned long>(emit.track_id),
                        utils::msSince(mqtt_start_s));
      LOG_INFO("event: camera=%s plate=%s", camera.code.c_str(), emit.plate.c_str());
      done |= kindBit(EventKind::kPlate);
    }
  }

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
  return done;
}

bool EventPublisher::canPublishNoHelmet(const Camera& camera, const PlateEmit& emit) const {
  const HelmetViolationConfig& helmet = config_.violation().helmet;
  if (!helmet.enabled) return false;
  if (emit.vehicle_cls != kClassMotorbike) return false;
  if (emit.no_helmet_frames < helmet.min_hits) return false;

  if (violations_ == nullptr ||
      !violations_->allows(business::violation::kNoHelmet, camera.id)) {
    LOG_DEBUG("track %lu: NO_HELMET (%d frame) nhưng camera %s chưa bật mã này — bỏ qua",
              static_cast<unsigned long>(emit.track_id), emit.no_helmet_frames,
              camera.code.c_str());
    return false;
  }
  return true;
}

bool EventPublisher::canPublishWrongLane(const Camera& camera, const PlateEmit& emit) const {
  const LaneViolationConfig& wrong_lane = config_.violation().wrong_lane;
  if (!wrong_lane.enabled) return false;
  if (emit.wrong_lane_frames < wrong_lane.min_hits) return false;

  if (violations_ == nullptr ||
      !violations_->allows(business::violation::kWrongLane, camera.id)) {
    LOG_DEBUG("track %lu: WRONG_LANE (%d frame) nhưng camera %s chưa bật mã này — bỏ qua",
              static_cast<unsigned long>(emit.track_id), emit.wrong_lane_frames,
              camera.code.c_str());
    return false;
  }
  return true;
}

bool EventPublisher::canPublishWrongWay(const Camera& camera, const PlateEmit& emit) const {
  const WrongWayViolationConfig& wrong_way = config_.violation().wrong_way;
  if (!wrong_way.enabled) return false;
  // Bắt buộc đã CHẠM line REVERSE_DIRECTION ngược chiều.
  if (emit.wrong_way_hits < wrong_way.min_hits) return false;

  if (violations_ == nullptr ||
      !violations_->allows(business::violation::kWrongWay, camera.id)) {
    LOG_DEBUG("track %lu: WRONG_WAY (%d lần cắt) nhưng camera %s chưa bật mã này — bỏ qua",
              static_cast<unsigned long>(emit.track_id), emit.wrong_way_hits,
              camera.code.c_str());
    return false;
  }
  return true;
}

bool EventPublisher::publishOneViolation(const Camera& camera, const PlateEmit& emit,
                                         const business::plate::EventParams& params,
                                         const std::string& violation_type_code,
                                         const Json::Value& evidence) {
  const Json::Value event =
      business::plate::buildViolationEvent(params, violation_type_code, evidence);
  const std::string topic = config_.eventTopic();
  const std::string json = business::plate::toCompactJson(event);
  if (dry_run_) {
    LOG_INFO("dry-run violation → %s\n%s", topic.c_str(), json.c_str());
    return true;
  }
  if (!mqtt_->publish(topic, json)) {
    LOG_WARN("track %lu: publish %s thất bại", static_cast<unsigned long>(emit.track_id),
             violation_type_code.c_str());
    return false;
  }
  LOG_INFO("violation: camera=%s plate=%s code=%s", camera.code.c_str(), emit.plate.c_str(),
           violation_type_code.c_str());
  return true;
}

EventPublisher::PublishOutcome EventPublisher::publishViolationKind(
    const Camera& camera, const PlateEmit& emit,
    const business::plate::EventParams& vparams, business::plate::EventKind kind) {
  using business::plate::EventKind;

  Json::Value evidence(Json::objectValue);
  business::plate::EventParams params = vparams;

  switch (kind) {
    case EventKind::kNoHelmet:
      if (!canPublishNoHelmet(camera, emit)) return PublishOutcome::kSkipped;
      evidence["road_type"] = "highway";
      // NO_HELMET dùng ảnh chung của track: không mũ là trạng thái kéo dài suốt
      // hành trình, không phải sự kiện tức thời cần ảnh riêng.
      break;

    case EventKind::kWrongLane:
      if (!canPublishWrongLane(camera, emit)) return PublishOutcome::kSkipped;
      evidence["lane_zone"] = emit.wrong_lane_zone;
      params = paramsWithViolationImages(camera, emit, vparams, emit.wrong_lane_full,
                                         emit.wrong_lane_crop, "wrong_lane",
                                         business::violation::kWrongLane);
      break;

    case EventKind::kWrongWay:
      if (!canPublishWrongWay(camera, emit)) return PublishOutcome::kSkipped;
      evidence["line_name"] = emit.wrong_way_line;
      evidence["road_type"] = "highway";
      params = paramsWithViolationImages(camera, emit, vparams, emit.wrong_way_full,
                                         emit.wrong_way_crop, "wrong_way",
                                         business::violation::kWrongWay);
      break;

    default:
      return PublishOutcome::kSkipped;  // kPlate không đi đường này
  }

  const char* code = business::plate::eventKindCode(kind);
  if (code == nullptr) return PublishOutcome::kSkipped;
  return publishOneViolation(camera, emit, params, code, evidence) ? PublishOutcome::kOk
                                                                   : PublishOutcome::kRetry;
}

business::plate::EventKindMask EventPublisher::publishViolations(
    const Camera& camera, const PlateEmit& emit,
    const business::plate::EventParams& params) {
  using business::plate::EventKind;
  using business::plate::EventKindMask;

  business::plate::EventParams vparams = params;
  if (vparams.direction.empty()) vparams.direction = "IN";

  EventKindMask done = 0;
  for (EventKind kind :
       {EventKind::kNoHelmet, EventKind::kWrongLane, EventKind::kWrongWay}) {
    // kOk và kSkipped đều là "xong": kSkipped nghĩa là không đủ điều kiện hoặc
    // VMS chưa bật mã — thử lại cũng cho kết quả y hệt. Chỉ kRetry mới để lại.
    if (publishViolationKind(camera, emit, vparams, kind) != PublishOutcome::kRetry)
      done |= business::plate::kindBit(kind);
  }
  return done;
}

business::plate::EventParams EventPublisher::paramsWithViolationImages(
    const Camera& camera, const PlateEmit& emit, const business::plate::EventParams& base_params,
    const JpegImage& full, const JpegImage& crop, const std::string& suffix,
    const std::string& violation_type_code) {
  // Ảnh chụp đúng lúc vi phạm; các event khác giữ ảnh đẹp nhất.
  business::plate::EventParams params = base_params;
  if (full.empty()) return params;

  const std::string base = "vehicle_" + std::to_string(emit.track_id) + "_" + suffix;
  if (dry_run_) {
    params.snapshot_url = "dry-run/" + base + ".jpg";
    params.snapshot_plate_key = crop.empty() ? "" : "dry-run/" + base + "_plate.jpg";
    return params;
  }

  const std::string key = uploader_->upload(camera.id, "vehicle", full.data, base + ".jpg");
  if (key.empty()) {
    LOG_WARN("track %lu: upload ảnh %s thất bại — dùng ảnh mặc định",
             static_cast<unsigned long>(emit.track_id), violation_type_code.c_str());
    return params;
  }
  params.snapshot_url = key;
  params.snapshot_plate_key =
      crop.empty() ? ""
                   : uploader_->upload(camera.id, "plate", crop.data, base + "_plate.jpg");
  return params;
}

}  // namespace comm
}  // namespace vehicle
