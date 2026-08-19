// Gộp upload snapshot + publish bbox/event lên Smart VMS.
#pragma once

#include <string>
#include <vector>

#include "business/plate/event.h"
#include "business/violation/config_store.h"
#include "common/config.h"
#include "common/types.h"
#include "communication/http_uploader.h"
#include "communication/mqtt_client.h"

namespace vehicle {
namespace comm {

class EventPublisher {
 public:
  // violations: cấu hình vi phạm theo camera từ VMS; nullptr → không bắn vi phạm.
  EventPublisher(const Config& config, MqttClient* mqtt, HttpUploader* uploader,
                 const business::violation::ConfigStore* violations = nullptr);

  // Dry-run: chỉ log payload, không upload/publish (dùng khi verify offline).
  void setDryRun(bool dry_run) { dry_run_ = dry_run; }

  void publishBbox(const std::string& camera_code, const std::vector<Detection>& detections);

  bool publishPlateEvent(const Camera& camera, const PlateEmit& emit);

 private:
  bool canPublishNoHelmet(const Camera& camera, const PlateEmit& emit) const;
  bool canPublishWrongLane(const Camera& camera, const PlateEmit& emit) const;
  bool canPublishWrongWay(const Camera& camera, const PlateEmit& emit) const;

  bool publishViolations(const Camera& camera, const PlateEmit& emit,
                         const business::plate::EventParams& params);

  // Upload ảnh bằng chứng vi phạm (nếu có) → params với snapshot riêng.
  business::plate::EventParams paramsWithViolationImages(
      const Camera& camera, const PlateEmit& emit,
      const business::plate::EventParams& base_params, const JpegImage& full,
      const JpegImage& crop, const std::string& suffix,
      const std::string& violation_type_code);

  bool publishOneViolation(const Camera& camera, const PlateEmit& emit,
                           const business::plate::EventParams& params,
                           const std::string& violation_type_code,
                           const Json::Value& evidence);

  const Config& config_;
  MqttClient* mqtt_;
  HttpUploader* uploader_;
  const business::violation::ConfigStore* violations_;
  bool dry_run_ = false;
};

}  // namespace comm
}  // namespace vehicle
