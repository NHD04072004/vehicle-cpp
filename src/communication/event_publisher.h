// Gộp upload snapshot + publish bbox/event lên Smart VMS.
#pragma once

#include <string>
#include <vector>

#include "business/plate/event.h"
#include "common/config.h"
#include "common/types.h"
#include "communication/http_uploader.h"
#include "communication/mqtt_client.h"

namespace vehicle {
namespace comm {

class EventPublisher {
 public:
  EventPublisher(const Config& config, MqttClient* mqtt, HttpUploader* uploader);

  // Dry-run: chỉ log payload, không upload/publish (dùng khi verify offline).
  void setDryRun(bool dry_run) { dry_run_ = dry_run; }

  void publishBbox(const std::string& camera_code, const std::vector<Detection>& detections);

  // Upload full-frame + crop biển rồi publish `pub_event`.
  // Trả về false nếu không upload/publish được (caller có thể retry).
  bool publishPlateEvent(const Camera& camera, const PlateEmit& emit);

 private:
  const Config& config_;
  MqttClient* mqtt_;
  HttpUploader* uploader_;
  bool dry_run_ = false;
};

}  // namespace comm
}  // namespace vehicle
