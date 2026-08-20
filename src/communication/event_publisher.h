// Gộp upload snapshot + publish bbox/event lên Smart VMS.
#pragma once

#include <string>
#include <vector>

#include "business/plate/event.h"
#include "business/plate/rules.h"  // EventKind / EventKindMask
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

  // Trả về mask các nghiệp vụ đã XONG: publish thành công, hoặc bị chặn vĩnh
  // viễn bởi config/VMS. Kind lỗi TẠM (upload/MQTT hỏng) không nằm trong mask
  // → caller retry riêng nó, không bắn lại kind đã thành công.
  business::plate::EventKindMask publishPlateEvent(const Camera& camera,
                                                   const PlateEmit& emit);

 private:
  // Ba trạng thái, không phải hai. Gộp "không đủ điều kiện" với "lỗi tạm" vào
  // một bool chính là nguồn gốc bug retry: 1 violation lỗi làm cả track bị thử
  // lại, bắn trùng những cái đã publish xong.
  enum class PublishOutcome {
    kOk,       // đã publish
    kSkipped,  // min_hits chưa đủ / VMS chưa bật mã → coi như xong, KHÔNG retry
    kRetry,    // lỗi tạm (upload/MQTT) → lần sau thử lại
  };

  // Nghiệp vụ đủ điều kiện bắn: module bật, đạt min_hits, và VMS đã bật mã cho
  // camera này. Ba nhánh chỉ khác nhau ở config + trường đếm.
  bool canPublish(const Camera& camera, const PlateEmit& emit,
                  business::plate::EventKind kind) const;

  // Publish 1 nghiệp vụ vi phạm theo kind. Trả trạng thái để caller gom mask.
  PublishOutcome publishViolationKind(const Camera& camera, const PlateEmit& emit,
                                      const business::plate::EventParams& params,
                                      business::plate::EventKind kind);

  business::plate::EventKindMask publishViolations(
      const Camera& camera, const PlateEmit& emit,
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
