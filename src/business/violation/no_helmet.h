// NO_HELMET — không đội mũ bảo hiểm.
//
// Luật: SGIE helmet gán class `no_helmet_class_id` cho người không đội mũ; đếm
// số người đó trên mỗi xe MÁY đang trong zone PLATE. Chỉ áp dụng cho xe máy.
//
// Khác hai nghiệp vụ kia ở chỗ KHÔNG cần ảnh bằng chứng riêng: không đội mũ là
// trạng thái kéo dài suốt hành trình chứ không phải sự kiện tức thời, nên event
// dùng chung ảnh đẹp nhất của track.
#pragma once

#include <json/json.h>

#include "common/config.h"
#include "common/types.h"

namespace vehicle {
namespace business {
namespace violation {

// Xe này có được xét NO_HELMET ở frame này không (chỉ xe máy, trong zone PLATE).
bool shouldEvaluateNoHelmet(const HelmetViolationConfig& config, int vehicle_cls,
                            bool in_plate_zone);

// Object của SGIE helmet có phải "người không đội mũ" không.
bool isNoHelmetClass(const HelmetViolationConfig& config, int class_id);

// Đủ ngưỡng để bắn event chưa (chưa xét cấu hình VMS).
// `vehicle_cls` từ vote của track — xe không phải xe máy không bao giờ vi phạm.
bool noHelmetMeetsThreshold(const HelmetViolationConfig& config, int vehicle_cls, int hits);

// `violation_evidence` của payload pub_event.
Json::Value buildNoHelmetEvidence();

}  // namespace violation
}  // namespace business
}  // namespace vehicle
