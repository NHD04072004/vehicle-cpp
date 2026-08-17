#include "business/violation/config_store.h"

#include "utils/string_utils.h"

namespace vehicle {
namespace business {
namespace violation {
namespace {

// VMS gửi "SPEEDING"; mã nội bộ là "SPEEDING_10_20" (gsan/violation/config_loader.py).
std::string normalizeCode(const std::string& raw) {
  const std::string code = utils::trim(raw);
  if (code == "SPEEDING") return "SPEEDING_10_20";
  return code;
}

}  // namespace

void ConfigStore::updateFromMqtt(const std::string& camera_id, bool module_enabled,
                                 const std::vector<std::string>& allowed_codes) {
  if (camera_id.empty()) return;

  CameraViolationConfig cfg;
  cfg.module_enabled = module_enabled;
  for (const std::string& raw : allowed_codes) {
    const std::string code = normalizeCode(raw);
    if (!code.empty()) cfg.allowed_codes.insert(code);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  by_camera_[camera_id] = std::move(cfg);
}

bool ConfigStore::allows(const std::string& code, const std::string& camera_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = by_camera_.find(camera_id);
  if (it == by_camera_.end()) return false;
  if (!it->second.module_enabled) return false;
  return it->second.allowed_codes.count(code) > 0;
}

bool ConfigStore::hasCamera(const std::string& camera_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return by_camera_.count(camera_id) > 0;
}

}  // namespace violation
}  // namespace business
}  // namespace vehicle
