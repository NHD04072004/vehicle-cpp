// Cấu hình vi phạm theo camera nhận từ MQTT `get_violations`.
// Port của gsan/violation/{config_loader,filter}.py — inject thay vì singleton.
#pragma once

#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace vehicle {
namespace business {
namespace violation {

struct CameraViolationConfig {
  bool module_enabled = false;
  std::set<std::string> allowed_codes;
};

class ConfigStore {
 public:
  // camera_id rỗng → bỏ qua. Code rỗng/khoảng trắng bị loại.
  void updateFromMqtt(const std::string& camera_id, bool module_enabled,
                      const std::vector<std::string>& allowed_codes);

  // false nếu chưa nhận config, module tắt, hoặc code không nằm trong allowed_codes.
  bool allows(const std::string& code, const std::string& camera_id) const;

  bool hasCamera(const std::string& camera_id) const;

 private:
  mutable std::mutex mutex_;
  std::map<std::string, CameraViolationConfig> by_camera_;
};

}  // namespace violation
}  // namespace business
}  // namespace vehicle
