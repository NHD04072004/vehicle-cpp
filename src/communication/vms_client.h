// Subscribe camera_list + get_polygon từ Smart VMS, parse thành DTO.
#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "common/config.h"
#include "common/types.h"
#include "communication/mqtt_client.h"

namespace vehicle {
namespace comm {

class VmsClient {
 public:
  using CamerasCallback = std::function<void(const std::vector<Camera>&)>;
  using ZonesCallback = std::function<void(const ZoneSet&)>;

  VmsClient(const Config& config, MqttClient* mqtt);

  // Subscribe camera_list và zones (wildcard '+' cho camera_code).
  bool start();

  void setCamerasCallback(CamerasCallback cb) { cameras_cb_ = std::move(cb); }
  void setZonesCallback(ZonesCallback cb) { zones_cb_ = std::move(cb); }

  std::vector<Camera> cameras() const;
  ZoneSet zones(const std::string& camera_code) const;

  // Chờ tới khi nhận được camera list (hoặc hết timeout).
  bool waitForCameras(double timeout_s);

  // Lọc theo ai_modules: chấp nhận list, "PLATE", hoặc chuỗi JSON "[\"PLATE\"]".
  static std::vector<std::string> normalizeAiModules(const std::string& raw_json_field);

 private:
  void onMessage(const std::string& topic, const std::string& payload);
  void handleCameraList(const std::string& payload);
  void handleZones(const std::string& camera_code, const std::string& payload);
  bool matchZonesTopic(const std::string& topic, std::string* camera_code) const;

  const Config& config_;
  MqttClient* mqtt_;
  mutable std::mutex mutex_;
  std::vector<Camera> cameras_;
  std::map<std::string, ZoneSet> zones_;
  CamerasCallback cameras_cb_;
  ZonesCallback zones_cb_;
};

}  // namespace comm
}  // namespace vehicle
