#include "communication/vms_client.h"

#include <json/json.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>

#include "common/logging.h"
#include "utils/string_utils.h"

namespace vehicle {
namespace comm {
namespace {

bool parseJson(const std::string& text, Json::Value* out) {
  Json::CharReaderBuilder builder;
  std::string errors;
  const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  return reader->parse(text.data(), text.data() + text.size(), out, &errors);
}

std::string asString(const Json::Value& node, const char* key, const std::string& fallback = "") {
  if (!node.isObject() || !node.isMember(key)) return fallback;
  const Json::Value& value = node[key];
  if (value.isString()) return value.asString();
  if (value.isIntegral()) return std::to_string(value.asInt64());
  if (value.isDouble()) return std::to_string(value.asDouble());
  return fallback;
}

// ai_modules có thể là mảng, chuỗi "PLATE", hoặc chuỗi JSON "[\"PLATE\"]".
std::vector<std::string> aiModulesOf(const Json::Value& camera) {
  std::vector<std::string> modules;
  if (!camera.isObject() || !camera.isMember("ai_modules")) return modules;
  const Json::Value& field = camera["ai_modules"];
  if (field.isArray()) {
    for (const Json::Value& item : field)
      if (item.isString()) modules.push_back(item.asString());
    return modules;
  }
  if (field.isString()) return VmsClient::normalizeAiModules(field.asString());
  return modules;
}

std::string restreamUrlOf(const Json::Value& camera, const std::string& ai_module) {
  if (!camera.isObject()) return "";
  if (camera.isMember("restream_urls")) {
    const Json::Value& urls = camera["restream_urls"];
    if (urls.isObject()) {
      if (urls.isMember(ai_module) && urls[ai_module].isString())
        return urls[ai_module].asString();
      // Một số VMS chỉ gửi 1 url chung.
      for (const std::string& key : urls.getMemberNames())
        if (urls[key].isString() && !urls[key].asString().empty()) return urls[key].asString();
    }
    if (urls.isString()) return urls.asString();
  }
  for (const char* key : {"restream_url", "rtsp_url", "url", "stream_url"}) {
    const std::string value = asString(camera, key);
    if (!value.empty()) return value;
  }
  return "";
}

// Đọc mảng điểm: [[x,y],...] | [{"x":..,"y":..}] | [x1,y1,x2,y2,...].
std::vector<Point> parsePoints(const Json::Value& node) {
  std::vector<Point> points;
  if (!node.isArray()) return points;

  const bool flat = std::all_of(node.begin(), node.end(),
                                [](const Json::Value& v) { return v.isNumeric(); });
  if (flat) {
    for (Json::ArrayIndex i = 0; i + 1 < node.size(); i += 2)
      points.push_back({node[i].asDouble(), node[i + 1].asDouble()});
    return points;
  }
  for (const Json::Value& item : node) {
    if (item.isArray() && item.size() >= 2 && item[0].isNumeric() && item[1].isNumeric()) {
      points.push_back({item[0].asDouble(), item[1].asDouble()});
    } else if (item.isObject()) {
      const char* kx[] = {"x", "X", "left"};
      const char* ky[] = {"y", "Y", "top"};
      double x = 0.0, y = 0.0;
      bool has_x = false, has_y = false;
      for (const char* key : kx)
        if (item.isMember(key) && item[key].isNumeric()) { x = item[key].asDouble(); has_x = true; break; }
      for (const char* key : ky)
        if (item.isMember(key) && item[key].isNumeric()) { y = item[key].asDouble(); has_y = true; break; }
      if (has_x && has_y) points.push_back({x, y});
    }
  }
  return points;
}

const Json::Value* findPointsNode(const Json::Value& zone) {
  for (const char* key : {"points", "polygon", "coordinates", "coords", "vertices", "area"}) {
    if (zone.isObject() && zone.isMember(key) && zone[key].isArray()) return &zone[key];
  }
  return nullptr;
}

// Node zones có thể nằm ở gốc, trong "zones", hoặc trong "data"/"payload".
const Json::Value* findZonesArray(const Json::Value& root) {
  if (root.isArray()) return &root;
  if (!root.isObject()) return nullptr;
  for (const char* key : {"zones", "polygons", "regions"}) {
    if (root.isMember(key) && root[key].isArray()) return &root[key];
  }
  for (const char* key : {"data", "payload", "result"}) {
    if (root.isMember(key)) {
      const Json::Value* nested = findZonesArray(root[key]);
      if (nested != nullptr) return nested;
    }
  }
  return nullptr;
}

// Mảng "lines" nằm song song với "zones" trong payload get_polygon.
const Json::Value* findLinesArray(const Json::Value& root) {
  if (!root.isObject()) return nullptr;
  if (root.isMember("lines") && root["lines"].isArray()) return &root["lines"];
  for (const char* key : {"data", "payload", "result"}) {
    if (root.isMember(key)) {
      const Json::Value* nested = findLinesArray(root[key]);
      if (nested != nullptr) return nested;
    }
  }
  return nullptr;
}

// direction_vector: [dx, dy] hoặc {"x":..,"y":..}. Trả false nếu vector ~0.
bool parseDirection(const Json::Value& node, Point* out) {
  if (node.isArray() && node.size() >= 2 && node[0].isNumeric() && node[1].isNumeric()) {
    *out = {node[0].asDouble(), node[1].asDouble()};
  } else if (node.isObject() && node.isMember("x") && node.isMember("y") &&
             node["x"].isNumeric() && node["y"].isNumeric()) {
    *out = {node["x"].asDouble(), node["y"].asDouble()};
  } else {
    return false;
  }
  return std::abs(out->x) > 1e-9 || std::abs(out->y) > 1e-9;
}

bool hasDirectionFlag(const Json::Value& line) {
  if (!line.isObject()) return false;
  const Json::Value& cfg = line["config"];
  if (cfg.isObject() && cfg.isMember("has_direction") && cfg["has_direction"].isBool())
    return cfg["has_direction"].asBool();
  if (line.isMember("has_direction") && line["has_direction"].isBool())
    return line["has_direction"].asBool();
  return false;
}

// Tách giá trị ở vị trí `marker` của template khi topic khớp prefix/suffix.
bool matchTemplate(const std::string& topic, const std::string& tpl, const char* marker,
                   std::string* value) {
  const size_t at = tpl.find(marker);
  if (at == std::string::npos) return false;
  const std::string prefix = tpl.substr(0, at);
  const std::string suffix = tpl.substr(at + std::strlen(marker));
  if (topic.size() <= prefix.size() + suffix.size()) return false;
  if (topic.compare(0, prefix.size(), prefix) != 0) return false;
  if (!suffix.empty() &&
      topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0)
    return false;
  *value = topic.substr(prefix.size(), topic.size() - prefix.size() - suffix.size());
  return !value->empty();
}

}  // namespace

VmsClient::VmsClient(const Config& config, MqttClient* mqtt)
    : config_(config), mqtt_(mqtt) {}

std::vector<std::string> VmsClient::normalizeAiModules(const std::string& raw) {
  const std::string text = utils::trim(raw);
  if (text.empty()) return {};
  if (text.front() == '[') {
    Json::Value parsed;
    if (parseJson(text, &parsed) && parsed.isArray()) {
      std::vector<std::string> modules;
      for (const Json::Value& item : parsed)
        if (item.isString()) modules.push_back(item.asString());
      return modules;
    }
  }
  return {text};
}

bool VmsClient::start() {
  if (mqtt_ == nullptr) return false;
  mqtt_->setMessageCallback(
      [this](const std::string& topic, const std::string& payload) {
        onMessage(topic, payload);
      });
  const std::string zones_wildcard = config_.zonesTopic("+");
  const std::string violations_wildcard = config_.violationsTopic("+");
  return mqtt_->subscribe(
      {config_.cameraListTopic(), zones_wildcard, violations_wildcard});
}

void VmsClient::onMessage(const std::string& topic, const std::string& payload) {
  if (topic == config_.cameraListTopic()) {
    handleCameraList(payload);
    return;
  }
  std::string key;
  if (matchZonesTopic(topic, &key)) {
    handleZones(key, payload);
    return;
  }
  if (matchViolationsTopic(topic, &key)) {
    handleViolations(key, payload);
    return;
  }
  LOG_DEBUG("mqtt: bỏ qua topic lạ %s", topic.c_str());
}

bool VmsClient::matchZonesTopic(const std::string& topic, std::string* camera_code) const {
  return matchTemplate(topic, config_.mqtt().get_polygon_tpl, "{camera_code}", camera_code);
}

bool VmsClient::matchViolationsTopic(const std::string& topic, std::string* camera_id) const {
  // violationsTopic() đã thay {ai_modules}; chỉ còn {camera_id} là biến.
  return matchTemplate(topic, config_.violationsTopic("{camera_id}"), "{camera_id}",
                       camera_id);
}

void VmsClient::handleCameraList(const std::string& payload) {
  Json::Value root;
  if (!parseJson(payload, &root)) {
    LOG_WARN("camera_list: payload không phải JSON hợp lệ");
    return;
  }
  const Json::Value* array = nullptr;
  if (root.isArray()) {
    array = &root;
  } else if (root.isObject() && root.isMember("cameras") && root["cameras"].isArray()) {
    array = &root["cameras"];
  }
  if (array == nullptr) {
    LOG_WARN("camera_list: không tìm thấy mảng cameras");
    return;
  }

  std::vector<Camera> matched;
  for (const Json::Value& node : *array) {
    Camera camera;
    camera.id = asString(node, "id");
    camera.code = asString(node, "code", asString(node, "camera_code"));
    camera.name = asString(node, "name");
    camera.ai_modules = aiModulesOf(node);
    camera.restream_url = restreamUrlOf(node, config_.aiModule());
    const bool wanted = std::find(camera.ai_modules.begin(), camera.ai_modules.end(),
                                  config_.aiModule()) != camera.ai_modules.end();
    if (!wanted) continue;
    if (camera.code.empty()) {
      LOG_WARN("camera_list: bỏ qua camera thiếu 'code' (id=%s)", camera.id.c_str());
      continue;
    }
    matched.push_back(camera);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cameras_ = matched;
  }
  if (cameras_cb_) cameras_cb_(matched);
}

void VmsClient::handleZones(const std::string& camera_code, const std::string& payload) {
  Json::Value root;
  if (!parseJson(payload, &root)) {
    LOG_WARN("zones[%s]: payload không phải JSON hợp lệ", camera_code.c_str());
    return;
  }
  const Json::Value* array = findZonesArray(root);
  const Json::Value* lines_array = findLinesArray(root);
  if (array == nullptr && lines_array == nullptr) {
    LOG_WARN("zones[%s]: không tìm thấy mảng zones/lines", camera_code.c_str());
    return;
  }

  ZoneSet set;
  set.camera_code = camera_code;
  double max_coord = 0.0;
  if (array != nullptr) {
    for (const Json::Value& node : *array) {
      const Json::Value* points_node = node.isArray() ? &node : findPointsNode(node);
      if (points_node == nullptr) continue;
      Zone zone;
      zone.name = asString(node, "name", asString(node, "zone_name"));
      zone.ai_modules = aiModulesOf(node);
      zone.points = parsePoints(*points_node);
      if (zone.points.size() < 3) continue;
      for (const Point& p : zone.points)
        max_coord = std::max(max_coord, std::max(p.x, p.y));
      set.zones.push_back(std::move(zone));
    }
  }

  // Lines: chỉ cần 2 điểm (khác zone >= 3) — dùng cho vi phạm cắt vạch.
  if (lines_array != nullptr) {
    for (const Json::Value& node : *lines_array) {
      const Json::Value* points_node = node.isArray() ? &node : findPointsNode(node);
      if (points_node == nullptr) continue;
      Line line;
      line.name = asString(node, "name", asString(node, "zone_name"));
      line.ai_modules = aiModulesOf(node);
      line.points = parsePoints(*points_node);
      if (line.points.size() < 2) continue;
      if (hasDirectionFlag(node) && node.isObject())
        line.has_direction = parseDirection(node["direction_vector"], &line.direction);
      for (const Point& p : line.points)
        max_coord = std::max(max_coord, std::max(p.x, p.y));
      set.lines.push_back(std::move(line));
    }
  }

  // Toạ độ > 1 → VMS gửi theo pixel; ngược lại là chuẩn hoá 0..1.
  const bool normalized = max_coord <= 1.0;
  for (Zone& zone : set.zones) zone.normalized = normalized;
  for (Line& line : set.lines) line.normalized = normalized;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = zones_.find(camera_code);
    // Chỉ ghi đè phần payload thực sự mang theo. Payload chỉ có `lines` mà gán
    // đè cả ZoneSet sẽ xoá sạch polygon PLATE của camera → handleMeta bỏ mọi
    // frame, mất OCR và toàn bộ vi phạm cho tới lần cập nhật zone tiếp theo.
    // Đối xứng: payload chỉ có `zones` sẽ xoá line REVERSE_DIRECTION.
    if (it != zones_.end()) {
      if (array == nullptr) set.zones = it->second.zones;
      if (lines_array == nullptr) set.lines = it->second.lines;
    }
    set.version = (it == zones_.end() ? 0 : it->second.version) + 1;
    zones_[camera_code] = set;
  }
  if (zones_cb_) zones_cb_(set);
}

void VmsClient::handleViolations(const std::string& camera_id, const std::string& payload) {
  if (violations_ == nullptr) return;
  Json::Value root;
  if (!parseJson(payload, &root)) {
    LOG_WARN("violations[%s]: payload không phải JSON hợp lệ", camera_id.c_str());
    return;
  }

  // Payload chuẩn là object (schema_version=1); chấp nhận list trần phòng VMS đổi format.
  const Json::Value* codes = nullptr;
  bool module_enabled = true;
  if (root.isArray()) {
    codes = &root;
  } else if (root.isObject()) {
    if (root.isMember("module_enabled") && root["module_enabled"].isBool())
      module_enabled = root["module_enabled"].asBool();
    for (const char* key : {"allowed_codes", "violations", "data", "items"}) {
      if (root.isMember(key) && root[key].isArray()) {
        codes = &root[key];
        break;
      }
    }
  }
  if (codes == nullptr) {
    LOG_WARN("violations[%s]: không tìm thấy allowed_codes", camera_id.c_str());
    return;
  }

  std::vector<std::string> allowed;
  for (const Json::Value& item : *codes)
    if (item.isString()) allowed.push_back(item.asString());

  violations_->updateFromMqtt(camera_id, module_enabled, allowed);
}

std::vector<Camera> VmsClient::cameras() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cameras_;
}

ZoneSet VmsClient::zones(const std::string& camera_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = zones_.find(camera_code);
  if (it == zones_.end()) return ZoneSet{};
  return it->second;
}

bool VmsClient::waitForCameras(double timeout_s) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(timeout_s * 1000));
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!cameras_.empty()) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return !cameras_.empty();
}

}  // namespace comm
}  // namespace vehicle
