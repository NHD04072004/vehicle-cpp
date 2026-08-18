#include "common/config.h"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <stdexcept>
#include <sys/stat.h>

#include "common/logging.h"
#include "utils/string_utils.h"

namespace vehicle {
namespace {

bool fileExists(const std::string& path) {
  struct stat st{};
  return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

YAML::Node loadFile(const std::string& path, bool required) {
  if (!fileExists(path)) {
    if (required) throw std::runtime_error("missing config file: " + path);
    LOG_WARN("config file not found, dùng mặc định: %s", path.c_str());
    return YAML::Node(YAML::NodeType::Map);
  }
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("invalid YAML in " + path + ": " + e.what());
  }
}

template <typename T>
T get(const YAML::Node& node, const std::string& key, const T& fallback) {
  if (!node || !node.IsMap()) return fallback;
  const YAML::Node child = node[key];
  if (!child || child.IsNull()) return fallback;
  try {
    return child.as<T>();
  } catch (const YAML::Exception&) {
    LOG_WARN("config key '%s' sai kiểu — dùng mặc định", key.c_str());
    return fallback;
  }
}

void loadGie(const YAML::Node& node, GieConfig* out) {
  if (!node || !node.IsMap()) return;
  out->config = get<std::string>(node, "config", out->config);
  out->unique_id = get<int>(node, "unique_id", out->unique_id);
}

void loadPipeline(const YAML::Node& pipe, PipelineConfig* p) {
  if (!pipe || !pipe.IsMap()) return;

  const YAML::Node rtsp = pipe["rtsp"];
  p->rtsp.select_rtp_protocol =
      get<int>(rtsp, "select_rtp_protocol", p->rtsp.select_rtp_protocol);
  p->rtsp.reconnect_interval =
      get<int>(rtsp, "reconnect_interval", p->rtsp.reconnect_interval);
  p->rtsp.reconnect_attempts =
      get<int>(rtsp, "reconnect_attempts", p->rtsp.reconnect_attempts);
  p->rtsp.drop_on_latency = get<bool>(rtsp, "drop_on_latency", p->rtsp.drop_on_latency);
  p->rtsp.latency = get<int>(rtsp, "latency", p->rtsp.latency);
  p->rtsp.low_latency_mode = get<bool>(rtsp, "low_latency_mode", p->rtsp.low_latency_mode);
  p->rtsp.num_extra_surfaces =
      get<int>(rtsp, "num_extra_surfaces", p->rtsp.num_extra_surfaces);
  p->rtsp.drop_frame_interval =
      get<int>(rtsp, "drop_frame_interval", p->rtsp.drop_frame_interval);
  if (p->rtsp.drop_frame_interval < 0) p->rtsp.drop_frame_interval = 0;
  p->rtsp.require_ready = get<bool>(rtsp, "require_ready", p->rtsp.require_ready);
  p->rtsp.ready_timeout_ms = get<int>(rtsp, "ready_timeout_ms", p->rtsp.ready_timeout_ms);
  p->rtsp.watch_interval_ms =
      get<int>(rtsp, "watch_interval_ms", p->rtsp.watch_interval_ms);

  const YAML::Node mux = pipe["streammux"];
  p->streammux.batch_size = get<int>(mux, "batch_size", p->streammux.batch_size);
  p->streammux.batched_push_timeout =
      get<int>(mux, "batched_push_timeout", p->streammux.batched_push_timeout);
  p->streammux.sync_inputs = get<bool>(mux, "sync_inputs", p->streammux.sync_inputs);
  p->streammux.attach_sys_ts = get<bool>(mux, "attach_sys_ts", p->streammux.attach_sys_ts);
  p->streammux.max_latency =
      get<uint64_t>(mux, "max_latency", p->streammux.max_latency);
  p->streammux.num_surfaces_per_frame =
      get<unsigned int>(mux, "num_surfaces_per_frame", p->streammux.num_surfaces_per_frame);
  p->streammux.config_file = get<std::string>(mux, "config_file", p->streammux.config_file);

  const YAML::Node pgie = pipe["pgie"];
  if (pgie && pgie.IsMap()) {
    p->pgie.config = get<std::string>(pgie, "config", p->pgie.config);
    p->pgie.unique_id = get<int>(pgie, "unique_id", p->pgie.unique_id);
  }

  loadGie(pipe["sgie_plate"], &p->sgie_plate);
  {
    const YAML::Node pre = pipe["preprocess_plate"];
    if (pre && pre.IsMap()) {
      p->preprocess_plate.config =
          get<std::string>(pre, "config", p->preprocess_plate.config);
      p->preprocess_plate.unique_id =
          get<int>(pre, "unique_id", p->preprocess_plate.unique_id);
    }
  }
  loadGie(pipe["sgie_digit"], &p->sgie_digit);
  loadGie(pipe["sgie_helmet"], &p->sgie_helmet);

  const YAML::Node tracker = pipe["tracker"];
  p->tracker.ll_config = get<std::string>(tracker, "ll_config", p->tracker.ll_config);
  p->tracker.ll_lib = get<std::string>(tracker, "ll_lib", p->tracker.ll_lib);
  p->tracker.width = get<int>(tracker, "width", p->tracker.width);
  p->tracker.height = get<int>(tracker, "height", p->tracker.height);

  const YAML::Node sink = pipe["sink"];
  p->sink.type = get<std::string>(sink, "type", p->sink.type);
  p->sink.output_file = get<std::string>(sink, "output_file", p->sink.output_file);

  const YAML::Node probe = pipe["probe"];
  p->probe.plate_expand_ratio =
      get<double>(probe, "plate_expand_ratio", p->probe.plate_expand_ratio);
  p->probe.anchor_bottom_ratio =
      get<double>(probe, "anchor_bottom_ratio", p->probe.anchor_bottom_ratio);
  p->probe.char_iou_dedup = get<double>(probe, "char_iou_dedup", p->probe.char_iou_dedup);
  p->probe.square_plate_ratio =
      get<double>(probe, "square_plate_ratio", p->probe.square_plate_ratio);
  p->probe.min_plate_width = get<int>(probe, "min_plate_width", p->probe.min_plate_width);
  p->probe.vehicle_bottom_pad =
      get<double>(probe, "vehicle_bottom_pad", p->probe.vehicle_bottom_pad);
  p->probe.keypoint_min_confidence = static_cast<float>(
      get<double>(probe, "keypoint_min_confidence", p->probe.keypoint_min_confidence));
  p->probe.publish_bbox = get<bool>(probe, "publish_bbox", p->probe.publish_bbox);
}

void loadViolation(const YAML::Node& node, ViolationConfig* v) {
  if (!node || !node.IsMap()) return;
  const YAML::Node helmet = node["helmet"];
  v->helmet.enabled = get<bool>(helmet, "enabled", v->helmet.enabled);
  v->helmet.no_helmet_class_id =
      get<int>(helmet, "no_helmet_class_id", v->helmet.no_helmet_class_id);
  v->helmet.min_hits = get<int>(helmet, "min_hits", v->helmet.min_hits);
  if (v->helmet.min_hits < 1) v->helmet.min_hits = 1;

  const YAML::Node wrong_lane = node["wrong_lane"];
  v->wrong_lane.enabled = get<bool>(wrong_lane, "enabled", v->wrong_lane.enabled);
  v->wrong_lane.min_hits = get<int>(wrong_lane, "min_hits", v->wrong_lane.min_hits);
  if (v->wrong_lane.min_hits < 1) v->wrong_lane.min_hits = 1;
}

}  // namespace

Config Config::load(const std::string& root_dir) {
  Config cfg;
  cfg.root_dir_ = utils::trimTrailingSlash(root_dir);

  const YAML::Node app = loadFile(cfg.resolvePath("resources/config/config.yaml"), true);
  const YAML::Node mqtt = loadFile(cfg.resolvePath("resources/config/mqtt.yaml"), true);
  const YAML::Node restful = loadFile(cfg.resolvePath("resources/config/restful.yaml"), false);
  const YAML::Node violations =
      loadFile(cfg.resolvePath("resources/config/violations.yaml"), false);

  cfg.ai_module_ = get<std::string>(app, "AI_MODULE", "");
  if (cfg.ai_module_.empty())
    cfg.ai_module_ = get<std::string>(mqtt, "ai_modules", "PLATE");

  const YAML::Node plate = app["plate"];
  cfg.plate_.send_mode = get<int>(plate, "send_mode", cfg.plate_.send_mode);
  cfg.plate_.max_recognize_times =
      get<int>(plate, "max_recognize_times", cfg.plate_.max_recognize_times);
  if (plate && plate["plate_style"] && plate["plate_style"].IsSequence()) {
    for (const auto& item : plate["plate_style"])
      cfg.plate_.plate_style.push_back(utils::toUpper(item.as<std::string>()));
  }

  loadPipeline(app["pipeline"], &cfg.pipeline_);
  loadViolation(violations["violation"], &cfg.violation_);

  MqttConfig& m = cfg.mqtt_;
  m.host = get<std::string>(mqtt, "host", m.host);
  m.port = get<int>(mqtt, "port", m.port);
  m.username = get<std::string>(mqtt, "username", m.username);
  m.password = get<std::string>(mqtt, "password", m.password);
  m.keep_alive = get<int>(mqtt, "keep_alive", m.keep_alive);
  m.reconnect_interval_s = get<int>(mqtt, "reconnect_interval_s", m.reconnect_interval_s);
  if (m.reconnect_interval_s < 1) m.reconnect_interval_s = 1;
  // company_id có thể là số hoặc chuỗi trong YAML.
  m.company_id = get<std::string>(mqtt, "company_id", m.company_id);
  m.ai_modules = cfg.ai_module_;
  m.camera_list_tpl = get<std::string>(mqtt, "camera_list", m.camera_list_tpl);
  m.get_polygon_tpl = get<std::string>(mqtt, "get_polygon", m.get_polygon_tpl);
  m.get_violations_tpl = get<std::string>(mqtt, "get_violations", m.get_violations_tpl);
  m.pub_bbox_tpl = get<std::string>(mqtt, "pub_bbox", m.pub_bbox_tpl);
  m.pub_event_tpl = get<std::string>(mqtt, "pub_event", m.pub_event_tpl);

  const YAML::Node snap = restful["snapshot_api"];
  cfg.snapshot_.base_url =
      utils::trimTrailingSlash(get<std::string>(snap, "base_url", cfg.snapshot_.base_url));
  cfg.snapshot_.endpoint = get<std::string>(snap, "endpoint", cfg.snapshot_.endpoint);
  if (!cfg.snapshot_.endpoint.empty() && cfg.snapshot_.endpoint.front() != '/')
    cfg.snapshot_.endpoint = "/" + cfg.snapshot_.endpoint;

  if (m.host.empty()) throw std::runtime_error("mqtt.yaml: thiếu host");

  LOG_INFO("config: AI_MODULE=%s send_mode=%d max_recognize_times=%d styles=%zu",
           cfg.ai_module_.c_str(), cfg.plate_.send_mode,
           cfg.plate_.max_recognize_times, cfg.plate_.plate_style.size());
  return cfg;
}

std::string Config::cameraListTopic() const {
  return utils::replaceAll(mqtt_.camera_list_tpl, "{company_id}", mqtt_.company_id);
}

std::string Config::zonesTopic(const std::string& camera_code) const {
  return utils::replaceAll(mqtt_.get_polygon_tpl, "{camera_code}", camera_code);
}

std::string Config::violationsTopic(const std::string& camera_id) const {
  std::string topic = utils::replaceAll(mqtt_.get_violations_tpl, "{camera_id}", camera_id);
  topic = utils::replaceAll(topic, "{ai_modules}", mqtt_.ai_modules);
  return utils::replaceAll(topic, "{ai_module}", mqtt_.ai_modules);
}

std::string Config::bboxTopic(const std::string& camera_code) const {
  return utils::replaceAll(mqtt_.pub_bbox_tpl, "{camera_code}", camera_code);
}

std::string Config::eventTopic() const {
  std::string topic = utils::replaceAll(mqtt_.pub_event_tpl, "{ai_modules}", mqtt_.ai_modules);
  return utils::replaceAll(topic, "{ai_module}", mqtt_.ai_modules);
}

std::string Config::uploadUrl() const {
  return snapshot_.base_url + snapshot_.endpoint;
}

std::string Config::resolvePath(const std::string& path) const {
  if (path.empty() || path.front() == '/') return path;
  return root_dir_ + "/" + path;
}

}  // namespace vehicle
