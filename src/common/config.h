#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vehicle {

// `plate.*` trong config.yaml — business dùng.
struct PlateConfig {
  int send_mode = 2;
  int max_recognize_times = 5;
  std::vector<std::string> plate_style;
};

// `pipeline.rtsp.*` — nvurisrcbin decode / RTSP.
struct RtspConfig {
  int select_rtp_protocol = 4;  // 4 = TCP
  int reconnect_interval = 10;  // giây; 0 = tắt
  int reconnect_attempts = -1;  // -1 = vô hạn; 0 = không retry thêm
  bool drop_on_latency = true;
  int latency = 100;            // ms
  bool low_latency_mode = false;
  int num_extra_surfaces = 0;   // 0 = giữ mặc định của nvurisrcbin
  int drop_frame_interval = 0;
  bool require_ready = true;
  int ready_timeout_ms = 3000;
  int watch_interval_ms = 5000;
};

// `pipeline.streammux.*` — nvstreammux v2 (USE_NEW_NVSTREAMMUX=yes).
struct StreammuxConfig {
  int batch_size = 0;
  int batched_push_timeout = 40000; // µs
  bool sync_inputs = false;
  bool attach_sys_ts = true;
  uint64_t max_latency = 0;         // ns
  unsigned int num_surfaces_per_frame = 1;
  std::string config_file = "resources/ds/streammux/config_mux.txt";
};

// `pipeline.pgie` / `sgie_*` — nvinfer.
struct GieConfig {
  std::string config;
  int unique_id = 0;
};

struct PreprocessConfig {
  std::string config = "resources/ds/infer/config_preprocess_warp_plate.txt";
  int unique_id = 6;
};

struct PgieConfig {
  std::string config = "resources/ds/infer/pgie_vehicle.txt";
  int unique_id = 1;
};

struct TrackerStageConfig {
  std::string ll_config = "resources/ds/tracker/config_tracker_NvDCF_perf.yml";
  std::string ll_lib =
      "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so";
  int width = 640;
  int height = 384;
};

struct SinkConfig {
  std::string type = "fake";
  std::string output_file = "tests/output/test_result.mp4";
};

// `pipeline.probe.*` — knobs plate_probe / char_assembler.
struct ProbeConfig {
  double plate_expand_ratio = 0.30;
  double anchor_bottom_ratio = 0.12;
  double char_iou_dedup = 0.40;
  double square_plate_ratio = 0.50;
  int min_plate_width = 20;
  double vehicle_bottom_pad = 10.0;
  float keypoint_min_confidence = 0.25f;
  bool publish_bbox = true;
};

// `pipeline.*` trong config.yaml — pipeline/probes dùng.
struct PipelineConfig {
  RtspConfig rtsp;
  StreammuxConfig streammux;
  PgieConfig pgie;
  GieConfig sgie_plate = {"resources/ds/infer/sgie1_plate_pose.txt", 2};
  PreprocessConfig preprocess_plate;
  GieConfig sgie_digit = {"resources/ds/infer/sgie2_digit.txt", 3};
  GieConfig sgie_helmet = {"resources/ds/infer/sgie3_helmet.txt", 4};
  TrackerStageConfig tracker;
  SinkConfig sink;
  ProbeConfig probe;
};

// `violation.helmet.*` — vi phạm không đội mũ bảo hiểm (chỉ xe máy).
struct HelmetViolationConfig {
  bool enabled = true;
  int no_helmet_class_id = 0;  // class 0 của helmet model = người không đội mũ
  int min_hits = 1;            // số frame tối thiểu detect được mới chốt vi phạm
};

// `violation.wrong_lane.*` — vi phạm đi sai làn (theo tên zone *_LANE, mọi loại xe).
struct LaneViolationConfig {
  bool enabled = true;
  int min_hits = 1;  // số frame tối thiểu detect được mới chốt vi phạm
};

struct ViolationConfig {
  HelmetViolationConfig helmet;
  LaneViolationConfig wrong_lane;
};

struct MqttConfig {
  std::string host;
  int port = 1883;
  std::string username;
  std::string password;
  int keep_alive = 60;
  int reconnect_interval_s = 5;

  std::string company_id = "1";
  std::string ai_modules = "PLATE";

  std::string camera_list_tpl = "smart_vms/cameras/company/{company_id}";
  std::string get_polygon_tpl = "smart_vms/cameras/{camera_code}/zones";
  std::string get_violations_tpl =
      "smart_vms/ai_config/state/{camera_id}/{ai_modules}/violations";
  std::string pub_bbox_tpl = "smart_vms/ai/bbox/{camera_code}";
  std::string pub_event_tpl = "smart_vms/ai_events/{ai_modules}";
};

struct SnapshotApiConfig {
  std::string base_url;
  std::string endpoint = "/upload/file";
};

class Config {
 public:
  static Config load(const std::string& root_dir);

  const std::string& rootDir() const { return root_dir_; }
  const std::string& aiModule() const { return ai_module_; }
  const PlateConfig& plate() const { return plate_; }
  const PipelineConfig& pipeline() const { return pipeline_; }
  const ViolationConfig& violation() const { return violation_; }
  const MqttConfig& mqtt() const { return mqtt_; }
  const SnapshotApiConfig& snapshotApi() const { return snapshot_; }

  PipelineConfig& mutablePipeline() { return pipeline_; }
  PlateConfig& mutablePlate() { return plate_; }

  std::string cameraListTopic() const;
  std::string zonesTopic(const std::string& camera_code) const;
  std::string violationsTopic(const std::string& camera_id) const;
  std::string bboxTopic(const std::string& camera_code) const;
  std::string eventTopic() const;
  std::string uploadUrl() const;

  std::string resolvePath(const std::string& path) const;

 private:
  std::string root_dir_;
  std::string ai_module_ = "PLATE";
  PlateConfig plate_;
  PipelineConfig pipeline_;
  ViolationConfig violation_;
  MqttConfig mqtt_;
  SnapshotApiConfig snapshot_;
};

}  // namespace vehicle
