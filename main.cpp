#include <glib-unix.h>
#include <gst/gst.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common/config.h"
#include "common/logging.h"
#include "communication/event_publisher.h"
#include "communication/http_uploader.h"
#include "communication/mqtt_client.h"
#include "communication/vms_client.h"
#include "pipeline/pipeline.h"
#include "pipeline/watchers.h"
#include "probes/plate_probe.h"
#include "utils/latency.h"

namespace {

struct Options {
  std::string root_dir;
  std::vector<std::string> sources;  // override RTSP/file, bỏ qua camera list
  std::string sink;
  double camera_timeout_s = 15.0;
  bool dry_run = false;
  int max_recognize = 0;   // 0 = giữ theo config.yaml
};

void printUsage(const char* argv0) {
  std::cout
      << "Dùng: " << argv0 << " [tuỳ chọn]\n"
      << "  --root <dir>          Thư mục project chứa resources/ (mặc định: $VEHICLE_ROOT hoặc .)\n"
      << "  --source <uri>        Chạy với nguồn chỉ định (rtsp://… hoặc file), lặp được\n"
      << "  --sink fake|file      Ghi đè pipeline.sink.type trong config.yaml\n"
      << "  --camera-timeout <s>  Thời gian chờ camera_list từ MQTT (mặc định 15)\n"
      << "  --dry-run             Không upload/publish, chỉ log payload (verify offline)\n"
      << "  --max-recognize <n>   Ghi đè plate.max_recognize_times\n"
      << "  --help                In hướng dẫn này\n";
}

bool parseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = (i + 1) < argc;
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return false;
    }
    if (arg == "--root" && has_value) {
      options->root_dir = argv[++i];
    } else if (arg == "--source" && has_value) {
      options->sources.push_back(argv[++i]);
    } else if (arg == "--sink" && has_value) {
      options->sink = argv[++i];
    } else if (arg == "--camera-timeout" && has_value) {
      options->camera_timeout_s = std::atof(argv[++i]);
    } else if (arg == "--dry-run") {
      options->dry_run = true;
    } else if (arg == "--max-recognize" && has_value) {
      options->max_recognize = std::atoi(argv[++i]);
    } else if (arg == "--muxer-size" && has_value) {
      ++i;
      std::cerr << "cảnh báo: --muxer-size bỏ qua (nvstreammux v2 giữ resolution nguồn)\n";
    } else {
      std::cerr << "Tham số không hợp lệ: " << arg << "\n";
      printUsage(argv[0]);
      return false;
    }
  }
  if (options->root_dir.empty()) {
    const char* env_root = std::getenv("VEHICLE_ROOT");
    options->root_dir = env_root != nullptr ? env_root : ".";
  }
  return true;
}

GMainLoop* g_loop = nullptr;

gboolean onSignal(gpointer) {
  LOG_INFO("nhận tín hiệu dừng — thoát");
  if (g_loop != nullptr) g_main_loop_quit(g_loop);
  return G_SOURCE_REMOVE;
}

struct PipelineConnIdle {
  vehicle::pipeline::Pipeline* pipeline = nullptr;
  bool resume = false;
};

gboolean onMqttConnectionIdle(gpointer data) {
  auto* ctx = static_cast<PipelineConnIdle*>(data);
  if (ctx->pipeline != nullptr) {
    if (ctx->resume)
      ctx->pipeline->resume();
    else
      ctx->pipeline->pause();
  }
  delete ctx;
  return G_SOURCE_REMOVE;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, &options)) return 0;

  gst_init(&argc, &argv);

  std::unique_ptr<vehicle::Config> config;
  try {
    config = std::make_unique<vehicle::Config>(vehicle::Config::load(options.root_dir));
  } catch (const std::exception& e) {
    LOG_ERROR("config: %s", e.what());
    return 1;
  }
  if (!options.sink.empty()) config->mutablePipeline().sink.type = options.sink;
  if (options.max_recognize > 0) {
    config->mutablePlate().max_recognize_times = options.max_recognize;
    LOG_INFO("ghi đè max_recognize_times=%d", options.max_recognize);
  }
  if (vehicle::utils::latencyEnabled()) {
    LOG_DEBUG("latency: VEHICLE_LATENCY bật — log từng đoạn (vote/encode/upload/mqtt/E2E)");
    LOG_DEBUG("latency: set NVDS_ENABLE_LATENCY_MEASUREMENT=1 để đo DS pipeline (decoder→probe)");
  }

  // Nguồn: --source (ưu tiên) hoặc camera list từ VMS + watchers.
  const bool dynamic_sources = options.sources.empty();

  vehicle::comm::MqttClient mqtt;
  bool mqtt_ok = false;
  if (dynamic_sources) {
    // Dynamic mode: bắt buộc MQTT — retry đến khi broker sẵn sàng.
    const int interval_s = std::max(1, config->mqtt().reconnect_interval_s);
    while (!mqtt.connect(config->mqtt())) {
      LOG_WARN("mqtt: chờ broker — retry sau %ds", interval_s);
      std::this_thread::sleep_for(std::chrono::seconds(interval_s));
    }
    mqtt_ok = true;
  } else {
    mqtt_ok = mqtt.connect(config->mqtt());
    if (!mqtt_ok)
      LOG_WARN("mqtt: không kết nối được broker — chạy tiếp (--source)");
  }

  vehicle::comm::VmsClient vms(*config, &mqtt);
  if (mqtt_ok && !vms.start()) LOG_WARN("mqtt: subscribe camera_list/zones thất bại");

  vehicle::comm::HttpUploader uploader(*config);
  vehicle::comm::EventPublisher publisher(*config, &mqtt, &uploader);
  publisher.setDryRun(options.dry_run);
  if (options.dry_run) LOG_INFO("chế độ dry-run: không upload/publish ra ngoài");
  vehicle::probes::PlateProbe probe(*config, &publisher);

  std::vector<vehicle::Camera> cameras;
  std::vector<vehicle::pipeline::SourceSpec> sources;
  if (!dynamic_sources) {
    for (size_t i = 0; i < options.sources.size(); ++i) {
      vehicle::Camera camera;
      camera.code = "local_" + std::to_string(i);
      camera.id = camera.code;
      camera.restream_url = options.sources[i];
      cameras.push_back(camera);
      sources.push_back({options.sources[i], camera.code});
    }
    LOG_INFO("chạy với %zu source chỉ định qua --source (không dynamic MQTT)",
             sources.size());
  } else {
    if (!vms.waitForCameras(options.camera_timeout_s)) {
      LOG_ERROR("không nhận được camera nào cho ai_modules=%s sau %.0fs",
                config->aiModule().c_str(), options.camera_timeout_s);
      return 1;
    }
    for (const vehicle::Camera& camera : vms.cameras()) {
      if (camera.restream_url.empty()) {
        LOG_WARN("camera %s: thiếu restream_urls.%s — bỏ qua", camera.code.c_str(),
                 config->aiModule().c_str());
        continue;
      }
      cameras.push_back(camera);
    }
    if (cameras.empty()) {
      LOG_ERROR("không có camera nào có restream_urls.%s", config->aiModule().c_str());
      return 1;
    }
  }

  probe.setCameras(cameras);
  probe.start();

  vehicle::pipeline::Pipeline pipeline(*config, &probe);
  if (!pipeline.build(sources)) {
    LOG_ERROR("không dựng được pipeline");
    return 1;
  }

  g_loop = g_main_loop_new(nullptr, FALSE);
  pipeline.setMainLoop(g_loop);
  g_unix_signal_add(SIGINT, onSignal, nullptr);
  g_unix_signal_add(SIGTERM, onSignal, nullptr);

  if (!pipeline.start()) return 1;

  // Mất MQTT → PAUSED; reconnect + re-subscribe → PLAYING (trên main loop).
  if (mqtt_ok) {
    mqtt.setConnectionCallback([&pipeline](bool ok) {
      g_idle_add(onMqttConnectionIdle, new PipelineConnIdle{&pipeline, ok});
    });
  }

  std::unique_ptr<vehicle::pipeline::CameraWatcher> camera_watcher;
  std::unique_ptr<vehicle::pipeline::PolygonWatcher> polygon_watcher;

  // PolygonWatcher: luôn bật khi có MQTT (kể cả --source, nếu VMS vẫn gửi zones).
  polygon_watcher = std::make_unique<vehicle::pipeline::PolygonWatcher>(&probe);
  polygon_watcher->start("polygon_watcher");
  vms.setZonesCallback([&polygon_watcher](const vehicle::ZoneSet& zones) {
    if (polygon_watcher) polygon_watcher->setDesired(zones);
  });
  // Seed ROI đã nhận trong lúc waitForCameras.
  for (const vehicle::Camera& camera : cameras) {
    vehicle::ZoneSet zones = vms.zones(camera.code);
    if (!zones.camera_code.empty() || !zones.zones.empty()) {
      if (zones.camera_code.empty()) zones.camera_code = camera.code;
      polygon_watcher->setDesired(std::move(zones));
    }
  }

  if (dynamic_sources) {
    camera_watcher = std::make_unique<vehicle::pipeline::CameraWatcher>(*config, &pipeline);
    camera_watcher->setDesired(cameras);
    camera_watcher->start("camera_watcher");
    vms.setCamerasCallback([&camera_watcher](const std::vector<vehicle::Camera>& next) {
      if (camera_watcher) camera_watcher->setDesired(next);
    });
    LOG_INFO("mqtt: bật CameraWatcher + PolygonWatcher (capacity=%u)",
             pipeline.maxSources());
  } else {
    LOG_INFO("mqtt: bật PolygonWatcher (--source, không dynamic camera)");
  }

  g_main_loop_run(g_loop);

  mqtt.setConnectionCallback(nullptr);
  vms.setCamerasCallback(nullptr);
  vms.setZonesCallback(nullptr);
  if (camera_watcher) camera_watcher->stop();
  if (polygon_watcher) polygon_watcher->stop();
  pipeline.stop();
  probe.stop();
  mqtt.disconnect();
  g_main_loop_unref(g_loop);
  g_loop = nullptr;
  LOG_INFO("thoát sạch");
  return 0;
}
