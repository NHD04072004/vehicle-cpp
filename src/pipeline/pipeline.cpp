#include "pipeline/pipeline.h"

#include <sys/stat.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <string>

#include "common/logging.h"

namespace vehicle {
namespace pipeline {
namespace {

bool fileExists(const std::string& path) {
  struct stat st{};
  return !path.empty() && ::stat(path.c_str(), &st) == 0;
}

void configureGpuOsd(GstElement* osd) {
  if (osd == nullptr) return;
  g_object_set(G_OBJECT(osd), "display-text", FALSE, "display-bbox", FALSE, nullptr);
  GObjectClass* klass = G_OBJECT_GET_CLASS(osd);
  if (g_object_class_find_property(klass, "display-mask") != nullptr)
    g_object_set(G_OBJECT(osd), "display-mask", FALSE, nullptr);
}

std::string dirName(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

// Resolve đường dẫn tương đối trong file config nvinfer/nvdspreprocess.
std::string resolveConfigPath(const std::string& base, const std::string& value) {
  if (value.empty()) return value;
  if (value.front() == '/') return value;
  return base + "/" + value;
}

bool configKeyPathExists(const std::string& config_path, const char* want_key) {
  if (!fileExists(config_path)) return false;
  std::ifstream file(config_path);
  if (!file) return false;
  const std::string base = dirName(config_path);
  std::string line;
  while (std::getline(file, line)) {
    const size_t comment = line.find_first_not_of(" \t");
    if (comment == std::string::npos || line[comment] == '#') continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string key = line.substr(comment, eq - comment);
    while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
    if (key != want_key) continue;
    std::string value = line.substr(eq + 1);
    while (!value.empty() && (value.back() == '\r' || value.back() == ' ')) value.pop_back();
    if (fileExists(resolveConfigPath(base, value))) return true;
  }
  return false;
}

// nvinfer chỉ chạy được khi file config tồn tại VÀ trỏ tới model có thật.
bool inferReady(const std::string& config_path) {
  if (!fileExists(config_path)) return false;
  std::ifstream file(config_path);
  if (!file) return false;
  const std::string base = dirName(config_path);
  std::string line;
  while (std::getline(file, line)) {
    const size_t comment = line.find_first_not_of(" \t");
    if (comment == std::string::npos || line[comment] == '#') continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string key = line.substr(comment, eq - comment);
    while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
    if (key != "onnx-file" && key != "model-engine-file" && key != "tlt-encoded-model" &&
        key != "uff-file" && key != "caffemodel")
      continue;
    std::string value = line.substr(eq + 1);
    while (!value.empty() && (value.back() == '\r' || value.back() == ' ')) value.pop_back();
    if (fileExists(resolveConfigPath(base, value))) return true;
  }
  return false;
}

// nvdspreprocess: cần config + custom-lib (.so) tồn tại.
bool preprocessReady(const std::string& config_path) {
  return configKeyPathExists(config_path, "custom-lib-path");
}

// nvurisrcbin cần uri có scheme; đường dẫn file thô → file://
std::string toUri(const std::string& source) {
  if (source.find("://") != std::string::npos) return source;
  if (!source.empty() && source.front() == '/') return "file://" + source;
  return source;
}

Camera cameraFromSpec(const SourceSpec& spec) {
  Camera camera;
  camera.code = spec.camera_code;
  camera.id = spec.camera_code;
  camera.restream_url = spec.uri;
  return camera;
}

bool nameStartsWith(const char* name, const char* prefix) {
  if (name == nullptr || prefix == nullptr) return false;
  const size_t n = std::strlen(prefix);
  return std::strncmp(name, prefix, n) == 0;
}

}  // namespace

Pipeline::Pipeline(const Config& config, probes::PlateProbe* probe)
    : config_(config), probe_(probe) {}

Pipeline::~Pipeline() { stop(); }

GstElement* Pipeline::makeElement(const char* factory, const char* name) {
  GstElement* element = gst_element_factory_make(factory, name);
  if (element == nullptr) LOG_ERROR("pipeline: không tạo được element %s", factory);
  return element;
}

void Pipeline::onPadAdded(GstElement* src, GstPad* pad, gpointer data) {
  GstPad* sink_pad = static_cast<GstPad*>(data);
  if (gst_pad_is_linked(sink_pad)) return;
  GstCaps* caps = gst_pad_get_current_caps(pad);
  if (caps == nullptr) caps = gst_pad_query_caps(pad, nullptr);
  const GstStructure* structure = gst_caps_get_structure(caps, 0);
  const gchar* name = gst_structure_get_name(structure);
  if (g_strrstr(name, "video") != nullptr) {
    if (gst_pad_link(pad, sink_pad) != GST_PAD_LINK_OK)
      LOG_ERROR("pipeline: link %s → muxer thất bại", GST_ELEMENT_NAME(src));
  }
  gst_caps_unref(caps);
}

void Pipeline::configureRtsp(GstElement* source, const std::string& uri) {
  const RtspConfig& rtsp = config_.pipeline().rtsp;
  g_object_set(G_OBJECT(source), "uri", uri.c_str(), "gpu-id", 0, "latency", rtsp.latency,
               "drop-on-latency", rtsp.drop_on_latency ? TRUE : FALSE, nullptr);

  GObjectClass* base_klass = G_OBJECT_GET_CLASS(source);
  if (rtsp.low_latency_mode &&
      g_object_class_find_property(base_klass, "low-latency-mode") != nullptr)
    g_object_set(G_OBJECT(source), "low-latency-mode", TRUE, nullptr);
  if (rtsp.num_extra_surfaces > 0 &&
      g_object_class_find_property(base_klass, "num-extra-surfaces") != nullptr)
    g_object_set(G_OBJECT(source), "num-extra-surfaces", rtsp.num_extra_surfaces, nullptr);
  if (rtsp.drop_frame_interval > 0 &&
      g_object_class_find_property(base_klass, "drop-frame-interval") != nullptr)
    g_object_set(G_OBJECT(source), "drop-frame-interval", rtsp.drop_frame_interval,
                 nullptr);

  if (uri.rfind("rtsp://", 0) != 0) return;

  g_object_set(G_OBJECT(source), "select-rtp-protocol", rtsp.select_rtp_protocol, nullptr);
  if (rtsp.reconnect_interval > 0)
    g_object_set(G_OBJECT(source), "rtsp-reconnect-interval", rtsp.reconnect_interval,
                 nullptr);

  GObjectClass* klass = G_OBJECT_GET_CLASS(source);
  if (g_object_class_find_property(klass, "rtsp-reconnect-attempts") != nullptr)
    g_object_set(G_OBJECT(source), "rtsp-reconnect-attempts", rtsp.reconnect_attempts,
                 nullptr);
}

int Pipeline::allocateSourceId(int preferred_id) {
  if (preferred_id >= 0 && preferred_id < static_cast<int>(max_sources_) &&
      free_ids_.count(static_cast<unsigned int>(preferred_id)) > 0) {
    free_ids_.erase(static_cast<unsigned int>(preferred_id));
    return preferred_id;
  }
  if (free_ids_.empty()) return -1;
  const unsigned int id = *free_ids_.begin();
  free_ids_.erase(free_ids_.begin());
  return static_cast<int>(id);
}

bool Pipeline::addSourceInternal(const Camera& camera, int preferred_id) {
  if (pipeline_ == nullptr || muxer_ == nullptr) return false;
  if (camera.code.empty()) {
    LOG_WARN("pipeline: bỏ qua camera thiếu code");
    return false;
  }
  if (camera.restream_url.empty()) {
    LOG_WARN("pipeline: camera %s thiếu restream_url — bỏ qua", camera.code.c_str());
    return false;
  }
  if (slots_.count(camera.code) > 0) {
    LOG_WARN("pipeline: camera %s đã tồn tại — dùng updateSource", camera.code.c_str());
    return false;
  }
  if (slots_.size() >= max_sources_) {
    LOG_WARN("pipeline: hết capacity (%u) — không thêm được %s", max_sources_,
             camera.code.c_str());
    return false;
  }

  const int id = allocateSourceId(preferred_id);
  if (id < 0) {
    LOG_ERROR("pipeline: không còn source_id trống");
    return false;
  }
  const unsigned int source_id = static_cast<unsigned int>(id);
  const std::string name = "source_" + std::to_string(source_id);
  GstElement* source = makeElement("nvurisrcbin", name.c_str());
  if (source == nullptr) {
    free_ids_.insert(source_id);
    return false;
  }

  const std::string uri = toUri(camera.restream_url);
  configureRtsp(source, uri);
  gst_bin_add(GST_BIN(pipeline_), source);

  const std::string pad_name = "sink_" + std::to_string(source_id);
  GstPad* mux_pad = gst_element_request_pad_simple(muxer_, pad_name.c_str());
  if (mux_pad == nullptr) {
    LOG_ERROR("pipeline: xin pad %s của muxer thất bại", pad_name.c_str());
    gst_bin_remove(GST_BIN(pipeline_), source);
    free_ids_.insert(source_id);
    return false;
  }
  g_signal_connect(source, "pad-added", G_CALLBACK(onPadAdded), mux_pad);

  SourceSlot slot;
  slot.source_id = source_id;
  slot.camera_code = camera.code;
  slot.uri = uri;
  slot.camera = camera;
  slot.element = source;
  slot.mux_pad = mux_pad;
  slots_[camera.code] = slot;

  if (probe_ != nullptr) probe_->bindCamera(source_id, camera);

  if (playing_) {
    if (!gst_element_sync_state_with_parent(source)) {
      LOG_ERROR("pipeline: sync state source_%u thất bại", source_id);
      removeSourceInternal(camera.code, false);
      return false;
    }
  }

  LOG_INFO("pipeline: add source[%u] camera=%s uri=%s", source_id, camera.code.c_str(),
           uri.c_str());
  return true;
}

bool Pipeline::removeSourceInternal(const std::string& camera_code, bool /*keep_slot_id*/) {
  auto it = slots_.find(camera_code);
  if (it == slots_.end()) return false;

  SourceSlot slot = it->second;
  slots_.erase(it);

  if (slot.element != nullptr) {
    gst_element_set_state(slot.element, GST_STATE_NULL);
    gst_bin_remove(GST_BIN(pipeline_), slot.element);
  }

  if (slot.mux_pad != nullptr) {
    gst_element_release_request_pad(muxer_, slot.mux_pad);
    gst_object_unref(slot.mux_pad);
  }

  free_ids_.insert(slot.source_id);
  if (probe_ != nullptr) probe_->unbindCamera(slot.source_id);

  LOG_INFO("pipeline: remove source[%u] camera=%s", slot.source_id, camera_code.c_str());
  return true;
}

bool Pipeline::addSource(const Camera& camera) { return addSourceInternal(camera, -1); }

bool Pipeline::removeSource(const std::string& camera_code) {
  return removeSourceInternal(camera_code, false);
}

bool Pipeline::updateSource(const Camera& camera) {
  auto it = slots_.find(camera.code);
  if (it == slots_.end()) {
    LOG_WARN("pipeline: update — không thấy camera %s", camera.code.c_str());
    return false;
  }

  const std::string new_uri = toUri(camera.restream_url);
  if (new_uri == it->second.uri) {
    it->second.camera = camera;
    if (probe_ != nullptr) probe_->updateCamera(it->second.source_id, camera);
    return true;
  }

  // URI đổi: recreate element, giữ source_id để probe/meta không lệch.
  const unsigned int keep_id = it->second.source_id;
  if (!removeSourceInternal(camera.code, false)) return false;
  return addSourceInternal(camera, static_cast<int>(keep_id));
}

void Pipeline::applyCameraList(const std::vector<Camera>& cameras) {
  if (pipeline_ == nullptr) return;

  std::map<std::string, Camera> desired;
  for (const Camera& camera : cameras) {
    if (camera.code.empty() || camera.restream_url.empty()) continue;
    desired[camera.code] = camera;
  }

  std::vector<std::string> to_remove;
  for (const auto& entry : slots_) {
    if (desired.count(entry.first) == 0) to_remove.push_back(entry.first);
  }
  for (const std::string& code : to_remove) removeSource(code);

  for (const auto& entry : desired) {
    const Camera& camera = entry.second;
    if (slots_.count(camera.code) == 0) {
      if (!addSource(camera))
        LOG_WARN("pipeline: applyCameraList — không thêm được %s", camera.code.c_str());
      continue;
    }
    if (!updateSource(camera))
      LOG_WARN("pipeline: applyCameraList — không cập nhật được %s", camera.code.c_str());
  }
}

bool Pipeline::buildSink(GstElement** first, GstElement** last) {
  const SinkConfig& sink_cfg = config_.pipeline().sink;
  GstElement* converter = makeElement("nvvideoconvert", "sink_converter");
  GstElement* osd = makeElement("nvdsosd", "osd");
  if (converter == nullptr || osd == nullptr) return false;
  g_object_set(G_OBJECT(converter), "nvbuf-memory-type", 0, nullptr);
  configureGpuOsd(osd);
  gst_bin_add_many(GST_BIN(pipeline_), converter, osd, nullptr);
  if (!gst_element_link(converter, osd)) return false;
  *first = converter;
  *last = osd;

  if (sink_cfg.type == "file") {
    GstElement* post = makeElement("nvvideoconvert", "encoder_converter");
    GstElement* encoder = makeElement("nvv4l2h264enc", "encoder");
    GstElement* parser = makeElement("h264parse", "parser");
    GstElement* muxer = makeElement("qtmux", "mp4mux");
    GstElement* sink = makeElement("filesink", "file_sink");
    if (post == nullptr || encoder == nullptr || parser == nullptr || muxer == nullptr ||
        sink == nullptr)
      return false;
    const std::string path = config_.resolvePath(sink_cfg.output_file);
    // async=0 bắt buộc khi dynamic source — tránh deadlock state transition.
    g_object_set(G_OBJECT(sink), "location", path.c_str(), "sync", FALSE, "async", FALSE,
                 nullptr);
    gst_bin_add_many(GST_BIN(pipeline_), post, encoder, parser, muxer, sink, nullptr);
    if (!gst_element_link_many(osd, post, encoder, parser, muxer, sink, nullptr)) {
      LOG_ERROR("pipeline: link nhánh ghi file thất bại");
      return false;
    }
    LOG_INFO("pipeline: sink=file → %s", path.c_str());
    return true;
  }

  if (sink_cfg.type != "fake")
    LOG_WARN("pipeline: sink.type='%s' không hỗ trợ — dùng fakesink (fake|file)",
             sink_cfg.type.c_str());

  GstElement* sink = makeElement("fakesink", "video_sink");
  if (sink == nullptr) return false;
  g_object_set(G_OBJECT(sink), "sync", FALSE, "async", FALSE, nullptr);
  gst_bin_add(GST_BIN(pipeline_), sink);
  if (!gst_element_link(osd, sink)) return false;
  LOG_INFO("pipeline: sink=fake");
  return true;
}

bool Pipeline::build(const std::vector<SourceSpec>& sources) {
  const PipelineConfig& cfg = config_.pipeline();
  const StreammuxConfig& mux = cfg.streammux;

  if (mux.batch_size > 0) {
    max_sources_ = static_cast<unsigned int>(mux.batch_size);
  } else if (!sources.empty()) {
    max_sources_ = static_cast<unsigned int>(sources.size());
  } else {
    LOG_ERROR("pipeline: không có source và batch_size=0 — không dựng được");
    return false;
  }

  if (sources.size() > max_sources_) {
    LOG_ERROR("pipeline: %zu source > capacity %u", sources.size(), max_sources_);
    return false;
  }

  // nvstreammux v2: cần USE_NEW_NVSTREAMMUX=yes.
  pipeline_ = gst_pipeline_new("vehicle-pipeline");
  muxer_ = makeElement("nvstreammux", "streammux");
  if (pipeline_ == nullptr || muxer_ == nullptr) return false;
  gst_bin_add(GST_BIN(pipeline_), muxer_);

  const int batch_size = static_cast<int>(max_sources_);
  g_object_set(G_OBJECT(muxer_), "batch-size", batch_size, "batched-push-timeout",
               mux.batched_push_timeout, "sync-inputs", mux.sync_inputs ? TRUE : FALSE,
               "attach-sys-ts", mux.attach_sys_ts ? TRUE : FALSE, "max-latency",
               static_cast<guint64>(mux.max_latency), "num-surfaces-per-frame",
               mux.num_surfaces_per_frame, nullptr);
  if (!mux.config_file.empty()) {
    const std::string mux_cfg = config_.resolvePath(mux.config_file);
    if (fileExists(mux_cfg))
      g_object_set(G_OBJECT(muxer_), "config-file-path", mux_cfg.c_str(), nullptr);
    else
      LOG_WARN("pipeline: thiếu streammux config %s — dùng property mặc định",
               mux_cfg.c_str());
  }
  free_ids_.clear();
  slots_.clear();
  for (unsigned int i = 0; i < max_sources_; ++i) free_ids_.insert(i);

  for (size_t i = 0; i < sources.size(); ++i) {
    if (!addSourceInternal(cameraFromSpec(sources[i]), static_cast<int>(i))) return false;
  }

  // Chuỗi suy luận: chỉ thêm stage có file config tồn tại.
  std::vector<GstElement*> stages;
  const std::string pgie_path = config_.resolvePath(cfg.pgie.config);
  if (inferReady(pgie_path)) {
    GstElement* pgie = makeElement("nvinfer", "pgie_vehicle");
    if (pgie == nullptr) return false;
    g_object_set(G_OBJECT(pgie), "config-file-path", pgie_path.c_str(), "unique-id",
                 cfg.pgie.unique_id, "batch-size", batch_size, nullptr);
    stages.push_back(pgie);

    GstElement* tracker = makeElement("nvtracker", "tracker");
    if (tracker == nullptr) return false;
    g_object_set(G_OBJECT(tracker), "ll-lib-file", cfg.tracker.ll_lib.c_str(),
                 "tracker-width", cfg.tracker.width, "tracker-height", cfg.tracker.height,
                 nullptr);
    const std::string tracker_cfg = config_.resolvePath(cfg.tracker.ll_config);
    if (fileExists(tracker_cfg))
      g_object_set(G_OBJECT(tracker), "ll-config-file", tracker_cfg.c_str(), nullptr);
    else
      LOG_WARN("pipeline: thiếu %s — tracker dùng cấu hình mặc định", tracker_cfg.c_str());
    stages.push_back(tracker);

    const std::string sgie1_path = config_.resolvePath(cfg.sgie_plate.config);
    bool have_plate = false;
    if (inferReady(sgie1_path)) {
      GstElement* sgie1 = makeElement("nvinfer", "sgie_plate");
      if (sgie1 == nullptr) return false;
      g_object_set(G_OBJECT(sgie1), "config-file-path", sgie1_path.c_str(), "unique-id",
                   cfg.sgie_plate.unique_id, "process-mode", 2, nullptr);
      stages.push_back(sgie1);
      have_plate = true;
    } else {
      LOG_WARN("pipeline: %s chưa có model — bỏ SGIE1 (không detect biển)", sgie1_path.c_str());
    }

    // Warp keypoints → tensor digit (sau plate, trước SGIE2). Skill DS: input-tensor-meta trên GObject.
    bool have_warp = false;
    const std::string pre_path = config_.resolvePath(cfg.preprocess_plate.config);
    if (have_plate && preprocessReady(pre_path)) {
      GstElement* pre = makeElement("nvdspreprocess", "preprocess_plate");
      if (pre == nullptr) return false;
      g_object_set(G_OBJECT(pre), "config-file", pre_path.c_str(), nullptr);
      stages.push_back(pre);
      have_warp = true;
      LOG_INFO("pipeline: warp preprocess %s (unique-id=%d)", pre_path.c_str(),
               cfg.preprocess_plate.unique_id);
    } else if (have_plate) {
      LOG_WARN("pipeline: %s chưa sẵn sàng — SGIE2 crop bbox (không warp)", pre_path.c_str());
    }

    const std::string sgie2_path = config_.resolvePath(cfg.sgie_digit.config);
    if (inferReady(sgie2_path)) {
      GstElement* sgie2 = makeElement("nvinfer", "sgie_digit");
      if (sgie2 == nullptr) return false;
      g_object_set(G_OBJECT(sgie2), "config-file-path", sgie2_path.c_str(), "unique-id",
                   cfg.sgie_digit.unique_id, "process-mode", 2, "input-tensor-meta",
                   have_warp ? TRUE : FALSE, nullptr);
      stages.push_back(sgie2);
    } else {
      LOG_WARN("pipeline: %s chưa có model — bỏ SGIE2 (không OCR ký tự)", sgie2_path.c_str());
    }

    // SGIE3 mũ bảo hiểm — PHẢI đứng sau mốc restore ROI.
    const std::string sgie3_path = config_.resolvePath(cfg.sgie_helmet.config);
    if (inferReady(sgie3_path)) {
      GstElement* sgie3 = makeElement("nvinfer", "sgie_helmet");
      if (sgie3 == nullptr) return false;
      g_object_set(G_OBJECT(sgie3), "config-file-path", sgie3_path.c_str(), "unique-id",
                   cfg.sgie_helmet.unique_id, "process-mode", 2, nullptr);
      stages.push_back(sgie3);
      LOG_INFO("pipeline: sgie_helmet %s (unique-id=%d, chỉ xe máy)", sgie3_path.c_str(),
               cfg.sgie_helmet.unique_id);
    } else {
      LOG_WARN("pipeline: %s chưa có model — bỏ SGIE3 (không bắn NO_HELMET)",
               sgie3_path.c_str());
    }
  } else {
    LOG_WARN("pipeline: %s chưa có model — chạy passthrough (chỉ decode + sink)",
             pgie_path.c_str());
  }

  GstElement* sink_first = nullptr;
  GstElement* sink_last = nullptr;
  if (!buildSink(&sink_first, &sink_last)) return false;

  GstElement* previous = muxer_;
  GstElement* tracker_el = nullptr;
  GstElement* sgie1_el = nullptr;
  GstElement* preprocess_el = nullptr;
  GstElement* sgie2_el = nullptr;
  for (GstElement* stage : stages) {
    gst_bin_add(GST_BIN(pipeline_), stage);
    if (!gst_element_link(previous, stage)) {
      LOG_ERROR("pipeline: link %s → %s thất bại", GST_ELEMENT_NAME(previous),
                GST_ELEMENT_NAME(stage));
      return false;
    }
    if (g_strcmp0(GST_ELEMENT_NAME(stage), "tracker") == 0) tracker_el = stage;
    if (g_strcmp0(GST_ELEMENT_NAME(stage), "sgie_plate") == 0) sgie1_el = stage;
    if (g_strcmp0(GST_ELEMENT_NAME(stage), "preprocess_plate") == 0) preprocess_el = stage;
    if (g_strcmp0(GST_ELEMENT_NAME(stage), "sgie_digit") == 0) sgie2_el = stage;
    previous = stage;
  }
  if (!gst_element_link(previous, sink_first)) {
    LOG_ERROR("pipeline: link %s → sink thất bại", GST_ELEMENT_NAME(previous));
    return false;
  }

  if (probe_ != nullptr && !stages.empty()) {
    // Bbox sớm: sau tracker (chỉ loại xe). OCR/event: sau SGIE cuối.
    if (tracker_el != nullptr) {
      GstPad* bbox_pad = gst_element_get_static_pad(tracker_el, "src");
      if (bbox_pad != nullptr) {
        probe_->attachBboxProbe(bbox_pad);
        gst_object_unref(bbox_pad);
      }
    }
    // Expand ROI xe trên sink SGIE1; restore SAU digit/warp — keypoints unletterbox theo ROI đã nới.
    if (sgie1_el != nullptr) {
      GstPad* expand_pad = gst_element_get_static_pad(sgie1_el, "sink");
      if (expand_pad != nullptr) {
        probe_->attachRoiExpandProbe(expand_pad);
        gst_object_unref(expand_pad);
      }
      GstElement* restore_el = sgie2_el != nullptr
                                   ? sgie2_el
                                   : (preprocess_el != nullptr ? preprocess_el : sgie1_el);
      GstPad* restore_pad = gst_element_get_static_pad(restore_el, "src");
      if (restore_pad != nullptr) {
        probe_->attachRoiRestoreProbe(restore_pad);
        gst_object_unref(restore_pad);
      }
    }
    GstPad* meta_pad = gst_element_get_static_pad(previous, "src");
    if (meta_pad != nullptr) {
      probe_->attachMetaProbe(meta_pad);
      gst_object_unref(meta_pad);
    }
    // JPEG full-frame sau nvdsosd — bbox đã blend trên GPU.
    if (sink_last != nullptr) {
      GstPad* image_pad = gst_element_get_static_pad(sink_last, "src");
      if (image_pad != nullptr) {
        probe_->attachImageProbe(image_pad);
        gst_object_unref(image_pad);
      }
    }
  }

  GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  bus_watch_id_ = gst_bus_add_watch(bus, onBusMessage, this);
  gst_object_unref(bus);
  return true;
}

bool Pipeline::isSourceElement(GstObject* obj) const {
  if (obj == nullptr) return false;
  // Lỗi có thể đến từ element con trong nvurisrcbin — đi lên tới source_*.
  for (GstObject* cur = obj; cur != nullptr; cur = GST_OBJECT_PARENT(cur)) {
    const gchar* name = GST_OBJECT_NAME(cur);
    if (nameStartsWith(name, "source_")) return true;
  }
  return false;
}

gboolean Pipeline::onBusMessage(GstBus*, GstMessage* msg, gpointer data) {
  auto* self = static_cast<Pipeline*>(data);
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
      // 0 source (MQTT tạm trống / gỡ hết): giữ app — chờ gắn lại / reconnect.
      // Còn source (vd. hết file --source): thoát bình thường.
      if (self->slots_.empty()) {
        LOG_WARN("pipeline: EOS với 0 source — không thoát, chờ camera/reconnect");
        break;
      }
      LOG_INFO("pipeline: EOS");
      if (self->loop_ != nullptr) g_main_loop_quit(self->loop_);
      break;
    case GST_MESSAGE_ERROR: {
      GError* error = nullptr;
      gchar* debug = nullptr;
      gst_message_parse_error(msg, &error, &debug);
      const bool from_source = self->isSourceElement(msg->src);
      if (from_source) {
        // Một RTSP lỗi: để nvurisrcbin tự reconnect — không tắt cả app.
        LOG_ERROR("pipeline: lỗi source %s: %s (%s) — giữ pipeline chạy",
                  GST_OBJECT_NAME(msg->src), error->message, debug ? debug : "");
      } else {
        LOG_ERROR("pipeline: lỗi từ %s: %s (%s)", GST_OBJECT_NAME(msg->src), error->message,
                  debug ? debug : "");
        if (self->loop_ != nullptr) g_main_loop_quit(self->loop_);
      }
      g_clear_error(&error);
      g_free(debug);
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError* error = nullptr;
      gchar* debug = nullptr;
      gst_message_parse_warning(msg, &error, &debug);
      LOG_WARN("pipeline: cảnh báo từ %s: %s", GST_OBJECT_NAME(msg->src), error->message);
      g_clear_error(&error);
      g_free(debug);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

bool Pipeline::start() {
  if (pipeline_ == nullptr) return false;
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    LOG_ERROR("pipeline: không chuyển được sang PLAYING");
    return false;
  }
  playing_ = true;
  return true;
}

bool Pipeline::pause() {
  if (pipeline_ == nullptr) return false;
  if (!playing_) return true;
  if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE) {
    LOG_ERROR("pipeline: không chuyển được sang PAUSED");
    return false;
  }
  playing_ = false;
  LOG_WARN("pipeline: PAUSED — chờ MQTT reconnect");
  return true;
}

bool Pipeline::resume() {
  if (pipeline_ == nullptr) return false;
  if (playing_) return true;
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    LOG_ERROR("pipeline: không resume được sang PLAYING");
    return false;
  }
  playing_ = true;
  LOG_INFO("pipeline: PLAYING — MQTT đã sẵn sàng");
  return true;
}

void Pipeline::stop() {
  if (pipeline_ == nullptr) return;
  playing_ = false;
  gst_element_send_event(pipeline_, gst_event_new_eos());
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  if (bus_watch_id_ != 0) {
    g_source_remove(bus_watch_id_);
    bus_watch_id_ = 0;
  }
  slots_.clear();
  free_ids_.clear();
  gst_object_unref(GST_OBJECT(pipeline_));
  pipeline_ = nullptr;
  muxer_ = nullptr;
  LOG_INFO("pipeline: đã dừng");
}

}  // namespace pipeline
}  // namespace vehicle
