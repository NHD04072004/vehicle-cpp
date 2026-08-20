#include "probes/plate_probe.h"

#include <gstnvdsmeta.h>
#include <nvbufsurface.h>
#include <nvds_latency_meta.h>
#include <nvds_obj_encode.h>
#include <nvdsmeta.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <strings.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "business/violation/constants.h"
#include "common/logging.h"
#include "probes/char_assembler.h"
#include "utils/geometry.h"
#include "utils/jpeg_draw.h"
#include "utils/latency.h"
#include "utils/plate_snapshot_warp.h"
#include "utils/string_utils.h"
#include "utils/time_utils.h"

namespace vehicle {
namespace probes {
namespace {

constexpr size_t kMaxCachedImages = 16;
constexpr size_t kMaxEmitQueue = 8;
constexpr int kJpegQuality = 80;
constexpr double kMemStatsIntervalS = 5.0;
// Hạn chờ crop đã submit encode. Đủ dài để JPEG kịp về ở image probe, đủ ngắn
// để encode fail không khoá track khỏi việc chụp lại.
constexpr double kCropPendingTtlS = 0.5;
constexpr const char* kPlateZoneName = "PLATE";
constexpr const char* kWrongLaneSnapshotKey = "__WRONG_LANE__";
constexpr const char* kWrongWaySnapshotKey = "__WRONG_WAY__";

bool isPlateZone(const Zone& zone) {
  if (!laneAllowedClasses(zone.name).empty()) return false;
  if (zone.hasAiModule(kPlateZoneName)) return true;
  return utils::toUpper(utils::trim(zone.name)) == kPlateZoneName;
}

bool memStatsEnabled() {
  static const bool enabled = [] {
    const char* v = std::getenv("VEHICLE_MEM_STATS");
    if (v == nullptr || v[0] == '\0') return false;
    if (std::strcmp(v, "1") == 0) return true;
    if (::strcasecmp(v, "true") == 0) return true;
    if (::strcasecmp(v, "yes") == 0) return true;
    if (::strcasecmp(v, "on") == 0) return true;
    return false;
  }();
  return enabled;
}

size_t jpegBytes(const std::map<uint64_t, TrackSnapshots>& banks) {
  size_t n = 0;
  for (const auto& track : banks) {
    for (const auto& sample : track.second.by_plate) {
      n += sample.second.full.data.size();
      n += sample.second.crop.data.size();
    }
  }
  return n;
}

size_t sampleCount(const std::map<uint64_t, TrackSnapshots>& banks) {
  size_t n = 0;
  for (const auto& track : banks) n += track.second.by_plate.size();
  return n;
}

BoundingBox toBox(const NvOSD_RectParams& rect) {
  BoundingBox box;
  box.x1 = rect.left;
  box.y1 = rect.top;
  box.x2 = rect.left + rect.width;
  box.y2 = rect.top + rect.height;
  return box;
}

// Ảnh bằng chứng vi phạm gắn với đúng thời điểm vi phạm (không phải chất lượng)
// nên không tái tạo được ở frame sau → luôn giữ lại.
bool isViolationSnapshotKey(const std::string& key) {
  return key == kWrongLaneSnapshotKey || key == kWrongWaySnapshotKey;
}

// Key được bảo vệ khỏi retainSingleSnapshot (giữ nguyên cả full lẫn crop).
bool isProtectedSnapshotKey(const std::string& key) {
  return isViolationSnapshotKey(key);
}

// Chỉ giữ 1 full-frame (ảnh phương tiện nặng) là của `prefer_key`; các key khác
// bị bỏ full nhưng GIỮ crop biển. Chuỗi biển nào thắng vote lúc chốt cũng vẫn
// còn crop của đúng frame đã đọc ra chuỗi đó.
// `now_s` để bỏ entry có hạn chờ crop đã quá hạn (encode fail, ảnh không về).
void retainSingleSnapshot(TrackSnapshots* bank, const std::string& prefer_key, double now_s) {
  if (bank == nullptr || prefer_key.empty() || bank->by_plate.size() <= 1) return;
  if (bank->by_plate.find(prefer_key) == bank->by_plate.end()) return;

  for (auto it = bank->by_plate.begin(); it != bank->by_plate.end();) {
    if (it->first == prefer_key || isProtectedSnapshotKey(it->first)) {
      ++it;
      continue;
    }
    // Crop đang bay trong pipeline: xoá bây giờ thì JPEG về sau rơi vào entry
    // mới không có full → coi như mất crop. Giữ lại, chỉ bỏ full nặng.
    if (it->second.crop.empty() && now_s >= it->second.crop_pending_until_s) {
      it = bank->by_plate.erase(it);  // không còn gì đáng giữ
      continue;
    }
    it->second.full = JpegImage{};  // bỏ full nặng, giữ crop biển
    ++it;
  }
}

// Có nên chụp crop cho `key` ở frame này không — hỏi thẳng KHO ẢNH.
// Chưa có ảnh và không có ảnh nào đang bay → chụp. Đã có ảnh → chỉ chụp khi
// frame này chấm điểm cao hơn. Đang chờ ảnh về → không submit trùng.
bool wantsBetterCrop(const std::map<uint64_t, TrackSnapshots>& banks, uint64_t track_id,
                     const std::string& key, const business::plate::CropScore& cand,
                     double now_s) {
  if (!cand.valid()) return false;
  auto bank_it = banks.find(track_id);
  if (bank_it == banks.end()) return true;
  auto it = bank_it->second.by_plate.find(key);
  if (it == bank_it->second.by_plate.end()) return true;
  // Chưa có ảnh: chụp, trừ khi lần submit trước còn trong hạn chờ.
  if (it->second.crop.empty()) return now_s >= it->second.crop_pending_until_s;
  return business::plate::cropScoreBetter(cand, it->second.crop_score);
}

void pruneSnapshotBanks(std::map<uint64_t, TrackSnapshots>* banks) {
  if (banks == nullptr) return;
  while (banks->size() > kMaxCachedImages) banks->erase(banks->begin());
}

void pruneOrphanSnapshots(business::plate::PlateRecognizer* manager,
                          std::map<uint64_t, TrackSnapshots>* banks) {
  if (manager == nullptr || banks == nullptr) return;
  for (auto it = banks->begin(); it != banks->end();) {
    if (!manager->hasTrack(it->first))
      it = banks->erase(it);
    else
      ++it;
  }
}

double meanCharConf(const PlateReading& reading) {
  if (reading.chars.empty()) return 0.0;
  double sum = 0.0;
  for (const CharReading& ch : reading.chars) sum += ch.confidence;
  return sum / static_cast<double>(reading.chars.size());
}

bool pickSnapshotImages(const TrackSnapshots& bank, const std::string& key, JpegImage* full,
                        JpegImage* crop, float* left = nullptr, float* top = nullptr,
                        float* width = nullptr, float* height = nullptr) {
  // Bắt buộc có full-frame. Chỉ crop → coi như chưa sẵn sàng.
  auto try_key = [&](const std::string& k) {
    auto it = bank.by_plate.find(k);
    if (it == bank.by_plate.end()) return false;
    if (it->second.full.empty()) return false;
    if (full != nullptr) *full = it->second.full;
    if (crop != nullptr && !it->second.crop.empty()) *crop = it->second.crop;
    if (left != nullptr) *left = it->second.left;
    if (top != nullptr) *top = it->second.top;
    if (width != nullptr) *width = it->second.width;
    if (height != nullptr) *height = it->second.height;
    return true;
  };

  if (!key.empty() && try_key(key)) return true;
  for (const auto& kv : bank.by_plate) {
    if (kv.first.empty() || kv.first == business::plate::kEmptyPlate) continue;
    if (isProtectedSnapshotKey(kv.first)) continue;
    if (try_key(kv.first)) return true;
  }
  for (const auto& kv : bank.by_plate) {
    if (isProtectedSnapshotKey(kv.first)) continue;
    if (try_key(kv.first)) return true;
  }
  return false;
}

// Crop dự phòng khi mẫu ảnh phương tiện được chọn không kèm crop.
// `plate` là chuỗi ĐÃ CHỐT: ưu tiên tuyệt đối crop của chính chuỗi đó để ảnh
// biển và kết quả nhận diện luôn khớp nhau. Chỉ khi chuỗi đã chốt không có crop
// nào mới đành lấy crop khác (tốt hơn là gửi event không có ảnh biển).
const JpegImage* fallbackCrop(const TrackSnapshots& bank, const std::string& plate,
                              bool* exact_match) {
  *exact_match = false;
  auto it = bank.by_plate.find(plate);
  if (it != bank.by_plate.end() && !it->second.crop.empty()) {
    *exact_match = true;
    return &it->second.crop;
  }
  for (const auto& kv : bank.by_plate) {
    if (isProtectedSnapshotKey(kv.first)) continue;
    if (!kv.second.crop.empty()) return &kv.second.crop;
  }
  return nullptr;
}

// RSS hiện tại (MB) — để đối chiếu số container với RAM thật trong cùng một dòng log.
size_t processRssMb() {
  FILE* f = std::fopen("/proc/self/statm", "r");
  if (f == nullptr) return 0;
  unsigned long size_pages = 0;
  unsigned long rss_pages = 0;
  const int n = std::fscanf(f, "%lu %lu", &size_pages, &rss_pages);
  std::fclose(f);
  if (n != 2) return 0;
  const size_t page_kb = static_cast<size_t>(sysconf(_SC_PAGESIZE)) / 1024;
  return (static_cast<size_t>(rss_pages) * page_kb) / 1024;
}

bool envOn(const char* name) {
  const char* v = std::getenv(name);
  return v != nullptr && (std::strcmp(v, "1") == 0 || ::strcasecmp(v, "true") == 0);
}

}  // namespace

PlateProbe::PlateProbe(const Config& config, comm::EventPublisher* publisher)
    : config_(config), publisher_(publisher) {
  enc_ctx_ = nvds_obj_enc_create_context(0);
  if (enc_ctx_ == nullptr)
    LOG_ERROR("probe: không tạo được object encoder — snapshot sẽ trống");
}

PlateProbe::~PlateProbe() {
  stop();
  if (enc_ctx_ != nullptr) {
    nvds_obj_enc_destroy_context(enc_ctx_);
    enc_ctx_ = nullptr;
  }
}

std::unique_ptr<business::plate::PlateRecognizer> PlateProbe::makeRecognizer() const {
  auto mgr = std::make_unique<business::plate::PlateRecognizer>(config_.plate());
  const WrongWayViolationConfig& ww = config_.violation().wrong_way;
  mgr->setWrongWayTiming(ww.enabled ? ww.settle_s : 0.0,
                         ww.enabled ? ww.wait_pair_s : 0.0);
  mgr->setWrongWayMaxAngle(ww.max_angle_deg);
  return mgr;
}

void PlateProbe::setCameras(const std::vector<Camera>& cameras) {
  std::lock_guard<std::mutex> lock(sources_mutex_);
  sources_.clear();
  for (size_t i = 0; i < cameras.size(); ++i) {
    auto state = std::make_unique<SourceState>();
    state->camera = cameras[i];
    state->manager = makeRecognizer();
    sources_[static_cast<unsigned int>(i)] = std::move(state);
  }
}

void PlateProbe::bindCamera(unsigned int source_id, const Camera& camera) {
  std::lock_guard<std::mutex> lock(sources_mutex_);
  auto& slot = sources_[source_id];
  if (slot == nullptr) {
    slot = std::make_unique<SourceState>();
    slot->manager = makeRecognizer();
  }
  slot->camera = camera;
}

void PlateProbe::unbindCamera(unsigned int source_id) {
  {
    std::lock_guard<std::mutex> lock(sources_mutex_);
    sources_.erase(source_id);
  }
  clearPendingFor(source_id);
  LOG_INFO("probe: unbind source_id=%u", source_id);
}

void PlateProbe::updateCamera(unsigned int source_id, const Camera& camera) {
  std::lock_guard<std::mutex> lock(sources_mutex_);
  auto it = sources_.find(source_id);
  if (it == sources_.end() || it->second == nullptr) {
    // Chưa bind — tạo mới.
    auto state = std::make_unique<SourceState>();
    state->camera = camera;
    state->manager = makeRecognizer();
    sources_[source_id] = std::move(state);
    return;
  }
  it->second->camera = camera;
}

void PlateProbe::clearPendingFor(unsigned int source_id) {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  for (auto it = pending_frames_.begin(); it != pending_frames_.end();) {
    if (it->first.first == source_id)
      it = pending_frames_.erase(it);
    else
      ++it;
  }
}

void PlateProbe::updateZones(const ZoneSet& zones) {
  std::lock_guard<std::mutex> lock(zones_mutex_);
  zones_[zones.camera_code] = zones;
}

void PlateProbe::attachBboxProbe(GstPad* pad) {
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, bboxProbeCb, this, nullptr);
}

void PlateProbe::attachRoiExpandProbe(GstPad* pad) {
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, roiExpandProbeCb, this, nullptr);
}

void PlateProbe::attachRoiRestoreProbe(GstPad* pad) {
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, roiRestoreProbeCb, this, nullptr);
}

void PlateProbe::attachMetaProbe(GstPad* pad) {
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, metaProbeCb, this, nullptr);
}

void PlateProbe::attachImageProbe(GstPad* pad) {
  gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, imageProbeCb, this, nullptr);
}

GstPadProbeReturn PlateProbe::bboxProbeCb(GstPad*, GstPadProbeInfo* info, gpointer data) {
  return static_cast<PlateProbe*>(data)->handleBbox(info);
}

GstPadProbeReturn PlateProbe::roiExpandProbeCb(GstPad*, GstPadProbeInfo* info, gpointer data) {
  return static_cast<PlateProbe*>(data)->handleRoiExpand(info);
}

GstPadProbeReturn PlateProbe::roiRestoreProbeCb(GstPad*, GstPadProbeInfo* info, gpointer data) {
  return static_cast<PlateProbe*>(data)->handleRoiRestore(info);
}

GstPadProbeReturn PlateProbe::metaProbeCb(GstPad*, GstPadProbeInfo* info, gpointer data) {
  return static_cast<PlateProbe*>(data)->handleMeta(info);
}

GstPadProbeReturn PlateProbe::imageProbeCb(GstPad*, GstPadProbeInfo* info, gpointer data) {
  return static_cast<PlateProbe*>(data)->handleImages(info);
}

PlateProbe::SourceState* PlateProbe::sourceState(unsigned int source_id) {
  std::lock_guard<std::mutex> lock(sources_mutex_);
  auto it = sources_.find(source_id);
  if (it == sources_.end()) return nullptr;
  return it->second.get();
}

PlateProbe::PlateZones PlateProbe::plateZonesFor(const std::string& camera_code, double frame_w,
                                                 double frame_h, double source_w,
                                                 double source_h) {
  PlateZones out;
  ZoneSet set;
  {
    std::lock_guard<std::mutex> lock(zones_mutex_);
    auto it = zones_.find(camera_code);
    if (it == zones_.end()) return out;  // configured=false → chưa nhận ROI từ VMS
    set = it->second;
  }
  out.configured = true;

  out.polygons.reserve(set.zones.size());
  for (const Zone& zone : set.zones) {
    if (!isPlateZone(zone)) continue;
    if (zone.normalized) {
      out.polygons.push_back(utils::scaleZone(zone.points, frame_w, frame_h));
      continue;
    }
    // Toạ độ pixel theo độ phân giải nguồn → quy về không gian của muxer.
    const double sx = (source_w > 0.0) ? frame_w / source_w : 1.0;
    const double sy = (source_h > 0.0) ? frame_h / source_h : 1.0;
    std::vector<Point> scaled;
    scaled.reserve(zone.points.size());
    for (const Point& p : zone.points) scaled.push_back({p.x * sx, p.y * sy});
    out.polygons.push_back(std::move(scaled));
  }

  if (out.polygons.empty()) warnMissingPlateZone(camera_code, set);
  return out;
}

void PlateProbe::warnMissingPlateZone(const std::string& camera_code, const ZoneSet& set) {
  {
    // Mỗi camera chỉ cảnh báo lại khi ZoneSet đổi version — tránh spam mỗi frame.
    std::lock_guard<std::mutex> lock(zones_mutex_);
    auto it = warned_zone_version_.find(camera_code);
    if (it != warned_zone_version_.end() && it->second == set.version) return;
    warned_zone_version_[camera_code] = set.version;
  }

  std::string names;
  for (const Zone& zone : set.zones) {
    if (!names.empty()) names += ", ";
    names += zone.name.empty() ? "<trống>" : ("'" + zone.name + "'");
    names += "(ai_modules=";
    if (zone.ai_modules.empty()) {
      names += "<trống>";
    } else {
      for (size_t i = 0; i < zone.ai_modules.size(); ++i) {
        if (i > 0) names += "|";
        names += zone.ai_modules[i];
      }
    }
    names += ")";
  }
  if (names.empty()) names = "<không có zone nào>";
  LOG_WARN(
      "zones[%s]: có %zu polygon nhưng không zone nào thuộc module '%s' (đã nhận: %s) — "
      "tắt nhận diện biển/vi phạm cho camera này, chỉ giữ bbox tracking",
      camera_code.c_str(), set.zones.size(), kPlateZoneName, names.c_str());
}

std::vector<PlateProbe::LanePolygon> PlateProbe::lanePolygonsFor(
    const std::string& camera_code, double frame_w, double frame_h, double source_w,
    double source_h) {
  ZoneSet set;
  {
    std::lock_guard<std::mutex> lock(zones_mutex_);
    auto it = zones_.find(camera_code);
    if (it == zones_.end()) return {};
    set = it->second;
  }

  std::vector<LanePolygon> lanes;
  for (const Zone& zone : set.zones) {
    const std::set<int> allowed = laneAllowedClasses(zone.name);
    if (allowed.empty()) continue;  // không phải zone *_LANE hợp lệ

    LanePolygon lane;
    lane.allowed_classes = allowed;
    lane.zone_name = zone.name;
    if (zone.normalized) {
      lane.polygon = utils::scaleZone(zone.points, frame_w, frame_h);
    } else {
      // Toạ độ pixel theo độ phân giải nguồn → quy về không gian của muxer.
      const double sx = (source_w > 0.0) ? frame_w / source_w : 1.0;
      const double sy = (source_h > 0.0) ? frame_h / source_h : 1.0;
      lane.polygon.reserve(zone.points.size());
      for (const Point& p : zone.points) lane.polygon.push_back({p.x * sx, p.y * sy});
    }
    lanes.push_back(std::move(lane));
  }
  return lanes;
}

std::vector<business::plate::WrongWayLine> PlateProbe::wrongWayLinesFor(
    const std::string& camera_code, double frame_w, double frame_h, double source_w,
    double source_h) {
  ZoneSet set;
  {
    std::lock_guard<std::mutex> lock(zones_mutex_);
    auto it = zones_.find(camera_code);
    if (it == zones_.end()) return {};
    set = it->second;
  }

  std::vector<business::plate::WrongWayLine> lines;
  for (const Line& line : set.lines) {
    if (!isReverseDirectionLine(line.name)) continue;
    // Không có chiều đi đúng → không kết luận được ngược chiều.
    if (!line.has_direction) {
      warnLineWithoutDirection(camera_code, set, line.name);
      continue;
    }
    if (line.points.size() < 2) continue;

    business::plate::WrongWayLine out;
    out.name = line.name;
    if (line.normalized) {
      out.a = {line.points[0].x * frame_w, line.points[0].y * frame_h};
      out.b = {line.points[1].x * frame_w, line.points[1].y * frame_h};
    } else {
      const double sx = (source_w > 0.0) ? frame_w / source_w : 1.0;
      const double sy = (source_h > 0.0) ? frame_h / source_h : 1.0;
      out.a = {line.points[0].x * sx, line.points[0].y * sy};
      out.b = {line.points[1].x * sx, line.points[1].y * sy};
    }
    // direction cùng hệ toạ độ với points → scale y như điểm (chỉ cần đúng dấu).
    out.direction = {line.direction.x * frame_w, line.direction.y * frame_h};
    lines.push_back(std::move(out));
  }
  return lines;
}

bool PlateProbe::hasWrongWayLine(const std::string& camera_code) {
  std::lock_guard<std::mutex> lock(zones_mutex_);
  auto it = zones_.find(camera_code);
  if (it == zones_.end()) return false;
  for (const Line& line : it->second.lines) {
    if (isReverseDirectionLine(line.name) && line.has_direction) return true;
  }
  return false;
}

void PlateProbe::warnLineWithoutDirection(const std::string& camera_code, const ZoneSet& set,
                                          const std::string& line_name) {
  std::lock_guard<std::mutex> lock(zones_mutex_);
  auto it = warned_line_version_.find(camera_code);
  if (it != warned_line_version_.end() && it->second == set.version) return;
  warned_line_version_[camera_code] = set.version;
  LOG_WARN(
      "zones[%s]: line '%s' thiếu direction_vector/has_direction — không xác định được "
      "chiều đi đúng, bỏ qua vi phạm %s",
      camera_code.c_str(), line_name.c_str(), business::violation::kWrongWay);
}

namespace {

void logComponentLatency(NvDsBatchMeta* batch, const char* label) {
  if (batch == nullptr) return;
  for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
       l_frame = l_frame->next) {
    auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
    for (NvDsUserMetaList* l_user = frame->frame_user_meta_list; l_user != nullptr;
         l_user = l_user->next) {
      auto* user_meta = static_cast<NvDsUserMeta*>(l_user->data);
      if (user_meta->base_meta.meta_type != NVDS_LATENCY_MEASUREMENT_META) continue;
      auto* comp = static_cast<NvDsMetaCompLatency*>(user_meta->user_meta_data);
      if (comp == nullptr) continue;
      utils::latencyLog("comp_%s src=%u frame=%u %s: %.1fms", label, comp->source_id,
                        comp->frame_num, comp->component_name,
                        comp->out_system_timestamp - comp->in_system_timestamp);
    }
    return;  // 1 frame/batch là đủ để thấy chặng nào tốn
  }
}

void maybeLogDsLatency(GstBuffer* buf, NvDsBatchMeta* batch, int batch_capacity,
                       const char* label) {
  if (!utils::latencyEnabled()) return;
  static const bool ds_measure = envOn("NVDS_ENABLE_LATENCY_MEASUREMENT");
  if (!ds_measure) return;

  // Mỗi label log thưa ~2s để dễ so tracker vs full pipeline.
  static double last_bbox_s = 0.0;
  static double last_meta_s = 0.0;
  const double now_s = utils::monotonicSeconds();
  double* last = (std::strcmp(label, "tracker") == 0) ? &last_bbox_s : &last_meta_s;
  if (now_s - *last < 2.0) return;
  *last = now_s;

  const guint capacity = static_cast<guint>(std::max(1, batch_capacity));
  const guint alloc_n = std::max(capacity, 32u);
  std::vector<NvDsFrameLatencyInfo> latency_info(alloc_n);
  std::memset(latency_info.data(), 0, sizeof(NvDsFrameLatencyInfo) * alloc_n);
  const guint n = nvds_measure_buffer_latency(buf, latency_info.data());
  for (guint i = 0; i < n && i < alloc_n; ++i) {
    utils::latencyLog("ds_%s src=%u frame=%u latency=%.1fms", label, latency_info[i].source_id,
                      latency_info[i].frame_num, latency_info[i].latency);
  }

  static const bool comp_measure = envOn("NVDS_ENABLE_COMPONENT_LATENCY_MEASUREMENT");
  if (comp_measure) logComponentLatency(batch, label);
}

}  // namespace

namespace {

// Ghi rect float-safe: left+width ≤ frame
void setClampedRect(NvOSD_RectParams* rect, double x1, double y1, double x2, double y2,
                    double frame_w, double frame_h) {
  if (rect == nullptr || frame_w <= 1.0 || frame_h <= 1.0) return;
  const double left = std::max(0.0, std::min(x1, frame_w - 1.0));
  const double top = std::max(0.0, std::min(y1, frame_h - 1.0));
  const double right = std::max(left + 1.0, std::min(x2, frame_w));
  const double bottom = std::max(top + 1.0, std::min(y2, frame_h));
  float w = static_cast<float>(right - left);
  float h = static_cast<float>(bottom - top);
  rect->left = static_cast<float>(left);
  rect->top = static_cast<float>(top);
  if (rect->left + w > static_cast<float>(frame_w))
    w = static_cast<float>(frame_w) - rect->left;
  if (rect->top + h > static_cast<float>(frame_h))
    h = static_cast<float>(frame_h) - rect->top;
  rect->width = std::max(0.0f, w);
  rect->height = std::max(0.0f, h);
}

constexpr gint64 kOrigRectTag = 0x5645484f524543LL;  // "VEHOREC"

gint64 packFloats(float a, float b) {
  uint32_t ua = 0;
  uint32_t ub = 0;
  std::memcpy(&ua, &a, sizeof(ua));
  std::memcpy(&ub, &b, sizeof(ub));
  return static_cast<gint64>((static_cast<uint64_t>(ua) << 32) | static_cast<uint64_t>(ub));
}

void unpackFloats(gint64 packed, float* a, float* b) {
  const uint64_t raw = static_cast<uint64_t>(packed);
  const uint32_t ua = static_cast<uint32_t>(raw >> 32);
  const uint32_t ub = static_cast<uint32_t>(raw & 0xffffffffULL);
  std::memcpy(a, &ua, sizeof(ua));
  std::memcpy(b, &ub, sizeof(ub));
}

void saveOrigRect(NvDsObjectMeta* obj) {
  if (obj == nullptr) return;
  obj->misc_obj_info[0] = kOrigRectTag;
  obj->misc_obj_info[1] = packFloats(obj->rect_params.left, obj->rect_params.top);
  obj->misc_obj_info[2] = packFloats(obj->rect_params.width, obj->rect_params.height);
}

// Trả bbox về giá trị gốc; no-op nếu object không do probe expand này đánh dấu.
void restoreOrigRect(NvDsObjectMeta* obj) {
  if (obj == nullptr || obj->misc_obj_info[0] != kOrigRectTag) return;
  unpackFloats(obj->misc_obj_info[1], &obj->rect_params.left, &obj->rect_params.top);
  unpackFloats(obj->misc_obj_info[2], &obj->rect_params.width, &obj->rect_params.height);
  obj->misc_obj_info[0] = 0;  // tránh restore lặp nếu buffer qua probe nhiều lần
}

void clampRectInFrame(NvOSD_RectParams* rect, double frame_w, double frame_h) {
  if (rect == nullptr) return;
  setClampedRect(rect, rect->left, rect->top, rect->left + rect->width,
                 rect->top + rect->height, frame_w, frame_h);
}

}  // namespace

GstPadProbeReturn PlateProbe::handleRoiExpand(GstPadProbeInfo* info) {
  GstBuffer* buf = static_cast<GstBuffer*>(info->data);
  NvDsBatchMeta* batch = gst_buffer_get_nvds_batch_meta(buf);
  if (batch == nullptr) return GST_PAD_PROBE_OK;

  const ProbeConfig& probe_cfg = config_.pipeline().probe;
  const int pgie_id = config_.pipeline().pgie.unique_id;
  if (probe_cfg.plate_expand_ratio <= 0.0 && probe_cfg.vehicle_bottom_pad <= 0)
    return GST_PAD_PROBE_OK;

  for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
       l_frame = l_frame->next) {
    auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
    if (frame->source_frame_width == 0 || frame->source_frame_height == 0) continue;
    const double frame_w = static_cast<double>(frame->source_frame_width);
    const double frame_h = static_cast<double>(frame->source_frame_height);

    for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
      auto* obj = static_cast<NvDsObjectMeta*>(l_obj->data);
      if (static_cast<int>(obj->unique_component_id) != pgie_id) continue;
      if (obj->parent != nullptr) continue;

      saveOrigRect(obj);

      BoundingBox box = toBox(obj->rect_params);
      box.y2 += probe_cfg.vehicle_bottom_pad;
      if (probe_cfg.plate_expand_ratio > 0.0)
        box = utils::expandBox(box, probe_cfg.plate_expand_ratio, frame_w, frame_h);
      else
        box = utils::clampBox(box, frame_w, frame_h);

      setClampedRect(&obj->rect_params, box.x1, box.y1, box.x2, box.y2, frame_w, frame_h);
    }
  }
  return GST_PAD_PROBE_OK;
}

GstPadProbeReturn PlateProbe::handleRoiRestore(GstPadProbeInfo* info) {
  GstBuffer* buf = static_cast<GstBuffer*>(info->data);
  NvDsBatchMeta* batch = gst_buffer_get_nvds_batch_meta(buf);
  if (batch != nullptr) {
    // Restore từ chính object meta — không cần tra map theo GstBuffer*.
    for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
         l_frame = l_frame->next) {
      auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
      for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
        restoreOrigRect(static_cast<NvDsObjectMeta*>(l_obj->data));
      }
    }
  }

  if (batch == nullptr) return GST_PAD_PROBE_OK;

  const int plate_id = config_.pipeline().sgie_plate.unique_id;
  constexpr float kMinPlateWidth = 20.0f;

  for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
       l_frame = l_frame->next) {
    auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
    if (frame->source_frame_width == 0 || frame->source_frame_height == 0) continue;
    const double frame_w = static_cast<double>(frame->source_frame_width);
    const double frame_h = static_cast<double>(frame->source_frame_height);

    for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
      auto* obj = static_cast<NvDsObjectMeta*>(l_obj->data);
      if (static_cast<int>(obj->unique_component_id) != plate_id) continue;
      clampRectInFrame(&obj->rect_params, frame_w, frame_h);
      // Biển quá hẹp: zero size để nvinfer bỏ qua (tránh upscale >16x).
      if (obj->rect_params.width < kMinPlateWidth || obj->rect_params.height < 1.0f) {
        obj->rect_params.width = 0.0f;
        obj->rect_params.height = 0.0f;
      }
    }
  }
  return GST_PAD_PROBE_OK;
}

GstPadProbeReturn PlateProbe::handleBbox(GstPadProbeInfo* info) {
  GstBuffer* buf = static_cast<GstBuffer*>(info->data);
  NvDsBatchMeta* batch = gst_buffer_get_nvds_batch_meta(buf);
  if (batch == nullptr) return GST_PAD_PROBE_OK;

  maybeLogDsLatency(buf, batch, config_.pipeline().streammux.batch_size, "tracker");

  if (publisher_ == nullptr || !config_.pipeline().probe.publish_bbox)
    return GST_PAD_PROBE_OK;

  const PipelineConfig& pipe = config_.pipeline();
  const ProbeConfig& probe_cfg = pipe.probe;
  const double now_s = utils::monotonicSeconds();

  for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
       l_frame = l_frame->next) {
    auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
    SourceState* state = sourceState(frame->source_id);
    if (state == nullptr) continue;
    if (frame->source_frame_width == 0 || frame->source_frame_height == 0) continue;

    const double frame_w = static_cast<double>(frame->source_frame_width);
    const double frame_h = static_cast<double>(frame->source_frame_height);
    const PlateZones plate_zones =
        plateZonesFor(state->camera.code, frame_w, frame_h, frame_w, frame_h);
    const std::vector<std::vector<Point>>& polygons = plate_zones.polygons;
    // Chưa có polygon PLATE → vẫn vẽ bbox tracking, nhưng không xe nào được coi
    // là trong zone (không OCR, không event).
    const bool zone_ready = !polygons.empty();

    std::vector<Detection> detections;
    for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
      auto* obj = static_cast<NvDsObjectMeta*>(l_obj->data);
      if (static_cast<int>(obj->unique_component_id) != pipe.pgie.unique_id) continue;
      if (obj->object_id == UNTRACKED_OBJECT_ID) continue;

      const uint64_t track_id = obj->object_id;
      BoundingBox box = toBox(obj->rect_params);
      box.y2 += probe_cfg.vehicle_bottom_pad;
      box = utils::clampBox(box, frame_w, frame_h);

      const int vehicle_cls = obj->class_id;
      const Point anchor = utils::anchorPoint(box, probe_cfg.anchor_bottom_ratio);
      bool in_zone = false;
      for (const std::vector<Point>& polygon : polygons) {
        if (utils::pointInPolygon(anchor, polygon)) {
          in_zone = true;
          break;
        }
      }

      // Tạo/cập nhật track sớm (trước SGIE) — OCR vẫn chạy ở probe meta.
      if (zone_ready) {
        state->manager->observeVehicle(track_id, vehicle_cls, obj->confidence, in_zone, now_s);
        if (!in_zone) continue;
      }

      Detection det;
      det.id = "vehicle_" + std::to_string(track_id);
      det.cls = vehicleClassName(vehicle_cls);
      det.confidence = obj->confidence;
      det.bbox_norm = utils::normalizeBox(box, frame_w, frame_h);
      det.label = det.cls;  // chỉ loại xe — không chờ biển
      det.color = "#00FF00";
      detections.push_back(std::move(det));
    }

    if (!detections.empty()) publisher_->publishBbox(state->camera.code, detections);
  }
  return GST_PAD_PROBE_OK;
}

GstPadProbeReturn PlateProbe::handleMeta(GstPadProbeInfo* info) {
  GstBuffer* buf = static_cast<GstBuffer*>(info->data);
  NvDsBatchMeta* batch = gst_buffer_get_nvds_batch_meta(buf);
  if (batch == nullptr) return GST_PAD_PROBE_OK;

  const double probe_start_s = utils::monotonicSeconds();
  maybeLogDsLatency(buf, batch, config_.pipeline().streammux.batch_size, "full");

  const PipelineConfig& pipe = config_.pipeline();
  const ProbeConfig& probe_cfg = pipe.probe;
  const HelmetViolationConfig& helmet_cfg = config_.violation().helmet;
  const LaneViolationConfig& wrong_lane_cfg = config_.violation().wrong_lane;
  const WrongWayViolationConfig& wrong_way_cfg = config_.violation().wrong_way;
  const double now_s = probe_start_s;

  NvBufSurface* surface = nullptr;
  GstMapInfo map_info;
  const bool mapped = enc_ctx_ != nullptr && gst_buffer_map(buf, &map_info, GST_MAP_READ);
  if (mapped) surface = reinterpret_cast<NvBufSurface*>(map_info.data);
  bool encode_requested = false;

  for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
       l_frame = l_frame->next) {
    auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
    SourceState* state = sourceState(frame->source_id);
    if (state == nullptr) continue;

    // nvstreammux v2 không scale — kích thước khung = resolution nguồn trong frame meta.
    if (frame->source_frame_width == 0 || frame->source_frame_height == 0) {
      LOG_WARN("frame src%u: thiếu source_frame_width/height — bỏ frame", frame->source_id);
      continue;
    }
    const double frame_w = static_cast<double>(frame->source_frame_width);
    const double frame_h = static_cast<double>(frame->source_frame_height);

    const PlateZones plate_zones =
        plateZonesFor(state->camera.code, frame_w, frame_h, frame_w, frame_h);
    // Không có polygon PLATE → không OCR, không vi phạm, không event. Bbox tracking
    // vẫn được publish ở handleBbox.
    if (plate_zones.polygons.empty()) continue;
    const std::vector<std::vector<Point>>& polygons = plate_zones.polygons;
    const std::vector<LanePolygon> lane_polygons =
        lanePolygonsFor(state->camera.code, frame_w, frame_h, frame_w, frame_h);
    const std::vector<business::plate::WrongWayLine> wrong_way_lines =
        wrong_way_cfg.enabled
            ? wrongWayLinesFor(state->camera.code, frame_w, frame_h, frame_w, frame_h)
            : std::vector<business::plate::WrongWayLine>{};

    // Phân tầng metadata: xe (PGIE) → biển (SGIE1) → ký tự (SGIE2).
    std::vector<NvDsObjectMeta*> vehicles;
    std::map<NvDsObjectMeta*, NvDsObjectMeta*> plate_of;      // vehicle → biển conf cao nhất
    std::map<NvDsObjectMeta*, std::vector<CharBox>> chars_of;  // biển → ký tự
    std::map<NvDsObjectMeta*, int> no_helmet_of;               // xe máy → số người không mũ

    for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
      auto* obj = static_cast<NvDsObjectMeta*>(l_obj->data);
      const int component = static_cast<int>(obj->unique_component_id);
      if (component == pipe.pgie.unique_id) {
        vehicles.push_back(obj);
      } else if (component == pipe.sgie_helmet.unique_id) {
        if (obj->parent == nullptr) continue;
        if (obj->class_id != helmet_cfg.no_helmet_class_id) continue;
        ++no_helmet_of[obj->parent];
      } else if (component == pipe.sgie_plate.unique_id) {
        NvDsObjectMeta* parent = obj->parent;
        if (parent == nullptr) continue;
        auto it = plate_of.find(parent);
        if (it == plate_of.end() || obj->confidence > it->second->confidence)
          plate_of[parent] = obj;
      } else if (component == pipe.sgie_digit.unique_id) {
        NvDsObjectMeta* parent = obj->parent;
        if (parent == nullptr) continue;
        CharBox ch;
        ch.text = obj->obj_label;
        ch.confidence = obj->confidence;
        ch.box = toBox(obj->rect_params);
        chars_of[parent].push_back(std::move(ch));
      }
    }

    LOG_DEBUG(
        "frame %u/src%u: %zu xe (PGIE), %zu biển (SGIE1), %zu nhóm ký tự (SGIE2), "
        "%zu xe không mũ (SGIE3)",
        frame->frame_num, frame->source_id, vehicles.size(), plate_of.size(),
        chars_of.size(), no_helmet_of.size());

    for (NvDsObjectMeta* vehicle : vehicles) {
      if (vehicle->object_id == UNTRACKED_OBJECT_ID) {
        LOG_DEBUG("frame %u: bỏ xe chưa có track_id (conf=%.2f)", frame->frame_num,
                  vehicle->confidence);
        continue;
      }
      const uint64_t track_id = vehicle->object_id;
      BoundingBox box = toBox(vehicle->rect_params);
      box.y2 += probe_cfg.vehicle_bottom_pad;  // nới đáy cho khỏi cắt biển
      box = utils::clampBox(box, frame_w, frame_h);

      const int vehicle_cls = vehicle->class_id;
      const Point anchor = utils::anchorPoint(box, probe_cfg.anchor_bottom_ratio);
      bool in_zone = false;  // polygons chắc chắn không rỗng ở đây
      for (const std::vector<Point>& polygon : polygons) {
        if (utils::pointInPolygon(anchor, polygon)) {
          in_zone = true;
          break;
        }
      }

      state->manager->observeVehicle(track_id, vehicle_cls, vehicle->confidence, in_zone, now_s);

      if (wrong_lane_cfg.enabled &&
          (in_zone || state->manager->everEnteredPlateZone(track_id))) {
        for (const LanePolygon& lane : lane_polygons) {
          if (lane.allowed_classes.count(vehicle_cls) > 0) continue;  // đúng làn
          if (utils::pointInPolygon(anchor, lane.polygon)) {
            state->manager->observeLane(track_id, lane.zone_name);
            if (state->manager->needsWrongLaneSnapshot(track_id)) {
              std::lock_guard<std::mutex> lock(pending_mutex_);
              PendingEncode pend;
              pend.track_id = track_id;
              pend.plate_key = kWrongLaneSnapshotKey;
              pend.left = vehicle->rect_params.left;
              pend.top = vehicle->rect_params.top;
              pend.width = vehicle->rect_params.width;
              pend.height = vehicle->rect_params.height;
              pend.encode_plate_crop = false;
              pending_frames_[{frame->source_id, frame->frame_num}].push_back(std::move(pend));
              state->manager->markWrongLaneSnapshotTaken(track_id);
            }
            break;
          }
        }
      }

      // WRONG_WAY: xe phải CHẠM line REVERSE_DIRECTION mới tính vi phạm.
      // KHÔNG gate theo zone PLATE: phải cập nhật anchor ở MỌI frame thì mới có
      // vị trí frame trước để so — nếu bỏ frame, đoạn di chuyển bị đứt và lần
      // cắt vạch không được ghi nhận. Line cũng có thể nằm ngoài zone PLATE.
      if (!wrong_way_lines.empty()) {
        if (state->manager->observeWrongWay(track_id, anchor, wrong_way_lines, now_s) &&
            state->manager->needsWrongWaySnapshot(track_id)) {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          PendingEncode pend;
          pend.track_id = track_id;
          pend.plate_key = kWrongWaySnapshotKey;
          pend.left = vehicle->rect_params.left;
          pend.top = vehicle->rect_params.top;
          pend.width = vehicle->rect_params.width;
          pend.height = vehicle->rect_params.height;
          pend.encode_plate_crop = false;
          pending_frames_[{frame->source_id, frame->frame_num}].push_back(std::move(pend));
          state->manager->markWrongWaySnapshotTaken(track_id);
        }
      }

      if (!in_zone) continue;

      // NO_HELMET chỉ áp dụng cho xe máy.
      if (helmet_cfg.enabled && vehicle_cls == kClassMotorbike) {
        auto helmet_it = no_helmet_of.find(vehicle);
        if (helmet_it != no_helmet_of.end())
          state->manager->observeHelmet(track_id, helmet_it->second);
      }

      // OCR: ghép ký tự; giữ JPEG theo chuỗi biển khi mẫu đẹp hơn.
      auto plate_it = plate_of.find(vehicle);
      business::plate::PlateOcrStatus ocr_status =
          business::plate::PlateOcrStatus::kRejected;
      std::string snap_plate_key;
      if (plate_it != plate_of.end()) {
        NvDsObjectMeta* plate = plate_it->second;
        const BoundingBox plate_box = toBox(plate->rect_params);
        auto chars_it = chars_of.find(plate);
        if (plate_box.width() >= probe_cfg.min_plate_width && chars_it != chars_of.end()) {
          AssembleOptions options;
          options.iou_dedup = probe_cfg.char_iou_dedup;
          options.square_ratio = probe_cfg.square_plate_ratio;
          if (::vehicle::log::Level::kDebug >= ::vehicle::log::minLevel()) {
            for (const CharBox& ch : chars_it->second)
              LOG_DEBUG("track %lu: ký tự '%s' conf=%.2f box=[%.0f,%.0f,%.0f,%.0f]",
                        static_cast<unsigned long>(track_id), ch.text.c_str(), ch.confidence,
                        ch.box.x1, ch.box.y1, ch.box.x2, ch.box.y2);
            LOG_DEBUG("track %lu: plate box=[%.0f,%.0f,%.0f,%.0f]",
                      static_cast<unsigned long>(track_id), plate_box.x1, plate_box.y1,
                      plate_box.x2, plate_box.y2);
          }
          const PlateReading reading = assemblePlateText(chars_it->second, options);
          if (!reading.chars.empty()) {
            const double area = box.width() * box.height();
            // area = bbox XE (ảnh phương tiện), plate_area = bbox BIỂN (crop).
            const double plate_area = plate_box.width() * plate_box.height();
            ocr_status =
                state->manager->addOcrReading(track_id, reading.chars, reading.raw, now_s,
                                              meanCharConf(reading), area, plate_area);
            snap_plate_key = utils::toUpper(reading.raw);
          }
        }
      }

      const bool just_final = business::plate::plateOcrJustFinal(ocr_status);
      const bool better_snap = business::plate::plateOcrBetterSnapshot(ocr_status);
      const bool want_full = just_final || better_snap;
      // Crop biển KHÔNG bám theo area xe: chụp khi biển đẹp hơn crop ĐANG CÓ
      // trong kho ảnh của đúng chuỗi này. Hỏi thẳng kho — không qua sổ ghi ở
      // TrackPlateState — để "đã chụp" luôn đồng nghĩa "ảnh có thật".
      const bool want_plate =
          business::plate::plateOcrAccepted(ocr_status) && !snap_plate_key.empty() &&
          wantsBetterCrop(state->snapshots, track_id, snap_plate_key,
                          state->manager->lastCropCandidate(track_id), now_s);
      if (want_full && !snap_plate_key.empty()) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        PendingEncode pend;
        pend.track_id = track_id;
        pend.plate_key = snap_plate_key;
        pend.left = vehicle->rect_params.left;
        pend.top = vehicle->rect_params.top;
        pend.width = vehicle->rect_params.width;
        pend.height = vehicle->rect_params.height;
        pend.encode_plate_crop = want_plate && plate_it != plate_of.end();
        pending_frames_[{frame->source_id, frame->frame_num}].push_back(std::move(pend));
      }
      if (want_plate && !snap_plate_key.empty() && surface != nullptr &&
          plate_it != plate_of.end()) {
        const double enc_start_s = utils::monotonicSeconds();
        utils::PlateWarpSnapshotParams warp_params;
        warp_params.keypoint_min_confidence = probe_cfg.keypoint_min_confidence;
        warp_params.plate_expand_ratio = probe_cfg.plate_expand_ratio;
        warp_params.vehicle_bottom_pad = probe_cfg.vehicle_bottom_pad;
        warp_params.jpeg_quality = kJpegQuality;
        std::vector<uint8_t> warped_jpeg;
        const bool warped = utils::encodeWarpedPlateJpeg(
            surface, static_cast<int>(frame->batch_id), plate_it->second, frame, warp_params,
            &warped_jpeg);
        if (warped && !warped_jpeg.empty()) {
          JpegImage image;
          image.data = std::move(warped_jpeg);
          TrackSnapshots& bank = state->snapshots[track_id];
          // Crop lưu ĐÚNG dưới chuỗi biển đọc được ở frame này, kèm điểm chất
          // lượng — lần sau so trực tiếp với ảnh đang giữ.
          SnapshotImages& slot = bank.by_plate[snap_plate_key];
          slot.crop = std::move(image);
          slot.crop_score = state->manager->lastCropCandidate(track_id);
          slot.crop_pending_until_s = 0.0;
          retainSingleSnapshot(&bank, snap_plate_key, now_s);
          utils::latencyLog("track %lu jpeg_enc_plate_warp: %.1fms [%s%s]",
                            static_cast<unsigned long>(track_id), utils::msSince(enc_start_s),
                            better_snap ? "best" : "", just_final ? "final" : "");
        } else {
          NvDsObjEncUsrArgs obj_args{};
          obj_args.saveImg = false;
          obj_args.attachUsrMeta = true;
          obj_args.isFrame = 0;
          obj_args.quality = kJpegQuality;
          nvds_obj_enc_process(enc_ctx_, &obj_args, surface, plate_it->second, frame);
          encode_requested = true;
          // JPEG chỉ về ở image probe: giữ chỗ nhận ảnh + ghi sẵn điểm (probe đó
          // không biết điểm), có hạn để encode fail vẫn được chụp lại.
          SnapshotImages& slot = state->snapshots[track_id].by_plate[snap_plate_key];
          slot.crop_score = state->manager->lastCropCandidate(track_id);
          slot.crop_pending_until_s = now_s + kCropPendingTtlS;
          utils::latencyLog("track %lu jpeg_enc_plate: %.1fms [%s%s] (aabb-fallback)",
                            static_cast<unsigned long>(track_id), utils::msSince(enc_start_s),
                            better_snap ? "best" : "", just_final ? "final" : "");
        }
      }

      // Track đã chốt (miss) nhưng chưa có full — snapshot ngay nếu xe còn trên frame.
      if (!want_full && state->manager->awaitingSnapshot(track_id)) {
        const bool have_full = [&]() {
          auto bank_it = state->snapshots.find(track_id);
          if (bank_it == state->snapshots.end()) return false;
          JpegImage tmp;
          return pickSnapshotImages(bank_it->second, state->manager->bestSnapshotKey(track_id),
                                    &tmp, nullptr);
        }();
        if (!have_full) {
          std::string key = state->manager->bestSnapshotKey(track_id);
          if (key.empty()) key = snap_plate_key;
          if (key.empty()) key = business::plate::kEmptyPlate;
          const bool has_plate_now = plate_it != plate_of.end();
          {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            PendingEncode pend;
            pend.track_id = track_id;
            pend.plate_key = key;
            pend.left = vehicle->rect_params.left;
            pend.top = vehicle->rect_params.top;
            pend.width = vehicle->rect_params.width;
            pend.height = vehicle->rect_params.height;
            pend.encode_plate_crop = has_plate_now;
            pending_frames_[{frame->source_id, frame->frame_num}].push_back(std::move(pend));
          }
          if (has_plate_now && surface != nullptr) {
            NvDsObjEncUsrArgs obj_args{};
            obj_args.saveImg = false;
            obj_args.attachUsrMeta = true;
            obj_args.isFrame = 0;
            obj_args.quality = kJpegQuality;
            nvds_obj_enc_process(enc_ctx_, &obj_args, surface, plate_it->second, frame);
            encode_requested = true;
            state->snapshots[track_id].by_plate[key].crop_pending_until_s =
                now_s + kCropPendingTtlS;
          }
        }
      }
    }
  }

  if (encode_requested) {
    const double finish_s = utils::monotonicSeconds();
    nvds_obj_enc_finish(enc_ctx_);
    utils::latencyLog("jpeg_enc_finish: %.1fms", utils::msSince(finish_s));
  }
  if (mapped) gst_buffer_unmap(buf, &map_info);

  const double probe_ms = utils::msSince(probe_start_s);
  if (encode_requested || probe_ms >= 10.0)
    utils::latencyLog("probe_meta: %.1fms encode=%d frames=%u", probe_ms,
                      encode_requested ? 1 : 0, batch->num_frames_in_batch);
  return GST_PAD_PROBE_OK;
}

namespace {

// Cập nhật rect xe từ PGIE meta trên frame (nếu track còn); giữ fallback đã lưu.
void refreshVehicleRect(NvDsFrameMeta* frame, int pgie_id, PendingEncode* pend) {
  if (frame == nullptr || pend == nullptr) return;
  for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
    auto* obj = static_cast<NvDsObjectMeta*>(l_obj->data);
    if (obj->parent != nullptr) continue;
    if (obj->object_id != pend->track_id) continue;
    if (static_cast<int>(obj->unique_component_id) != pgie_id) continue;
    pend->left = obj->rect_params.left;
    pend->top = obj->rect_params.top;
    pend->width = obj->rect_params.width;
    pend->height = obj->rect_params.height;
    return;
  }
}

void storeFullSnapshot(SnapshotImages* snap, const JpegImage& full, const PendingEncode& pend) {
  if (snap == nullptr || full.empty()) return;
  snap->full = full;
  snap->left = pend.left;
  snap->top = pend.top;
  snap->width = pend.width;
  snap->height = pend.height;
}

}  // namespace

GstPadProbeReturn PlateProbe::handleImages(GstPadProbeInfo* info) {
  GstBuffer* buf = static_cast<GstBuffer*>(info->data);
  NvDsBatchMeta* batch = gst_buffer_get_nvds_batch_meta(buf);
  if (batch == nullptr) return GST_PAD_PROBE_OK;

  const double now_s = utils::monotonicSeconds();
  const int pgie_id = config_.pipeline().pgie.unique_id;
  const WrongWayViolationConfig& wrong_way_cfg = config_.violation().wrong_way;

  // Track chờ full-frame theo (source, frame_num) — lấy trước khi encode.
  std::map<std::pair<unsigned int, uint64_t>, std::vector<PendingEncode>> waiting_by_frame;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
         l_frame = l_frame->next) {
      auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
      auto it = pending_frames_.find({frame->source_id, frame->frame_num});
      if (it == pending_frames_.end()) continue;
      waiting_by_frame[it->first] = std::move(it->second);
      pending_frames_.erase(it);
    }
  }

  if (!waiting_by_frame.empty() && enc_ctx_ != nullptr) {
    GstMapInfo map_info;
    if (gst_buffer_map(buf, &map_info, GST_MAP_READ)) {
      auto* surface = reinterpret_cast<NvBufSurface*>(map_info.data);
      const double enc_start_s = utils::monotonicSeconds();
      bool submitted = false;
      for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
           l_frame = l_frame->next) {
        auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
        auto wait_it = waiting_by_frame.find({frame->source_id, frame->frame_num});
        if (wait_it == waiting_by_frame.end()) continue;

        for (PendingEncode& pend : wait_it->second) refreshVehicleRect(frame, pgie_id, &pend);

        NvDsObjEncUsrArgs frame_args{};
        frame_args.saveImg = false;
        frame_args.attachUsrMeta = true;
        frame_args.isFrame = 1;
        frame_args.quality = kJpegQuality;
        nvds_obj_enc_process(enc_ctx_, &frame_args, surface, nullptr, frame);
        submitted = true;
      }
      if (submitted) {
        nvds_obj_enc_finish(enc_ctx_);
        utils::latencyLog("jpeg_enc_full: %.1fms frames=%zu", utils::msSince(enc_start_s),
                          waiting_by_frame.size());
      }
      gst_buffer_unmap(buf, &map_info);
    }
  }

  for (NvDsMetaList* l_frame = batch->frame_meta_list; l_frame != nullptr;
       l_frame = l_frame->next) {
    auto* frame = static_cast<NvDsFrameMeta*>(l_frame->data);
    SourceState* state = sourceState(frame->source_id);
    if (state == nullptr) continue;

    auto wait_it = waiting_by_frame.find({frame->source_id, frame->frame_num});
    if (wait_it != waiting_by_frame.end()) {
      const std::vector<PendingEncode>& waiting = wait_it->second;
      // Lấy meta JPEG mới nhất (tránh meta cũ còn sót trên frame pool).
      JpegImage full_frame;
      for (NvDsUserMetaList* l_user = frame->frame_user_meta_list; l_user != nullptr;
           l_user = l_user->next) {
        auto* user_meta = static_cast<NvDsUserMeta*>(l_user->data);
        if (user_meta->base_meta.meta_type != NVDS_CROP_IMAGE_META) continue;
        auto* enc = static_cast<NvDsObjEncOutParams*>(user_meta->user_meta_data);
        if (enc == nullptr || enc->outBuffer == nullptr || enc->outLen == 0) continue;
        full_frame.data.assign(enc->outBuffer, enc->outBuffer + enc->outLen);
      }
      if (!full_frame.empty()) {
        for (const PendingEncode& pend : waiting) {
          TrackSnapshots& bank = state->snapshots[pend.track_id];
          storeFullSnapshot(&bank.by_plate[pend.plate_key], full_frame, pend);
          if (!isProtectedSnapshotKey(pend.plate_key))
            retainSingleSnapshot(&bank, pend.plate_key, now_s);
        }
      }
    }

    // Crop biển → map parent track + plate_key pending cùng frame.
    for (NvDsMetaList* l_obj = frame->obj_meta_list; l_obj != nullptr; l_obj = l_obj->next) {
      auto* obj = static_cast<NvDsObjectMeta*>(l_obj->data);
      if (obj->parent == nullptr) continue;
      if (obj->parent->object_id == UNTRACKED_OBJECT_ID) continue;
      const uint64_t track_id = obj->parent->object_id;
      std::string plate_key;
      if (wait_it != waiting_by_frame.end()) {
        for (const PendingEncode& pend : wait_it->second) {
          if (pend.track_id != track_id) continue;
          if (plate_key.empty() || isProtectedSnapshotKey(plate_key))
            plate_key = pend.plate_key;
          if (!isProtectedSnapshotKey(plate_key)) break;
        }
      }
      if (plate_key.empty()) continue;
      // Không ghi đè crop đã warp ở meta probe.
      TrackSnapshots& bank = state->snapshots[track_id];
      if (!bank.by_plate[plate_key].crop.empty()) continue;
      for (NvDsUserMetaList* l_user = obj->obj_user_meta_list; l_user != nullptr;
           l_user = l_user->next) {
        auto* user_meta = static_cast<NvDsUserMeta*>(l_user->data);
        if (user_meta->base_meta.meta_type != NVDS_CROP_IMAGE_META) continue;
        auto* enc = static_cast<NvDsObjEncOutParams*>(user_meta->user_meta_data);
        if (enc == nullptr || enc->outBuffer == nullptr || enc->outLen == 0) continue;
        JpegImage image;
        image.data.assign(enc->outBuffer, enc->outBuffer + enc->outLen);
        SnapshotImages& slot = bank.by_plate[plate_key];
        slot.crop = std::move(image);
        slot.crop_pending_until_s = 0.0;
        if (!isProtectedSnapshotKey(plate_key)) retainSingleSnapshot(&bank, plate_key, now_s);
        break;
      }
    }
    pruneSnapshotBanks(&state->snapshots);

    // Miss-finalize (xe biến mất / ngoài zone > 1s) rồi emit.
    state->manager->finalizeMissed(now_s);

    // Cắt vạch rồi mà quá hạn vẫn không có biển số → bỏ track, không bắn event.
    if (wrong_way_cfg.enabled) state->manager->clearStaleWrongWay(now_s);

    for (const business::plate::PendingEmit& pending : state->manager->collectReady(now_s)) {
      JpegImage full;
      JpegImage crop;
      float left = 0.0f;
      float top = 0.0f;
      float width = 0.0f;
      float height = 0.0f;
      auto bank_it = state->snapshots.find(pending.track_id);
      const bool have_images =
          bank_it != state->snapshots.end() &&
          pickSnapshotImages(bank_it->second, pending.snapshot_key, &full, &crop, &left, &top,
                             &width, &height);
      if (!have_images) {
        LOG_DEBUG("track %lu: chưa có ảnh snapshot key='%s' — chờ frame sau",
                  static_cast<unsigned long>(pending.track_id), pending.snapshot_key.c_str());
        continue;
      }

      // Mẫu ảnh phương tiện được chọn có thể không kèm crop → lấy crop của ĐÚNG
      // chuỗi biển đã chốt (snapshot_key), để ảnh biển khớp kết quả nhận diện.
      if (crop.empty()) {
        bool exact = false;
        const JpegImage* best = fallbackCrop(bank_it->second, pending.snapshot_key, &exact);
        if (best != nullptr) {
          crop = *best;
          if (exact) {
            LOG_DEBUG("track %lu: dùng crop của chuỗi đã chốt '%s'",
                      static_cast<unsigned long>(pending.track_id),
                      pending.snapshot_key.c_str());
          } else {
            LOG_WARN("track %lu: chuỗi chốt '%s' không có crop — dùng crop frame khác",
                     static_cast<unsigned long>(pending.track_id),
                     pending.snapshot_key.c_str());
          }
        }
      }

      // Event vi phạm ngược chiều BẮT BUỘC có đủ 2 ảnh: crop biển + full-frame.
      // Thiếu ảnh → chờ frame sau (dropStaleWrongWay sẽ bỏ track nếu quá hạn).
      if (pending.wrong_way_hits > 0 && (crop.empty() || full.empty())) {
        LOG_DEBUG("track %lu: WRONG_WAY chờ đủ ảnh (crop=%zuB full=%zuB)",
                  static_cast<unsigned long>(pending.track_id), crop.data.size(),
                  full.data.size());
        continue;
      }

      if (!state->manager->commitEmit(pending.track_id, pending.plate, now_s)) continue;

      if (crop.empty()) {
        LOG_WARN("track %lu: event '%s' không có crop biển (key='%s', %d reading)",
                 static_cast<unsigned long>(pending.track_id), pending.plate.c_str(),
                 pending.snapshot_key.c_str(), pending.recognize_count);
      }

      // 1 bbox xanh / event trên đúng JPEG của track — không phụ thuộc nvdsosd.
      if (width >= 1.0f && height >= 1.0f) {
        if (!utils::drawGreenRectOnJpeg(&full.data, left, top, width, height)) {
          LOG_WARN("track %lu: vẽ bbox xanh lên JPEG thất bại (%.0fx%.0f @%.0f,%.0f)",
                   static_cast<unsigned long>(pending.track_id), width, height, left, top);
        }
      } else {
        LOG_WARN("track %lu: thiếu rect xe cho snapshot — gửi ảnh không bbox",
                 static_cast<unsigned long>(pending.track_id));
      }

      EmitJob job;
      job.camera = state->camera;
      job.manager = state->manager.get();
      job.emit.track_id = pending.track_id;
      job.emit.plate = pending.plate;
      job.emit.vehicle_cls = pending.vehicle_cls;
      job.emit.created_at_s = pending.created_at_s;
      job.emit.first_ocr_at_s = pending.first_ocr_at_s;
      job.emit.final_at_s = pending.final_at_s;
      job.emit.recognize_count = pending.recognize_count;
      job.emit.no_helmet_frames = pending.no_helmet_frames;
      job.emit.no_helmet_count = pending.no_helmet_count;
      job.emit.wrong_lane_frames = pending.wrong_lane_frames;
      job.emit.wrong_lane_zone = pending.wrong_lane_zone;
      if (pending.has_wrong_lane_snapshot) {
        auto lane_it = bank_it->second.by_plate.find(kWrongLaneSnapshotKey);
        if (lane_it != bank_it->second.by_plate.end() && !lane_it->second.full.empty()) {
          JpegImage lane_full = lane_it->second.full;
          if (lane_it->second.width >= 1.0f && lane_it->second.height >= 1.0f) {
            utils::drawGreenRectOnJpeg(&lane_full.data, lane_it->second.left,
                                       lane_it->second.top, lane_it->second.width,
                                       lane_it->second.height);
          }
          job.emit.wrong_lane_full = std::move(lane_full);
          job.emit.wrong_lane_crop = lane_it->second.crop;
        } else {
          LOG_WARN("track %lu: WRONG_LANE thiếu ảnh lúc vi phạm — dùng ảnh mặc định",
                   static_cast<unsigned long>(pending.track_id));
        }
      }
      job.emit.wrong_way_hits = pending.wrong_way_hits;
      job.emit.wrong_way_line = pending.wrong_way_line;
      if (pending.has_wrong_way_snapshot) {
        auto way_it = bank_it->second.by_plate.find(kWrongWaySnapshotKey);
        if (way_it != bank_it->second.by_plate.end() && !way_it->second.full.empty()) {
          JpegImage way_full = way_it->second.full;
          if (way_it->second.width >= 1.0f && way_it->second.height >= 1.0f) {
            utils::drawGreenRectOnJpeg(&way_full.data, way_it->second.left,
                                       way_it->second.top, way_it->second.width,
                                       way_it->second.height);
          }
          job.emit.wrong_way_full = std::move(way_full);
          job.emit.wrong_way_crop = way_it->second.crop;
        } else {
          LOG_WARN("track %lu: WRONG_WAY thiếu ảnh lúc vi phạm — dùng ảnh mặc định",
                   static_cast<unsigned long>(pending.track_id));
        }
      }
      job.emit.images_ready_at_s = now_s;
      job.emit.full_frame = std::move(full);
      job.emit.plate_crop = std::move(crop);
      state->snapshots.erase(pending.track_id);
      utils::latencyLog(
          "track %lu images_ready: final→images=%.0fms full=%zuB plate=%zuB key='%s'",
          static_cast<unsigned long>(pending.track_id),
          (pending.final_at_s > 0.0) ? (now_s - pending.final_at_s) * 1000.0 : 0.0,
          job.emit.full_frame.data.size(), job.emit.plate_crop.data.size(),
          pending.snapshot_key.c_str());
      job.emit.enqueue_at_s = utils::monotonicSeconds();
      enqueue(std::move(job));
    }
  }

  if (now_s - last_cleanup_s_ > 1.0) {
    last_cleanup_s_ = now_s;
    std::lock_guard<std::mutex> lock(sources_mutex_);
    for (auto& entry : sources_) {
      if (entry.second == nullptr || entry.second->manager == nullptr) continue;
      entry.second->manager->finalizeMissed(now_s);
      entry.second->manager->cleanup(now_s);
      pruneOrphanSnapshots(entry.second->manager.get(), &entry.second->snapshots);
    }
  }
  maybeLogMemStats(now_s);
  return GST_PAD_PROBE_OK;
}

void PlateProbe::maybeLogMemStats(double now_s) {
  if (!memStatsEnabled()) return;
  if (now_s - last_mem_stats_s_ < kMemStatsIntervalS) return;
  last_mem_stats_s_ = now_s;

  size_t tracks = 0;
  size_t sample_n = 0;
  size_t jpeg_b = 0;
  {
    std::lock_guard<std::mutex> lock(sources_mutex_);
    for (const auto& entry : sources_) {
      if (entry.second == nullptr) continue;
      if (entry.second->manager != nullptr) tracks += entry.second->manager->trackCount();
      sample_n += sampleCount(entry.second->snapshots);
      jpeg_b += jpegBytes(entry.second->snapshots);
    }
  }
  size_t pending = 0;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending = pending_frames_.size();
  }
  size_t queue = 0;
  size_t queue_jpeg_b = 0;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue = queue_.size();
    for (const EmitJob& job : queue_) {
      queue_jpeg_b += job.emit.full_frame.data.size();
      queue_jpeg_b += job.emit.plate_crop.data.size();
    }
  }
  size_t zones_n = 0;
  {
    std::lock_guard<std::mutex> lock(zones_mutex_);
    zones_n = zones_.size();
  }
  LOG_DEBUG(
      "mem_stats: tracks=%zu queue=%zu pending=%zu samples=%zu jpeg_kb=%zu queue_jpeg_kb=%zu "
      "zones=%zu rss_mb=%zu",
      tracks, queue, pending, sample_n, jpeg_b / 1024, queue_jpeg_b / 1024, zones_n,
      processRssMb());
}

void PlateProbe::enqueue(EmitJob job) {
  size_t dropped = 0;
  uint64_t dropped_track = 0;
  std::string dropped_plate;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (queue_.size() >= kMaxEmitQueue) {
      dropped_track = queue_.front().emit.track_id;
      dropped_plate = queue_.front().emit.plate;
      queue_.pop_front();
      ++dropped;
    }
    queue_.push_back(std::move(job));
  }
  if (dropped > 0)
    LOG_WARN("emit queue đầy (%zu) — drop %zu job cũ (track %lu plate=%s)", kMaxEmitQueue,
             dropped, static_cast<unsigned long>(dropped_track), dropped_plate.c_str());
  queue_cv_.notify_one();
}

void PlateProbe::start() {
  if (running_) return;
  running_ = true;
  worker_ = std::thread(&PlateProbe::workerLoop, this);
}

void PlateProbe::stop() {
  if (!running_) return;
  running_ = false;
  queue_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

void PlateProbe::workerLoop() {
  while (running_) {
    EmitJob job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(lock, std::chrono::milliseconds(200),
                         [this] { return !queue_.empty() || !running_; });
      if (!running_ && queue_.empty()) return;
      if (queue_.empty()) continue;
      job = std::move(queue_.front());
      queue_.pop_front();
    }
    if (publisher_ == nullptr) continue;
    const double worker_start_s = utils::monotonicSeconds();
    utils::latencyLog("track %lu worker_wait: enqueue→start=%.0fms",
                      static_cast<unsigned long>(job.emit.track_id),
                      (job.emit.enqueue_at_s > 0.0)
                          ? (worker_start_s - job.emit.enqueue_at_s) * 1000.0
                          : 0.0);
    if (publisher_->publishPlateEvent(job.camera, job.emit) && job.manager != nullptr)
      job.manager->markPosted(job.emit.track_id);
    utils::latencyLog("track %lu worker_total: %.0fms",
                      static_cast<unsigned long>(job.emit.track_id),
                      utils::msSince(worker_start_s));
  }
}

}  // namespace probes
}  // namespace vehicle
