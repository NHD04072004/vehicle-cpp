#include "business/plate/plate_recognizer.h"

#include "business/plate/rules.h"
#include "common/logging.h"
#include "utils/geometry.h"
#include "utils/latency.h"

namespace vehicle {
namespace business {
namespace plate {
namespace {

void logFinalPlate(uint64_t track_id, const TrackPlateState& st) {
  utils::latencyLog(
      "track %lu plate='%s' vote_ms: zone→1st_ocr=%.0f 1st_ocr→final=%.0f "
      "zone→final=%.0f readings=%d snap='%s'",
      static_cast<unsigned long>(track_id), st.plate().c_str(),
      (st.firstOcrAtS() > 0.0 && st.createdAtS() > 0.0)
          ? (st.firstOcrAtS() - st.createdAtS()) * 1000.0
          : 0.0,
      (st.finalAtS() > 0.0 && st.firstOcrAtS() > 0.0)
          ? (st.finalAtS() - st.firstOcrAtS()) * 1000.0
          : 0.0,
      (st.finalAtS() > 0.0 && st.createdAtS() > 0.0)
          ? (st.finalAtS() - st.createdAtS()) * 1000.0
          : 0.0,
      st.plateRecognizeCount(), st.bestSnapshotKey().c_str());
}

}  // namespace

PlateRecognizer::PlateRecognizer(const PlateConfig& config) : config_(config) {
  if (config_.plate_style.empty()) config_.plate_style = defaultPlateStyles();
  if (config_.max_recognize_times <= 0)
    config_.max_recognize_times = kDefaultMaxRecognizeTimes;
}

bool PlateRecognizer::observeVehicle(uint64_t track_id, int cls, double cls_conf, bool in_zone,
                                     double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) {
    it = tracks_.emplace(track_id, TrackPlateState(track_id, now_s,
                                                   config_.max_recognize_times,
                                                   config_.plate_style)).first;
    LOG_DEBUG("track %lu: xuất hiện, tạo state", static_cast<unsigned long>(track_id));
  }

  if (in_zone) {
    it->second.onEnterPolygon(now_s);
    if (cls >= 0) it->second.addClass(cls, cls_conf);
    return false;
  }

  if (it->second.inPolygon()) {
    it->second.onLeavePolygon(now_s);
    LOG_DEBUG("track %lu: rời zone — chờ miss %.0fs", static_cast<unsigned long>(track_id),
              kMissTrackIdleS);
  }

  if (!it->second.everEnteredPolygon()) return false;

  const bool just_final = it->second.tryFinalizeMiss(now_s);
  if (just_final) {
    LOG_DEBUG("track %lu: miss-finalize biển '%s' (%d/%d) snap='%s'",
              static_cast<unsigned long>(track_id), it->second.plate().c_str(),
              it->second.plateRecognizeCount(), config_.max_recognize_times,
              it->second.bestSnapshotKey().c_str());
    logFinalPlate(track_id, it->second);
  }
  return just_final;
}

void PlateRecognizer::observeHelmet(uint64_t track_id, int no_helmet_count) {
  if (no_helmet_count <= 0) return;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return;
  it->second.addHelmetObservation(no_helmet_count);
  LOG_DEBUG("track %lu: helmet %d người không đội mũ (frame thứ %d)",
            static_cast<unsigned long>(track_id), no_helmet_count,
            it->second.noHelmetFrames());
}

void PlateRecognizer::observeLane(uint64_t track_id, const std::string& zone_name) {
  if (zone_name.empty()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return;
  if (!it->second.everEnteredPolygon()) return;
  it->second.addWrongLaneObservation(zone_name);
  LOG_DEBUG("track %lu: wrong_lane zone='%s' (frame thứ %d)",
            static_cast<unsigned long>(track_id), zone_name.c_str(),
            it->second.wrongLaneFrames());
}

bool PlateRecognizer::observeWrongWay(uint64_t track_id, const Point& anchor,
                                      const std::vector<WrongWayLine>& lines, double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return false;
  TrackPlateState& state = it->second;

  const bool had_prev = state.hasLastAnchor();
  const Point prev = state.lastAnchor();
  state.setLastAnchor(anchor);
  state.pushAnchorHistory(anchor);
  if (!had_prev || lines.empty()) return false;

  const double dx = anchor.x - prev.x;
  const double dy = anchor.y - prev.y;
  if (dx * dx + dy * dy < kMinWrongWayMovePx * kMinWrongWayMovePx) return false;

  // Xe đỗ ngay trên vạch: bbox vẫn nhấp nháy vài px mỗi frame nên đoạn
  // prev→anchor liên tục "cắt" line. Xét trạng thái đứng yên trên cả history
  // (tán xạ quanh 1 tâm) mới loại được, ngưỡng 1 frame là không đủ.
  if (state.isStationary()) return false;

  // Hướng chuyển động lấy từ 3-4 anchor gần nhất: hiệu 2 frame liên tiếp quá
  // nhạy với jitter bbox, đủ để lật ngược dấu tích vô hướng ở xe đi chậm.
  const Point motion = state.motionVector();
  if (motion.x == 0.0 && motion.y == 0.0) return false;  // chưa đủ cơ sở về hướng

  for (const WrongWayLine& line : lines) {
    // Điều kiện 1: phải thực sự cắt qua line REVERSE_DIRECTION.
    if (!utils::segmentsIntersect(prev, anchor, line.a, line.b)) continue;

    // Điều kiện 2: hướng đi lệch <= ngưỡng so với direction_vector (chiều CẤM
    // — mũi tên vẽ trên line). Lệch > 90 độ là đi ngược mũi tên → hợp lệ.
    const double angle = utils::angleBetweenDeg(motion, line.direction);
    if (angle < 0.0 || angle > ww_max_angle_deg_) {
      LOG_DEBUG("track %lu: cắt line '%s' nhưng lệch %.1f độ (> %.1f) — bỏ qua",
                static_cast<unsigned long>(track_id), line.name.c_str(), angle,
                ww_max_angle_deg_);
      continue;
    }

    state.addWrongWayObservation(line.name, now_s);
    LOG_INFO("track %lu: WRONG_WAY cắt line '%s' cùng chiều cấm, lệch %.1f độ (lần %d)",
             static_cast<unsigned long>(track_id), line.name.c_str(), angle,
             state.wrongWayHits());
    return true;
  }
  return false;
}

CropScore PlateRecognizer::lastCropCandidate(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return CropScore{};
  return it->second.lastCropCandidate();
}

bool PlateRecognizer::needsWrongWaySnapshot(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  return it != tracks_.end() && it->second.needsWrongWaySnapshot();
}

void PlateRecognizer::markWrongWaySnapshotTaken(uint64_t track_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return;
  it->second.markWrongWaySnapshotTaken();
  it->second.markHasWrongWaySnapshot();
}

void PlateRecognizer::setWrongWayTiming(double settle_s, double wait_pair_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  ww_settle_s_ = std::max(0.0, settle_s);
  ww_wait_pair_s_ = std::max(0.0, wait_pair_s);
}

void PlateRecognizer::setWrongWayMaxAngle(double max_angle_deg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_angle_deg <= 0.0 || max_angle_deg >= 90.0) return;  // giữ mặc định
  ww_max_angle_deg_ = max_angle_deg;
}

size_t PlateRecognizer::dropStaleWrongWay(double now_s) {
  if (ww_wait_pair_s_ <= 0.0) return 0;
  std::lock_guard<std::mutex> lock(mutex_);
  size_t dropped = 0;
  for (auto& entry : tracks_) {
    TrackPlateState& state = entry.second;
    if (state.isPushed() || state.isPosted()) continue;
    if (state.wrongWayHits() <= 0) continue;
    const double hit_at = state.wrongWayHitAtS();
    if (hit_at <= 0.0 || now_s < hit_at + ww_wait_pair_s_) continue;

    LOG_INFO("track %lu: WRONG_WAY quá %.1fs %s — bỏ track",
             static_cast<unsigned long>(entry.first), ww_wait_pair_s_,
             state.hasFinalPlate() ? "không đủ ảnh (crop biển + full-frame)"
                                   : "không có biển số");
    state.markPushed();
    state.markPosted();
    ++dropped;
  }
  return dropped;
}

PlateOcrStatus PlateRecognizer::addOcrReading(uint64_t track_id, const CharSequence& chars,
                                              const std::string& raw, double now_s,
                                              double mean_conf, double sample_area,
                                              double plate_area) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return PlateOcrStatus::kRejected;
  const PlateOcrStatus status =
      it->second.addOcrReading(chars, raw, now_s, mean_conf, sample_area, plate_area);
  if (plateOcrAccepted(status)) {
    LOG_DEBUG("track %lu: OCR '%s' (%d/%d)%s%s", static_cast<unsigned long>(track_id),
              raw.c_str(), it->second.plateRecognizeCount(), config_.max_recognize_times,
              plateOcrBetterSnapshot(status) ? " [best_snap]" : "",
              plateOcrJustFinal(status) ? " [final]" : "");
  }
  if (plateOcrJustFinal(status)) logFinalPlate(track_id, it->second);
  return status;
}

size_t PlateRecognizer::finalizeMissed(double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  size_t n = 0;
  for (auto& entry : tracks_) {
    if (entry.second.tryFinalizeMiss(now_s)) {
      logFinalPlate(entry.first, entry.second);
      ++n;
    }
  }
  return n;
}

std::string PlateRecognizer::bestSnapshotKey(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return {};
  return it->second.bestSnapshotKey();
}

bool PlateRecognizer::hasSnapshotSamples(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  return it != tracks_.end() && it->second.hasSnapshotSamples();
}

bool PlateRecognizer::everEnteredPlateZone(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  return it != tracks_.end() && it->second.everEnteredPolygon();
}

bool PlateRecognizer::needsWrongLaneSnapshot(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  return it != tracks_.end() && it->second.needsWrongLaneSnapshot();
}

void PlateRecognizer::markWrongLaneSnapshotTaken(uint64_t track_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return;
  it->second.markWrongLaneSnapshotTaken();
  it->second.markHasWrongLaneSnapshot();
}

bool PlateRecognizer::awaitingSnapshot(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return false;
  return it->second.hasFinalPlate() && !it->second.isPushed() && !it->second.isPosted();
}

std::vector<PendingEmit> PlateRecognizer::collectReady(double now_s) {
  std::vector<PendingEmit> ready;
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& entry : tracks_) {
    TrackPlateState& state = entry.second;
    if (!state.hasFinalPlate() || state.isPushed() || state.isPosted()) continue;

    // Có vi phạm ngược chiều → chờ settle sau khi đủ CẢ biển lẫn vi phạm, để gom
    // nốt ảnh (crop biển + full-frame) của những frame cuối.
    if (ww_settle_s_ > 0.0 && state.wrongWayHits() > 0) {
      const double paired = state.wrongWayPairedAtS();
      if (paired > 0.0 && now_s < paired + ww_settle_s_) continue;
    }

    auto attempt = last_attempt_s_.find(entry.first);
    if (attempt != last_attempt_s_.end() && now_s - attempt->second < retry_interval_s_)
      continue;
    last_attempt_s_[entry.first] = now_s;

    const std::string plate = normalizePlateForEmit(state.plate(), config_.plate_style);
    if (!shouldEmitPlate(plate, config_.send_mode)) {
      LOG_DEBUG("track %lu: '%s' bị send_mode=%d chặn",
                static_cast<unsigned long>(entry.first), plate.c_str(), config_.send_mode);
      state.markPushed();
      state.markPosted();
      continue;
    }
    if (dedup_.alreadyEmitted(entry.first, plate)) {
      LOG_DEBUG("track %lu: '%s' trùng dedup cache",
                static_cast<unsigned long>(entry.first), plate.c_str());
      state.markPushed();
      state.markPosted();
      continue;
    }
    ready.push_back({entry.first, plate, state.votedCls(), state.bestSnapshotKey(),
                     state.createdAtS(), state.firstOcrAtS(), state.finalAtS(),
                     state.plateRecognizeCount(), state.noHelmetFrames(),
                     state.noHelmetCount(), state.wrongLaneFrames(), state.wrongLaneZone(),
                     state.hasWrongLaneSnapshot(), state.wrongWayHits(),
                     state.wrongWayLine(), state.hasWrongWaySnapshot()});
  }
  return ready;
}

bool PlateRecognizer::commitEmit(uint64_t track_id, const std::string& plate) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return false;
  if (!dedup_.tryEmit(track_id, plate)) return false;
  it->second.markPushed();
  return true;
}

void PlateRecognizer::markPosted(uint64_t track_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it != tracks_.end()) it->second.markPosted();
}

size_t PlateRecognizer::cleanup(double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  size_t removed = 0;
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (it->second.shouldForceDelete(now_s)) {
      last_attempt_s_.erase(it->first);
      it = tracks_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

size_t PlateRecognizer::trackCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tracks_.size();
}

bool PlateRecognizer::hasTrack(uint64_t track_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tracks_.count(track_id) > 0;
}

}  // namespace plate
}  // namespace business
}  // namespace vehicle
