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

size_t PlateRecognizer::clearStaleWrongWay(double now_s) {
  if (ww_wait_pair_s_ <= 0.0) return 0;
  std::lock_guard<std::mutex> lock(mutex_);
  size_t cleared = 0;
  for (auto& entry : tracks_) {
    TrackPlateState& state = entry.second;
    const ViolationState& ww = state.kind(EventKind::kWrongWay);
    if (ww.hits <= 0 || ww.posted) continue;
    if (ww.first_hit_at_s <= 0.0 || now_s < ww.first_hit_at_s + ww_wait_pair_s_) continue;

    // Chỉ bỏ vế WRONG_WAY. Track vẫn sống để PLATE/NO_HELMET/WRONG_LANE bắn
    // bình thường — chúng không cần crop biển của riêng nghiệp vụ ngược chiều.
    LOG_INFO("track %lu: WRONG_WAY quá %.1fs %s — bỏ RIÊNG vế ngược chiều",
             static_cast<unsigned long>(entry.first), ww_wait_pair_s_,
             state.hasFinalPlate() ? "không đủ ảnh (crop biển + full-frame)"
                                   : "không có biển số");
    state.clearKind(EventKind::kWrongWay);
    ++cleared;
  }
  return cleared;
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

EventKindMask PlateRecognizer::readyMaskLocked(TrackPlateState& state, double now_s) {
  // Cổng CHUNG duy nhất: không có biển thì không nghiệp vụ nào xử phạt được.
  if (!state.hasFinalPlate()) return 0;

  const std::string& plate = state.emitPlate();  // đã normalize sẵn lúc chốt

  // Biển bị send_mode chặn → không kind nào bắn được. Đây là quyết định chốt
  // VĨNH VIỄN (không phải lỗi tạm), nên đánh dấu posted để thôi thử lại.
  if (!shouldEmitPlate(plate, config_.send_mode)) {
    if (!state.allSettled()) {
      LOG_DEBUG("track %lu: '%s' bị send_mode=%d chặn — chốt mọi nghiệp vụ",
                static_cast<unsigned long>(state.trackId()), plate.c_str(),
                config_.send_mode);
      state.markAllPosted();
    }
    return 0;
  }

  EventKindMask ready = 0;
  for (size_t i = 0; i < kEventKindCount; ++i) {
    const EventKind k = static_cast<EventKind>(i);
    ViolationState& ks = state.kindMut(k);

    // kPlate ăn theo hasFinalPlate; nghiệp vụ khác cần có hit.
    if (k != EventKind::kPlate && ks.hits <= 0) continue;
    if (ks.pushed || ks.posted) continue;

    // CHỈ WRONG_WAY: chờ settle sau khi đủ cả biển lẫn vi phạm, để gom nốt ảnh
    // của những frame cuối. Không chặn các nghiệp vụ khác.
    if (k == EventKind::kWrongWay && ww_settle_s_ > 0.0 && ks.paired_at_s > 0.0 &&
        now_s < ks.paired_at_s + ww_settle_s_) {
      continue;
    }

    // Retry throttle riêng từng nghiệp vụ.
    if (ks.last_attempt_s > 0.0 && now_s - ks.last_attempt_s < retry_interval_s_) continue;

    if (dedup_.alreadyEmitted(state.trackId(), plate, k, now_s)) {
      LOG_DEBUG("track %lu: '%s' kind=%s trùng dedup cache",
                static_cast<unsigned long>(state.trackId()), plate.c_str(),
                eventKindName(k));
      state.markPosted(k);  // chỉ chốt kind này, không đụng kind khác
      continue;
    }

    ks.last_attempt_s = now_s;
    ready |= static_cast<EventKindMask>(1u << i);
  }
  return ready;
}

void PlateRecognizer::collectReady(double now_s, std::vector<ReadyEmit>* out) {
  if (out == nullptr) return;
  out->clear();
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& entry : tracks_) {
    TrackPlateState& state = entry.second;
    const EventKindMask ready = readyMaskLocked(state, now_s);
    // Lọc TRƯỚC khi chạm string: track đang chờ ảnh không còn copy plate +
    // snapshot_key mỗi frame như trước.
    if (ready == 0) continue;

    out->emplace_back();
    ReadyEmit& re = out->back();
    re.track_id = entry.first;
    re.ready = ready;
    re.plate = state.emitPlate();
    re.snapshot_key = state.bestSnapshotKey();
    re.vehicle_cls = state.votedCls();
    re.created_at_s = state.createdAtS();
    re.first_ocr_at_s = state.firstOcrAtS();
    re.final_at_s = state.finalAtS();
    re.recognize_count = state.plateRecognizeCount();
    for (size_t i = 0; i < kEventKindCount; ++i) {
      if ((ready & (1u << i)) == 0) continue;  // chỉ copy kind đang sẵn sàng
      const ViolationState& ks = state.kind(static_cast<EventKind>(i));
      re.payload[i] = KindPayload{ks.hits, ks.detail, ks.has_snapshot};
      re.labels[i] = state.kindLabel(static_cast<EventKind>(i));
    }
  }
}

bool PlateRecognizer::commitEmit(uint64_t track_id, const std::string& plate,
                                 EventKindMask kinds, double now_s) {
  if (kinds == 0) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return false;

  EventKindMask committed = 0;
  for (size_t i = 0; i < kEventKindCount; ++i) {
    const EventKind k = static_cast<EventKind>(i);
    if (!maskHas(kinds, k)) continue;
    if (!dedup_.tryEmit(track_id, plate, k, now_s)) continue;
    it->second.markPushed(k);
    committed |= kindBit(k);
  }
  return committed != 0;
}

void PlateRecognizer::settleKinds(uint64_t track_id, EventKindMask want,
                                  EventKindMask done) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tracks_.find(track_id);
  if (it == tracks_.end()) return;
  TrackPlateState& state = it->second;
  const std::string& plate = state.emitPlate();
  for (size_t i = 0; i < kEventKindCount; ++i) {
    const EventKind k = static_cast<EventKind>(i);
    if (!maskHas(want, k)) continue;
    if (maskHas(done, k)) {
      state.markPosted(k);
      continue;
    }
    // Lỗi tạm: mở lại cờ pushed VÀ gỡ khoá dedup, nếu không lần retry sau sẽ bị
    // chính dedup chặn (cache được ghi lúc commitEmit, trước khi publish).
    state.unmarkPushed(k);
    dedup_.forget(track_id, plate, k);
  }
}

size_t PlateRecognizer::cleanup(double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  size_t removed = 0;
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (it->second.shouldForceDelete(now_s)) {
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
