#include "business/plate/track.h"

#include <algorithm>
#include <utility>

#include "utils/geometry.h"
#include "utils/string_utils.h"

namespace vehicle {
namespace business {
namespace plate {
namespace {

bool isBlankPlate(const std::string& plate) {
  return plate.empty() || plate == kEmptyPlate;
}

}  // namespace

TrackPlateState::TrackPlateState(uint64_t track_id, double created_at_s,
                                 int max_recognize_times, std::vector<std::string> styles)
    : track_id_(track_id),
      max_recognize_times_(max_recognize_times),
      styles_(std::move(styles)),
      created_at_s_(created_at_s) {
  if (styles_.empty()) styles_ = defaultPlateStyles();
}

void TrackPlateState::onEnterPolygon(double now_s) {
  in_polygon_ = true;
  ever_entered_polygon_ = true;
  last_in_zone_at_s_ = now_s;
}

void TrackPlateState::onLeavePolygon(double now_s) {
  in_polygon_ = false;
  last_in_zone_at_s_ = now_s;
}

bool TrackPlateState::tryFinalizeMiss(double now_s) {
  if (has_final_plate_ || in_polygon_) return false;
  if (idleOutOfZoneS(now_s) <= kMissTrackIdleS) return false;
  return finalizePlate(now_s);
}

bool TrackPlateState::canOcr() const {
  // Cho thêm slack để chờ xe gần hơn trước khi hard-cap chốt.
  const int cap = std::max(max_recognize_times_, max_recognize_times_ * 2);
  return in_polygon_ && !has_final_plate_ && plate_recognize_count_ < cap;
}

void TrackPlateState::pickBestSnapshotKey() {
  if (samples_by_plate_.empty()) {
    best_snapshot_key_.clear();
    return;
  }

  // 1) Ưu tiên mẫu khớp biển đã chốt (phatnguoi finalize_best_assets).
  if (!isBlankPlate(plate_)) {
    auto it = samples_by_plate_.find(plate_);
    if (it != samples_by_plate_.end()) {
      best_snapshot_key_ = it->first;
      return;
    }
  }

  // 2) Mẫu không EMPTY — điểm (conf, area) cao nhất.
  auto prefer = [&](const std::string& a, const std::string& b) {
    return snapshotScoreBetter(samples_by_plate_.at(a), samples_by_plate_.at(b));
  };

  std::string best;
  for (const auto& kv : samples_by_plate_) {
    if (isBlankPlate(kv.first)) continue;
    if (best.empty() || prefer(kv.first, best)) best = kv.first;
  }
  if (!best.empty()) {
    best_snapshot_key_ = best;
    return;
  }

  // 3) Fallback bất kỳ.
  best = samples_by_plate_.begin()->first;
  for (const auto& kv : samples_by_plate_) {
    if (prefer(kv.first, best)) best = kv.first;
  }
  best_snapshot_key_ = best;
}

bool TrackPlateState::finalizePlate(double now_s) {
  if (has_final_plate_) return false;
  if (list_plate_chars_.empty()) return false;

  const std::vector<std::string>& styles = styles_;
  plate_ = selectBestPlate(list_plate_chars_, [&styles](const std::string& text) {
    return isPlateValid(text, styles);
  });
  has_final_plate_ = true;
  if (now_s > 0.0) final_at_s_ = now_s;
  // Biển đến sau vi phạm → cặp đã đủ ngay tại đây.
  if (wrong_way_hits_ > 0 && now_s > 0.0) setWrongWayPairedAt(now_s);
  pickBestSnapshotKey();
  return true;
}

PlateOcrStatus TrackPlateState::addOcrReading(const CharSequence& chars, const std::string& raw,
                                              double now_s, double mean_conf,
                                              double sample_area, double plate_area) {
  if (!canOcr()) return PlateOcrStatus::kRejected;
  if (chars.empty()) return PlateOcrStatus::kRejected;

  CharSequence upper;
  upper.reserve(chars.size());
  double conf_sum = 0.0;
  for (const CharReading& ch : chars) {
    upper.push_back({utils::toUpper(ch.text), ch.confidence});
    conf_sum += ch.confidence;
  }
  const std::string plate_key = utils::toUpper(raw);
  list_plate_chars_.push_back(std::move(upper));
  list_plate_number_.push_back(plate_key);
  plate_recognize_count_ += 1;
  if (first_ocr_at_s_ <= 0.0 && now_s > 0.0) first_ocr_at_s_ = now_s;

  // mean_conf từ caller; nếu 0 thì tự tính từ ký tự.
  SnapshotScore cand;
  cand.mean_conf =
      mean_conf > 0.0 ? mean_conf : conf_sum / static_cast<double>(chars.size());
  cand.area = std::max(0.0, sample_area);
  last_sample_area_ = cand.area;
  // Chỉ coi là "đang tiến gần" khi đã có baseline và area tăng >5%.
  const bool area_grew = max_sample_area_ > 0.0 && cand.area > max_sample_area_ * 1.05;
  max_sample_area_ = std::max(max_sample_area_, cand.area);

  bool better_snap = false;
  auto it = samples_by_plate_.find(plate_key);
  if (it == samples_by_plate_.end() || snapshotScoreBetter(cand, it->second)) {
    samples_by_plate_[plate_key] = cand;
    better_snap = true;
  }

  // Điểm crop chấm theo chất lượng BIỂN (không phải độ gần của xe). Probe so
  // điểm này với crop đang có trong kho ảnh của đúng chuỗi `plate_key`.
  last_crop_cand_.mean_conf = cand.mean_conf;
  last_crop_cand_.plate_area = std::max(0.0, plate_area);

  // Chốt sớm khi đủ N reading và xe không còn đang tiến gần (area không tăng).
  // Nếu vẫn tiến gần → giữ OCR tới hard-cap 2×N hoặc miss-finalize.
  const bool hard_cap = plate_recognize_count_ >= max_recognize_times_ * 2;
  const bool ready = plate_recognize_count_ >= max_recognize_times_ && (!area_grew || hard_cap);
  if (ready && finalizePlate(now_s)) {
    return better_snap ? PlateOcrStatus::kFinalizedBestSnapshot : PlateOcrStatus::kFinalized;
  }
  return better_snap ? PlateOcrStatus::kBestSnapshot : PlateOcrStatus::kRecognized;
}

double TrackPlateState::idleOutOfZoneS(double now_s) const {
  if (in_polygon_) return 0.0;
  return std::max(0.0, now_s - last_in_zone_at_s_);
}

double TrackPlateState::ageS(double now_s) const {
  return std::max(0.0, now_s - created_at_s_);
}

bool TrackPlateState::shouldRetryMissPush(double now_s) const {
  return has_final_plate_ && !is_pushed_ && !in_polygon_ &&
         idleOutOfZoneS(now_s) > kMissTrackIdleS;
}

bool TrackPlateState::shouldForceDelete(double now_s) const {
  if (is_posted_) return true;
  if (!in_polygon_ && idleOutOfZoneS(now_s) > kForceDeleteIdleS) return true;
  if (ageS(now_s) > kForceDeleteAgeS) return true;
  return false;
}

void TrackPlateState::addClass(int cls, double conf) {
  list_cls_.emplace_back(cls, conf);
}

void TrackPlateState::addHelmetObservation(int no_helmet_count) {
  if (no_helmet_count <= 0) return;
  ++no_helmet_frames_;
  if (no_helmet_count > max_no_helmet_count_) max_no_helmet_count_ = no_helmet_count;
}

void TrackPlateState::addWrongLaneObservation(const std::string& zone_name) {
  ++wrong_lane_frames_;
  if (wrong_lane_zone_.empty()) wrong_lane_zone_ = zone_name;
  if (!has_wrong_lane_snapshot_) needs_wrong_lane_snapshot_ = true;
}

void TrackPlateState::pushAnchorHistory(const Point& p) {
  anchor_history_.push_back(p);
  if (anchor_history_.size() > kMotionHistoryLen)
    anchor_history_.erase(anchor_history_.begin());
}

Point TrackPlateState::motionVector() const {
  if (anchor_history_.size() < kMinMotionHistoryLen) return Point{0.0, 0.0};
  if (isStationary()) return Point{0.0, 0.0};
  const Point v = utils::motionVector(anchor_history_);
  if (v.x * v.x + v.y * v.y < kMinMotionLenPx * kMinMotionLenPx) return Point{0.0, 0.0};
  return v;
}

bool TrackPlateState::isStationary() const {
  const size_t n = anchor_history_.size();
  if (n < kMinMotionHistoryLen) return true;  // chưa đủ cơ sở kết luận đang chạy

  // Quãng đường tịnh: xe bò chậm vẫn trôi đều một hướng, xe đỗ thì quay về chỗ cũ.
  const Point& first = anchor_history_.front();
  const Point& last = anchor_history_.back();
  const double net_dx = last.x - first.x;
  const double net_dy = last.y - first.y;
  if (net_dx * net_dx + net_dy * net_dy > kStationaryNetPx * kStationaryNetPx)
    return false;

  // Tán xạ quanh tâm: mọi anchor nằm gọn trong bán kính = chỉ là nhiễu detection.
  double cx = 0.0, cy = 0.0;
  for (const Point& p : anchor_history_) {
    cx += p.x;
    cy += p.y;
  }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);
  for (const Point& p : anchor_history_) {
    const double ddx = p.x - cx;
    const double ddy = p.y - cy;
    if (ddx * ddx + ddy * ddy > kStationaryRadiusPx * kStationaryRadiusPx) return false;
  }
  return true;
}

void TrackPlateState::addWrongWayObservation(const std::string& line_name, double now_s) {
  ++wrong_way_hits_;
  if (wrong_way_line_.empty()) wrong_way_line_ = line_name;
  if (!has_wrong_way_snapshot_) needs_wrong_way_snapshot_ = true;
  if (wrong_way_hit_at_s_ <= 0.0 && now_s > 0.0) wrong_way_hit_at_s_ = now_s;
  // Vi phạm đến sau biển → cặp đã đủ ngay tại đây.
  if (has_final_plate_ && now_s > 0.0) setWrongWayPairedAt(now_s);
}

int TrackPlateState::votedCls() const {
  if (list_cls_.empty()) return -1;

  // (votes, conf_sum) — khớp phatnguoi resolve_best_cls; hoà phiếu → tổng conf.
  std::vector<std::pair<int, std::pair<int, double>>> stats;  // cls → (votes, conf_sum)
  for (const auto& sample : list_cls_) {
    const int cls = sample.first;
    const double conf = sample.second;
    auto it = std::find_if(stats.begin(), stats.end(),
                           [cls](const auto& kv) { return kv.first == cls; });
    if (it == stats.end())
      stats.push_back({cls, {1, conf}});
    else {
      it->second.first += 1;
      it->second.second += conf;
    }
  }

  const auto* best = &stats.front();
  for (const auto& kv : stats) {
    if (kv.second.first > best->second.first ||
        (kv.second.first == best->second.first && kv.second.second > best->second.second)) {
      best = &kv;
    }
  }
  return best->first;
}

bool DedupCache::alreadyEmitted(uint64_t track_id, const std::string& plate) const {
  if (std::find(track_ids_.begin(), track_ids_.end(), track_id) != track_ids_.end())
    return true;
  return std::find(plates_.begin(), plates_.end(), plate) != plates_.end();
}

void DedupCache::remember(uint64_t track_id, const std::string& plate) {
  track_ids_.push_back(track_id);
  plates_.push_back(plate);
  while (track_ids_.size() > maxlen_) track_ids_.pop_front();
  while (plates_.size() > maxlen_) plates_.pop_front();
}

bool DedupCache::tryEmit(uint64_t track_id, const std::string& plate) {
  if (alreadyEmitted(track_id, plate)) return false;
  remember(track_id, plate);
  return true;
}

}  // namespace plate
}  // namespace business
}  // namespace vehicle
