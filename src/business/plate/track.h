#pragma once

#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "business/plate/rules.h"

namespace vehicle {
namespace business {
namespace plate {

// Trạng thái biển sau 1 lần addOcrReading.
enum class PlateOcrStatus {
  kRejected = 0,          // không nhận reading
  kRecognized,            // nhận OCR, chưa chốt, snapshot không tốt hơn
  kBestSnapshot,          // nhận OCR + mẫu snapshot đẹp hơn cho chuỗi này
  kFinalized,             // vừa chốt biển (đủ max)
  kFinalizedBestSnapshot, // vừa chốt + mẫu snapshot đẹp hơn
};

inline bool plateOcrAccepted(PlateOcrStatus s) {
  return s != PlateOcrStatus::kRejected;
}
inline bool plateOcrJustFinal(PlateOcrStatus s) {
  return s == PlateOcrStatus::kFinalized || s == PlateOcrStatus::kFinalizedBestSnapshot;
}
inline bool plateOcrBetterSnapshot(PlateOcrStatus s) {
  return s == PlateOcrStatus::kBestSnapshot || s == PlateOcrStatus::kFinalizedBestSnapshot;
}

// Điểm 1 mẫu snapshot theo chuỗi biển (khớp phatnguoi add_recognize_sample).
struct SnapshotScore {
  double mean_conf = 0.0;
  double area = 0.0;  // diện tích bbox xe (px²)
};

// Ưu tiên xe gần (diện tích lớn), rồi mean conf ký tự.
inline bool snapshotScoreBetter(const SnapshotScore& cand, const SnapshotScore& cur) {
  if (cand.area != cur.area) return cand.area > cur.area;
  return cand.mean_conf > cur.mean_conf;
}

class TrackPlateState {
 public:
  TrackPlateState() = default;
  TrackPlateState(uint64_t track_id, double created_at_s,
                  int max_recognize_times = kDefaultMaxRecognizeTimes,
                  std::vector<std::string> styles = defaultPlateStyles());

  void onEnterPolygon(double now_s);
  void onLeavePolygon(double now_s);
  bool tryFinalizeMiss(double now_s);
  bool canOcr() const;

  // Thêm 1 lần đọc OCR.
  // mean_conf: mean conf ký tự; sample_area: diện tích bbox xe — chọn snapshot theo chuỗi biển.
  PlateOcrStatus addOcrReading(const CharSequence& chars, const std::string& raw,
                               double now_s = 0.0, double mean_conf = 0.0,
                               double sample_area = 0.0);

  double idleOutOfZoneS(double now_s) const;
  double ageS(double now_s) const;

  bool shouldRetryMissPush(double now_s) const;
  bool shouldForceDelete(double now_s) const;

  void markPushed() { is_pushed_ = true; }
  void markPosted() { is_posted_ = true; }
  void addClass(int cls, double conf = 0.0);

  // Class: nhiều phiếu nhất; hoà → tổng conf cao hơn. -1 nếu chưa có.
  int votedCls() const;

  uint64_t trackId() const { return track_id_; }
  const std::string& plate() const { return plate_; }
  int plateRecognizeCount() const { return plate_recognize_count_; }
  bool hasFinalPlate() const { return has_final_plate_; }
  bool isPushed() const { return is_pushed_; }
  bool isPosted() const { return is_posted_; }
  bool inPolygon() const { return in_polygon_; }
  bool hasSnapshotSamples() const { return !samples_by_plate_.empty(); }
  // Key ảnh sau chốt (chuỗi biển của mẫu được chọn). Rỗng nếu chưa chốt / chưa có mẫu.
  const std::string& bestSnapshotKey() const { return best_snapshot_key_; }
  double createdAtS() const { return created_at_s_; }
  double firstOcrAtS() const { return first_ocr_at_s_; }
  double finalAtS() const { return final_at_s_; }

 private:
  bool finalizePlate(double now_s);
  void pickBestSnapshotKey();

  uint64_t track_id_ = 0;
  int max_recognize_times_ = kDefaultMaxRecognizeTimes;
  std::vector<std::string> styles_ = defaultPlateStyles();
  double created_at_s_ = 0.0;
  double last_in_zone_at_s_ = 0.0;
  double first_ocr_at_s_ = 0.0;
  double final_at_s_ = 0.0;
  double max_sample_area_ = 0.0;
  double last_sample_area_ = 0.0;
  bool in_polygon_ = false;
  int plate_recognize_count_ = 0;
  std::vector<CharSequence> list_plate_chars_;
  std::vector<std::string> list_plate_number_;
  std::string plate_;
  bool has_final_plate_ = false;
  bool is_pushed_ = false;
  bool is_posted_ = false;
  std::vector<std::pair<int, double>> list_cls_;  // (cls, conf)
  // Chuỗi biển raw → điểm mẫu tốt nhất (phatnguoi _sample_by_plate).
  std::map<std::string, SnapshotScore> samples_by_plate_;
  std::string best_snapshot_key_;
};

class DedupCache {
 public:
  explicit DedupCache(size_t maxlen = kDedupCacheSize) : maxlen_(maxlen) {}

  bool alreadyEmitted(uint64_t track_id, const std::string& plate) const;
  void remember(uint64_t track_id, const std::string& plate);
  bool tryEmit(uint64_t track_id, const std::string& plate);

 private:
  size_t maxlen_;
  std::deque<uint64_t> track_ids_;
  std::deque<std::string> plates_;
};

}  // namespace plate
}  // namespace business
}  // namespace vehicle
