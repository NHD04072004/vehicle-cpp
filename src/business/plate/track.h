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
// Dùng cho ảnh PHƯƠNG TIỆN (full-frame + bbox xanh) và mọi event vi phạm.
inline bool snapshotScoreBetter(const SnapshotScore& cand, const SnapshotScore& cur) {
  if (cand.area != cur.area) return cand.area > cur.area;
  return cand.mean_conf > cur.mean_conf;
}

// Điểm riêng cho ảnh CROP BIỂN: chất lượng biển, không phải độ gần của xe.
// Xe to nhưng biển mờ không được thắng frame biển nét.
struct CropScore {
  double mean_conf = 0.0;   // mean conf ký tự OCR
  double plate_area = 0.0;  // diện tích bbox BIỂN (px²)
  bool valid() const { return plate_area > 0.0; }
};

// Conf là tiêu chí chính (biển đọc rõ); hoà conf → biển to hơn.
// So sánh conf theo bước 0.02 để nhiễu nhỏ không lật ngược lựa chọn.
inline bool cropScoreBetter(const CropScore& cand, const CropScore& cur) {
  if (!cur.valid()) return cand.valid();
  if (!cand.valid()) return false;
  const double delta = cand.mean_conf - cur.mean_conf;
  if (delta > 0.02) return true;
  if (delta < -0.02) return false;
  return cand.plate_area > cur.plate_area;
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
                               double sample_area = 0.0, double plate_area = 0.0);

  double idleOutOfZoneS(double now_s) const;
  double ageS(double now_s) const;

  bool shouldRetryMissPush(double now_s) const;
  bool shouldForceDelete(double now_s) const;

  void markPushed() { is_pushed_ = true; }
  void markPosted() { is_posted_ = true; }
  void addClass(int cls, double conf = 0.0);

  void addHelmetObservation(int no_helmet_count);

  void addWrongLaneObservation(const std::string& zone_name);
  bool needsWrongLaneSnapshot() const { return needs_wrong_lane_snapshot_; }
  void markWrongLaneSnapshotTaken() { needs_wrong_lane_snapshot_ = false; }
  bool hasWrongLaneSnapshot() const { return has_wrong_lane_snapshot_; }
  void markHasWrongLaneSnapshot() { has_wrong_lane_snapshot_ = true; }

  // Vị trí anchor frame trước — để phát hiện cắt vạch giữa 2 frame.
  bool hasLastAnchor() const { return has_last_anchor_; }
  const Point& lastAnchor() const { return last_anchor_; }
  void setLastAnchor(const Point& p) { last_anchor_ = p; has_last_anchor_ = true; }

  // Lịch sử anchor (cũ → mới, tối đa kMotionHistoryLen) để tính vector hướng
  // chuyển động mượt hơn hiệu 2 frame liên tiếp.
  void pushAnchorHistory(const Point& p);
  const std::vector<Point>& anchorHistory() const { return anchor_history_; }
  // Hướng chuyển động hiện tại; {0,0} nếu chưa đủ điểm hoặc xe đứng yên.
  Point motionVector() const;
  // Xe đang đứng yên: anchor chỉ dao động quanh 1 tâm (sai số detection), không
  // trôi theo hướng nào. Chưa đủ history cũng coi là chưa đủ cơ sở → true.
  bool isStationary() const;

  void addWrongWayObservation(const std::string& line_name, double now_s = 0.0);
  bool needsWrongWaySnapshot() const { return needs_wrong_way_snapshot_; }
  void markWrongWaySnapshotTaken() { needs_wrong_way_snapshot_ = false; }
  bool hasWrongWaySnapshot() const { return has_wrong_way_snapshot_; }
  void markHasWrongWaySnapshot() { has_wrong_way_snapshot_ = true; }

  // --- Rendezvous biển số ↔ vi phạm ngược chiều ---
  // Bên nào đến trước thì cache lại; đủ cả hai → chờ settle rồi mới bắn event.
  // Quá hạn chờ mà thiếu vế còn lại → bỏ track.
  double wrongWayHitAtS() const { return wrong_way_hit_at_s_; }
  // Mốc "đủ cả hai vế" (0 nếu chưa đủ). Emit sau mốc này + settle.
  double wrongWayPairedAtS() const { return wrong_way_paired_at_s_; }
  void setWrongWayPairedAt(double now_s) {
    if (wrong_way_paired_at_s_ <= 0.0) wrong_way_paired_at_s_ = now_s;
  }

  // Class: nhiều phiếu nhất; hoà → tổng conf cao hơn. -1 nếu chưa có.
  int votedCls() const;

  uint64_t trackId() const { return track_id_; }
  const std::string& plate() const { return plate_; }
  int plateRecognizeCount() const { return plate_recognize_count_; }
  bool hasFinalPlate() const { return has_final_plate_; }
  bool isPushed() const { return is_pushed_; }
  bool isPosted() const { return is_posted_; }
  bool inPolygon() const { return in_polygon_; }
  bool everEnteredPolygon() const { return ever_entered_polygon_; }
  bool hasSnapshotSamples() const { return !samples_by_plate_.empty(); }
  const CropScore& lastCropCandidate() const { return last_crop_cand_; }
  // Key ảnh sau chốt (chuỗi biển của mẫu được chọn). Rỗng nếu chưa chốt / chưa có mẫu.
  const std::string& bestSnapshotKey() const { return best_snapshot_key_; }
  double createdAtS() const { return created_at_s_; }
  double firstOcrAtS() const { return first_ocr_at_s_; }
  double finalAtS() const { return final_at_s_; }
  int noHelmetFrames() const { return no_helmet_frames_; }
  int noHelmetCount() const { return max_no_helmet_count_; }
  int wrongLaneFrames() const { return wrong_lane_frames_; }
  const std::string& wrongLaneZone() const { return wrong_lane_zone_; }
  int wrongWayHits() const { return wrong_way_hits_; }
  const std::string& wrongWayLine() const { return wrong_way_line_; }

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
  bool ever_entered_polygon_ = false;
  int plate_recognize_count_ = 0;
  std::vector<CharSequence> list_plate_chars_;
  std::vector<std::string> list_plate_number_;
  std::string plate_;
  bool has_final_plate_ = false;
  bool is_pushed_ = false;
  bool is_posted_ = false;
  std::vector<std::pair<int, double>> list_cls_;  // (cls, conf)
  int no_helmet_frames_ = 0;
  int max_no_helmet_count_ = 0;
  int wrong_lane_frames_ = 0;
  std::string wrong_lane_zone_;
  bool needs_wrong_lane_snapshot_ = false;  // đang chờ probe chụp ảnh vi phạm
  bool has_wrong_lane_snapshot_ = false;    // đã có ảnh lúc sai làn
  int wrong_way_hits_ = 0;
  std::string wrong_way_line_;
  bool needs_wrong_way_snapshot_ = false;
  bool has_wrong_way_snapshot_ = false;
  Point last_anchor_;
  bool has_last_anchor_ = false;
  std::vector<Point> anchor_history_;
  double wrong_way_hit_at_s_ = 0.0;     // lúc cắt vạch (0 = chưa vi phạm)
  double wrong_way_paired_at_s_ = 0.0;  // lúc đủ CẢ biển + vi phạm
  // Chuỗi biển raw → điểm mẫu tốt nhất (phatnguoi _sample_by_plate).
  std::map<std::string, SnapshotScore> samples_by_plate_;
  std::string best_snapshot_key_;
  // Điểm crop của reading GẦN NHẤT. Không có sổ "đã chụp" ở đây: kho ảnh trong
  // probe là nguồn sự thật duy nhất, tránh sổ và kho lệch nhau.
  CropScore last_crop_cand_;
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
