#pragma once

#include <array>
#include <deque>
#include <map>
#include <string>
#include <unordered_map>
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

  bool shouldForceDelete(double now_s) const;

  // --- Vòng đời per-kind ----------------------------------------------------
  // Mỗi nghiệp vụ có bộ đếm + cờ riêng, không giết nhau. kinds_ là std::array
  // (EventKind là enum đóng, cố định 4 phần tử) → O(1) index, 0 cấp phát, nằm
  // liền khối trong TrackPlateState; map sẽ tốn 4 node rời + pointer-chase mỗi
  // lần duyệt, mà collectReady duyệt toàn bộ tracks_ mỗi frame.
  const ViolationState& kind(EventKind k) const { return kinds_[kindIndex(k)]; }
  ViolationState& kindMut(EventKind k) { return kinds_[kindIndex(k)]; }
  const std::string& kindLabel(EventKind k) const { return kind_label_[kindIndex(k)]; }

  // Ghi nhận 1 lần vi phạm. label rỗng = kind không có nhãn (kPlate/kNoHelmet).
  void addKindHit(EventKind k, double now_s = 0.0, int detail = 0,
                  const std::string& label = std::string());

  void markPushed(EventKind k) { kinds_[kindIndex(k)].pushed = true; }
  void markPosted(EventKind k) {
    ViolationState& v = kinds_[kindIndex(k)];
    v.pushed = true;
    v.posted = true;
  }
  void unmarkPushed(EventKind k) { kinds_[kindIndex(k)].pushed = false; }
  void markAllPosted();
  // Reset riêng 1 nghiệp vụ; track vẫn sống để bắn các kind còn lại.
  void clearKind(EventKind k);

  // kPlate active khi đã chốt biển; kind khác active khi có hit.
  EventKindMask activeKinds() const;
  EventKindMask pendingKinds() const;  // active && chưa posted
  bool allSettled() const { return pendingKinds() == 0; }

  void markSnapshotTaken(EventKind k) { kinds_[kindIndex(k)].needs_snapshot = false; }
  void markHasSnapshot(EventKind k) { kinds_[kindIndex(k)].has_snapshot = true; }

  void addClass(int cls, double conf = 0.0);

  void addHelmetObservation(int no_helmet_count);

  void addWrongLaneObservation(const std::string& zone_name);
  bool needsWrongLaneSnapshot() const { return kind(EventKind::kWrongLane).needs_snapshot; }
  void markWrongLaneSnapshotTaken() { markSnapshotTaken(EventKind::kWrongLane); }
  bool hasWrongLaneSnapshot() const { return kind(EventKind::kWrongLane).has_snapshot; }
  void markHasWrongLaneSnapshot() { markHasSnapshot(EventKind::kWrongLane); }

  // Vị trí anchor frame trước — để phát hiện cắt vạch giữa 2 frame.
  bool hasLastAnchor() const { return has_last_anchor_; }
  const Point& lastAnchor() const { return last_anchor_; }
  void setLastAnchor(const Point& p) { last_anchor_ = p; has_last_anchor_ = true; }

  // Lịch sử anchor (cũ → mới, tối đa kMotionHistoryLen) để tính vector hướng
  // chuyển động mượt hơn hiệu 2 frame liên tiếp.
  void pushAnchorHistory(const Point& p);
  // Hướng chuyển động hiện tại; {0,0} nếu chưa đủ điểm hoặc xe đứng yên.
  Point motionVector() const;
  // Xe đang đứng yên: anchor chỉ dao động quanh 1 tâm (sai số detection), không
  // trôi theo hướng nào. Chưa đủ history cũng coi là chưa đủ cơ sở → true.
  bool isStationary() const;

  void addWrongWayObservation(const std::string& line_name, double now_s = 0.0);
  bool needsWrongWaySnapshot() const { return kind(EventKind::kWrongWay).needs_snapshot; }
  void markWrongWaySnapshotTaken() { markSnapshotTaken(EventKind::kWrongWay); }
  bool hasWrongWaySnapshot() const { return kind(EventKind::kWrongWay).has_snapshot; }
  void markHasWrongWaySnapshot() { markHasSnapshot(EventKind::kWrongWay); }

  // --- Rendezvous biển số ↔ vi phạm ngược chiều ---
  // Bên nào đến trước thì cache lại; đủ cả hai → chờ settle rồi mới bắn event.
  // Quá hạn chờ mà thiếu vế còn lại → chỉ bỏ vế WRONG_WAY, track vẫn sống.
  double wrongWayHitAtS() const { return kind(EventKind::kWrongWay).first_hit_at_s; }
  // Mốc "đủ cả hai vế" (0 nếu chưa đủ). Emit sau mốc này + settle.
  double wrongWayPairedAtS() const { return kind(EventKind::kWrongWay).paired_at_s; }
  void setWrongWayPairedAt(double now_s) {
    ViolationState& v = kindMut(EventKind::kWrongWay);
    if (v.paired_at_s <= 0.0) v.paired_at_s = now_s;
  }

  // Class: nhiều phiếu nhất; hoà → tổng conf cao hơn. -1 nếu chưa có.
  int votedCls() const;

  uint64_t trackId() const { return track_id_; }
  const std::string& plate() const { return plate_; }
  int plateRecognizeCount() const { return plate_recognize_count_; }
  bool hasFinalPlate() const { return has_final_plate_; }
  // Biển đã normalize sẵn lúc chốt — normalizePlateForEmit không phải chạy lại
  // mỗi frame cho mỗi track trong collectReady.
  const std::string& emitPlate() const { return emit_plate_; }
  bool inPolygon() const { return in_polygon_; }
  bool everEnteredPolygon() const { return ever_entered_polygon_; }
  const CropScore& lastCropCandidate() const { return last_crop_cand_; }
  // Key ảnh sau chốt (chuỗi biển của mẫu được chọn). Rỗng nếu chưa chốt / chưa có mẫu.
  const std::string& bestSnapshotKey() const { return best_snapshot_key_; }
  double createdAtS() const { return created_at_s_; }
  double firstOcrAtS() const { return first_ocr_at_s_; }
  double finalAtS() const { return final_at_s_; }
  int noHelmetFrames() const { return kind(EventKind::kNoHelmet).hits; }
  int noHelmetCount() const { return kind(EventKind::kNoHelmet).detail; }
  int wrongLaneFrames() const { return kind(EventKind::kWrongLane).hits; }
  const std::string& wrongLaneZone() const { return kindLabel(EventKind::kWrongLane); }
  int wrongWayHits() const { return kind(EventKind::kWrongWay).hits; }
  const std::string& wrongWayLine() const { return kindLabel(EventKind::kWrongWay); }

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
  bool in_polygon_ = false;
  bool ever_entered_polygon_ = false;
  int plate_recognize_count_ = 0;
  std::vector<CharSequence> list_plate_chars_;
  std::vector<std::string> list_plate_number_;
  std::string plate_;
  std::string emit_plate_;  // plate_ đã normalize; set 1 lần lúc chốt
  bool has_final_plate_ = false;
  std::vector<std::pair<int, double>> list_cls_;  // (cls, conf)
  // Vòng đời 4 nghiệp vụ. Mảng cố định → 0 cấp phát, truy cập O(1) theo index.
  std::array<ViolationState, kEventKindCount> kinds_{};
  // Nhãn bằng chứng theo kind (zone LANE / line REVERSE_DIRECTION). Tách khỏi
  // ViolationState để struct đó giữ nguyên POD và không cấp phát.
  std::array<std::string, kEventKindCount> kind_label_{};
  Point last_anchor_;
  bool has_last_anchor_ = false;
  std::vector<Point> anchor_history_;
  // Chuỗi biển raw → điểm mẫu tốt nhất (phatnguoi _sample_by_plate).
  std::map<std::string, SnapshotScore> samples_by_plate_;
  std::string best_snapshot_key_;
  // Điểm crop của reading GẦN NHẤT. Không có sổ "đã chụp" ở đây: kho ảnh trong
  // probe là nguồn sự thật duy nhất, tránh sổ và kho lệch nhau.
  CropScore last_crop_cand_;
};

// Chống bắn trùng theo bộ ba (track_id, plate, kind).
//
// Bản cũ giữ 2 deque rời và khớp theo OR (trùng track_id HOẶC trùng plate), nên
// hai XE KHÁC NHAU cùng đọc ra một chuỗi biển thì xe thứ hai bị nuốt hoàn toàn
// — rất phổ biến với 'UNKOWN'. Khoá theo bộ ba sửa đúng chỗ đó: cùng xe cùng
// nghiệp vụ mới bị chặn.
//
// Tra cứu O(1) trung bình (bản cũ là std::find O(n) chạy 2 lần mỗi track mỗi
// frame). FIFO giữ trần maxlen; TTL để entry không sống mãi vì tracker có thể
// tái sử dụng track_id sau thời gian dài.
class DedupCache {
 public:
  explicit DedupCache(size_t maxlen = kDedupCacheSize, double ttl_s = kDedupTtlS)
      : maxlen_(maxlen), ttl_s_(ttl_s) {}

  bool alreadyEmitted(uint64_t track_id, const std::string& plate, EventKind kind,
                      double now_s = 0.0) const;
  void remember(uint64_t track_id, const std::string& plate, EventKind kind,
                double now_s = 0.0);
  bool tryEmit(uint64_t track_id, const std::string& plate, EventKind kind,
               double now_s = 0.0);
  // Publish thất bại → gỡ khoá để lần retry sau không bị chính dedup chặn.
  void forget(uint64_t track_id, const std::string& plate, EventKind kind);

 private:
  struct Key {
    uint64_t track_id = 0;
    std::string plate;
    EventKind kind = EventKind::kPlate;
    bool operator==(const Key& o) const {
      return track_id == o.track_id && kind == o.kind && plate == o.plate;
    }
  };
  struct KeyHash {
    size_t operator()(const Key& k) const noexcept;
  };

  // Bỏ entry hết hạn và ép trần maxlen. Amortized O(1).
  void evict(double now_s);

  size_t maxlen_;
  double ttl_s_;
  std::unordered_map<Key, double, KeyHash> seen_;  // key → mốc hết hạn (0 = không TTL)
  std::deque<Key> fifo_;                           // thứ tự chèn, để ép trần
};

}  // namespace plate
}  // namespace business
}  // namespace vehicle
