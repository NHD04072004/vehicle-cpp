// Quản lý vòng đời track + cổng emit cho 1 camera (PIPELINE.md §5).
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "business/plate/track.h"
#include "common/config.h"

namespace vehicle {
namespace business {
namespace plate {

// Line REVERSE_DIRECTION đã scale sang pixel của frame hiện tại.
struct WrongWayLine {
  Point a;
  Point b;
  Point direction;  // chiều CẤM (mũi tên): đi cùng chiều này = vi phạm
  std::string name;
};

struct PendingEmit {
  uint64_t track_id = 0;
  std::string plate;     // đã normalize
  int vehicle_cls = -1;
  std::string snapshot_key;  // chuỗi biển của mẫu ảnh đẹp nhất
  double created_at_s = 0.0;
  double first_ocr_at_s = 0.0;
  double final_at_s = 0.0;
  int recognize_count = 0;
  int no_helmet_frames = 0;
  int no_helmet_count = 0;
  int wrong_lane_frames = 0;
  std::string wrong_lane_zone;
  bool has_wrong_lane_snapshot = false;
  int wrong_way_hits = 0;
  std::string wrong_way_line;
  bool has_wrong_way_snapshot = false;
};

class PlateRecognizer {
 public:
  explicit PlateRecognizer(const PlateConfig& config);

  // Cập nhật in/out zone + vote class. true nếu vừa chốt (miss grace).
  bool observeVehicle(uint64_t track_id, int cls, double cls_conf, bool in_zone,
                      double now_s);

  // Kết quả SGIE helmet của 1 frame (chỉ gọi cho xe máy đang trong zone).
  void observeHelmet(uint64_t track_id, int no_helmet_count);

  // Ghi nhận 1 frame xe đi sai làn (zone_name: LANE zone gây vi phạm).
  void observeLane(uint64_t track_id, const std::string& zone_name);

  // Anchor xe frame này; nếu đoạn di chuyển cắt 1 line REVERSE_DIRECTION ngược
  // chiều `direction` thì ghi nhận vi phạm. Trả true nếu vừa ghi nhận.
  bool observeWrongWay(uint64_t track_id, const Point& anchor,
                       const std::vector<WrongWayLine>& lines, double now_s = 0.0);

  bool everEnteredPlateZone(uint64_t track_id) const;

  // OCR + cập nhật mẫu snapshot theo chuỗi biển.
  PlateOcrStatus addOcrReading(uint64_t track_id, const CharSequence& chars,
                               const std::string& raw, double now_s = 0.0,
                               double mean_conf = 0.0, double sample_area = 0.0,
                               double plate_area = 0.0);

  // Điểm crop của reading vừa nhận. Probe so với crop đang có trong kho ảnh để
  // quyết định chụp — không giữ sổ "đã chụp" ở tầng business.
  CropScore lastCropCandidate(uint64_t track_id) const;

  // Chốt các track đã rời zone > grace (kể cả không còn meta trong frame).
  // Trả về số track vừa chốt.
  size_t finalizeMissed(double now_s);

  std::string bestSnapshotKey(uint64_t track_id) const;
  bool hasSnapshotSamples(uint64_t track_id) const;
  bool needsWrongLaneSnapshot(uint64_t track_id) const;
  void markWrongLaneSnapshotTaken(uint64_t track_id);
  bool needsWrongWaySnapshot(uint64_t track_id) const;
  void markWrongWaySnapshotTaken(uint64_t track_id);
  // Cấu hình rendezvous biển ↔ vi phạm ngược chiều (giây).
  void setWrongWayTiming(double settle_s, double wait_pair_s);
  // Góc lệch tối đa (độ) giữa hướng chuyển động và direction_vector để tính vi phạm.
  void setWrongWayMaxAngle(double max_angle_deg);
  // Bỏ các track đã cắt vạch nhưng quá hạn vẫn không có biển. Trả số track bỏ.
  size_t dropStaleWrongWay(double now_s);
  // Đã chốt biển, chưa emit — cần (hoặc đang chờ) snapshot vehicle.
  bool awaitingSnapshot(uint64_t track_id) const;

  std::vector<PendingEmit> collectReady(double now_s);
  bool commitEmit(uint64_t track_id, const std::string& plate, double now_s = 0.0);
  void markPosted(uint64_t track_id);
  size_t cleanup(double now_s);

  size_t trackCount() const;
  bool hasTrack(uint64_t track_id) const;

 private:
  mutable std::mutex mutex_;
  PlateConfig config_;
  std::map<uint64_t, TrackPlateState> tracks_;
  std::map<uint64_t, double> last_attempt_s_;
  DedupCache dedup_;
  double retry_interval_s_ = 1.0;
  double ww_settle_s_ = 0.0;     // chờ sau khi đủ cả biển + vi phạm
  double ww_wait_pair_s_ = 0.0;  // hạn chờ vế còn lại
  double ww_max_angle_deg_ = 40.0;  // ngưỡng "cùng chiều" với direction_vector
};

}  // namespace plate
}  // namespace business
}  // namespace vehicle
