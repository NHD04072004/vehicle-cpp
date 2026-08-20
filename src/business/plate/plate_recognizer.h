// Quản lý vòng đời track + cổng emit cho 1 camera (PIPELINE.md §5).
#pragma once

#include <array>
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

// Số liệu của MỘT nghiệp vụ tại thời điểm sẵn sàng emit. Không giữ string —
// nhãn nằm ở ReadyEmit::labels để struct này POD.
struct KindPayload {
  int hits = 0;
  int detail = 0;  // kNoHelmet: số người không mũ tối đa
  bool has_snapshot = false;
};

// Một track có ÍT NHẤT một nghiệp vụ sẵn sàng emit ở frame này.
//
// Giữ 1 phần tử/track (không phải 1/kind) để mỗi track vẫn chỉ tra kho ảnh một
// lần và dựng đúng một EmitJob — số job không tăng, nên kMaxEmitQueue không bị
// đầy sớm hơn hiện tại. Phần tách theo nghiệp vụ nằm ở `ready` + `payload`.
struct ReadyEmit {
  uint64_t track_id = 0;
  EventKindMask ready = 0;  // bit nào set = kind đó sẵn sàng NGAY frame này
  // --- dùng chung mọi kind, chỉ copy 1 lần/track/frame ---
  std::string plate;         // đã normalize
  std::string snapshot_key;  // chuỗi biển của mẫu ảnh đẹp nhất
  int vehicle_cls = -1;
  double created_at_s = 0.0;
  double first_ocr_at_s = 0.0;
  double final_at_s = 0.0;
  int recognize_count = 0;
  // --- riêng từng nghiệp vụ ---
  std::array<KindPayload, kEventKindCount> payload{};
  std::array<std::string, kEventKindCount> labels{};  // zone LANE / line REVERSE
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
  // Track đã cắt vạch nhưng quá hạn vẫn thiếu vế còn lại (biển hoặc ảnh) → bỏ
  // RIÊNG vế WRONG_WAY, track vẫn sống cho các nghiệp vụ khác. Trả số vế đã bỏ.
  size_t clearStaleWrongWay(double now_s);
  // Đã chốt biển, chưa emit — cần (hoặc đang chờ) snapshot vehicle.
  bool awaitingSnapshot(uint64_t track_id) const;

  // Track có ít nhất 1 nghiệp vụ sẵn sàng emit. `out` là buffer tái dùng của
  // caller (clear rồi ghi đè) → không cấp phát vector mỗi frame.
  void collectReady(double now_s, std::vector<ReadyEmit>* out);
  // Ghi dedup + đánh dấu pushed cho ĐÚNG các kind trong `kinds`.
  bool commitEmit(uint64_t track_id, const std::string& plate, EventKindMask kinds,
                  double now_s = 0.0);
  // done: kind đã publish xong (hoặc bị chặn vĩnh viễn) → posted.
  // want & ~done: lỗi tạm → gỡ cờ pushed và gỡ khoá dedup để retry.
  void settleKinds(uint64_t track_id, EventKindMask want, EventKindMask done);
  size_t cleanup(double now_s);

  size_t trackCount() const;
  bool hasTrack(uint64_t track_id) const;

 private:
  // Bitmask kind sẵn sàng emit ngay frame này; 0 = chưa có gì. Gọi trong lock.
  EventKindMask readyMaskLocked(TrackPlateState& state, double now_s);

  mutable std::mutex mutex_;
  PlateConfig config_;
  std::map<uint64_t, TrackPlateState> tracks_;
  DedupCache dedup_;
  double retry_interval_s_ = 1.0;
  double ww_settle_s_ = 0.0;     // chờ sau khi đủ cả biển + vi phạm
  double ww_wait_pair_s_ = 0.0;  // hạn chờ vế còn lại
  double ww_max_angle_deg_ = 40.0;  // ngưỡng "cùng chiều" với direction_vector
};

}  // namespace plate
}  // namespace business
}  // namespace vehicle
