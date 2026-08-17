// Quản lý vòng đời track + cổng emit cho 1 camera (PIPELINE.md §5).
#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "business/plate/track.h"
#include "common/config.h"

namespace vehicle {
namespace business {
namespace plate {

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
};

class PlateRecognizer {
 public:
  explicit PlateRecognizer(const PlateConfig& config);

  // Cập nhật in/out zone + vote class. true nếu vừa chốt (miss grace).
  bool observeVehicle(uint64_t track_id, int cls, double cls_conf, bool in_zone,
                      double now_s);

  // Kết quả SGIE helmet của 1 frame (chỉ gọi cho xe máy đang trong zone).
  void observeHelmet(uint64_t track_id, int no_helmet_count);

  // OCR + cập nhật mẫu snapshot theo chuỗi biển.
  PlateOcrStatus addOcrReading(uint64_t track_id, const CharSequence& chars,
                               const std::string& raw, double now_s = 0.0,
                               double mean_conf = 0.0, double sample_area = 0.0);

  // Chốt các track đã rời zone > grace (kể cả không còn meta trong frame).
  // Trả về số track vừa chốt.
  size_t finalizeMissed(double now_s);

  std::string bestSnapshotKey(uint64_t track_id) const;
  bool hasSnapshotSamples(uint64_t track_id) const;
  // Đã chốt biển, chưa emit — cần (hoặc đang chờ) snapshot vehicle.
  bool awaitingSnapshot(uint64_t track_id) const;

  std::vector<PendingEmit> collectReady(double now_s);
  bool commitEmit(uint64_t track_id, const std::string& plate);
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
};

}  // namespace plate
}  // namespace business
}  // namespace vehicle
