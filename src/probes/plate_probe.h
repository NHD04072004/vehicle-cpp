// Pad probe: đọc NvDsBatchMeta → lọc ROI → OCR → business → communication.
#pragma once

#include <gst/gst.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "business/plate/plate_recognizer.h"
#include "common/config.h"
#include "common/types.h"
#include "communication/event_publisher.h"

struct _NvDsObjEncCtx;
struct _NvDsObjectMeta;

namespace vehicle {
namespace probes {

struct SnapshotImages {
  JpegImage full;
  JpegImage crop;
  // Điểm chất lượng của `crop` đang giữ — nguồn sự thật để quyết định có chụp
  // lại hay không (thay cho sổ ghi cũ ở TrackPlateState).
  business::plate::CropScore crop_score;
  // Hạn chờ crop đã submit (monotonic giây, 0 = không chờ). nvds_obj_enc bất
  // đồng bộ: JPEG chỉ về ở image probe. Chặn retainSingleSnapshot xoá mất chỗ
  // nhận ảnh, nhưng có hạn — encode fail thì phải được chụp lại.
  double crop_pending_until_s = 0.0;
  // Bbox xe (px) gắn với full — vẽ xanh lên JPEG lúc emit.
  float left = 0.0f;
  float top = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
};

// Ảnh theo track → chuỗi biển (mẫu tốt nhất từng reading).
struct TrackSnapshots {
  std::map<std::string, SnapshotImages> by_plate;
};

struct PendingEncode {
  uint64_t track_id = 0;
  std::string plate_key;
  float left = 0.0f;
  float top = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  bool encode_plate_crop = false;
};

class PlateProbe {
 public:
  PlateProbe(const Config& config, comm::EventPublisher* publisher);
  ~PlateProbe();

  void setCameras(const std::vector<Camera>& cameras);
  void bindCamera(unsigned int source_id, const Camera& camera);
  void unbindCamera(unsigned int source_id);
  void updateCamera(unsigned int source_id, const Camera& camera);
  void updateZones(const ZoneSet& zones);

  void attachBboxProbe(GstPad* pad);
  void attachRoiExpandProbe(GstPad* pad);
  void attachRoiRestoreProbe(GstPad* pad);
  void attachMetaProbe(GstPad* pad);
  void attachImageProbe(GstPad* pad);

  void start();
  void stop();

 private:
  struct SourceState {
    Camera camera;
    std::unique_ptr<business::plate::PlateRecognizer> manager;
    std::map<uint64_t, TrackSnapshots> snapshots;
  };

  struct EmitJob {
    Camera camera;
    PlateEmit emit;
    business::plate::PlateRecognizer* manager = nullptr;
    // Nghiệp vụ mà job này đang bắn — worker báo lại kết quả theo đúng tập đó.
    business::plate::EventKindMask want = 0;
  };

  static GstPadProbeReturn bboxProbeCb(GstPad* pad, GstPadProbeInfo* info, gpointer data);
  static GstPadProbeReturn roiExpandProbeCb(GstPad* pad, GstPadProbeInfo* info, gpointer data);
  static GstPadProbeReturn roiRestoreProbeCb(GstPad* pad, GstPadProbeInfo* info, gpointer data);
  static GstPadProbeReturn metaProbeCb(GstPad* pad, GstPadProbeInfo* info, gpointer data);
  static GstPadProbeReturn imageProbeCb(GstPad* pad, GstPadProbeInfo* info, gpointer data);

  GstPadProbeReturn handleBbox(GstPadProbeInfo* info);
  GstPadProbeReturn handleRoiExpand(GstPadProbeInfo* info);
  GstPadProbeReturn handleRoiRestore(GstPadProbeInfo* info);
  GstPadProbeReturn handleMeta(GstPadProbeInfo* info);
  GstPadProbeReturn handleImages(GstPadProbeInfo* info);

  struct LanePolygon {
    std::vector<Point> polygon;
    std::set<int> allowed_classes;  // loại xe được phép ở làn này
    std::string zone_name;
  };

  // Kết quả tra polygon PLATE: phân biệt "chưa có ROI" với "có ROI nhưng rỗng".
  struct PlateZones {
    std::vector<std::vector<Point>> polygons;
    bool configured = false;
  };

  // Tạo recognizer đã nạp sẵn timing rendezvous WRONG_WAY.
  std::unique_ptr<business::plate::PlateRecognizer> makeRecognizer() const;
  SourceState* sourceState(unsigned int source_id);
  PlateZones plateZonesFor(const std::string& camera_code, double frame_w, double frame_h,
                           double source_w, double source_h);
  std::vector<LanePolygon> lanePolygonsFor(const std::string& camera_code,
                                           double frame_w, double frame_h,
                                           double source_w, double source_h);
  // Line REVERSE_DIRECTION đã scale sang pixel; bỏ line thiếu direction.
  std::vector<business::plate::WrongWayLine> wrongWayLinesFor(
      const std::string& camera_code, double frame_w, double frame_h, double source_w,
      double source_h);
  // Cảnh báo 1 lần/version khi ZoneSet không chứa zone PLATE nào.
  void warnMissingPlateZone(const std::string& camera_code, const ZoneSet& set);
  void warnLineWithoutDirection(const std::string& camera_code, const ZoneSet& set,
                                const std::string& line_name);
  // Đẩy yêu cầu chụp ảnh bằng chứng cho 1 nghiệp vụ (WRONG_LANE / WRONG_WAY)
  // tại đúng frame vi phạm. No-op nếu track đã có ảnh của nghiệp vụ đó.
  // Nhận bbox rời thay vì NvOSD_RectParams để header không phải kéo theo
  // nvdsmeta.h (chỉ hai struct DeepStream được forward declare ở trên).
  void submitViolationSnapshot(SourceState* state, uint64_t track_id,
                               business::plate::EventKind kind, const char* snapshot_key,
                               float left, float top, float width, float height,
                               unsigned int source_id, uint64_t frame_num);
  void enqueue(EmitJob job);
  void workerLoop();
  void clearPendingFor(unsigned int source_id);
  void maybeLogMemStats(double now_s);

  const Config& config_;
  comm::EventPublisher* publisher_;

  std::mutex sources_mutex_;
  std::map<unsigned int, std::unique_ptr<SourceState>> sources_;

  std::mutex zones_mutex_;
  std::map<std::string, ZoneSet> zones_;
  // camera_code → version đã cảnh báo thiếu zone PLATE (bảo vệ bởi zones_mutex_).
  std::map<std::string, uint64_t> warned_zone_version_;
  // camera_code → version đã cảnh báo line thiếu direction (bảo vệ bởi zones_mutex_).
  std::map<std::string, uint64_t> warned_line_version_;

  std::mutex pending_mutex_;
  std::map<std::pair<unsigned int, uint64_t>, std::vector<PendingEncode>> pending_frames_;

  _NvDsObjEncCtx* enc_ctx_ = nullptr;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<EmitJob> queue_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  double last_cleanup_s_ = 0.0;
  double last_mem_stats_s_ = 0.0;
  // Buffer tái dùng cho collectReady — chỉ handleImages đụng tới (một thread),
  // tránh cấp phát vector mới mỗi buffer.
  std::vector<business::plate::ReadyEmit> ready_buf_;
};

}  // namespace probes
}  // namespace vehicle
