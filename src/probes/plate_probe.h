// Pad probe: đọc NvDsBatchMeta → lọc ROI → OCR → business → communication.
#pragma once

#include <gst/gst.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
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
    bool warned_no_zone = false;
  };

  struct EmitJob {
    Camera camera;
    PlateEmit emit;
    business::plate::PlateRecognizer* manager = nullptr;
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

  SourceState* sourceState(unsigned int source_id);
  std::vector<std::vector<Point>> polygonsFor(const std::string& camera_code,
                                              double frame_w, double frame_h,
                                              double source_w, double source_h);
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
};

}  // namespace probes
}  // namespace vehicle
