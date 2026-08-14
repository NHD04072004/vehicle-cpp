// Watcher lấy bản mới nhất từ MQTT (camera list / polygon) rồi áp dụng an toàn.
#pragma once

#include <glib.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "common/config.h"
#include "common/types.h"

namespace vehicle {
namespace probes {
class PlateProbe;
}  // namespace probes

namespace pipeline {

class Pipeline;

// Vòng lặp nền chung: MQTT setDesired → dirty → thread onWake().
class MqttWatcherBase {
 public:
  MqttWatcherBase() = default;
  virtual ~MqttWatcherBase();

  MqttWatcherBase(const MqttWatcherBase&) = delete;
  MqttWatcherBase& operator=(const MqttWatcherBase&) = delete;

  void start(const char* name);
  void stop();

 protected:
  void wake();
  // Chờ interval hoặc wake/stop. false → đã stop.
  bool sleepFor(int interval_ms);
  bool stopping() const;

  virtual void onWake() = 0;

 private:
  void threadMain();

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = true;
  bool dirty_ = false;
  std::string name_;
  std::thread thread_;
};

// Camera list MQTT → probe RTSP → add/remove source trên GLib main loop.
class CameraWatcher : public MqttWatcherBase {
 public:
  CameraWatcher(const Config& config, Pipeline* pipeline);
  ~CameraWatcher() override;

  void setDesired(std::vector<Camera> cameras);

 protected:
  void onWake() override;

 private:
  struct ApplyIdle;

  std::vector<Camera> snapshotDesired() const;
  std::vector<Camera> filterReady(const std::vector<Camera>& cameras);
  void scheduleApply(std::vector<Camera> ready);
  static gboolean onApplyIdle(gpointer data);

  const Config& config_;
  Pipeline* pipeline_;

  mutable std::mutex state_mutex_;
  std::vector<Camera> desired_;
  std::vector<Camera> last_applied_;
  std::set<std::string> logged_not_ready_;
  std::atomic<bool> apply_pending_{false};
};

// Zones MQTT → giữ latest theo camera_code → đẩy vào PlateProbe trên main loop.
class PolygonWatcher : public MqttWatcherBase {
 public:
  explicit PolygonWatcher(probes::PlateProbe* probe);
  ~PolygonWatcher() override;

  // Một ZoneSet từ topic get_polygon (thread MQTT).
  void setDesired(ZoneSet zones);

 protected:
  void onWake() override;

 private:
  struct ApplyIdle;

  void scheduleApply(std::vector<ZoneSet> pending);
  static gboolean onApplyIdle(gpointer data);

  probes::PlateProbe* probe_;

  mutable std::mutex state_mutex_;
  std::map<std::string, ZoneSet> latest_;   // camera_code → ROI mới nhất
  std::set<std::string> dirty_codes_;       // chờ apply
  std::map<std::string, uint64_t> applied_version_;
  std::atomic<bool> apply_pending_{false};
};

}  // namespace pipeline
}  // namespace vehicle
