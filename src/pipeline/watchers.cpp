#include "pipeline/watchers.h"

#include <curl/curl.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <utility>

#include "common/logging.h"
#include "pipeline/pipeline.h"
#include "probes/plate_probe.h"

namespace vehicle {
namespace pipeline {
namespace {

bool sameCameraSet(const std::vector<Camera>& a, const std::vector<Camera>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].code != b[i].code || a[i].restream_url != b[i].restream_url ||
        a[i].id != b[i].id)
      return false;
  }
  return true;
}

void sortByCode(std::vector<Camera>* cameras) {
  std::sort(cameras->begin(), cameras->end(),
            [](const Camera& x, const Camera& y) { return x.code < y.code; });
}

size_t discardBody(char* /*ptr*/, size_t size, size_t nmemb, void* /*userdata*/) {
  return size * nmemb;
}

bool startsWithIgnoreCase(const std::string& s, const char* prefix) {
  const size_t n = std::char_traits<char>::length(prefix);
  if (s.size() < n) return false;
  for (size_t i = 0; i < n; ++i) {
    if (std::tolower(static_cast<unsigned char>(s[i])) !=
        std::tolower(static_cast<unsigned char>(prefix[i])))
      return false;
  }
  return true;
}

bool rtspReady(const std::string& uri, int timeout_ms) {
  if (uri.empty()) return false;
  if (!startsWithIgnoreCase(uri, "rtsp://") && !startsWithIgnoreCase(uri, "rtsps://"))
    return true;

  CURL* curl = curl_easy_init();
  if (curl == nullptr) return false;

  const long timeout = static_cast<long>(std::max(500, timeout_ms));
  curl_easy_setopt(curl, CURLOPT_URL, uri.c_str());
  curl_easy_setopt(curl, CURLOPT_RTSP_STREAM_URI, uri.c_str());
  curl_easy_setopt(curl, CURLOPT_RTSP_REQUEST, (long)CURL_RTSPREQ_DESCRIBE);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discardBody);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, timeout);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

  const CURLcode rc = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK) return false;
  return status >= 200 && status < 300;
}

}  // namespace

// ---------------------------------------------------------------------------
// MqttWatcherBase
// ---------------------------------------------------------------------------

MqttWatcherBase::~MqttWatcherBase() { stop(); }

void MqttWatcherBase::start(const char* name) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stop_) return;
    stop_ = false;
    dirty_ = true;
    name_ = name != nullptr ? name : "watcher";
  }
  thread_ = std::thread(&MqttWatcherBase::threadMain, this);
  LOG_INFO("%s: started", name_.c_str());
}

void MqttWatcherBase::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_) return;
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  if (!name_.empty()) LOG_INFO("%s: stopped", name_.c_str());
}

void MqttWatcherBase::wake() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dirty_ = true;
  }
  cv_.notify_all();
}

bool MqttWatcherBase::sleepFor(int interval_ms) {
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_for(lock, std::chrono::milliseconds(std::max(100, interval_ms)),
               [this] { return stop_ || dirty_; });
  return !stop_;
}

bool MqttWatcherBase::stopping() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_;
}

void MqttWatcherBase::threadMain() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_) break;
      dirty_ = false;
    }
    onWake();  // derived tự sleepFor(...) cuối chu kỳ
    if (stopping()) break;
  }
}

// ---------------------------------------------------------------------------
// CameraWatcher
// ---------------------------------------------------------------------------

struct CameraWatcher::ApplyIdle {
  Pipeline* pipeline = nullptr;
  std::vector<Camera> cameras;
  CameraWatcher* watcher = nullptr;
};

CameraWatcher::CameraWatcher(const Config& config, Pipeline* pipeline)
    : config_(config), pipeline_(pipeline) {}

CameraWatcher::~CameraWatcher() { stop(); }

void CameraWatcher::setDesired(std::vector<Camera> cameras) {
  std::vector<Camera> cleaned;
  cleaned.reserve(cameras.size());
  for (Camera& camera : cameras) {
    if (camera.code.empty() || camera.restream_url.empty()) {
      LOG_WARN("camera_watcher: bỏ %s — thiếu code/restream_url", camera.code.c_str());
      continue;
    }
    cleaned.push_back(std::move(camera));
  }
  sortByCode(&cleaned);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (sameCameraSet(desired_, cleaned)) return;
    desired_ = std::move(cleaned);
  }
  wake();
}

std::vector<Camera> CameraWatcher::snapshotDesired() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return desired_;
}

std::vector<Camera> CameraWatcher::filterReady(const std::vector<Camera>& cameras) {
  const RtspConfig& rtsp = config_.pipeline().rtsp;
  if (!rtsp.require_ready) return cameras;

  std::set<std::string> applied_codes;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const Camera& camera : last_applied_) applied_codes.insert(camera.code);
  }

  std::vector<Camera> ready;
  ready.reserve(cameras.size());
  std::set<std::string> desired_codes;
  for (const Camera& camera : cameras) {
    desired_codes.insert(camera.code);
    if (applied_codes.count(camera.code) > 0) {
      ready.push_back(camera);
      logged_not_ready_.erase(camera.code);
      continue;
    }
    if (rtspReady(camera.restream_url, rtsp.ready_timeout_ms)) {
      if (logged_not_ready_.erase(camera.code) > 0)
        LOG_INFO("camera_watcher: %s đã ready — sẽ gắn pipeline", camera.code.c_str());
      ready.push_back(camera);
    } else if (logged_not_ready_.insert(camera.code).second) {
      LOG_WARN("camera_watcher: %s chưa ready — chưa gắn pipeline (%s)", camera.code.c_str(),
               camera.restream_url.c_str());
    }
  }
  for (auto it = logged_not_ready_.begin(); it != logged_not_ready_.end();) {
    if (desired_codes.count(*it) == 0)
      it = logged_not_ready_.erase(it);
    else
      ++it;
  }
  return ready;
}

void CameraWatcher::scheduleApply(std::vector<Camera> ready) {
  bool expected = false;
  if (!apply_pending_.compare_exchange_strong(expected, true)) return;

  auto* ctx = new ApplyIdle;
  ctx->pipeline = pipeline_;
  ctx->cameras = std::move(ready);
  ctx->watcher = this;
  g_idle_add(&CameraWatcher::onApplyIdle, ctx);
}

gboolean CameraWatcher::onApplyIdle(gpointer data) {
  auto* ctx = static_cast<ApplyIdle*>(data);
  if (ctx->pipeline != nullptr) ctx->pipeline->applyCameraList(ctx->cameras);
  if (ctx->watcher != nullptr) {
    {
      std::lock_guard<std::mutex> lock(ctx->watcher->state_mutex_);
      ctx->watcher->last_applied_ = ctx->cameras;
    }
    ctx->watcher->apply_pending_.store(false);
  }
  delete ctx;
  return G_SOURCE_REMOVE;
}

void CameraWatcher::onWake() {
  std::vector<Camera> desired = snapshotDesired();
  std::vector<Camera> ready = filterReady(desired);
  sortByCode(&ready);

  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    changed = !sameCameraSet(ready, last_applied_);
  }
  if (changed && !apply_pending_.load()) {
    scheduleApply(std::move(ready));
  }

  // Chu kỳ probe RTSP (wake sớm nếu MQTT đổi list).
  const int interval_ms = std::max(500, config_.pipeline().rtsp.watch_interval_ms);
  sleepFor(interval_ms);
}

// ---------------------------------------------------------------------------
// PolygonWatcher
// ---------------------------------------------------------------------------

struct PolygonWatcher::ApplyIdle {
  probes::PlateProbe* probe = nullptr;
  std::vector<ZoneSet> zones;
  PolygonWatcher* watcher = nullptr;
};

PolygonWatcher::PolygonWatcher(probes::PlateProbe* probe) : probe_(probe) {}

PolygonWatcher::~PolygonWatcher() { stop(); }

void PolygonWatcher::setDesired(ZoneSet zones) {
  if (zones.camera_code.empty()) return;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = latest_.find(zones.camera_code);
    if (it != latest_.end() && it->second.version == zones.version &&
        it->second.zones.size() == zones.zones.size()) {
      // Cùng version — bỏ qua.
      return;
    }
    const std::string code = zones.camera_code;
    latest_[code] = std::move(zones);
    dirty_codes_.insert(code);
  }
  wake();
}

void PolygonWatcher::scheduleApply(std::vector<ZoneSet> pending) {
  bool expected = false;
  if (!apply_pending_.compare_exchange_strong(expected, true)) return;

  auto* ctx = new ApplyIdle;
  ctx->probe = probe_;
  ctx->zones = std::move(pending);
  ctx->watcher = this;
  g_idle_add(&PolygonWatcher::onApplyIdle, ctx);
}

gboolean PolygonWatcher::onApplyIdle(gpointer data) {
  auto* ctx = static_cast<ApplyIdle*>(data);
  if (ctx->probe != nullptr) {
    for (const ZoneSet& zones : ctx->zones) {
      ctx->probe->updateZones(zones);
      LOG_DEBUG("polygon_watcher: apply %s version=%lu (%zu zone)", zones.camera_code.c_str(),
                static_cast<unsigned long>(zones.version), zones.zones.size());
    }
  }
  if (ctx->watcher != nullptr) {
    {
      std::lock_guard<std::mutex> lock(ctx->watcher->state_mutex_);
      for (const ZoneSet& zones : ctx->zones)
        ctx->watcher->applied_version_[zones.camera_code] = zones.version;
    }
    ctx->watcher->apply_pending_.store(false);
  }
  delete ctx;
  return G_SOURCE_REMOVE;
}

void PolygonWatcher::onWake() {
  std::vector<ZoneSet> pending;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const std::string& code : dirty_codes_) {
      auto it = latest_.find(code);
      if (it == latest_.end()) continue;
      auto ap = applied_version_.find(code);
      if (ap != applied_version_.end() && ap->second == it->second.version) continue;
      pending.push_back(it->second);
    }
  }

  if (!pending.empty() && !apply_pending_.load()) {
    LOG_INFO("polygon_watcher: cập nhật %zu camera ROI", pending.size());
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (const ZoneSet& z : pending) dirty_codes_.erase(z.camera_code);
    }
    scheduleApply(std::move(pending));
  }

  // Chủ yếu chờ MQTT wake; interval dài để retry nếu idle còn pending.
  sleepFor(5000);
}

}  // namespace pipeline
}  // namespace vehicle
