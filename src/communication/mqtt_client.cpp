#include "communication/mqtt_client.h"

#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "common/logging.h"
#include "nvds_msgapi.h"

namespace vehicle {
namespace comm {
namespace {

constexpr const char* kDefaultProtoLib =
    "/opt/nvidia/deepstream/deepstream/lib/libnvds_mqtt_proto.so";

constexpr unsigned long kMaxReconnectAttempts = 20000;

// Con trỏ hàm của adapter (nạp bằng dlsym như sample DeepStream).
NvDsMsgApiHandle (*g_connect)(char*, nvds_msgapi_connect_cb_t, char*) = nullptr;
NvDsMsgApiErrorType (*g_send_async)(NvDsMsgApiHandle, char*, const uint8_t*, size_t,
                                    nvds_msgapi_send_cb_t, void*) = nullptr;
NvDsMsgApiErrorType (*g_subscribe)(NvDsMsgApiHandle, char**, int,
                                   nvds_msgapi_subscribe_request_cb_t, void*) = nullptr;
NvDsMsgApiErrorType (*g_disconnect)(NvDsMsgApiHandle) = nullptr;
void (*g_do_work)(NvDsMsgApiHandle) = nullptr;

// nvds_msgapi connect callback không có user_data.
MqttClient* g_mqtt_self = nullptr;

void requestProcessRestart() { ::kill(::getpid(), SIGTERM); }

void connectCallback(NvDsMsgApiHandle handle, NvDsMsgApiEventType event) {
  if (g_mqtt_self != nullptr) g_mqtt_self->onBrokerEvent(handle, static_cast<int>(event));
}

void sendCallback(void* /*user_ptr*/, NvDsMsgApiErrorType flag) {
  if (flag != NVDS_MSGAPI_OK) LOG_WARN("mqtt: publish thất bại (flag=%d)", static_cast<int>(flag));
}

void subscribeCallback(NvDsMsgApiErrorType flag, void* msg, int msg_len, char* topic,
                       void* user_ptr) {
  if (flag != NVDS_MSGAPI_OK) {
    LOG_WARN("mqtt: nhận message lỗi trên topic %s", topic ? topic : "?");
    return;
  }
  auto* client = static_cast<MqttClient*>(user_ptr);
  if (client == nullptr || msg == nullptr || msg_len <= 0) return;
  client->dispatchMessage(topic ? std::string(topic) : std::string(),
                          std::string(static_cast<const char*>(msg),
                                      static_cast<size_t>(msg_len)));
}

}  // namespace

MqttClient::~MqttClient() { disconnect(); }

bool MqttClient::loadAdapter() {
  if (so_handle_ != nullptr) return true;
  const char* env_path = std::getenv("VEHICLE_MQTT_PROTO_LIB");
  const std::string path = env_path != nullptr ? env_path : kDefaultProtoLib;
  so_handle_ = dlopen(path.c_str(), RTLD_LAZY);
  if (so_handle_ == nullptr) {
    LOG_ERROR("mqtt: không nạp được %s (%s)", path.c_str(), dlerror());
    return false;
  }
  *reinterpret_cast<void**>(&g_connect) = dlsym(so_handle_, "nvds_msgapi_connect");
  *reinterpret_cast<void**>(&g_send_async) = dlsym(so_handle_, "nvds_msgapi_send_async");
  *reinterpret_cast<void**>(&g_subscribe) = dlsym(so_handle_, "nvds_msgapi_subscribe");
  *reinterpret_cast<void**>(&g_disconnect) = dlsym(so_handle_, "nvds_msgapi_disconnect");
  *reinterpret_cast<void**>(&g_do_work) = dlsym(so_handle_, "nvds_msgapi_do_work");
  if (g_connect == nullptr || g_send_async == nullptr || g_subscribe == nullptr ||
      g_disconnect == nullptr || g_do_work == nullptr) {
    LOG_ERROR("mqtt: thiếu symbol nvds_msgapi_* trong %s", path.c_str());
    dlclose(so_handle_);
    so_handle_ = nullptr;
    return false;
  }
  return true;
}

std::string MqttClient::writeBrokerConfig(const MqttConfig& config) {
  char path[64];
  std::snprintf(path, sizeof(path), "/tmp/vehicle-mqtt-%d.cfg", static_cast<int>(::getpid()));
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    LOG_WARN("mqtt: không ghi được cfg tạm %s — dùng mặc định adapter", path);
    return std::string();
  }
  out << "[message-broker]\n"
      << "username = " << config.username << "\n"
      << "password = " << config.password << "\n"
      << "client-id = vehicle-" << config.ai_modules << "-" << ::getpid() << "\n"
      << "keep-alive = " << config.keep_alive << "\n"
      << "loop-timeout = 200\n"
      << "set-threaded = 1\n";
  out.close();
  ::chmod(path, 0600);
  return std::string(path);
}

bool MqttClient::openSession() {
  if (handle_ != nullptr) return true;
  if (config_path_.empty()) config_path_ = writeBrokerConfig(config_);
  std::string connection = config_.host + ";" + std::to_string(config_.port);

  live_handle_.store(nullptr);
  handle_ = g_connect(const_cast<char*>(connection.c_str()), connectCallback,
                      config_path_.empty() ? nullptr : const_cast<char*>(config_path_.c_str()));
  if (handle_ == nullptr) {
    const unsigned long n = retry_count_.fetch_add(1) + 1;
    // Outage dài (giờ/ngày) → log thưa dần: 10 lần đầu, rồi mỗi 60 lần (~5 phút @5s).
    if (n <= 10 || n % 60 == 0)
      LOG_ERROR("mqtt: connect %s thất bại (lần %lu)", connection.c_str(), n);
    return false;
  }
  live_handle_.store(handle_);
  retry_count_.store(0);
  need_reconnect_.store(false);
  LOG_INFO("mqtt: connect %s (user=%s)", connection.c_str(), config_.username.c_str());
  return true;
}

void MqttClient::setConnected(bool ok) {
  const bool prev = connected_.exchange(ok);
  if (prev == ok) return;
  if (ok)
    LOG_INFO("mqtt: kết nối broker thành công");
  else
    LOG_ERROR("mqtt: mất kết nối broker");
  notifyConnection(ok);
}

void MqttClient::notifyConnection(bool ok) {
  ConnectionCallback cb;
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    cb = connection_cb_;
  }
  if (cb) cb(ok);
}

void MqttClient::onBrokerEvent(void* handle, int event) {
  if (shutting_down_.load()) return;

  void* live = live_handle_.load();
  if (live == nullptr || (handle != nullptr && handle != live)) return;

  if (event == static_cast<int>(NVDS_MSGAPI_EVT_SUCCESS)) {
    connected_.store(true);
    return;
  }
  if (connected_.load()) LOG_ERROR("mqtt: broker event=%d — sẽ reconnect", event);
  setConnected(false);
  need_reconnect_.store(true);
  cv_.notify_all();
}

bool MqttClient::connect(const MqttConfig& config) {
  if (connected_.load() && handle_ != nullptr) return true;
  if (!loadAdapter()) return false;

  config_ = config;
  g_mqtt_self = this;
  shutting_down_.store(false);
  need_reconnect_.store(false);

  if (!openSession()) {
    if (!config_path_.empty()) {
      ::unlink(config_path_.c_str());
      config_path_.clear();
    }
    return false;
  }
  setConnected(true);

  running_ = true;
  if (!worker_.joinable()) worker_ = std::thread(&MqttClient::workerLoop, this);
  return true;
}

void MqttClient::workerLoop() {
  while (running_.load()) {
    void* handle = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handle = handle_;
    }

    if (handle != nullptr && connected_.load() && !need_reconnect_.load()) {
      g_do_work(handle);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    if (shutting_down_.load() || !running_.load()) break;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      handle_ = nullptr;
      live_handle_.store(nullptr);
    }
    if (connected_.load()) setConnected(false);

    const int interval_s = std::max(1, config_.reconnect_interval_s);
    const unsigned long n = retry_count_.load();
    if (n <= 10 || n % 60 == 0) LOG_WARN("mqtt: reconnect sau %ds...", interval_s);
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait_for(lock, std::chrono::seconds(interval_s),
                   [this] { return !running_.load() || shutting_down_.load(); });
    }
    if (!running_.load() || shutting_down_.load()) break;

    const unsigned long failures = retry_count_.load() + 1;  // openSession() sẽ reset về 0
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ok = openSession();
      if (ok) {
        need_reconnect_.store(false);
        resubscribe();
      }
    }
    if (ok) {
      LOG_INFO("mqtt: reconnect thành công sau %lu lần thử", failures);
      setConnected(true);  // sau re-subscribe, ngoài mutex_
      continue;
    }

    if (retry_count_.load() >= kMaxReconnectAttempts) {
      LOG_ERROR(
          "mqtt: reconnect thất bại %lu lần liên tiếp — thoát để restart tiến trình sạch",
          retry_count_.load());
      requestProcessRestart();
      break;
    }
  }
}

void MqttClient::disconnect() {
  shutting_down_.store(true);
  running_.store(false);
  need_reconnect_.store(false);
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    live_handle_.store(nullptr);
    if (handle_ != nullptr && g_disconnect != nullptr) {
      g_disconnect(handle_);
      handle_ = nullptr;
      LOG_INFO("mqtt: đã ngắt kết nối");
    }
  }
  connected_.store(false);

  if (!config_path_.empty()) {
    ::unlink(config_path_.c_str());
    config_path_.clear();
  }
  if (g_mqtt_self == this) g_mqtt_self = nullptr;
  if (so_handle_ != nullptr) {
    dlclose(so_handle_);
    so_handle_ = nullptr;
  }
}

void MqttClient::setMessageCallback(MessageCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  message_cb_ = std::move(cb);
}

void MqttClient::setConnectionCallback(ConnectionCallback cb) {
  std::lock_guard<std::mutex> lock(cb_mutex_);
  connection_cb_ = std::move(cb);
}

void MqttClient::dispatchMessage(const std::string& topic, const std::string& payload) {
  MessageCallback cb;
  {
    std::lock_guard<std::mutex> lock(cb_mutex_);
    cb = message_cb_;
  }
  if (cb) cb(topic, payload);
}

bool MqttClient::subscribeLocked(const std::vector<std::string>& topics) {
  if (handle_ == nullptr || topics.empty()) return false;
  std::vector<char*> raw;
  raw.reserve(topics.size());
  for (const std::string& topic : topics) raw.push_back(const_cast<char*>(topic.c_str()));
  const NvDsMsgApiErrorType rc =
      g_subscribe(handle_, raw.data(), static_cast<int>(raw.size()), subscribeCallback, this);
  if (rc != NVDS_MSGAPI_OK) {
    LOG_ERROR("mqtt: subscribe thất bại (rc=%d)", static_cast<int>(rc));
    return false;
  }
  return true;
}

void MqttClient::resubscribe() {
  if (subscribed_topics_.empty()) return;
  LOG_INFO("mqtt: re-subscribe %zu topic sau reconnect", subscribed_topics_.size());
  subscribeLocked(subscribed_topics_);
}

bool MqttClient::subscribe(const std::vector<std::string>& topics) {
  if (topics.empty()) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const std::string& topic : topics) {
    bool exists = false;
    for (const std::string& t : subscribed_topics_) {
      if (t == topic) {
        exists = true;
        break;
      }
    }
    if (!exists) subscribed_topics_.push_back(topic);
  }
  if (handle_ == nullptr || !connected_.load()) return true;  // lưu topic, subscribe khi reconnect
  return subscribeLocked(topics);
}

bool MqttClient::publish(const std::string& topic, const std::string& payload) {
  if (!connected_.load() || handle_ == nullptr) return false;
  void* handle = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handle = handle_;
  }
  if (handle == nullptr) return false;
  const NvDsMsgApiErrorType rc =
      g_send_async(handle, const_cast<char*>(topic.c_str()),
                   reinterpret_cast<const uint8_t*>(payload.data()), payload.size(),
                   sendCallback, nullptr);
  if (rc != NVDS_MSGAPI_OK) {
    LOG_WARN("mqtt: publish %s thất bại (rc=%d)", topic.c_str(), static_cast<int>(rc));
    return false;
  }
  LOG_DEBUG("mqtt: publish %s (%zu bytes)", topic.c_str(), payload.size());
  return true;
}

}  // namespace comm
}  // namespace vehicle
