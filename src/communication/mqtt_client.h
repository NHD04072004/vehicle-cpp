// MQTT client dựa trên nvds_msgapi + libnvds_mqtt_proto.so của DeepStream.
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/config.h"

namespace vehicle {
namespace comm {

class MqttClient {
 public:
  using MessageCallback =
      std::function<void(const std::string& topic, const std::string& payload)>;
  using ConnectionCallback = std::function<void(bool connected)>;

  MqttClient() = default;
  ~MqttClient();
  MqttClient(const MqttClient&) = delete;
  MqttClient& operator=(const MqttClient&) = delete;

  // Nạp adapter (env VEHICLE_MQTT_PROTO_LIB để override đường dẫn .so) rồi connect.
  bool connect(const MqttConfig& config);
  void disconnect();
  bool connected() const { return connected_.load(); }

  void setMessageCallback(MessageCallback cb);
  void setConnectionCallback(ConnectionCallback cb);
  bool subscribe(const std::vector<std::string>& topics);
  // MQTT adapter chỉ hỗ trợ gửi bất đồng bộ.
  bool publish(const std::string& topic, const std::string& payload);

  // Dùng nội bộ bởi callback C của adapter.
  void dispatchMessage(const std::string& topic, const std::string& payload);
  void onBrokerEvent(int event);

 private:
  bool loadAdapter();
  std::string writeBrokerConfig(const MqttConfig& config);
  bool openSession();  // tạo handle; caller setConnected/resubscribe
  bool subscribeLocked(const std::vector<std::string>& topics);
  void resubscribe();
  void workerLoop();
  void setConnected(bool ok);
  void notifyConnection(bool ok);

  void* so_handle_ = nullptr;
  void* handle_ = nullptr;  // NvDsMsgApiHandle
  std::string config_path_;
  MqttConfig config_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> need_reconnect_{false};

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> subscribed_topics_;

  std::mutex cb_mutex_;
  MessageCallback message_cb_;
  ConnectionCallback connection_cb_;
};

}  // namespace comm
}  // namespace vehicle
