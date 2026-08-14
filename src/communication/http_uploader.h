// Upload snapshot lên API Smart VMS (multipart) → object_key.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "common/config.h"

namespace vehicle {
namespace comm {

class HttpUploader {
 public:
  explicit HttpUploader(const Config& config);
  ~HttpUploader();
  HttpUploader(const HttpUploader&) = delete;
  HttpUploader& operator=(const HttpUploader&) = delete;

  // POST {base_url}{endpoint} — multipart: file + camera_id + category.
  // Trả về object_key; rỗng nếu lỗi.
  std::string upload(const std::string& camera_id, const std::string& category,
                     const std::vector<uint8_t>& jpeg, const std::string& filename);

 private:
  std::string url_;
  long timeout_ms_ = 5000;
};

}  // namespace comm
}  // namespace vehicle
