#include "communication/http_uploader.h"

#include <curl/curl.h>
#include <json/json.h>

#include <memory>

#include "common/logging.h"
#include "utils/latency.h"
#include "utils/time_utils.h"

namespace vehicle {
namespace comm {
namespace {

size_t writeBody(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* out = static_cast<std::string*>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

// API trả {"object_key": "..."}; chấp nhận vài alias thường gặp.
std::string extractObjectKey(const std::string& body) {
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errors;
  const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(body.data(), body.data() + body.size(), &root, &errors)) return "";
  for (const char* key : {"object_key", "objectKey", "key", "url", "path"}) {
    if (root.isObject() && root.isMember(key) && root[key].isString())
      return root[key].asString();
  }
  if (root.isObject() && root.isMember("data")) {
    const Json::Value& data = root["data"];
    for (const char* key : {"object_key", "objectKey", "key", "url"}) {
      if (data.isObject() && data.isMember(key) && data[key].isString())
        return data[key].asString();
    }
  }
  return "";
}

}  // namespace

HttpUploader::HttpUploader(const Config& config) : url_(config.uploadUrl()) {
  curl_global_init(CURL_GLOBAL_DEFAULT);
  if (config.snapshotApi().base_url.empty())
    LOG_WARN("restful.yaml: snapshot_api.base_url rỗng — upload sẽ bị bỏ qua");
}

HttpUploader::~HttpUploader() { curl_global_cleanup(); }

std::string HttpUploader::upload(const std::string& camera_id, const std::string& category,
                                 const std::vector<uint8_t>& jpeg,
                                 const std::string& filename) {
  if (url_.empty() || jpeg.empty()) return "";

  const double start_s = utils::monotonicSeconds();
  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    LOG_ERROR("upload: curl_easy_init thất bại");
    return "";
  }

  curl_mime* mime = curl_mime_init(curl);
  curl_mimepart* part = curl_mime_addpart(mime);
  curl_mime_name(part, "file");
  curl_mime_filename(part, filename.c_str());
  curl_mime_type(part, "image/jpeg");
  curl_mime_data(part, reinterpret_cast<const char*>(jpeg.data()), jpeg.size());

  part = curl_mime_addpart(mime);
  curl_mime_name(part, "camera_id");
  curl_mime_data(part, camera_id.c_str(), CURL_ZERO_TERMINATED);

  part = curl_mime_addpart(mime);
  curl_mime_name(part, "category");
  curl_mime_data(part, category.c_str(), CURL_ZERO_TERMINATED);

  std::string body;
  curl_easy_setopt(curl, CURLOPT_URL, url_.c_str());
  curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeBody);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms_);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

  const CURLcode rc = curl_easy_perform(curl);
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  curl_mime_free(mime);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK) {
    LOG_WARN("upload %s: %s", category.c_str(), curl_easy_strerror(rc));
    utils::latencyLog("upload %s FAIL curl=%.0fms size=%zu", category.c_str(),
                      utils::msSince(start_s), jpeg.size());
    return "";
  }
  if (status < 200 || status >= 300) {
    LOG_WARN("upload %s: HTTP %ld — %s", category.c_str(), status, body.c_str());
    utils::latencyLog("upload %s FAIL http=%ld %.0fms size=%zu", category.c_str(), status,
                      utils::msSince(start_s), jpeg.size());
    return "";
  }

  const std::string object_key = extractObjectKey(body);
  if (object_key.empty())
    LOG_WARN("upload %s: response thiếu object_key — %s", category.c_str(), body.c_str());
  else
    LOG_DEBUG("upload %s → %s", category.c_str(), object_key.c_str());
  utils::latencyLog("upload %s ok: %.0fms size=%zuB key=%s", category.c_str(),
                    utils::msSince(start_s), jpeg.size(),
                    object_key.empty() ? "(none)" : object_key.c_str());
  return object_key;
}

}  // namespace comm
}  // namespace vehicle
