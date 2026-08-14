// DTO dùng chung giữa communication / pipeline / probes / business.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vehicle {

enum VehicleClass : int {
  kClassBus = 0,
  kClassCar = 1,
  kClassMotorbike = 2,
  kClassTruck = 3,
};

struct Point {
  double x = 0.0;
  double y = 0.0;
};

// Bbox pixel theo frame gốc: [x1, y1, x2, y2].
struct BoundingBox {
  double x1 = 0.0;
  double y1 = 0.0;
  double x2 = 0.0;
  double y2 = 0.0;

  double width() const { return x2 - x1; }
  double height() const { return y2 - y1; }
  bool valid() const { return x2 > x1 && y2 > y1; }
};

// Vùng ROI nhận từ MQTT `get_polygon` (chỉ zones, bỏ lines).
struct Zone {
  std::string name;
  std::vector<Point> points;
  // true → points là 0..1 (probe scale sang pixel); false → đã là pixel.
  bool normalized = true;
};

struct ZoneSet {
  std::string camera_code;
  std::vector<Zone> zones;
  uint64_t version = 0;  // tăng mỗi lần VMS cập nhật → invalidate cache.
};

struct Camera {
  std::string id;    // uuid — dùng cho event/upload
  std::string code;  // camera_code — dùng cho topic bbox/zones
  std::string name;
  std::vector<std::string> ai_modules;
  std::string restream_url;  // restream_urls.<AI_MODULE>
};

// Một ký tự OCR do SGIE2 đọc được.
struct CharReading {
  std::string text;
  double confidence = 0.0;
};

// Một lần đọc biển hoàn chỉnh của 1 track trong 1 frame.
struct PlateReading {
  std::vector<CharReading> chars;
  std::string raw;  // chuỗi ghép trái→phải (trên→dưới nếu 2 dòng)
};

// Detection publish realtime qua `pub_bbox`.
struct Detection {
  std::string id;     // "vehicle_<track_id>"
  std::string cls;    // "car" | "motorbike" | "truck"
  double confidence = 0.0;
  BoundingBox bbox_norm;  // đã chuẩn hoá 0..1
  std::string label;      // "car 30A12345"
  std::string color;      // "#00FF00"
};

// Ảnh JPEG đã encode (full-frame hoặc crop biển).
struct JpegImage {
  std::vector<uint8_t> data;
  bool empty() const { return data.empty(); }
};

// Kết quả chốt biển của 1 track, sẵn sàng upload + pub_event.
struct PlateEmit {
  uint64_t track_id = 0;
  std::string plate;     // đã normalize (UNKOWN / *_unk / biển hợp lệ)
  int vehicle_cls = -1;  // -1 nếu chưa vote được
  JpegImage full_frame;
  JpegImage plate_crop;
  // Mốc monotonic (giây) — latency debug.
  double created_at_s = 0.0;
  double first_ocr_at_s = 0.0;
  double final_at_s = 0.0;
  double images_ready_at_s = 0.0;
  double enqueue_at_s = 0.0;
  int recognize_count = 0;
};

const char* vehicleClassName(int cls);

}  // namespace vehicle
