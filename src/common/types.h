// DTO dùng chung giữa communication / pipeline / probes / business.
#pragma once

#include <cstdint>
#include <set>
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

// Vùng ROI nhận từ MQTT `get_polygon`, mảng "zones".
struct Zone {
  std::string name;
  std::vector<std::string> ai_modules;
  std::vector<Point> points;
  // true → points là 0..1 (probe scale sang pixel); false → đã là pixel.
  bool normalized = true;

  bool hasAiModule(const std::string& module) const;
};

// Vạch nhận từ MQTT `get_polygon`, mảng "lines" (tách khỏi "zones"): 2 điểm +
// vector hướng. Xe cắt vạch khi đi CÙNG chiều `direction` → vi phạm.
struct Line {
  std::string name;
  std::vector<std::string> ai_modules;
  std::vector<Point> points;  // >= 2 điểm; dùng đoạn [0]→[1]
  bool normalized = true;
  // config.has_direction=true → direction hợp lệ (chiều CẤM — mũi tên vẽ
  // trên line; xe đi CÙNG chiều mũi tên mà cắt vạch là vi phạm).
  bool has_direction = false;
  Point direction;  // vector đơn vị, cùng hệ toạ độ với points

  bool hasAiModule(const std::string& module) const;
};

struct ZoneSet {
  std::string camera_code;
  std::vector<Zone> zones;
  std::vector<Line> lines;
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
  // Vi phạm không đội mũ (SGIE3, chỉ xe máy).
  int no_helmet_frames = 0;  // số frame detect được người không đội mũ
  int no_helmet_count = 0;   // số người không đội mũ nhiều nhất trong 1 frame
  // Vi phạm đi sai làn (zone tên *_LANE, mọi loại xe).
  int wrong_lane_frames = 0;   // số frame detect đi sai làn
  std::string wrong_lane_zone; // tên zone LANE đầu tiên gây vi phạm (evidence)
  JpegImage wrong_lane_full;
  JpegImage wrong_lane_crop;
  // Vi phạm đi ngược chiều (line tên REVERSE_DIRECTION, mọi loại xe).
  int wrong_way_hits = 0;        // số lần cắt vạch ngược chiều
  std::string wrong_way_line;    // tên line đầu tiên gây vi phạm (evidence)
  JpegImage wrong_way_full;
  JpegImage wrong_way_crop;
  // Mốc monotonic (giây) — latency debug.
  double created_at_s = 0.0;
  double first_ocr_at_s = 0.0;
  double final_at_s = 0.0;
  double images_ready_at_s = 0.0;
  double enqueue_at_s = 0.0;
  int recognize_count = 0;
};

const char* vehicleClassName(int cls);

std::set<int> laneAllowedClasses(const std::string& zone_name);

// Tên line VMS dùng cho vi phạm đi ngược chiều.
constexpr const char* kReverseDirectionLine = "REVERSE_DIRECTION";

bool isReverseDirectionLine(const std::string& line_name);

}  // namespace vehicle
