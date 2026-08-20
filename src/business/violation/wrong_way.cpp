#include "business/violation/wrong_way.h"

#include <utility>

#include "utils/geometry.h"

namespace vehicle {
namespace business {
namespace violation {

void MotionHistory::push(const Point& p) {
  last_ = p;
  has_last_ = true;
  anchors_.push_back(p);
  if (anchors_.size() > kMotionHistoryLen) anchors_.erase(anchors_.begin());
}

bool MotionHistory::movedEnough(const Point& prev, const Point& now) {
  const double dx = now.x - prev.x;
  const double dy = now.y - prev.y;
  return dx * dx + dy * dy >= kMinMovePx * kMinMovePx;
}

Point MotionHistory::direction() const {
  if (anchors_.size() < kMinMotionHistoryLen) return Point{0.0, 0.0};
  if (isStationary()) return Point{0.0, 0.0};
  const Point v = utils::motionVector(anchors_);
  if (v.x * v.x + v.y * v.y < kMinMotionLenPx * kMinMotionLenPx) return Point{0.0, 0.0};
  return v;
}

bool MotionHistory::isStationary() const {
  const size_t n = anchors_.size();
  if (n < kMinMotionHistoryLen) return true;  // chưa đủ cơ sở kết luận đang chạy

  // Quãng đường tịnh: xe bò chậm vẫn trôi đều một hướng, xe đỗ thì quay về chỗ cũ.
  const Point& first = anchors_.front();
  const Point& last = anchors_.back();
  const double net_dx = last.x - first.x;
  const double net_dy = last.y - first.y;
  if (net_dx * net_dx + net_dy * net_dy > kStationaryNetPx * kStationaryNetPx) return false;

  // Tán xạ quanh tâm: mọi anchor nằm gọn trong bán kính = chỉ là nhiễu detection.
  double cx = 0.0, cy = 0.0;
  for (const Point& p : anchors_) {
    cx += p.x;
    cy += p.y;
  }
  cx /= static_cast<double>(n);
  cy /= static_cast<double>(n);
  for (const Point& p : anchors_) {
    const double ddx = p.x - cx;
    const double ddy = p.y - cy;
    if (ddx * ddx + ddy * ddy > kStationaryRadiusPx * kStationaryRadiusPx) return false;
  }
  return true;
}

bool isWrongWayLine(const Line& line) {
  return isReverseDirectionLine(line.name) && line.has_direction && line.points.size() >= 2;
}

std::vector<WrongWayLine> wrongWayLines(const std::vector<Line>& lines,
                                        const FrameScale& scale,
                                        std::string* out_missing_direction) {
  std::vector<WrongWayLine> out;
  for (const Line& line : lines) {
    if (!isReverseDirectionLine(line.name)) continue;
    // Không có chiều đi đúng → không kết luận được ngược chiều.
    if (!line.has_direction) {
      if (out_missing_direction != nullptr && out_missing_direction->empty())
        *out_missing_direction = line.name;
      continue;
    }
    if (line.points.size() < 2) continue;

    WrongWayLine scaled;
    scaled.name = line.name;
    scaled.a = scalePoint(line.points[0], line.normalized, scale);
    scaled.b = scalePoint(line.points[1], line.normalized, scale);
    // Giữ nguyên vector gốc: angleBetweenDeg tự chuẩn hoá độ dài nên chỉ cần
    // đúng HƯỚNG. Scale bất đẳng hướng (1920 vs 1080) sẽ làm méo góc — vector
    // 45 độ (1,1) sau khi nhân thành (1920,1080) chỉ còn 29.4 độ, lệch 15.6 độ
    // — đủ để phép so với max_angle_deg sai ở cả hai chiều.
    scaled.direction = line.direction;
    out.push_back(std::move(scaled));
  }
  return out;
}

CrossResult detectCrossing(const Point& prev, const Point& now, const MotionHistory& history,
                           const std::vector<WrongWayLine>& lines, double max_angle_deg) {
  CrossResult result;
  if (lines.empty()) return result;
  if (!MotionHistory::movedEnough(prev, now)) return result;

  // Xe đỗ ngay trên vạch: bbox vẫn nhấp nháy vài px mỗi frame nên đoạn
  // prev→now liên tục "cắt" line. Xét trạng thái đứng yên trên cả history
  // (tán xạ quanh 1 tâm) mới loại được, ngưỡng 1 frame là không đủ.
  const Point motion = history.direction();
  if (motion.x == 0.0 && motion.y == 0.0) return result;  // chưa đủ cơ sở về hướng

  for (const WrongWayLine& line : lines) {
    // Điều kiện 1: phải thực sự cắt qua line REVERSE_DIRECTION.
    if (!utils::segmentsIntersect(prev, now, line.a, line.b)) continue;

    // Điều kiện 2: hướng đi lệch <= ngưỡng so với direction_vector (chiều CẤM
    // — mũi tên vẽ trên line). Lệch > 90 độ là đi ngược mũi tên → hợp lệ.
    const double angle = utils::angleBetweenDeg(motion, line.direction);
    if (angle < 0.0 || angle > max_angle_deg) {
      if (result.rejected_line == nullptr) {
        result.rejected_line = &line;
        result.rejected_angle_deg = angle;
      }
      continue;
    }

    result.line = &line;
    result.angle_deg = angle;
    return result;
  }
  return result;
}

bool wrongWayMeetsThreshold(const WrongWayViolationConfig& config, int hits) {
  return config.enabled && hits >= config.min_hits;
}

Json::Value buildWrongWayEvidence(const std::string& line_name) {
  Json::Value evidence(Json::objectValue);
  evidence["line_name"] = line_name;
  evidence["road_type"] = "highway";
  return evidence;
}

}  // namespace violation
}  // namespace business
}  // namespace vehicle
