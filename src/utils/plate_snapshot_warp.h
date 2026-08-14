// Snapshot biển: perspective-warp từ YOLO-Pose keypoints (cùng convention preprocess).
// Fail → caller giữ path AABB cũ (nvds_obj_enc).
#pragma once

#include <cstdint>
#include <vector>

struct NvBufSurface;
struct _NvDsObjectMeta;
struct _NvDsFrameMeta;

namespace vehicle {
namespace utils {

struct PlateWarpSnapshotParams {
  float keypoint_min_confidence = 0.25f;  // mặc định = preprocess keypoint_min_confidence
  int net_width = 640;
  int net_height = 640;
  int max_width = 400;
  int max_height = 200;
  // Re-expand parent ROI (meta probe chạy SAU restore — cần ROI lúc SGIE1).
  double plate_expand_ratio = 0.30;
  int vehicle_bottom_pad = 10;
  int jpeg_quality = 80;
};

// Warp plate → JPEG. `batch_index` = frame->batch_id trong NvBufSurface.
bool encodeWarpedPlateJpeg(NvBufSurface* surface, int batch_index, _NvDsObjectMeta* plate,
                           _NvDsFrameMeta* frame, const PlateWarpSnapshotParams& params,
                           std::vector<uint8_t>* jpeg_out);

}  // namespace utils
}  // namespace vehicle
