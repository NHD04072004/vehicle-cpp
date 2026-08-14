#include "utils/plate_snapshot_warp.h"

#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <nvdsmeta.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "common/types.h"
#include "utils/geometry.h"
#include "utils/jpeg_draw.h"

namespace vehicle {
namespace utils {
namespace {

constexpr int kNumKeypoints = 4;

struct Point2f {
  float x = 0.f;
  float y = 0.f;
};

void unletterboxPoint(float kx, float ky, int net_w, int net_h, int roi_w, int roi_h, float& lx,
                      float& ly) {
  const float scale = std::min(static_cast<float>(net_w) / static_cast<float>(roi_w),
                               static_cast<float>(net_h) / static_cast<float>(roi_h));
  const float new_w = static_cast<float>(roi_w) * scale;
  const float new_h = static_cast<float>(roi_h) * scale;
  const float pad_x = (static_cast<float>(net_w) - new_w) * 0.5f;
  const float pad_y = (static_cast<float>(net_h) - new_h) * 0.5f;
  lx = (kx - pad_x) / scale;
  ly = (ky - pad_y) / scale;
}

void fillMissingKeypoint(Point2f pts[4], bool ok[4]) {
  for (int miss = 0; miss < kNumKeypoints; ++miss) {
    if (ok[miss]) continue;
    const int a = (miss + 3) % 4;
    const int b = (miss + 1) % 4;
    const int c = (miss + 2) % 4;
    if (ok[a] && ok[b] && ok[c]) {
      pts[miss].x = pts[a].x + pts[b].x - pts[c].x;
      pts[miss].y = pts[a].y + pts[b].y - pts[c].y;
      ok[miss] = true;
    }
  }
}

bool readKeypointsNet(NvDsObjectMeta* obj, float min_conf, Point2f out[4]) {
  if (obj == nullptr || obj->mask_params.data == nullptr) return false;
  if (obj->mask_params.size < kNumKeypoints * 3 * sizeof(float)) return false;
  const float* m = obj->mask_params.data;
  bool ok[4] = {false, false, false, false};
  int n_ok = 0;
  for (int i = 0; i < kNumKeypoints; ++i) {
    out[i].x = m[i * 3 + 0];
    out[i].y = m[i * 3 + 1];
    const float conf = m[i * 3 + 2];
    if (conf >= min_conf && std::isfinite(out[i].x) && std::isfinite(out[i].y)) {
      ok[i] = true;
      ++n_ok;
    }
  }
  if (n_ok == kNumKeypoints) return true;
  if (n_ok >= 3) {
    fillMissingKeypoint(out, ok);
    return ok[0] && ok[1] && ok[2] && ok[3];
  }
  return false;
}

/** Parent ROI lúc SGIE1 = bbox xe đã expand (meta probe chạy sau restore). */
bool inferenceParentRoi(NvDsObjectMeta* plate, NvDsFrameMeta* frame,
                        const PlateWarpSnapshotParams& params, float* roi_left, float* roi_top,
                        int* roi_w, int* roi_h) {
  if (plate == nullptr || frame == nullptr) return false;
  const int frame_w = static_cast<int>(frame->source_frame_width);
  const int frame_h = static_cast<int>(frame->source_frame_height);
  if (frame_w < 1 || frame_h < 1) return false;

  *roi_left = 0.f;
  *roi_top = 0.f;
  *roi_w = frame_w;
  *roi_h = frame_h;

  NvDsObjectMeta* parent = plate->parent;
  if (parent == nullptr || parent->rect_params.width <= 1.f || parent->rect_params.height <= 1.f)
    return true;

  BoundingBox box;
  box.x1 = parent->rect_params.left;
  box.y1 = parent->rect_params.top;
  box.x2 = parent->rect_params.left + parent->rect_params.width;
  box.y2 = parent->rect_params.top + parent->rect_params.height;
  box.y2 += static_cast<double>(params.vehicle_bottom_pad);
  if (params.plate_expand_ratio > 0.0)
    box = expandBox(box, params.plate_expand_ratio, frame_w, frame_h);
  else
    box = clampBox(box, frame_w, frame_h);

  *roi_left = static_cast<float>(box.x1);
  *roi_top = static_cast<float>(box.y1);
  *roi_w = std::max(1, static_cast<int>(std::lround(box.x2 - box.x1)));
  *roi_h = std::max(1, static_cast<int>(std::lround(box.y2 - box.y1)));
  return true;
}

bool keypointsToFrame(NvDsObjectMeta* plate, NvDsFrameMeta* frame,
                      const PlateWarpSnapshotParams& params, Point2f out[4]) {
  Point2f net[4];
  if (!readKeypointsNet(plate, params.keypoint_min_confidence, net)) return false;
  float roi_left = 0.f, roi_top = 0.f;
  int roi_w = 0, roi_h = 0;
  if (!inferenceParentRoi(plate, frame, params, &roi_left, &roi_top, &roi_w, &roi_h)) return false;
  for (int i = 0; i < kNumKeypoints; ++i) {
    float lx = 0.f, ly = 0.f;
    unletterboxPoint(net[i].x, net[i].y, params.net_width, params.net_height, roi_w, roi_h, lx,
                     ly);
    out[i].x = roi_left + lx;
    out[i].y = roi_top + ly;
  }
  return true;
}

bool computeWarpSize(const Point2f src[4], int max_w, int max_h, int* w, int* h) {
  auto dist = [](Point2f a, Point2f b) {
    const float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  };
  float wf = std::max(dist(src[0], src[1]), dist(src[3], src[2]));
  float hf = std::max(dist(src[0], src[3]), dist(src[1], src[2]));
  if (!(wf >= 1.f) || !(hf >= 1.f) || !std::isfinite(wf) || !std::isfinite(hf)) return false;
  const float up = std::max(1.f, std::max(128.f / wf, 40.f / hf));
  wf *= up;
  hf *= up;
  const float scale = std::min(static_cast<float>(max_w) / wf, static_cast<float>(max_h) / hf);
  if (scale < 1.f) {
    wf *= scale;
    hf *= scale;
  }
  *w = std::max(1, static_cast<int>(std::lround(wf)));
  *h = std::max(1, static_cast<int>(std::lround(hf)));
  return true;
}

bool getPerspectiveTransform(const Point2f src[4], const Point2f dst[4], double H[9]) {
  // 8x8 system for homography (same as preprocess lib).
  double A[8][8]{};
  double b[8]{};
  for (int i = 0; i < 4; ++i) {
    const double x = src[i].x, y = src[i].y;
    const double u = dst[i].x, v = dst[i].y;
    A[i * 2 + 0][0] = x;
    A[i * 2 + 0][1] = y;
    A[i * 2 + 0][2] = 1;
    A[i * 2 + 0][6] = -u * x;
    A[i * 2 + 0][7] = -u * y;
    b[i * 2 + 0] = u;
    A[i * 2 + 1][3] = x;
    A[i * 2 + 1][4] = y;
    A[i * 2 + 1][5] = 1;
    A[i * 2 + 1][6] = -v * x;
    A[i * 2 + 1][7] = -v * y;
    b[i * 2 + 1] = v;
  }
  // Gaussian elimination
  for (int col = 0; col < 8; ++col) {
    int pivot = col;
    for (int r = col + 1; r < 8; ++r)
      if (std::fabs(A[r][col]) > std::fabs(A[pivot][col])) pivot = r;
    if (std::fabs(A[pivot][col]) < 1e-12) return false;
    if (pivot != col) {
      for (int c = 0; c < 8; ++c) std::swap(A[pivot][c], A[col][c]);
      std::swap(b[pivot], b[col]);
    }
    const double div = A[col][col];
    for (int c = col; c < 8; ++c) A[col][c] /= div;
    b[col] /= div;
    for (int r = 0; r < 8; ++r) {
      if (r == col) continue;
      const double f = A[r][col];
      for (int c = col; c < 8; ++c) A[r][c] -= f * A[col][c];
      b[r] -= f * b[col];
    }
  }
  for (int i = 0; i < 8; ++i) H[i] = b[i];
  H[8] = 1.0;
  return true;
}

bool invert3x3(const double m[9], double inv[9]) {
  const double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7],
               i = m[8];
  const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (std::fabs(det) < 1e-12) return false;
  const double id = 1.0 / det;
  inv[0] = (e * i - f * h) * id;
  inv[1] = (c * h - b * i) * id;
  inv[2] = (b * f - c * e) * id;
  inv[3] = (f * g - d * i) * id;
  inv[4] = (a * i - c * g) * id;
  inv[5] = (c * d - a * f) * id;
  inv[6] = (d * h - e * g) * id;
  inv[7] = (b * g - a * h) * id;
  inv[8] = (a * e - b * d) * id;
  return true;
}

void sampleBilinearRgb(const uint8_t* src, int sw, int sh, int pitch, float sx, float sy,
                       uint8_t out[3]) {
  const int x0 = static_cast<int>(std::floor(sx));
  const int y0 = static_cast<int>(std::floor(sy));
  const float qx = sx - static_cast<float>(x0);
  const float qy = sy - static_cast<float>(y0);
  const float px = 1.f - qx;
  auto fetch = [&](int ix, int iy, int c) -> float {
    if (static_cast<unsigned>(ix) >= static_cast<unsigned>(sw) ||
        static_cast<unsigned>(iy) >= static_cast<unsigned>(sh))
      return 0.f;
    return static_cast<float>(src[iy * pitch + ix * 4 + c]);
  };
  for (int c = 0; c < 3; ++c) {
    const float s00 = fetch(x0, y0, c);
    const float s01 = fetch(x0 + 1, y0, c);
    const float s10 = fetch(x0, y0 + 1, c);
    const float s11 = fetch(x0 + 1, y0 + 1, c);
    const float s0 = s00 * px + s01 * qx;
    const float s1 = s10 * px + s11 * qx;
    out[c] = static_cast<uint8_t>(std::min(255.f, std::max(0.f, s0 + (s1 - s0) * qy + 0.5f)));
  }
}

bool cropFrameRgba(NvBufSurface* surface, int batch_index, int x0, int y0, int cw, int ch,
                   std::vector<uint8_t>* rgba, int* pitch_out) {
  if (surface == nullptr || rgba == nullptr || cw < 1 || ch < 1) return false;
  if (batch_index < 0 || batch_index >= static_cast<int>(surface->numFilled)) return false;

  NvBufSurfaceCreateParams cp{};
  cp.gpuId = surface->gpuId;
  cp.width = cw;
  cp.height = ch;
  cp.size = 0;
  cp.isContiguous = true;
  cp.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
  cp.layout = NVBUF_LAYOUT_PITCH;
  cp.memType = NVBUF_MEM_CUDA_PINNED;

  NvBufSurface* tmp = nullptr;
  if (NvBufSurfaceCreate(&tmp, 1, &cp) != 0 || tmp == nullptr) return false;

  NvBufSurfTransformConfigParams config{};
  config.compute_mode = NvBufSurfTransformCompute_GPU;
  config.gpu_id = surface->gpuId;
  config.cuda_stream = nullptr;
  if (NvBufSurfTransformSetSessionParams(&config) != NvBufSurfTransformError_Success) {
    NvBufSurfaceDestroy(tmp);
    return false;
  }

  NvBufSurfTransformParams transform{};
  NvBufSurfTransformRect src_rect{};
  src_rect.top = static_cast<uint32_t>(y0);
  src_rect.left = static_cast<uint32_t>(x0);
  src_rect.width = static_cast<uint32_t>(cw);
  src_rect.height = static_cast<uint32_t>(ch);
  NvBufSurfTransformRect dst_rect{};
  dst_rect.width = static_cast<uint32_t>(cw);
  dst_rect.height = static_cast<uint32_t>(ch);
  transform.transform_flag = NVBUFSURF_TRANSFORM_CROP_SRC | NVBUFSURF_TRANSFORM_FILTER;
  transform.transform_filter = NvBufSurfTransformInter_Bilinear;
  transform.transform_flip = NvBufSurfTransform_None;
  transform.src_rect = &src_rect;
  transform.dst_rect = &dst_rect;

  NvBufSurface in_view = *surface;
  in_view.numFilled = 1;
  in_view.batchSize = 1;
  in_view.surfaceList = &surface->surfaceList[batch_index];
  if (NvBufSurfTransform(&in_view, tmp, &transform) != NvBufSurfTransformError_Success) {
    NvBufSurfaceDestroy(tmp);
    return false;
  }

  if (NvBufSurfaceMap(tmp, 0, 0, NVBUF_MAP_READ) != 0) {
    NvBufSurfaceDestroy(tmp);
    return false;
  }
  NvBufSurfaceSyncForCpu(tmp, 0, 0);

  const int pitch = static_cast<int>(tmp->surfaceList[0].pitch);
  const uint8_t* src = static_cast<const uint8_t*>(tmp->surfaceList[0].mappedAddr.addr[0]);
  if (src == nullptr) {
    NvBufSurfaceUnMap(tmp, 0, 0);
    NvBufSurfaceDestroy(tmp);
    return false;
  }
  rgba->resize(static_cast<size_t>(pitch) * static_cast<size_t>(ch));
  std::memcpy(rgba->data(), src, rgba->size());
  if (pitch_out) *pitch_out = pitch;

  NvBufSurfaceUnMap(tmp, 0, 0);
  NvBufSurfaceDestroy(tmp);
  return true;
}

}  // namespace

bool encodeWarpedPlateJpeg(NvBufSurface* surface, int batch_index, _NvDsObjectMeta* plate_raw,
                           _NvDsFrameMeta* frame_raw, const PlateWarpSnapshotParams& params,
                           std::vector<uint8_t>* jpeg_out) {
  auto* plate = reinterpret_cast<NvDsObjectMeta*>(plate_raw);
  auto* frame = reinterpret_cast<NvDsFrameMeta*>(frame_raw);
  if (surface == nullptr || plate == nullptr || frame == nullptr || jpeg_out == nullptr)
    return false;

  Point2f src[4];
  if (!keypointsToFrame(plate, frame, params, src)) return false;

  int out_w = 0, out_h = 0;
  if (!computeWarpSize(src, params.max_width, params.max_height, &out_w, &out_h)) return false;

  const int frame_w = static_cast<int>(surface->surfaceList[batch_index].width);
  const int frame_h = static_cast<int>(surface->surfaceList[batch_index].height);
  float min_x = src[0].x, max_x = src[0].x, min_y = src[0].y, max_y = src[0].y;
  for (int i = 1; i < 4; ++i) {
    min_x = std::min(min_x, src[i].x);
    max_x = std::max(max_x, src[i].x);
    min_y = std::min(min_y, src[i].y);
    max_y = std::max(max_y, src[i].y);
  }
  const float pad = 4.f;
  min_x = std::max(0.f, min_x - pad);
  min_y = std::max(0.f, min_y - pad);
  max_x = std::min(static_cast<float>(frame_w), max_x + pad);
  max_y = std::min(static_cast<float>(frame_h), max_y + pad);
  const int x0 = static_cast<int>(std::floor(min_x));
  const int y0 = static_cast<int>(std::floor(min_y));
  const int cw = std::max(1, static_cast<int>(std::ceil(max_x)) - x0);
  const int ch = std::max(1, static_cast<int>(std::ceil(max_y)) - y0);
  if (x0 + cw > frame_w || y0 + ch > frame_h) return false;

  std::vector<uint8_t> rgba;
  int pitch = 0;
  if (!cropFrameRgba(surface, batch_index, x0, y0, cw, ch, &rgba, &pitch)) return false;

  Point2f local[4];
  for (int i = 0; i < 4; ++i) {
    local[i].x = src[i].x - static_cast<float>(x0);
    local[i].y = src[i].y - static_cast<float>(y0);
  }
  Point2f dstp[4] = {{0.f, 0.f},
                     {static_cast<float>(out_w - 1), 0.f},
                     {static_cast<float>(out_w - 1), static_cast<float>(out_h - 1)},
                     {0.f, static_cast<float>(out_h - 1)}};
  double H[9], inv[9];
  if (!getPerspectiveTransform(local, dstp, H) || !invert3x3(H, inv)) return false;

  std::vector<uint8_t> rgb(static_cast<size_t>(out_w) * static_cast<size_t>(out_h) * 3u);
  for (int y = 0; y < out_h; ++y) {
    for (int x = 0; x < out_w; ++x) {
      const double X = inv[0] * x + inv[1] * y + inv[2];
      const double Y = inv[3] * x + inv[4] * y + inv[5];
      const double W = inv[6] * x + inv[7] * y + inv[8];
      uint8_t px[3] = {0, 0, 0};
      if (std::fabs(W) > 1e-12) {
        sampleBilinearRgb(rgba.data(), cw, ch, pitch, static_cast<float>(X / W),
                          static_cast<float>(Y / W), px);
      }
      uint8_t* d = rgb.data() + (static_cast<size_t>(y) * out_w + x) * 3u;
      d[0] = px[0];
      d[1] = px[1];
      d[2] = px[2];
    }
  }
  return encodeRgbToJpeg(rgb.data(), out_w, out_h, params.jpeg_quality, jpeg_out);
}

}  // namespace utils
}  // namespace vehicle
