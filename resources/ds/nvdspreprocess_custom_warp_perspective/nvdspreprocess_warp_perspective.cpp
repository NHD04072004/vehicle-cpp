/*
 * nvdspreprocess custom lib: GPU perspective-warp plate ROIs from YOLO-Pose keypoints.
 *
 * Path (device-only, DALI-style bilinear):
 *   NV12 NvBufSurface → NvBufSurfTransform RGBA (CUDA device)
 *   → CUDA warp_perspective → RGBA 256x256 (device)
 *   → CUDA RGBA→NCHW float → nvinfer input-tensor-meta buffer
 *
 * No full-frame host copy / CPU bilinear.
 */

#include "nvdspreprocess_warp_perspective.h"
#include "warp_kernels.h"

#include "gstnvdsmeta.h"
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cuda_runtime_api.h>
#include <gst/gst.h>

namespace {

constexpr int kNumKeypoints = 4;
/** Max chi tiết dump mỗi process (tránh spam RTSP). WARP_DEBUG=0 tắt. */
constexpr int kMaxDebugDumps = 64;

static int warpDebugEnabled()
{
  static int enabled = -1;
  if (enabled < 0) {
    const char *e = std::getenv("WARP_DEBUG");
    enabled = (e == nullptr || e[0] == '\0' || e[0] == '0') ? 0 : 1;
  }
  return enabled;
}

static bool warpDebugTake()
{
  if (!warpDebugEnabled())
    return false;
  static std::atomic<int> n{0};
  int cur = n.fetch_add(1, std::memory_order_relaxed);
  return cur < kMaxDebugDumps;
}

enum class WarpFail : int {
  Ok = 0,
  NoObj,
  NoMask,
  MaskTooSmall,
  LowKeypointConf,
  WarpSize,
  Homography,
  Kernel,
};

static const char *warpFailName(WarpFail f)
{
  switch (f) {
  case WarpFail::Ok:
    return "ok";
  case WarpFail::NoObj:
    return "no_obj";
  case WarpFail::NoMask:
    return "no_mask";
  case WarpFail::MaskTooSmall:
    return "mask_too_small";
  case WarpFail::LowKeypointConf:
    return "low_kpt_conf";
  case WarpFail::WarpSize:
    return "warp_size";
  case WarpFail::Homography:
    return "homography";
  case WarpFail::Kernel:
    return "kernel";
  }
  return "?";
}

enum class CropFail : int {
  Ok = 0,
  BadArgs,
  EmptySize,
  OutsideFrame,
  CreateSurf,
  Session,
  Transform,
  Memcpy,
};

static const char *cropFailName(CropFail f)
{
  switch (f) {
  case CropFail::Ok:
    return "ok";
  case CropFail::BadArgs:
    return "bad_args";
  case CropFail::EmptySize:
    return "empty_size";
  case CropFail::OutsideFrame:
    return "outside_frame";
  case CropFail::CreateSurf:
    return "create_surf";
  case CropFail::Session:
    return "session";
  case CropFail::Transform:
    return "transform";
  case CropFail::Memcpy:
    return "memcpy";
  }
  return "?";
}

struct Point2f {
  float x = 0.f;
  float y = 0.f;
};

struct DeviceFrame {
  NvBufSurface *surf = nullptr;
  uint8_t *data = nullptr;
  int pitch = 0;
  int w = 0;
  int h = 0;
};

static bool parse_float(const std::unordered_map<std::string, std::string> &cfg, const char *key,
                        float default_v, float &out)
{
  auto it = cfg.find(key);
  if (it == cfg.end() || it->second.empty()) {
    out = default_v;
    return false;
  }
  try {
    out = std::stof(it->second);
    return true;
  } catch (...) {
    out = default_v;
    return false;
  }
}

static bool parse_int(const std::unordered_map<std::string, std::string> &cfg, const char *key,
                      int default_v, int &out)
{
  auto it = cfg.find(key);
  if (it == cfg.end() || it->second.empty()) {
    out = default_v;
    return false;
  }
  try {
    out = std::stoi(it->second);
    return true;
  } catch (...) {
    out = default_v;
    return false;
  }
}

static bool solve8(double A[8][8], double b[8], double x[8])
{
  double M[8][9];
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j)
      M[i][j] = A[i][j];
    M[i][8] = b[i];
  }
  for (int col = 0; col < 8; ++col) {
    int pivot = col;
    for (int r = col + 1; r < 8; ++r) {
      if (std::fabs(M[r][col]) > std::fabs(M[pivot][col]))
        pivot = r;
    }
    if (std::fabs(M[pivot][col]) < 1e-12)
      return false;
    if (pivot != col) {
      for (int c = col; c < 9; ++c)
        std::swap(M[pivot][c], M[col][c]);
    }
    double div = M[col][col];
    for (int c = col; c < 9; ++c)
      M[col][c] /= div;
    for (int r = 0; r < 8; ++r) {
      if (r == col)
        continue;
      double f = M[r][col];
      for (int c = col; c < 9; ++c)
        M[r][c] -= f * M[col][c];
    }
  }
  for (int i = 0; i < 8; ++i)
    x[i] = M[i][8];
  return true;
}

static bool getPerspectiveTransform(const Point2f src[4], const Point2f dst[4], double H[9])
{
  double A[8][8] = {};
  double b[8] = {};
  for (int i = 0; i < 4; ++i) {
    double x = src[i].x, y = src[i].y, u = dst[i].x, v = dst[i].y;
    A[i * 2 + 0][0] = x;
    A[i * 2 + 0][1] = y;
    A[i * 2 + 0][2] = 1;
    A[i * 2 + 0][6] = -x * u;
    A[i * 2 + 0][7] = -y * u;
    b[i * 2 + 0] = u;

    A[i * 2 + 1][3] = x;
    A[i * 2 + 1][4] = y;
    A[i * 2 + 1][5] = 1;
    A[i * 2 + 1][6] = -x * v;
    A[i * 2 + 1][7] = -y * v;
    b[i * 2 + 1] = v;
  }
  double h[8];
  if (!solve8(A, b, h))
    return false;
  for (int i = 0; i < 8; ++i)
    H[i] = h[i];
  H[8] = 1.0;
  return true;
}

static bool invert3x3(const double H[9], double inv[9])
{
  double a = H[0], b = H[1], c = H[2];
  double d = H[3], e = H[4], f = H[5];
  double g = H[6], h = H[7], i = H[8];
  double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (std::fabs(det) < 1e-12)
    return false;
  double id = 1.0 / det;
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

static void unletterboxPoint(float kx, float ky, int net_w, int net_h, int roi_w, int roi_h,
                             float &lx, float &ly)
{
  float scale = std::min((float)net_w / (float)roi_w, (float)net_h / (float)roi_h);
  float new_w = roi_w * scale;
  float new_h = roi_h * scale;
  float pad_x = (net_w - new_w) * 0.5f;
  float pad_y = (net_h - new_h) * 0.5f;
  lx = (kx - pad_x) / scale;
  ly = (ky - pad_y) / scale;
}

/** Bổ sung điểm thiếu theo hình bình hành: P_i = P_{i-1} + P_{i+1} - P_{i+2}. */
static void fillMissingKeypoint(Point2f pts[4], bool ok[4])
{
  for (int miss = 0; miss < kNumKeypoints; ++miss) {
    if (ok[miss])
      continue;
    int a = (miss + 3) % 4;
    int b = (miss + 1) % 4;
    int c = (miss + 2) % 4;
    if (ok[a] && ok[b] && ok[c]) {
      pts[miss].x = pts[a].x + pts[b].x - pts[c].x;
      pts[miss].y = pts[a].y + pts[b].y - pts[c].y;
      ok[miss] = true;
    }
  }
}

/** Đọc 4 keypoints; cho phép 3/4 đạt min_conf rồi suy điểm còn lại. */
static WarpFail readKeypoints(NvDsObjectMeta *obj, float min_conf, Point2f out[4], float conf_out[4])
{
  for (int i = 0; i < kNumKeypoints; ++i)
    conf_out[i] = -1.f;
  if (!obj)
    return WarpFail::NoObj;
  if (!obj->mask_params.data)
    return WarpFail::NoMask;
  if (obj->mask_params.size < kNumKeypoints * 3 * sizeof(float))
    return WarpFail::MaskTooSmall;
  const float *m = obj->mask_params.data;
  bool ok[4] = {false, false, false, false};
  int n_ok = 0;
  for (int i = 0; i < kNumKeypoints; ++i) {
    conf_out[i] = m[i * 3 + 2];
    out[i].x = m[i * 3 + 0];
    out[i].y = m[i * 3 + 1];
    if (conf_out[i] >= min_conf && std::isfinite(out[i].x) && std::isfinite(out[i].y)) {
      ok[i] = true;
      ++n_ok;
    }
  }
  if (n_ok == kNumKeypoints)
    return WarpFail::Ok;
  if (n_ok >= 3) {
    fillMissingKeypoint(out, ok);
    if (ok[0] && ok[1] && ok[2] && ok[3])
      return WarpFail::Ok;
  }
  return WarpFail::LowKeypointConf;
}

/** Map net-space keypoints → full-frame coords.
 *  SGIE pose: letterbox là ROI parent (xe); PGIE pose: letterbox là cả frame. */
static WarpFail keypointsToFrame(NvDsObjectMeta *obj, float min_conf, int net_w, int net_h,
                                 int frame_w, int frame_h, Point2f out[4], Point2f net_out[4],
                                 float conf_out[4], float *roi_left, float *roi_top, int *roi_w,
                                 int *roi_h, bool *has_parent)
{
  WarpFail rf = readKeypoints(obj, min_conf, net_out, conf_out);
  if (rf != WarpFail::Ok)
    return rf;

  *roi_left = 0.f;
  *roi_top = 0.f;
  *roi_w = frame_w;
  *roi_h = frame_h;
  *has_parent = false;
  if (obj->parent != nullptr && obj->parent->rect_params.width > 1.f &&
      obj->parent->rect_params.height > 1.f) {
    *roi_left = obj->parent->rect_params.left;
    *roi_top = obj->parent->rect_params.top;
    *roi_w = std::max(1, (int)std::lround(obj->parent->rect_params.width));
    *roi_h = std::max(1, (int)std::lround(obj->parent->rect_params.height));
    *has_parent = true;
  }

  for (int i = 0; i < kNumKeypoints; ++i) {
    float lx = 0.f, ly = 0.f;
    unletterboxPoint(net_out[i].x, net_out[i].y, net_w, net_h, *roi_w, *roi_h, lx, ly);
    out[i].x = *roi_left + lx;
    out[i].y = *roi_top + ly;
  }
  return WarpFail::Ok;
}

/** Kích thước biển theo cạnh keypoints (px frame) — dùng cho tỉ lệ letterbox. */
static bool plateNativeSize(const Point2f src[4], float &wf, float &hf)
{
  auto dist = [](Point2f a, Point2f b) {
    float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  };
  wf = std::max(dist(src[0], src[1]), dist(src[3], src[2]));
  hf = std::max(dist(src[0], src[3]), dist(src[1], src[2]));
  if (!(wf >= 1.f) || !(hf >= 1.f) || !std::isfinite(wf) || !std::isfinite(hf))
    return false;
  return true;
}

/** Fit giữ tỉ lệ vào out_w×out_h (YOLO maintain-aspect-ratio + symmetric pad). */
static void fitLetterbox(float native_w, float native_h, int out_w, int out_h, int &cw, int &ch,
                         int &pad_x, int &pad_y)
{
  float scale = std::min((float)out_w / native_w, (float)out_h / native_h);
  cw = std::max(1, std::min(out_w, (int)std::lround(native_w * scale)));
  ch = std::max(1, std::min(out_h, (int)std::lround(native_h * scale)));
  pad_x = (out_w - cw) / 2;
  pad_y = (out_h - ch) / 2;
}

static void dumpPpm(const std::string &path, const uint8_t *rgba, int w, int h)
{
  std::ofstream f(path, std::ios::binary);
  if (!f)
    return;
  f << "P6\n" << w << " " << h << "\n255\n";
  for (int i = 0; i < w * h; ++i) {
    f.put((char)rgba[i * 4 + 0]);
    f.put((char)rgba[i * 4 + 1]);
    f.put((char)rgba[i * 4 + 2]);
  }
}

static void destroyDeviceFrame(DeviceFrame &df)
{
  if (df.surf) {
    NvBufSurfaceDestroy(df.surf);
    df.surf = nullptr;
  }
  df.data = nullptr;
  df.pitch = df.w = df.h = 0;
}

/** NVMM → RGBA on CUDA device (no host sync). */
static bool frameToDeviceRGBA(NvBufSurface *in_surf, int batch_index, DeviceFrame &out)
{
  destroyDeviceFrame(out);
  if (!in_surf || batch_index < 0 || batch_index >= (int)in_surf->numFilled)
    return false;

  NvBufSurfaceParams *src = &in_surf->surfaceList[batch_index];
  int w = (int)src->width;
  int h = (int)src->height;

  NvBufSurfaceCreateParams cp{};
  cp.gpuId = in_surf->gpuId;
  cp.width = w;
  cp.height = h;
  cp.size = 0;
  cp.isContiguous = true;
  cp.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
  cp.layout = NVBUF_LAYOUT_PITCH;
  cp.memType = NVBUF_MEM_CUDA_DEVICE;

  NvBufSurface *out_surf = nullptr;
  if (NvBufSurfaceCreate(&out_surf, 1, &cp) != 0 || !out_surf)
    return false;

  NvBufSurfTransformConfigParams config{};
  config.compute_mode = NvBufSurfTransformCompute_GPU;
  config.gpu_id = in_surf->gpuId;
  config.cuda_stream = nullptr;
  if (NvBufSurfTransformSetSessionParams(&config) != NvBufSurfTransformError_Success) {
    NvBufSurfaceDestroy(out_surf);
    return false;
  }

  NvBufSurfTransformParams transform{};
  NvBufSurfTransformRect src_rect{0, 0, (uint32_t)w, (uint32_t)h};
  NvBufSurfTransformRect dst_rect{0, 0, (uint32_t)w, (uint32_t)h};
  transform.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
  transform.transform_filter = NvBufSurfTransformInter_Bilinear;
  transform.transform_flip = NvBufSurfTransform_None;
  transform.src_rect = &src_rect;
  transform.dst_rect = &dst_rect;

  NvBufSurface in_view = *in_surf;
  in_view.numFilled = 1;
  in_view.batchSize = 1;
  in_view.surfaceList = src;
  if (NvBufSurfTransform(&in_view, out_surf, &transform) != NvBufSurfTransformError_Success) {
    printf("warp_perspective: NvBufSurfTransform(device) failed bi=%d fmt=%d\n", batch_index,
           (int)src->colorFormat);
    fflush(stdout);
    NvBufSurfaceDestroy(out_surf);
    return false;
  }

  out.surf = out_surf;
  out.data = (uint8_t *)out_surf->surfaceList[0].dataPtr;
  out.pitch = (int)out_surf->surfaceList[0].pitch;
  out.w = w;
  out.h = h;
  return out.data != nullptr;
}

/** GPU bbox crop+resize fallback (CUDA bilinear — tránh NvBufSurfTransform fail ROI nhỏ/biên). */
static CropFail cropResizeDevice(const DeviceFrame &frame, float left, float top, float width,
                                 float height, uint8_t *dst_rgba, int dw, int dh,
                                 cudaStream_t stream, int *out_x0, int *out_y0, int *out_cw,
                                 int *out_ch)
{
  if (out_x0)
    *out_x0 = -1;
  if (out_y0)
    *out_y0 = -1;
  if (out_cw)
    *out_cw = 0;
  if (out_ch)
    *out_ch = 0;

  if (!frame.data || !dst_rgba)
    return CropFail::BadArgs;
  if (!(width >= 1.f) || !(height >= 1.f) || !std::isfinite(left) || !std::isfinite(top) ||
      !std::isfinite(width) || !std::isfinite(height))
    return CropFail::EmptySize;

  float x1 = left;
  float y1 = top;
  float x2 = left + width;
  float y2 = top + height;
  if (x1 < 0.f)
    x1 = 0.f;
  if (y1 < 0.f)
    y1 = 0.f;
  if (x2 > (float)frame.w)
    x2 = (float)frame.w;
  if (y2 > (float)frame.h)
    y2 = (float)frame.h;
  float cw_f = x2 - x1;
  float ch_f = y2 - y1;
  if (cw_f < 1.f || ch_f < 1.f)
    return CropFail::OutsideFrame;

  if (out_x0)
    *out_x0 = (int)std::floor(x1);
  if (out_y0)
    *out_y0 = (int)std::floor(y1);
  if (out_cw)
    *out_cw = std::max(1, (int)std::round(cw_f));
  if (out_ch)
    *out_ch = std::max(1, (int)std::round(ch_f));

  cudaError_t err =
      launch_crop_resize_rgba(frame.data, frame.w, frame.h, frame.pitch, x1, y1, cw_f, ch_f,
                              dst_rgba, dw, dh, stream);
  if (err != cudaSuccess)
    return CropFail::Transform;
  return CropFail::Ok;
}

static NvDsObjectMeta *unitObjectMeta(const NvDsPreProcessUnit &unit)
{
  if (unit.obj_meta)
    return unit.obj_meta;
  return unit.roi_meta.object_meta;
}

} // namespace

extern "C" NvDsPreProcessStatus
CustomTransformation(NvBufSurface *in_surf, NvBufSurface *out_surf, CustomTransformParams &params)
{
  NvBufSurfTransform_Error err = NvBufSurfTransformSetSessionParams(&params.transform_config_params);
  if (err != NvBufSurfTransformError_Success)
    return NVDSPREPROCESS_CUSTOM_TRANSFORMATION_FAILED;
  err = NvBufSurfTransform(in_surf, out_surf, &params.transform_params);
  if (err != NvBufSurfTransformError_Success)
    return NVDSPREPROCESS_CUSTOM_TRANSFORMATION_FAILED;
  return NVDSPREPROCESS_SUCCESS;
}

extern "C" NvDsPreProcessStatus
CustomAsyncTransformation(NvBufSurface *in_surf, NvBufSurface *out_surf, CustomTransformParams &params)
{
  NvBufSurfTransform_Error err = NvBufSurfTransformSetSessionParams(&params.transform_config_params);
  if (err != NvBufSurfTransformError_Success)
    return NVDSPREPROCESS_CUSTOM_TRANSFORMATION_FAILED;
  err = NvBufSurfTransformAsync(in_surf, out_surf, &params.transform_params, &params.sync_obj);
  if (err != NvBufSurfTransformError_Success)
    return NVDSPREPROCESS_CUSTOM_TRANSFORMATION_FAILED;
  return NVDSPREPROCESS_SUCCESS;
}

extern "C" NvDsPreProcessStatus
CustomTensorPreparation(CustomCtx *ctx, NvDsPreProcessBatch *batch, NvDsPreProcessCustomBuf *&buf,
                        CustomTensorParams &tensorParam, NvDsPreProcessAcquirer *acquirer)
{
  if (!ctx || !batch || !acquirer)
    return NVDSPREPROCESS_CUSTOM_TENSOR_FAILED;

  buf = acquirer->acquire();
  if (!buf || !buf->memory_ptr)
    return NVDSPREPROCESS_CUSTOM_TENSOR_FAILED;

  if (batch->units.empty()) {
    tensorParam.params.network_input_shape[0] = 0;
    return NVDSPREPROCESS_SUCCESS;
  }

  int out_c = 3, out_h = 256, out_w = 256;
  if (tensorParam.params.network_input_shape.size() >= 4) {
    out_c = tensorParam.params.network_input_shape[1];
    out_h = tensorParam.params.network_input_shape[2];
    out_w = tensorParam.params.network_input_shape[3];
  }
  (void)out_c;

  GstMapInfo inmap = GST_MAP_INFO_INIT;
  if (!batch->inbuf || !gst_buffer_map(batch->inbuf, &inmap, GST_MAP_READ)) {
    printf("warp_perspective: gst_buffer_map failed\n");
    fflush(stdout);
    acquirer->release(buf);
    return NVDSPREPROCESS_CUSTOM_TENSOR_FAILED;
  }
  NvBufSurface *in_surf = (NvBufSurface *)inmap.data;

  std::unordered_map<int, DeviceFrame> frame_cache;
  const size_t plate_bytes = (size_t)out_w * out_h * 4;
  uint8_t *d_plate = nullptr;
  uint8_t *d_content = nullptr;
  cudaError_t cerr = cudaMalloc(&d_plate, plate_bytes);
  if (cerr == cudaSuccess)
    cerr = cudaMalloc(&d_content, plate_bytes);
  if (cerr != cudaSuccess) {
    printf("warp_perspective: cudaMalloc plate failed: %s\n", cudaGetErrorString(cerr));
    fflush(stdout);
    if (d_plate)
      cudaFree(d_plate);
    if (d_content)
      cudaFree(d_content);
    gst_buffer_unmap(batch->inbuf, &inmap);
    acquirer->release(buf);
    return NVDSPREPROCESS_CUDA_ERROR;
  }

  const size_t unit_floats = (size_t)3 * out_h * out_w;
  bool ok = true;

  for (size_t ui = 0; ui < batch->units.size(); ++ui) {
    const NvDsPreProcessUnit &unit = batch->units[ui];
    int bi = (int)unit.batch_index;
    float *dst = (float *)buf->memory_ptr + ui * unit_floats;

    if (frame_cache.find(bi) == frame_cache.end()) {
      DeviceFrame df;
      if (!frameToDeviceRGBA(in_surf, bi, df)) {
        printf("warp_perspective: frameToDeviceRGBA failed batch_index=%d\n", bi);
        fflush(stdout);
        ok = false;
        break;
      }
      frame_cache.emplace(bi, df);
    }
    DeviceFrame &frame = frame_cache[bi];

    NvDsObjectMeta *obj = unitObjectMeta(unit);
    bool warped = false;
    WarpFail wfail = obj ? WarpFail::Ok : WarpFail::NoObj;

    Point2f src[4]{};
    Point2f net_kpts[4]{};
    float kpt_conf[4] = {-1.f, -1.f, -1.f, -1.f};
    float roi_left = 0.f, roi_top = 0.f;
    int roi_w = frame.w, roi_h = frame.h;
    bool has_parent = false;
    int native_w = 0, native_h = 0;
    int content_w = 0, content_h = 0, pad_x = 0, pad_y = 0;

    if (obj) {
      wfail = keypointsToFrame(obj, ctx->keypoint_min_confidence, ctx->net_width, ctx->net_height,
                               frame.w, frame.h, src, net_kpts, kpt_conf, &roi_left, &roi_top,
                               &roi_w, &roi_h, &has_parent);
      if (wfail == WarpFail::Ok) {
        float wf = 0.f, hf = 0.f;
        if (!plateNativeSize(src, wf, hf)) {
          wfail = WarpFail::WarpSize;
        } else {
          native_w = std::max(1, (int)std::lround(wf));
          native_h = std::max(1, (int)std::lround(hf));
          fitLetterbox(wf, hf, out_w, out_h, content_w, content_h, pad_x, pad_y);
          Point2f dstp[4] = {{0.f, 0.f},
                             {(float)(content_w - 1), 0.f},
                             {(float)(content_w - 1), (float)(content_h - 1)},
                             {0.f, (float)(content_h - 1)}};
          double H[9], inv[9];
          if (!getPerspectiveTransform(src, dstp, H) || !invert3x3(H, inv)) {
            wfail = WarpFail::Homography;
          } else {
            float inv_f[9];
            for (int i = 0; i < 9; ++i)
              inv_f[i] = (float)inv[i];
            cerr = cudaMemsetAsync(d_plate, 0, plate_bytes, ctx->stream);
            if (cerr == cudaSuccess)
              cerr = launch_warp_perspective_rgba(frame.data, frame.w, frame.h, frame.pitch,
                                                  d_content, content_w, content_h, inv_f,
                                                  ctx->stream);
            if (cerr == cudaSuccess)
              cerr = launch_paste_rgba(d_content, content_w, content_h, d_plate, out_w, out_h,
                                       pad_x, pad_y, ctx->stream);
            if (cerr == cudaSuccess) {
              warped = true;
              wfail = WarpFail::Ok;
            } else {
              wfail = WarpFail::Kernel;
              printf("warp_perspective: warp+letterbox failed: %s\n", cudaGetErrorString(cerr));
              fflush(stdout);
            }
          }
        }
      }
    }

    if (!warped) {
      float left = unit.roi_meta.roi.left;
      float top = unit.roi_meta.roi.top;
      float width = unit.roi_meta.roi.width;
      float height = unit.roi_meta.roi.height;
      if (obj) {
        left = obj->rect_params.left;
        top = obj->rect_params.top;
        width = obj->rect_params.width;
        height = obj->rect_params.height;
      }
      int cx0 = -1, cy0 = -1, ccw = 0, cch = 0;
      fitLetterbox(std::max(1.f, width), std::max(1.f, height), out_w, out_h, content_w, content_h,
                   pad_x, pad_y);
      CropFail cfail = CropFail::Ok;
      cerr = cudaMemsetAsync(d_plate, 0, plate_bytes, ctx->stream);
      if (cerr != cudaSuccess) {
        cfail = CropFail::Memcpy;
      } else {
        cfail = cropResizeDevice(frame, left, top, width, height, d_content, content_w, content_h,
                                 ctx->stream, &cx0, &cy0, &ccw, &cch);
        if (cfail == CropFail::Ok) {
          cerr = launch_paste_rgba(d_content, content_w, content_h, d_plate, out_w, out_h, pad_x,
                                   pad_y, ctx->stream);
          if (cerr != cudaSuccess)
            cfail = CropFail::Memcpy;
        }
      }
      if (cfail != CropFail::Ok) {
        printf("warp_perspective: unit %zu bbox-fallback failed reason=%s "
               "bbox=(%.1f,%.1f,%.1f,%.1f) frame=%dx%d crop=(%d,%d,%d,%d) warp_fail=%s "
               "letterbox=%dx%d pad=(%d,%d)\n",
               ui, cropFailName(cfail), left, top, width, height, frame.w, frame.h, cx0, cy0, ccw,
               cch, warpFailName(wfail), content_w, content_h, pad_x, pad_y);
        fflush(stdout);
        cudaMemsetAsync(d_plate, 0, plate_bytes, ctx->stream);
      } else if (warpDebugEnabled()) {
        printf("warp_perspective: unit %zu bbox-fallback (gpu) warp_fail=%s letterbox=%dx%d "
               "pad=(%d,%d)\n",
               ui, warpFailName(wfail), content_w, content_h, pad_x, pad_y);
        fflush(stdout);
      }

      if (warpDebugTake()) {
        const size_t mask_bytes = obj ? obj->mask_params.size : 0;
        const void *mask_ptr = obj ? (const void *)obj->mask_params.data : nullptr;
        printf("warp_perspective: DEBUG unit=%zu bi=%d frame=%dx%d warped=0 fail=%s "
               "mask_ptr=%p mask_bytes=%zu min_conf=%.2f "
               "kpt_conf=[%.3f,%.3f,%.3f,%.3f] "
               "net_kpt=[(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f)] "
               "frame_kpt=[(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f)] "
               "parent=%d parent_roi=(%.1f,%.1f,%d,%d) "
               "plate_bbox=(%.1f,%.1f,%.1f,%.1f) native_wh=%dx%d letterbox=%dx%d pad=(%d,%d) "
               "crop_fail=%s\n",
               ui, bi, frame.w, frame.h, warpFailName(wfail), mask_ptr, mask_bytes,
               ctx->keypoint_min_confidence, kpt_conf[0], kpt_conf[1], kpt_conf[2], kpt_conf[3],
               net_kpts[0].x, net_kpts[0].y, net_kpts[1].x, net_kpts[1].y, net_kpts[2].x,
               net_kpts[2].y, net_kpts[3].x, net_kpts[3].y, src[0].x, src[0].y, src[1].x,
               src[1].y, src[2].x, src[2].y, src[3].x, src[3].y, has_parent ? 1 : 0, roi_left,
               roi_top, roi_w, roi_h, left, top, width, height, native_w, native_h, content_w,
               content_h, pad_x, pad_y, cropFailName(cfail));
        fflush(stdout);
      }
    } else if (warpDebugTake()) {
      printf("warp_perspective: DEBUG unit=%zu bi=%d frame=%dx%d warped=1 "
             "kpt_conf=[%.3f,%.3f,%.3f,%.3f] "
             "frame_kpt=[(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f)] "
             "parent=%d parent_roi=(%.1f,%.1f,%d,%d) native_wh=%dx%d letterbox=%dx%d "
             "pad=(%d,%d)\n",
             ui, bi, frame.w, frame.h, kpt_conf[0], kpt_conf[1], kpt_conf[2], kpt_conf[3],
             src[0].x, src[0].y, src[1].x, src[1].y, src[2].x, src[2].y, src[3].x, src[3].y,
             has_parent ? 1 : 0, roi_left, roi_top, roi_w, roi_h, native_w, native_h, content_w,
             content_h, pad_x, pad_y);
      fflush(stdout);
    }

    cerr = launch_rgba_to_nchw_f32(d_plate, out_w, out_h, dst, ctx->pixel_normalization_factor,
                                   ctx->stream);
    if (cerr != cudaSuccess) {
      printf("warp_perspective: rgba_to_nchw failed: %s\n", cudaGetErrorString(cerr));
      fflush(stdout);
      ok = false;
      break;
    }

    if (warped && !ctx->dump_dir.empty()) {
      std::vector<uint8_t> host((size_t)out_w * out_h * 4);
      cerr = cudaMemcpyAsync(host.data(), d_plate, host.size(), cudaMemcpyDeviceToHost, ctx->stream);
      if (cerr == cudaSuccess)
        cerr = cudaStreamSynchronize(ctx->stream);
      if (cerr == cudaSuccess) {
        char path[512];
        std::snprintf(path, sizeof(path), "%s/warp_%05d_u%02zu.ppm", ctx->dump_dir.c_str(),
                      ctx->dump_counter++, ui);
        dumpPpm(path, host.data(), out_w, out_h);
      }
    }
  }

  cudaFree(d_plate);
  cudaFree(d_content);
  for (auto &kv : frame_cache)
    destroyDeviceFrame(kv.second);
  gst_buffer_unmap(batch->inbuf, &inmap);

  if (!ok) {
    acquirer->release(buf);
    return NVDSPREPROCESS_CUDA_ERROR;
  }

  cerr = cudaStreamSynchronize(ctx->stream);
  if (cerr != cudaSuccess) {
    printf("warp_perspective: stream sync failed: %s\n", cudaGetErrorString(cerr));
    fflush(stdout);
    acquirer->release(buf);
    return NVDSPREPROCESS_CUDA_ERROR;
  }

  tensorParam.params.network_input_shape[0] = (int)batch->units.size();
  return NVDSPREPROCESS_SUCCESS;
}

extern "C" CustomCtx *
initLib(CustomInitParams initparams)
{
  auto ctx = std::make_unique<CustomCtx>();
  ctx->initParams = initparams;
  parse_float(initparams.user_configs, NVDSPREPROCESS_USER_CONFIGS_PIXEL_NORMALIZATION_FACTOR,
              ctx->pixel_normalization_factor, ctx->pixel_normalization_factor);
  parse_float(initparams.user_configs, NVDSPREPROCESS_USER_CONFIGS_KEYPOINT_MIN_CONFIDENCE,
              ctx->keypoint_min_confidence, ctx->keypoint_min_confidence);
  parse_int(initparams.user_configs, NVDSPREPROCESS_USER_CONFIGS_MAX_WIDTH, ctx->max_width,
            ctx->max_width);
  parse_int(initparams.user_configs, NVDSPREPROCESS_USER_CONFIGS_MAX_HEIGHT, ctx->max_height,
            ctx->max_height);
  parse_int(initparams.user_configs, NVDSPREPROCESS_USER_CONFIGS_NET_WIDTH, ctx->net_width,
            ctx->net_width);
  parse_int(initparams.user_configs, NVDSPREPROCESS_USER_CONFIGS_NET_HEIGHT, ctx->net_height,
            ctx->net_height);
  auto it = initparams.user_configs.find(NVDSPREPROCESS_USER_CONFIGS_DUMP_DIR);
  if (it != initparams.user_configs.end())
    ctx->dump_dir = it->second;

  if (cudaStreamCreateWithFlags(&ctx->stream, cudaStreamNonBlocking) != cudaSuccess)
    ctx->stream = nullptr;

  printf("warp_perspective: init GPU path norm=%f kpt_min=%f max=%dx%d net=%dx%d dump=%s\n",
         ctx->pixel_normalization_factor, ctx->keypoint_min_confidence, ctx->max_width, ctx->max_height,
         ctx->net_width, ctx->net_height, ctx->dump_dir.empty() ? "(none)" : ctx->dump_dir.c_str());
  fflush(stdout);
  return ctx.release();
}

extern "C" void
deInitLib(CustomCtx *ctx)
{
  if (!ctx)
    return;
  if (ctx->stream)
    cudaStreamDestroy(ctx->stream);
  delete ctx;
}
