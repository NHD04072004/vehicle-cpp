#include "warp_kernels.h"

#include <math.h>

namespace {

__device__ __forceinline__ float clampf(float v, float lo, float hi)
{
  return fminf(hi, fmaxf(lo, v));
}

/**
 * Bilinear sample at floating source coords (OpenCV warpPerspective convention).
 * (DALI Sampler INTERP_LINEAR uses pos-0.5 / pixel-center; we keep OpenCV-compatible
 *  coords so inv(H_src_to_dst) from getPerspectiveTransform matches digit training.)
 */
__device__ void sample_bilinear_rgba(const uint8_t *src, int sw, int sh, int pitch, float sx,
                                    float sy, uint8_t out[4])
{
  float x = sx;
  float y = sy;
  int x0 = (int)floorf(x);
  int y0 = (int)floorf(y);
  float qx = x - (float)x0;
  float qy = y - (float)y0;
  float px = 1.f - qx;

  auto fetch = [&](int ix, int iy, int c) -> float {
    if ((unsigned)ix >= (unsigned)sw || (unsigned)iy >= (unsigned)sh)
      return 0.f;
    return (float)src[iy * pitch + ix * 4 + c];
  };

  for (int c = 0; c < 4; ++c) {
    float s00 = fetch(x0, y0, c);
    float s01 = fetch(x0 + 1, y0, c);
    float s10 = fetch(x0, y0 + 1, c);
    float s11 = fetch(x0 + 1, y0 + 1, c);
    float s0 = s00 * px + s01 * qx;
    float s1 = s10 * px + s11 * qx;
    out[c] = (uint8_t)clampf(s0 + (s1 - s0) * qy + 0.5f, 0.f, 255.f);
  }
}

__global__ void k_warp_perspective_rgba(const uint8_t *src, int sw, int sh, int pitch,
                                        uint8_t *dst, int dw, int dh, float h0, float h1, float h2,
                                        float h3, float h4, float h5, float h6, float h7, float h8)
{
  int x = (int)(blockIdx.x * blockDim.x + threadIdx.x);
  int y = (int)(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= dw || y >= dh)
    return;

  float X = h0 * x + h1 * y + h2;
  float Y = h3 * x + h4 * y + h5;
  float W = h6 * x + h7 * y + h8;
  uint8_t px[4] = {0, 0, 0, 0};
  if (fabsf(W) > 1e-12f) {
    float sx = X / W;
    float sy = Y / W;
    sample_bilinear_rgba(src, sw, sh, pitch, sx, sy, px);
  }
  uint8_t *d = dst + ((size_t)y * dw + x) * 4;
  d[0] = px[0];
  d[1] = px[1];
  d[2] = px[2];
  d[3] = px[3];
}

__global__ void k_rgba_to_nchw_f32(const uint8_t *rgba, int w, int h, float *dst, float scale)
{
  int x = (int)(blockIdx.x * blockDim.x + threadIdx.x);
  int y = (int)(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= w || y >= h)
    return;
  const size_t plane = (size_t)w * h;
  const size_t i = (size_t)y * w + x;
  const uint8_t *p = rgba + i * 4;
  dst[i] = p[0] * scale;
  dst[plane + i] = p[1] * scale;
  dst[2 * plane + i] = p[2] * scale;
}

} // namespace

__global__ void k_crop_resize_rgba(const uint8_t *src, int sw, int sh, int pitch, float left,
                                   float top, float width, float height, uint8_t *dst, int dw,
                                   int dh)
{
  int x = (int)(blockIdx.x * blockDim.x + threadIdx.x);
  int y = (int)(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= dw || y >= dh)
    return;
  float sx = left + (width * ((float)x + 0.5f) / (float)dw) - 0.5f;
  float sy = top + (height * ((float)y + 0.5f) / (float)dh) - 0.5f;
  uint8_t px[4] = {0, 0, 0, 0};
  sample_bilinear_rgba(src, sw, sh, pitch, sx, sy, px);
  uint8_t *d = dst + ((size_t)y * dw + x) * 4;
  d[0] = px[0];
  d[1] = px[1];
  d[2] = px[2];
  d[3] = px[3];
}

extern "C" cudaError_t launch_warp_perspective_rgba(const uint8_t *src_rgba, int src_w, int src_h,
                                                    int src_pitch, uint8_t *dst_rgba, int dst_w,
                                                    int dst_h, const float inv_h[9],
                                                    cudaStream_t stream)
{
  dim3 block(32, 8);
  dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y);
  k_warp_perspective_rgba<<<grid, block, 0, stream>>>(
      src_rgba, src_w, src_h, src_pitch, dst_rgba, dst_w, dst_h, inv_h[0], inv_h[1], inv_h[2],
      inv_h[3], inv_h[4], inv_h[5], inv_h[6], inv_h[7], inv_h[8]);
  return cudaGetLastError();
}

__global__ void k_paste_rgba(const uint8_t *src, int sw, int sh, uint8_t *dst, int dw, int dh,
                             int ox, int oy)
{
  int x = (int)(blockIdx.x * blockDim.x + threadIdx.x);
  int y = (int)(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= sw || y >= sh)
    return;
  int dx = x + ox;
  int dy = y + oy;
  if ((unsigned)dx >= (unsigned)dw || (unsigned)dy >= (unsigned)dh)
    return;
  const uint8_t *s = src + ((size_t)y * sw + x) * 4;
  uint8_t *d = dst + ((size_t)dy * dw + dx) * 4;
  d[0] = s[0];
  d[1] = s[1];
  d[2] = s[2];
  d[3] = s[3];
}

extern "C" cudaError_t launch_crop_resize_rgba(const uint8_t *src_rgba, int src_w, int src_h,
                                               int src_pitch, float src_left, float src_top,
                                               float src_width, float src_height, uint8_t *dst_rgba,
                                               int dst_w, int dst_h, cudaStream_t stream)
{
  if (!src_rgba || !dst_rgba || src_w < 1 || src_h < 1 || dst_w < 1 || dst_h < 1 ||
      !(src_width >= 1.f) || !(src_height >= 1.f))
    return cudaErrorInvalidValue;
  dim3 block(32, 8);
  dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y);
  k_crop_resize_rgba<<<grid, block, 0, stream>>>(src_rgba, src_w, src_h, src_pitch, src_left,
                                                 src_top, src_width, src_height, dst_rgba, dst_w,
                                                 dst_h);
  return cudaGetLastError();
}

extern "C" cudaError_t launch_paste_rgba(const uint8_t *src_rgba, int sw, int sh,
                                         uint8_t *dst_rgba, int dw, int dh, int ox, int oy,
                                         cudaStream_t stream)
{
  if (!src_rgba || !dst_rgba || sw < 1 || sh < 1 || dw < 1 || dh < 1)
    return cudaErrorInvalidValue;
  dim3 block(32, 8);
  dim3 grid((sw + block.x - 1) / block.x, (sh + block.y - 1) / block.y);
  k_paste_rgba<<<grid, block, 0, stream>>>(src_rgba, sw, sh, dst_rgba, dw, dh, ox, oy);
  return cudaGetLastError();
}

extern "C" cudaError_t launch_rgba_to_nchw_f32(const uint8_t *rgba, int w, int h, float *dst_nchw,
                                               float scale, cudaStream_t stream)
{
  dim3 block(32, 8);
  dim3 grid((w + block.x - 1) / block.x, (h + block.y - 1) / block.y);
  k_rgba_to_nchw_f32<<<grid, block, 0, stream>>>(rgba, w, h, dst_nchw, scale);
  return cudaGetLastError();
}
