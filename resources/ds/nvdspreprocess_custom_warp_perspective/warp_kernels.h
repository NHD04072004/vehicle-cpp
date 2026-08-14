/*
 * GPU warp (DALI-style bilinear, pixel-center) + RGBA→NCHW helpers.
 */
#ifndef WARP_KERNELS_H_
#define WARP_KERNELS_H_

#include <cuda_runtime_api.h>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/** invH: 3x3 row-major, destination→source (same as OpenCV WARP_INVERSE_MAP). */
cudaError_t launch_warp_perspective_rgba(
    const uint8_t *src_rgba, int src_w, int src_h, int src_pitch,
    uint8_t *dst_rgba, int dst_w, int dst_h,
    const float inv_h[9], cudaStream_t stream);

/** Bilinear crop+resize AABB → contiguous RGBA dst (fallback khi không warp được). */
cudaError_t launch_crop_resize_rgba(
    const uint8_t *src_rgba, int src_w, int src_h, int src_pitch,
    float src_left, float src_top, float src_width, float src_height,
    uint8_t *dst_rgba, int dst_w, int dst_h, cudaStream_t stream);

/** Dán ROI RGBA liên tục (sw×sh) vào dst tại (ox,oy). */
cudaError_t launch_paste_rgba(const uint8_t *src_rgba, int sw, int sh, uint8_t *dst_rgba,
                              int dw, int dh, int ox, int oy, cudaStream_t stream);

/** RGBA (pitch = dst_w*4 contiguous) → NCHW float RGB * scale. */
cudaError_t launch_rgba_to_nchw_f32(
    const uint8_t *rgba, int w, int h,
    float *dst_nchw, float scale, cudaStream_t stream);

#ifdef __cplusplus
}
#endif

#endif /* WARP_KERNELS_H_ */
