/*
 * Custom nvdspreprocess library: perspective-warp plate ROIs from YOLO-Pose
 * keypoints stored in NvDsObjectMeta.mask_params (4 x (x,y,conf)).
 */

#ifndef NVDSPREPROCESS_WARP_PERSPECTIVE_H_
#define NVDSPREPROCESS_WARP_PERSPECTIVE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include <cuda_runtime_api.h>

#include "nvdspreprocess_interface.h"

#define NVDSPREPROCESS_USER_CONFIGS_PIXEL_NORMALIZATION_FACTOR "pixel-normalization-factor"
#define NVDSPREPROCESS_USER_CONFIGS_KEYPOINT_MIN_CONFIDENCE "keypoint_min_confidence"
#define NVDSPREPROCESS_USER_CONFIGS_MAX_WIDTH "max_width"
#define NVDSPREPROCESS_USER_CONFIGS_MAX_HEIGHT "max_height"
#define NVDSPREPROCESS_USER_CONFIGS_NET_WIDTH "net_width"
#define NVDSPREPROCESS_USER_CONFIGS_NET_HEIGHT "net_height"
#define NVDSPREPROCESS_USER_CONFIGS_DUMP_DIR "dump_dir"

struct CustomCtx {
  CustomInitParams initParams;
  float pixel_normalization_factor = 0.003921568627f;
  float keypoint_min_confidence = 0.3f;
  int max_width = 400;
  int max_height = 200;
  int net_width = 640;
  int net_height = 640;
  std::string dump_dir;
  int dump_counter = 0;
  cudaStream_t stream = nullptr;
};

extern "C" NvDsPreProcessStatus CustomTransformation(NvBufSurface *in_surf, NvBufSurface *out_surf,
                                                     CustomTransformParams &params);

extern "C" NvDsPreProcessStatus CustomAsyncTransformation(NvBufSurface *in_surf, NvBufSurface *out_surf,
                                                          CustomTransformParams &params);

extern "C" NvDsPreProcessStatus CustomTensorPreparation(CustomCtx *ctx, NvDsPreProcessBatch *batch,
                                                        NvDsPreProcessCustomBuf *&buf,
                                                        CustomTensorParams &tensorParam,
                                                        NvDsPreProcessAcquirer *acquirer);

extern "C" CustomCtx *initLib(CustomInitParams initparams);

extern "C" void deInitLib(CustomCtx *ctx);

#endif /* NVDSPREPROCESS_WARP_PERSPECTIVE_H_ */
