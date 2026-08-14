# nvdspreprocess_custom_warp_perspective

Custom `nvdspreprocess` library: perspective-warp biển số từ 4 keypoints YOLO-Pose
(`tl,tr,br,bl` trong `mask_params`) trước SGIE digit.

## GPU path

```text
NV12 (NvBufSurface GPU)
  → NvBufSurfTransform → RGBA CUDA device   (1 color convert, no host)
  → CUDA warp_perspective (bilinear; OpenCV-compatible coords)
  → CUDA RGBA→NCHW float → nvinfer input-tensor-meta
```

Warp **giữ tỉ lệ** rồi letterbox pad vào `processing-width`×`processing-height`
(256×256) — khớp digit train `maintain-aspect-ratio=1`. Dump PPM chỉ
`cudaMemcpy` ROI 256×256 khi `dump_dir` bật — không kéo full-frame về CPU.

## Build

```bash
docker run --rm --gpus all --entrypoint bash \
  -v "$PWD":/app -w /app/resources/ds/nvdspreprocess_custom_warp_perspective \
  nhd04072004/ds_app:8.0-amd64 -c 'make clean CUDA_VER=12.8 && make CUDA_VER=12.8'
cp resources/ds/nvdspreprocess_custom_warp_perspective/libnvdspreprocess_custom_warp_perspective.so \
   resources/weights/
```

CMake target `nvdspreprocess_custom_warp_perspective` cũng build và copy `.so` vào
`resources/weights/` khi build app.

## Config (production)

- Preprocess: `resources/ds/infer/config_preprocess_warp_plate.txt`
- Plate pose: `resources/ds/infer/sgie1_plate_pose.txt`
- Digit: `resources/ds/infer/sgie2_digit.txt` (`input-tensor-meta=1`)

## Verify

```bash
python3 -u /app/tests/experiments/lpr_pose/dump_prod_cascade.py
```
