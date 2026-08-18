# scripts/

Tiện ích chuyển model và vận hành.

| File | Tác dụng |
|------|----------|
| `export_onnx.py` | `.pt` → `.onnx` theo layout DeepStream `[batch, num_boxes, 6]` = `[x1, y1, x2, y2, score, class_id]`. Tự nhận checkpoint **ultralytics** (YOLOv8/YOLO11) hay **yolov5** (repo gốc). |
| `build_engines.sh` | `.pt`/`.onnx` → TensorRT engine (`trtexec`). Gồm YOLO detect và YOLO-Pose `last_keypoint`. |
| `mem_watch.sh` | Đo RSS/GPU theo thời gian (mặc định 10 phút). `--restart` để restart compose rồi sample. |
| `bench_stages.sh` | Đo FPS từng chặng pipeline (gst-launch). |
| `vendor/yolov5/` | Repo `ultralytics/yolov5` (script tự clone khi cần) — chỉ dùng để nạp checkpoint YOLOv5. |

## Memory watch (leak / growth)

```bash
# Host, cạnh docker-compose.yaml — restart + đo 10 phút + log nội bộ tracks/queue:
VEHICLE_MEM_STATS=1 bash scripts/mem_watch.sh --restart --duration 600 --interval 15

# A/B: bỏ HTTP upload — nếu RAM hết tăng → nghi queue JPEG khi upload timeout
VEHICLE_MEM_STATS=1 VEHICLE_EXTRA_ARGS=--dry-run \
  bash scripts/mem_watch.sh --restart --csv logs/mem_dryrun.csv
```

CSV ghi vào `logs/mem_watch_*.csv`. Cột `queue` / `jpeg_kb` / `queue_jpeg_kb` (trong log
`mem_stats:`) cho biết backlog ảnh đang giữ RAM.

## Build engine

```bash
# trong container test (đã có GPU + trtexec)
docker exec vehicle_test bash -lc 'bash /app/scripts/build_engines.sh'

# hoặc image chuyên dụng (đã cài sẵn pandas/seaborn + repo yolov5)
docker compose -f scripts/docker-compose.export.yaml run --rm vehicle_export
```

Biến môi trường: `FORCE=1` (build lại), `PRECISION=fp32`, `YOLOV5_REPO=<path>`.

Model đang khai báo trong `build_engines.sh`:

| Tên | Weights | imgsz | max batch | Engine |
|-----|---------|-------|-----------|--------|
| `vehicle` | `vehicle_n_best.pt` (YOLO11n) | 640 | 8 | `vehicle_b8_fp16.engine` |
| `last_keypoint` | `last_keypoint.pt` (YOLO-Pose) | 640 | 8 | `last_keypoint_b8_fp16.engine` |
| `digit_n_p3p4_256` | `digit_n_p3p4_256.pt` (YOLO11n) | 256 | 16 | `digit_n_p3p4_256_b16_fp16.engine` |
| `helmet` | `helmet_ylv8_171125.pt` | 640 | 16 | `helmet_b16_fp16.engine` |

SGIE1 plate pose export bằng `scripts/vendor/deepstream_yolo_pose/export_yoloV8_pose.py`.

Engine phụ thuộc GPU + phiên bản TensorRT của máy chạy → **không commit, không đóng
vào image**; build lại trên máy đích rồi mount `resources/` (xem
`docker-compose.yaml`).

## Nhãn

`export_onnx.py` ghi tên class của model ra `resources/weights/<tên>.labels.txt`
để tham chiếu. File nhãn dùng thật khi chạy là `resources/ds/infer/labels_*.txt`:

- `labels_vehicle.txt` — `bus, car, motorbike, truck` (đúng thứ tự model; app
  remap sang chuẩn nghiệp vụ `0=car, 1=motorbike, 2=truck, 3=bus` qua
  `pipeline.pgie.vehicle_class_map` = `[3, 0, 1, 2]`).
- `labels_plate_pose.txt` — `plate` (YOLO-Pose SGIE1).
- `labels_digit.txt` — `0-9` rồi `A-Z`. Model digit trả class id 0..35 nhưng
  checkpoint không kèm tên ký tự; nếu dataset dùng thứ tự khác thì sửa file này.
