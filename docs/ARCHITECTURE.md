# Architecture — Vehicle AI Module

Module AI nhận diện phương tiện / biển số, chạy trên NVIDIA DeepStream, tích hợp Smart VMS qua MQTT và API upload snapshot.

## Cây thư mục

```text
vehicle/
├── vehicle.sh                    # build | run (Docker ENTRYPOINT)
├── CMakeLists.txt                # Build system (CMake)
├── Dockerfile                    # Image runtime cho app
├── docker-compose.yaml           # Compose môi trường (dev/prod tùy service)
├── docs/
│   ├── ARCHITECTURE.md           # Tài liệu kiến trúc (file này)
│   ├── PIPELINE.md               # Tài liệu pipeline GStreamer/DeepStream
│   └── license_plate_recognition.md
├── src/
│   ├── business/
│   │   ├── plate/                # rules, track, event, plate_recognizer (PLATE domain)
│   │   └── violation/            # constants, config_store (mã vi phạm VMS cho phép)
│   ├── common/                   # config (yaml-cpp), types (DTO), logging
│   ├── communication/            # mqtt_client, vms_client, http_uploader, event_publisher
│   ├── pipeline/                 # pipeline.{h,cpp} — dựng graph DeepStream
│   ├── probes/                   # plate_probe (NvDsBatchMeta, ROI), char_assembler
│   └── utils/                    # geometry, jpeg_draw, plate_snapshot_warp, …
├── resources/
│   ├── config/
│   │   ├── config.yaml           # AI module, plate.* (send_mode, styles, …), pipeline.*
│   │   ├── mqtt.yaml             # Broker MQTT + topic Smart VMS
│   │   └── restful.yaml          # snapshot_api (base_url, endpoint)
│   ├── ds/
│   │   ├── infer/                # pgie_vehicle / sgie1_plate_pose / warp / sgie2_digit / sgie3_helmet + labels
│   │   ├── streammux/            # config_mux.txt (nvstreammux v2)
│   │   ├── tracker/              # NvDCF ll-config + tracker_config.txt
│   │   ├── nvdsinfer_custom_impl_Yolo/       # bbox parser (vehicle/digit)
│   │   ├── nvdsinfer_custom_impl_Yolo_pose/  # pose parser (plate)
│   │   └── nvdspreprocess_custom_warp_perspective/  # warp plate → digit tensor
│   └── weights/                  # .pt → .onnx → .engine + custom .so
├── scripts/
│   ├── export_onnx.py            # .pt (ultralytics/yolov5) → ONNX layout DeepStream
│   ├── build_engines.sh          # ONNX → TensorRT engine (trtexec)
│   └── README.md
└── tests/
    ├── data/
    │   ├── test.mp4              # Video nguồn verify (đã kiểm chứng)
    │   ├── test_result.mp4       # Video kết quả OSD (đã kiểm chứng)
    │   ├── test_plates.json      # Ground truth plates / per-frame
    │   ├── plate/bien.jpg        # Ảnh crop biển số (proto upload)
    │   └── vehicle/xe.jpg        # Ảnh full-frame xe (proto upload)
    ├── business/                 # TDD unit test logic src/business/plate (Python ref)
    │   ├── plate_rules/          # Reference thuần ↔ C++ plate/{rules,track,event}
    │   ├── test_*.py
    │   └── requirements.txt
    ├── business_cpp/             # Parity test C++ (khoá src/business/plate ≡ plate_rules)
    │   ├── test_business.cpp
    │   └── run.sh
    ├── verify/                   # Verify business + pipeline vs golden media fixture
    │   ├── test_golden_plates.py
    │   ├── eval_pipeline.py      # Chạy pipeline thật trên test.mp4 → đối chiếu JSON
    │   └── run.sh
    └── test_proto/               # Prototype / smoke test giao thức
        ├── test_sub_mqtt_cameras.py
        ├── test_sub_mqtt_polygon.py
        ├── test_pub_mqtt_bbox.py
        ├── test_pub_mqtt_event.py
        └── test_api_upload.py
```




## Tác dụng từng phần



### Root — build & chạy


| Path                       | Tác dụng                                                                                                           |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `main.cpp`                 | Điểm vào chương trình: khởi tạo config, MQTT, pipeline DeepStream, vòng đời app.                                   |
| `CMakeLists.txt`           | Khai báo target, phụ thuộc (GStreamer, DeepStream, OpenCV, MQTT, …), cách link/build.                              |
| `vehicle.sh`               | `build` / `run` — biên dịch cmake và chạy binary; cũng là Docker ENTRYPOINT (sync `.so`). |
| `Dockerfile`               | Đóng gói môi trường runtime (DeepStream + app + custom `.so`).                                                     |
| `docker-compose.yaml`      | Dev/prod: mount repo / resources, GPU NVIDIA.                                                                     |
| `scripts/Dockerfile.export` | Toolchain convert model: torch/ultralytics + yolov5 + trtexec.                                                   |
| `scripts/docker-compose.export.yaml` | Chạy `scripts/build_engines.sh` trong image toolchain (cần GPU).                                          |




### `docs/`


| Path              | Tác dụng                                                              |
| ----------------- | --------------------------------------------------------------------- |
| `ARCHITECTURE.md` | Mô tả cấu trúc thư mục và vai trò từng thành phần.                    |
| `PIPELINE.md`     | Mô tả chi tiết pipeline video (nguồn → infer → track → probe → sink). |




### `src/` — mã nguồn ứng dụng


| Thư mục          | Tác dụng                                                                                                         |
| ---------------- | ---------------------------------------------------------------------------------------------------------------- |
| `business/`      | Nghiệp vụ domain theo module AI. `plate/` (chốt biển số, validate style, lọc zone, payload event) và `violation/` (mã vi phạm camera được bật, port `gsan/violation` của `phatnguoi_mbf`). |
| `common/`        | Shared nội bộ: config loader, struct/DTO, enum, hằng số — thay cho `include/` vì app không expose API ra ngoài.   |
| `communication/` | Giao tiếp ngoài: subscribe camera list / zones / violations, publish bbox & AI events qua MQTT; upload snapshot qua HTTP API. |
| `pipeline/`      | Ghép và điều khiển GStreamer/DeepStream pipeline (mux, nvinfer, tracker, OSD, …).                                |
| `probes/`        | Callback pad-probe đọc `NvDsBatchMeta`, lấy bbox/object, ROI check, đẩy dữ liệu sang business/communication.     |
| `utils/`         | Helper nhỏ (geometry, time, string, file, …).                                                                    |


> Không dùng `include/`: project là binary app, không phải thư viện public. Header dùng chung đặt trong `src/common/` (hoặc cạnh `.cpp` của từng module).

**Chia target khi build (CMake):**

| Target                  | Nguồn                                              | Phụ thuộc                           |
| ----------------------- | -------------------------------------------------- | ----------------------------------- |
| `vehicle_business`      | `common/`, `utils/`, `business/plate/`, `business/violation/`, `char_assembler` | yaml-cpp, jsoncpp (không có GStreamer) |
| `vehicle`               | `main.cpp`, `communication/`, `probes/`, `pipeline/` | + GStreamer, DeepStream, libcurl    |
| `vehicle_business_tests`| `tests/business_cpp/`                              | `vehicle_business`                  |

Logic thuần tách khỏi DeepStream để test chạy được không cần GPU/model.

```bash
docker exec vehicle_test bash -lc 'bash /app/scripts/build_engines.sh'   # 1 lần: .pt → engine
docker exec vehicle_test bash -lc 'bash /app/vehicle.sh build'            # app + parser + test
docker exec vehicle_test bash -lc 'bash /app/vehicle.sh run'              # camera từ MQTT
docker exec vehicle_test bash -lc 'bash /app/vehicle.sh run --source /app/tests/data/test.mp4 --sink file --dry-run'
```

Cờ CLI hữu ích: `--dry-run` (không upload/publish, chỉ log payload),
`--max-recognize <n>`, `--sink fake|osd|file`.
Env bắt buộc cho mux v2: `USE_NEW_NVSTREAMMUX=yes` (đã set trong `vehicle.sh` / compose).



### `resources/` — tài nguyên runtime


| Path | Tác dụng |
|------|----------|
| `config/config.yaml` | `AI_MODULE`, `plate.*`, `violation.*` (helmet), `pipeline.*` lồng nhau: `rtsp` / `streammux` (v2) / `pgie` / `sgie_*` / `tracker` / `sink` / `probe`. |
| `config/mqtt.yaml` | Broker MQTT + `company_id` / `ai_modules` + topic keys. |
| `config/restful.yaml` | `snapshot_api`: `base_url`, `endpoint` (upload snapshot). |
| `ds/infer/` | Config nvinfer/preprocess (`pgie_vehicle`, `sgie1_plate_pose`, warp, `sgie2_digit`, `sgie3_helmet`) + labels. |
| `ds/streammux/` | `config_mux.txt` cho nvstreammux v2 (`USE_NEW_NVSTREAMMUX=yes`). |
| `ds/tracker/` | `config_tracker_NvDCF_perf.yml` (ll-config) + `tracker_config.txt`. |
| `ds/nvdsinfer_custom_impl_Yolo/` | Custom YOLO bbox parser → `build/libs/libnvdsinfer_custom_impl_Yolo.so`. |
| `ds/nvdsinfer_custom_impl_Yolo_pose/` | YOLO-Pose parser → `build/libs/libnvdsinfer_custom_impl_Yolo_pose.so`. |
| `ds/nvdspreprocess_custom_warp_perspective/` | Warp biển → `build/libs/libnvdspreprocess_custom_warp_perspective.so`. |
| `weights/` | `*.pt` → `*.onnx` → `*_b{N}_{prec}.engine`. **Thiếu model → app tự bỏ stage nvinfer tương ứng** (log cảnh báo, chạy passthrough). |

| File trong `weights/` | Vai trò |
|-----------------------|---------|
| `vehicle_n_best.pt` → `vehicle.onnx` / `vehicle_b8_fp16.engine` | PGIE detect xe (YOLO11n 640; class model 0=bus, 1=car, 2=motobike, 3=truck) |
| `last_keypoint.pt` → `last_keypoint.onnx` / `last_keypoint_b8_fp16.engine` | SGIE1 detect biển + 4 keypoints (YOLO-Pose) |
| `digit_n_p3p4_256.pt` → `digit_n_p3p4_256.onnx` / `digit_n_p3p4_256_b16_fp16.engine` | SGIE2 OCR ký tự (YOLO11n, 256, 36 class) |
| `helmet_ylv8_171125.pt` → `helmet.onnx` / `helmet_b16_fp16.engine` | SGIE3 mũ bảo hiểm (YOLOv8s, 640, 3 class: 0=không mũ, 1=có mũ, 2=khác) |

Parser `.so` (build từ `ds/nvdsinfer_custom_impl_Yolo*`, `ds/nvdspreprocess_custom_warp_perspective`)
build vào `build/libs/`, không nằm trong `weights/`:

| File trong `build/libs/` | Vai trò |
|---------------------------|---------|
| `libnvdsinfer_custom_impl_Yolo.so` | Parser bbox vehicle/digit (`NvDsInferParseYolo`) |
| `libnvdsinfer_custom_impl_Yolo_pose.so` | Parser pose plate (`NvDsInferParseYoloPose`) |
| `libnvdspreprocess_custom_warp_perspective.so` | Warp keypoints → tensor cho digit |

Engine phụ thuộc GPU + TensorRT của máy chạy → build tại chỗ bằng
`bash scripts/build_engines.sh`, không commit và không đóng vào image.


**Topic MQTT (theo** `mqtt.yaml`**):**


| Key           | Pattern                                  | Chiều                                       |
| ------------- | ---------------------------------------- | ------------------------------------------- |
| `camera_list` | `smart_vms/cameras/company/{company_id}` | Subscribe — danh sách camera                |
| `get_polygon` | `smart_vms/cameras/{camera_code}/zones`  | Subscribe — vùng/line ROI                   |
| `pub_bbox`    | `smart_vms/ai/bbox/{camera_code}`        | Publish — bbox realtime                     |
| `pub_event`   | `smart_vms/ai_events/{ai_modules}`       | Publish — sự kiện AI (biển số + metadata xe) |




### `scripts/`

Script vận hành phụ (build engine, copy artifact, tiện ích deploy). Build/run app dùng `vehicle.sh` ở root.

### `tests/` — kiểm thử

| Path | Tác dụng |
|------|----------|
| `data/test.mp4` | Video nguồn **đã kiểm chứng** — input verify pipeline. |
| `data/test_result.mp4` | Video kết quả OSD **đã kiểm chứng** — tham chiếu visualize. |
| `data/test_plates.json` | Ground truth: plates theo track + `per_frame` objects/bbox. |
| `verify/` | Verify business rules khớp golden JSON/media (`bash tests/verify/run.sh`). |
| `business/` | **TDD unit test** cho logic `src/business/plate/` (rules, track, event). |
| `business_cpp/` | Bản C++ của cùng bộ case → khoá parity `src/business/plate/` ≡ `plate_rules/`. |
| `data/vehicle/xe.jpg` | Ảnh full-frame → upload proto `vehicle`. |
| `data/plate/bien.jpg` | Ảnh crop biển → upload proto `plate`. |
| `test_proto/` | Prototype / smoke schema MQTT & API. |

```bash
# TDD business (Python reference)
docker exec vehicle_test bash -lc 'bash /app/tests/business/run.sh'

# Parity C++ (cùng bộ case, chạy trên src/business/plate)
docker exec vehicle_test bash -lc 'bash /app/tests/business_cpp/run.sh'

# Verify golden fixture (in từng track)
docker exec vehicle_test bash -lc 'bash /app/tests/verify/run.sh'
```





## Luồng dữ liệu tổng quan

```text
VMS (MQTT)
  │  camera list + zones
  ▼
vehicle app ──► DeepStream pipeline ──► probes ──► business
  │                                              │
  │◄──────── MQTT: bbox / ai_events ─────────────┤
  │◄──────── HTTP: upload snapshot ──────────────┘
  ▼
Smart VMS / MinIO
```

1. App subscribe danh sách camera và polygon theo `ai_modules`.
2. Pipeline infer + track trên stream camera.
3. Probe đọc metadata → business chốt biển số → tạo event.
4. Publish bbox realtime; upload snapshot; publish AI event kèm object key ảnh.

