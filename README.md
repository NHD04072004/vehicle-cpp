# Vehicle AI — DeepStream LPR (PLATE)

Module AI nhận diện phương tiện / biển số trên **NVIDIA DeepStream**, tích hợp Smart VMS qua MQTT (camera list, zones, bbox, event) và HTTP upload snapshot.

Tài liệu chi tiết thêm: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md), [`docs/PIPELINE.md`](docs/PIPELINE.md).

---

## Mục lục

1. [Tổng quan](#1-tổng-quan)
2. [Cấu trúc repo](#2-cấu-trúc-repo)
3. [Pipeline DeepStream](#3-pipeline-deepstream)
4. [Probe](#4-probe)
5. [Logic nghiệp vụ (business)](#5-logic-nghiệp-vụ-business)
6. [Giao tiếp VMS](#6-giao-tiếp-vms)
7. [Build engine](#7-build-engine)
8. [Deploy / chạy production](#8-deploy--chạy-production)
9. [Config chính cần chú ý](#9-config-chính-cần-chú-ý)
10. [Chạy dev / verify](#10-chạy-dev--verify)
11. [Chú ý khi vận hành](#11-chú-ý-khi-vận-hành)

---

## 1. Tổng quan

```text
VMS (MQTT)
  │  camera_list + zones
  ▼
main.cpp
  ├─ pipeline/     DeepStream graph (RTSP → infer → track → sink)
  ├─ probes/       Đọc NvDsBatchMeta, ROI, ghép OCR
  ├─ business/     Vote biển, validate style VN, chốt event
  └─ communication MQTT bbox/event + HTTP upload snapshot
       │
       ▼
Smart VMS / storage
```

| Thành phần | Vai trò |
|------------|---------|
| PGIE | Detect xe (`car` / `motorbike` / `truck`) |
| Tracker | Gán `track_id` ổn định (NvDCF) |
| SGIE1 | Detect biển trên ROI xe |
| SGIE2 | Detect ký tự trên crop biển |
| Probe + business | Zone filter → vote multi-frame → emit event |

Phạm vi hiện tại: **LPR (`AI_MODULE=PLATE`)**. Không line crossing IN/OUT. `payload.direction` luôn `null`.

---

## 2. Cấu trúc repo

```text
vehicle/
├── main.cpp                 # Entry: config, MQTT, pipeline, vòng đời app
├── vehicle.sh               # build | run (cũng là Docker ENTRYPOINT)
├── Dockerfile               # Image runtime (binary + parser .so)
├── docker-compose.yaml      # Production compose
├── src/
│   ├── pipeline/            # Graph DeepStream + dynamic RTSP
│   ├── probes/              # plate_probe, char assembler
│   ├── business/plate/      # Rules, track state, event
│   ├── communication/       # MQTT, HTTP upload, event publisher
│   ├── common/              # Config YAML, DTO, logging
│   └── utils/
├── resources/
│   ├── config/              # config.yaml, mqtt.yaml, restful.yaml
│   ├── ds/                  # infer / preprocess warp / streammux / tracker / custom .so src
│   └── weights/             # .pt → .onnx → .engine (+ custom .so)
├── scripts/                 # export_onnx.py, build_engines.sh, docker-compose.export.yaml
└── tests/                   # business TDD, verify, lpr_pose smoke
```

---

## 3. Pipeline DeepStream

```text
RTSP / file
  → nvurisrcbin (decode)
  → nvstreammux v2          (USE_NEW_NVSTREAMMUX=yes)
  → PGIE nvinfer            vehicle_b8_fp16.engine
  → nvtracker               NvDCF
  → SGIE1 nvinfer           last_keypoint_b8_fp16.engine   (pose, operate-on PGIE)
  → nvdspreprocess          warp perspective (operate-on SGIE1)
  → SGIE2 nvinfer           digit_n_p3p4_256_b16_fp16.engine (input-tensor-meta)
  → pad-probe (plate_probe)
  → sink                    fake | file
```

**Hierarchy object:**

```text
Frame → Vehicle (PGIE + tracker) → Plate pose (SGIE1) → Digits (SGIE2, warped tensor)
```

| Stage | Model / note |
|-------|----------------|
| PGIE | YOLOv5s 640, batch 8, conf ≈ 0.55, class `0=car, 1=motorbike, 2=truck` |
| Tracker | `config_tracker_NvDCF_perf.yml` |
| SGIE1 | YOLO-Pose 640, batch 8, 4 keypoints; `libnvdsinfer_custom_impl_Yolo_pose.so` |
| Preprocess | `libnvdspreprocess_custom_warp_perspective.so` → 256×256 tensor |
| SGIE2 | YOLOv5s 256, 36 class (0–9, A–Z), batch 16; `input-tensor-meta=1` |
| Parser (vehicle/digit) | `libnvdsinfer_custom_impl_Yolo.so` — `NvDsInferParseYolo` |

### Dynamic camera (MQTT)

Control plane = MQTT `camera_list` (không dùng `nvmultiurisrcbin` REST).

| Thao tác | Hành vi |
|----------|---------|
| Add | Camera mới → `nvurisrcbin` + pad `sink_N` ổn định |
| Remove | Gỡ source; **không** đổi `source_id` camera còn lại |
| Update URL | Remove + add lại **cùng** `source_id` |
| Capacity | `pipeline.streammux.batch_size` (mặc định 8); vượt → bỏ camera thừa |
| `--source` | Tắt dynamic — list cố định lúc start (verify / file) |

Thiếu engine tương ứng → stage nvinfer đó bị bỏ (log cảnh báo, passthrough).

---

## 4. Probe

Probe đọc `NvDsBatchMeta` sau tracker/SGIE, lọc zone, ghép OCR, gọi business / communication.

### ROI / polygon

- Điểm neo (anchor): đáy-giữa bbox xe, nhích lên `anchor_bottom_ratio` (mặc định **12%** chiều cao).
- Chỉ OCR khi anchor nằm trong zone `PLATE` (MQTT `get_polygon`).
- Bỏ lines IN/OUT nếu VMS vẫn gửi kèm.

### Ghép ký tự (SGIE2)

1. Bỏ biển quá nhỏ (`min_plate_width`, mặc định 20 px).
2. Dedup box chồng: IoU > `char_iou_dedup` (0.4) → giữ conf cao hơn.
3. Khử nghiêng: hồi quy tuyến tính qua tâm ký tự.
4. Layout 1/2 dòng theo `square_plate_ratio` (residual spread).
5. Ghép trái→phải (và trên→dưới nếu 2 dòng) → đẩy reading theo `track_id`.

### Bbox realtime

Nếu `pipeline.probe.publish_bbox: true` → publish bbox xe (trong polygon) qua MQTT, không bắt buộc đã có text biển.

Knob chính nằm ở `pipeline.probe` trong `config.yaml` (xem [§9](#9-config-chính-cần-chú-ý)).

---

## 5. Logic nghiệp vụ (business)

State theo **`track_id`**:

| Trạng thái | Điều kiện |
|------------|-----------|
| Tạo | Xe vào polygon lần đầu |
| OCR | Trong polygon, chưa chốt, `count < max_recognize_times` |
| Chốt biển | Đủ `max_recognize_times` **hoặc** rời zone + idle > 1s (đã có ≥1 reading) |
| Snapshot | Giữ frame đẹp nhất `(mean_digit_conf, diện_tích_xe)` |
| Cleanup | Idle lâu / đã post event |

### Vote / fuse

- Gom readings cùng độ dài; từng vị trí vote theo số phiếu (hoà → tổng conf).
- Ưu tiên: đúng style VN → số lần đọc → mean conf.
- Validate pattern `plate.plate_style` (`N`=digit, `C`=alpha).
- Sai style → `"{digit}_unk"`; rỗng → `"UNKOWN"`.
- `send_mode=1`: chỉ biển hợp lệ; `send_mode=2`: bắn full (mặc định).
- Dedup: trùng `track_id` hoặc chuỗi biển đã bắn → bỏ.

### Checklist emit

1. Chưa bắn `track_id` / biển  
2. Đã chốt (đủ max hoặc rời zone)  
3. Pass `send_mode`  
4. Có ảnh → upload HTTP → `pub_event`

Map class: `0→Ô TÔ`, `1→XE MÁY`, `2→XE TẢI`.

---

## 6. Giao tiếp VMS

### MQTT (`resources/config/mqtt.yaml`)

| Key | Chiều | Dùng cho |
|-----|-------|----------|
| `camera_list` | Sub | Camera có `ai_modules` chứa `PLATE`, URL `restream_urls.PLATE` |
| `get_polygon` | Sub | Zone ROI |
| `pub_bbox` | Pub | Bbox realtime |
| `pub_event` | Pub | Event biển số |

Chỉnh `host`, `port`, `username`, `password`, `company_id` theo site trước khi deploy.

### HTTP snapshot (`resources/config/restful.yaml`)

`POST {base_url}{endpoint}` — multipart `file` + `camera_id` + `category`:

| category | Ảnh | Field trong event |
|----------|-----|-------------------|
| `vehicle` | full-frame | `snapshot_url` |
| `plate` | crop biển | `snapshot_base64` (= object key, **không** phải base64 thô) |

---

## 7. Build engine

Engine TensorRT **phụ thuộc GPU + version TensorRT** → build trên máy đích, **không** commit / không đóng vào image runtime.

### Quy trình

```text
.pt  →  (export_onnx.py)  →  .onnx  →  (trtexec)  →  *_b{N}_{prec}.engine
```

| Tên | Weights nguồn | imgsz | max batch | Engine |
|-----|---------------|-------|-----------|--------|
| vehicle | `vehicle_n_best.pt` (YOLO11n, 4 class) | 640 | 8 | `vehicle_b8_fp16.engine` |
| digit | `digit_n_p3p4_256.pt` (YOLO11n) | 256 | 16 | `digit_n_p3p4_256_b16_fp16.engine` |
| plate pose | `last_keypoint.pt` (export riêng) | 640 | 8 | `last_keypoint_b8_fp16.engine` |

### Cách chạy

```bash
# Cần GPU + đặt sẵn file .pt trong resources/weights/
docker compose -f scripts/docker-compose.export.yaml run --rm vehicle_export

# Build lại bắt buộc / đổi precision
FORCE=1 docker compose -f scripts/docker-compose.export.yaml run --rm -e FORCE vehicle_export
PRECISION=fp32 FORCE=1 docker compose -f scripts/docker-compose.export.yaml run --rm -e FORCE -e PRECISION vehicle_export
```

Sau khi xong, kiểm tra có:

```text
resources/weights/vehicle_b8_fp16.engine
resources/weights/last_keypoint_b8_fp16.engine
resources/weights/digit_n_p3p4_256_b16_fp16.engine
```

Tên engine phải khớp `model-engine-file` trong `resources/ds/infer/*.txt`. Đổi batch/precision → sửa tên file trong infer config cho khớp.

Chi tiết: [`scripts/README.md`](scripts/README.md).

---

## 8. Deploy / chạy production

### Máy đích cần

- NVIDIA driver + **nvidia-container-toolkit**
- Docker + Docker Compose
- GPU tương thích với file `.engine` (cùng family GPU / cùng TensorRT càng tốt)

### Mang theo gì

**Không cần cả repo.** Bộ tối thiểu:

```text
deploy/
├── docker-compose.yaml
└── resources/
    ├── config/          # mqtt / restful / plate / pipeline
    ├── ds/              # nvinfer, tracker, streammux
    └── weights/         # *.engine + *.labels.txt
```

+ image `nhd04072004/vehicle-app:1.0.0` (`docker pull` hoặc `docker load`).

| Có trong image | Mang riêng (mount) |
|----------------|--------------------|
| Binary `vehicle`, parser `.so`, default config/ds | `config/`, `ds/`, `weights/*.engine` |

Parser `.so` đã có sẵn trong image tại `/app/build/libs/` — host **không** cần mang `.so`.

### Chạy

```bash
# 1) (Nếu GPU khác máy build) build engine tại chỗ — cần .pt + Dockerfile.export
docker compose -f scripts/docker-compose.export.yaml run --rm vehicle_export

# 2) Pull / load image rồi start
docker pull nhd04072004/vehicle-app:1.0.0
docker compose up -d

# Log
docker logs -f vehicle
```

Build image từ source (khi chưa có registry):

```bash
docker compose up -d --build
```

Volumes hiện tại:

```yaml
volumes:
  - ./resources/config:/app/resources/config
  - ./resources/ds:/app/resources/ds
  - ./resources/weights:/app/resources/weights
```

Đổi config / engine trên host → `docker compose restart` (không cần rebuild image, trừ khi đổi code C++).

---

## 9. Config chính cần chú ý

### `resources/config/config.yaml`

| Key | Ý nghĩa |
|-----|---------|
| `AI_MODULE` | `PLATE` |
| `plate.send_mode` | `1` = chỉ biển hợp lệ; `2` = full |
| `plate.max_recognize_times` | Số lần OCR trước khi chốt (vd. 20) |
| `plate.plate_style` | Pattern biển VN |
| `pipeline.rtsp.*` | TCP (`select_rtp_protocol: 4`), reconnect, latency |
| `pipeline.streammux.batch_size` | Capacity camera tối đa (= PGIE batch) |
| `pipeline.pgie.vehicle_class_map` | Remap class model → chuẩn nghiệp vụ (`[3, 0, 1, 2]`) |
| `pipeline.sink.type` | Prod: `fake`; debug video: `file` |
| `pipeline.probe.anchor_bottom_ratio` | Neo ROI |
| `pipeline.probe.publish_bbox` | Bật/tắt MQTT bbox |

### `resources/config/mqtt.yaml` / `restful.yaml`

- Broker, credential, `company_id`
- `snapshot_api.base_url` / `endpoint` đúng máy storage

### `resources/ds/infer/*.txt`

| File | Chú ý |
|------|--------|
| `pgie_vehicle.txt` | `model-engine-file`, `batch-size`, conf (`pre-cluster-threshold`) |
| `sgie1_plate_pose.txt` | YOLO-Pose keypoint, `operate-on-gie-id=1` |
| `config_preprocess_warp_plate.txt` | Warp 4 keypoints → tensor digit |
| `sgie2_digit.txt` | Engine digit, `input-tensor-meta=1` khi có warp |
| `custom-lib-path` | `.so` trong `resources/weights/` (Yolo / Yolo_pose / warp) |

**`batch-size` trong nvinfer phải khớp tên engine** (`vehicle_b8_…` ↔ batch 8).

Env bắt buộc: `USE_NEW_NVSTREAMMUX=yes` (đã set trong compose / `vehicle.sh`).

---

## 10. Chạy dev / verify

### CLI

```bash
# Build trong container / máy có toolchain
bash vehicle.sh build
bash vehicle.sh build -DBUILD_TESTS=OFF

# Camera từ MQTT
bash vehicle.sh run
# hoặc tắt:
bash vehicle.sh

# File / RTSP cố định, không publish
bash vehicle.sh run --source tests/data/test.mp4 --sink file --dry-run

# Ghi đè số lần OCR
bash vehicle.sh run --max-recognize 5 --dry-run
```

| Cờ (truyền cho binary) | Ý nghĩa |
|------------------------|---------|
| `--root` | Thư mục chứa `resources/` (script tự truyền) |
| `--source` | RTSP/file; tắt dynamic MQTT camera list |
| `--sink fake\|file` | Ghi đè sink |
| `--dry-run` | Không upload/publish, chỉ log payload |
| `--max-recognize` | Ghi đè `plate.max_recognize_times` |
| `--camera-timeout` | Chờ `camera_list` (mặc định 15s) |

### Test

```bash
bash tests/business/run.sh          # TDD business (Python ref)
bash tests/business_cpp/run.sh      # Parity C++
bash tests/verify/run.sh            # Golden fixture vs pipeline
```

---

## 11. Chú ý khi vận hành

1. **Engine không portable** — đổi GPU architecture hoặc TensorRT version → build lại engine trên máy đó.
2. **Đồng bộ batch** — `streammux.batch_size`, PGIE `batch-size`, và tên `*_b{N}_*.engine` phải khớp.
3. **Mount đè image** — `config` / `ds` / `weights` trên host thắng bản trong image; thiếu engine trên host → stage infer tắt.
4. **`.so` không commit** — image giữ parser; entrypoint sync vào `weights/` lúc start.
5. **MQTT trước camera** — thiếu `camera_list` / zone đúng `company_id` → không có source hoặc không OCR.
6. **RTSP** — mặc định TCP; kiểm tra restream URL `restream_urls.PLATE` từ VMS.
7. **`network_mode: host`** — cần để RTSP/MQTT đơn giản trên LAN; firewall/host networking theo site.
8. **GPU visibility** — `NVIDIA_VISIBLE_DEVICES`, `NVIDIA_DRIVER_CAPABILITIES=compute,utility,video`.
9. **Class vehicle** — model trả `0=bus, 1=car, 2=motobike, 3=truck`; app remap sang chuẩn nghiệp vụ `0=car, 1=motorbike, 2=truck, 3=bus` bằng `vehicle_class_map`. Đổi model phải sửa lại map.
10. **Không commit secret** — đổi password MQTT / URL nội bộ theo môi trường; không đẩy credential thật lên git nếu repo public.
11. **Thiếu model** — app vẫn chạy nhưng bỏ nvinfer tương ứng (passthrough) — kiểm tra log khi deploy lần đầu.
12. **Debug** — `--dry-run` + `--sink file` + `VEHICLE_LOG_LEVEL=debug` để soi payload / video trước khi nối VMS thật.

---

## License / liên hệ

Nội bộ dự án Smart VMS — module `vehicle` (PLATE).
