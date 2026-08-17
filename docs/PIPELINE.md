# Pipeline — Nhận diện biển số (PLATE)

Bổ sung chi tiết cho [ARCHITECTURE.md](./ARCHITECTURE.md): luồng DeepStream, probe, nghiệp vụ chốt biển số, và hợp đồng MQTT/HTTP với Smart VMS.

Phạm vi: chỉ LPR (`ai_modules = PLATE`). Không line crossing IN/OUT. Logic tham chiếu từ `phatnguoi_mbf`, triển khai theo cây `src/` của project này.

---

## 1. Vị trí trong kiến trúc

Luồng tổng quan (lặp lại từ ARCHITECTURE):

```text
VMS (MQTT)
  │  camera_list + get_polygon
  ▼
main.cpp
  ├─ src/common          ← config, DTO, hằng số
  ├─ src/communication   ← MQTT / HTTP
  ├─ src/pipeline        ← GStreamer / DeepStream graph
  ├─ src/probes          ← NvDsBatchMeta, ROI, đẩy sang business
  ├─ src/business/plate  ← vote OCR, validate, chốt event
  └─ src/utils           ← geometry, crop, time, …
       │
       ▼
Smart VMS / MinIO  ← pub_bbox, pub_event, upload snapshot
```

| Module (`src/`) | Vai trò trong pipeline PLATE |
|-----------------|------------------------------|
| `pipeline/` | Source RTSP → mux → PGIE xe → tracker → SGIE1 biển → SGIE2 ký tự → (OSD) → sink |
| `probes/` | Đọc metadata; filter polygon; gắn plate/digit vào track; gọi business / communication |
| `business/` | State theo `track_id`; multi-frame vote; validate style; quyết định emit |
| `communication/` | Sub `camera_list` / `get_polygon`; pub `pub_bbox` / `pub_event`; HTTP upload |
| `common/` | Load `resources/config/*.yaml`; struct camera/zone/detection/event |
| `utils/` | Anchor point, expand ROI, IoU, rotate crop, JPEG encode |

Weights: `resources/weights/` (vehicle / plate / digit engines).

---

## 2. Chuỗi xử lý

```text
RTSP (restream_urls.PLATE)
  → src/pipeline: decode / nvstreammux
  → PGIE nvinfer — detect xe (car / motorbike / truck)
  → nvtracker — track_id
  → SGIE1 nvinfer — detect biển trên ROI xe (expand 30%)
  → SGIE2 nvinfer — detect ký tự trên crop biển
  → SGIE3 nvinfer — detect mũ bảo hiểm, CHỈ trên bbox xe máy
  → src/probes — ROI polygon + thu thập OCR / vi phạm theo track
  → src/business/plate — vote n lần → validate → payload
  → src/business/violation — lọc mã vi phạm VMS cho phép theo camera
  → src/communication — upload + MQTT bbox/event
```

**Hierarchy object DeepStream:**

```text
Frame → Vehicle (PGIE + tracker) → Plate (SGIE1) → Digits (SGIE2)
                                 → Helmet (SGIE3, chỉ xe máy)
```

**Nguyên tắc emit:** OCR liên tục trong polygon đến khi ra khỏi zone (cấp tối đa `max_recognize_times`, mặc định 5); chọn biển đẹp nhất + snapshot đẹp nhất; qua `send_mode` / dedup. Event xe: `payload.direction` luôn `null`. Event vi phạm: `direction = "IN"`.

---

## 3. `src/pipeline` — graph DeepStream

| Stage | Element | Model / tham số thực tế |
|-------|---------|-------------------------|
| Source | `nvurisrcbin` (RTSP) | URL từ camera MQTT: `restream_urls.PLATE`; file → `--source`. Runtime add/remove/update qua `applyCameraList`. |
| Batch | `nvstreammux` v2 (`USE_NEW_NVSTREAMMUX=yes`) | `pipeline.streammux.batch_size` = **capacity** tối đa (mặc định 8); `source_id` = slot pad `sink_N` ổn định khi add/remove |
| Detect xe | PGIE `nvinfer` | `vehicle_n_best.pt` (YOLO11n 640), 4 class model `0=bus`, `1=car`, `2=motobike`, `3=truck`; conf 0.55 |
| Track | `nvtracker` | NvDCF (`resources/ds/tracker/config_tracker_NvDCF_perf.yml`), `probationAge: 0` |
| Detect biển | SGIE1 `nvinfer` | `last_keypoint` YOLO-Pose 640, 4 keypoints; `sgie1_plate_pose.txt`; conf 0.25 |
| Warp biển | `nvdspreprocess` | `config_preprocess_warp_plate.txt` → `libnvdspreprocess_custom_warp_perspective.so` |
| OCR | SGIE2 `nvinfer` | `digit_n_p3p4_256.pt` (YOLO11n 256, 36 class), `input-tensor-meta=1`; conf 0.25 |
| Mũ bảo hiểm | SGIE3 `nvinfer` | `helmet_ylv8_171125.pt` (YOLOv8s 640, 3 class), `sgie3_helmet.txt`; `operate-on-class-ids=2` (chỉ xe máy); conf 0.85 |
| OSD / sink | `nvvideoconvert` → `nvdsosd` (tắt vẽ) → `fakesink` / file | Full-frame JPEG rồi **vẽ 1 bbox xanh lúc emit** trên đúng track. |

> **SGIE3 phải là stage nvinfer cuối cùng.** `attachRoiExpandProbe` nới bbox xe 30 %
> trên sink của SGIE1 và `attachRoiRestoreProbe` trả bbox gốc trên **src của SGIE2**.
> Nếu SGIE3 nằm giữa hai mốc đó, nó sẽ crop ROI đã phình thay vì bbox xe thật —
> khác với `phatnguoi_mbf` (chạy helmet trên đúng crop xe).

**Parser:** PGIE/digit dùng `NvDsInferParseYolo`
(`resources/ds/nvdsinfer_custom_impl_Yolo/` → `libnvdsinfer_custom_impl_Yolo.so`).
SGIE1 pose dùng `NvDsInferParseYoloPose`
(`resources/ds/nvdsinfer_custom_impl_Yolo_pose/` → `libnvdsinfer_custom_impl_Yolo_pose.so`).
Vehicle/digit ONNX từ `scripts/export_onnx.py`; keypoint từ
`scripts/vendor/deepstream_yolo_pose/`. Xem `scripts/README.md`.

> Model `vehicle_n_best.pt` trả class id theo thứ tự `0=bus, 1=car, 2=motobike,
> 3=truck` (đọc từ `names` trong checkpoint). Chuẩn nghiệp vụ của app là
> `0=car, 1=motorbike, 2=truck, 3=bus` → `pipeline.pgie.vehicle_class_map`
> để `[3, 0, 1, 2]`. Đổi model thì sửa lại map này cho khớp.

Pad đáy bbox xe sau detect (tham chiếu cũ `y2 += 10`) — cân nhắc trong parser/probe để khỏi cắt biển.

### 3.1 Dynamic RTSP (add / remove / update / reconnect)

Control plane vẫn là MQTT `camera_list` (không dùng REST `nvmultiurisrcbin`).

| Thao tác | Hành vi |
|----------|---------|
| Add | Camera mới trong list → `nvurisrcbin` + pad `sink_N` (N lấy từ free-list) → `bindCamera` |
| Remove | Camera khỏi list → `NULL` + `release_request_pad` → `unbindCamera`; **không** đổi `source_id` camera còn lại |
| Update URL | Đổi `restream_urls.*` → remove + add lại **cùng** `source_id` |
| Reconnect | `rtsp-reconnect-interval` / `rtsp-reconnect-attempts`; `GST_MESSAGE_ERROR` từ `source_*` chỉ log, không quit |
| Capacity | `pipeline.streammux.batch_size` (mặc định 8, khớp PGIE). Vượt → warn, bỏ camera thừa |
| `--source` | Tắt dynamic callback — list cố định lúc start |

Sink luôn `async=false` để tránh deadlock khi thêm source lúc `PLAYING`.

---

## 4. `src/probes` — metadata & ROI

Probe đọc `NvDsBatchMeta` sau tracker / SGIE, đẩy sang `business` và (tuỳ chọn) `communication` cho bbox realtime.

### 4.1 Anchor & polygon

- Điểm test: đáy-giữa bbox, nhích lên **12%** chiều cao (`utils` geometry).
- Chỉ xử lý OCR khi anchor nằm trong zone `PLATE` (từ MQTT `get_polygon`).
- Cache polygon theo `(W, H, version)`; invalidate khi VMS cập nhật zone.
- Bỏ qua lines IN/OUT nếu payload zones vẫn gửi kèm.

### 4.2 SGIE1 — biển trên ROI xe

1. Expand bbox xe **30%** (15%/cạnh) trước secondary infer.
2. Chọn plate **conf cao nhất** nếu nhiều box.
3. Remap toạ độ về crop xe gốc (bù offset expand).

### 4.3 SGIE2 — ký tự + layout (logic post-infer trong probe/utils)

1. Bỏ crop biển quá nhỏ (`w < 20`).
2. Dedup box chồng: IoU `> 0.4` giữ conf cao hơn.
3. Khử nghiêng: hồi quy tuyến tính qua tâm ký tự (`y = a·x + b`), lấy phần dư.
4. Layout: `residual_spread / avg_h > 0.5` → **2 dòng**, else **1 dòng**.
   (So phần dư thay vì `y_spread` thô: biển 1 dòng chụp xiên có `y_spread` lớn
   nhưng vẫn bám 1 đường thẳng — dùng `y_spread` sẽ tách nhầm thành 2 dòng và
   đảo thứ tự ký tự.)
5. Ghép trái→phải (và trên→dưới nếu 2 dòng); đẩy reading vào `business` theo `track_id`.

### 4.4 Publish bbox (qua communication)

Có thể lấy detection sau PGIE/tracker (trong polygon) để pub realtime — không bắt buộc đã có plate text.

---

## 5. `src/business/plate` — chốt biển & event

### 5.1 State theo `track_id`

| Trạng thái | Điều kiện |
|------------|-----------|
| Tạo object | Xe vào polygon lần đầu |
| OCR | Trong polygon, chưa `has_final_plate`, `count < max_recognize_times` |
| Chốt biển | Đủ `max_recognize_times` **hoặc** rời zone + idle > 1s (đã có ≥1 reading) |
| Snapshot | Mỗi chuỗi biển giữ mẫu tốt nhất `(mean_digit_conf, diện_tích_xe)`; lúc chốt ưu tiên khớp biển final |
| Class | Vote `(cls, conf)`: nhiều phiếu → tổng conf |
| Miss track | Idle > 1s khỏi plate zone → retry push nếu đã chốt |
| Force delete | Idle > 10s hoặc age > 120s |
| Cleanup | Đã post event |

Mỗi lần OCR có ký tự: tăng `plate_recognize_count`, lưu `list_plate_chars` / `list_plate_number`.

### 5.2 Fuse / vote

Khi chốt (`count >= max_recognize_times` hoặc rời zone, mặc định max = **5**):

- Gom readings cùng độ dài; từng vị trí vote theo số phiếu, hoà thì tổng conf.
- Ưu tiên: valid style → số lần đọc → mean conf.
- Fallback: most-frequent trên `list_plate_number` (ưu tiên len > 5, rồi khớp style).
- `has_final_plate = true` → chuẩn bị emit **một lần**.
- Snapshot gửi kèm = frame đẹp nhất trong các lần OCR (không nhất thiết frame chốt).

### 5.3 Validate style VN

Patterns (`N`=digit, `C`=alpha) từ `resources/config/config.yaml` → `plate.plate_style`:

```text
NNCNNNNN, NNCNNNNNN, CCNNNN, NNCNNNN, NNCCNNNN,
NNNNNCC, NNNNNCN, NNNNNCCNN, NNCCNNNNN, NNCCNNNNNN
```

Rule thêm:

- 2 ký tự đầu ∈ `DIGIT_CAR` (`00–10`, `44–46`, `87`, `91`, `96`) → invalid.
- 2 ký tự đầu là chữ → phải ∈ `ALPHA_ARMY`.

| Kết quả | Xử lý |
|---------|--------|
| empty | → `"UNKOWN"` |
| sai style | → `"{digit}_unk"` |
| `send_mode == 1` | bỏ `UNKOWN` / `*_unk` |
| `send_mode == 2` | bắn full (mặc định) |

Dedup cache (≈50): trùng `track_id` hoặc trùng chuỗi biển đã bắn → bỏ.

### 5.4 Checklist emit

1. `track_id` chưa bắn  
2. Biển chưa trong plate cache  
3. Đã chốt biển (đủ max **hoặc** rời zone) / miss-track retry  
4. Pass `send_mode`  
5. Có ảnh để upload  
6. `communication` upload + `pub_event` (xe: `direction=null`; vi phạm: `direction=IN`)

### 5.5 Attribute phụ (phase sau)

Màu nền biển / brand / color / seat — không bắt buộc phase 1.

---

## 6. `src/communication` — MQTT & HTTP

Topic keys theo `resources/config/mqtt.yaml` (ARCHITECTURE):

| Key | Pattern | Chiều | Dùng cho PLATE |
|-----|---------|-------|----------------|
| `camera_list` | `smart_vms/cameras/company/{company_id}` | Sub | Lọc camera `ai_modules` chứa `PLATE`; lấy `restream_urls.PLATE` |
| `get_polygon` | `smart_vms/cameras/{camera_code}/zones` | Sub | Chỉ lấy **zones** ROI; bỏ lines |
| `pub_bbox` | `smart_vms/ai/bbox/{camera_code}` | Pub | Bbox realtime |
| `pub_event` | `smart_vms/ai_events/{ai_modules}` | Pub | Event biển số (`PLATE`) |

### 6.1 Bbox payload

```json
{
  "camera_code": "cauhoabinh",
  "ai_modules": "PLATE",
  "timestamp": 1754200000.0,
  "detections": [
    {
      "id": "vehicle_1",
      "class": "car",
      "confidence": 0.94,
      "bbox": [0.38, 0.22, 0.58, 0.68],
      "label": "car 30A12345",
      "color": "#00FF00"
    }
  ]
}
```

`bbox` = `[x1, y1, x2, y2]` chuẩn hoá theo frame.

### 6.2 Upload snapshot

Config: `resources/config/restful.yaml` → `snapshot_api`

`POST {base_url}{endpoint}` (mặc định `endpoint=/upload/file`) — multipart `file` + `camera_id` + `category`:

| category | Ảnh | Field event |
|----------|-----|-------------|
| `vehicle` | full-frame | `snapshot_url` (= object_key) |
| `plate` | crop biển | `snapshot_base64` (= object_key, **không** phải base64) |

### 6.3 Event payload

```json
{
  "ai_modules": "PLATE",
  "camera_id": "<uuid>",
  "event_time": "<UTC ISO>",
  "entity_type": "VEHICLE",
  "entity_id": "vehicle_<track_id>",
  "payload": {
    "direction": null,
    "vehicle": {
      "license_plate": {
        "text": "30A12345",
        "status": "DETECTED",
        "plate_color": null
      },
      "vehicle_type": "Ô tô",
      "car_type": null,
      "manufacturer": null,
      "color": null,
      "total_number_of_seats": null
    }
  },
  "snapshot_url": "<object_key full-frame>",
  "snapshot_base64": "<object_key plate-crop>"
}
```

Map class sau vote: `1→Ô tô`, `2→Xe máy`, `3→Xe tải`, `0→Xe khách`.

### 6.4 Event vi phạm (NO_HELMET)

VMS gửi **retained** danh sách mã vi phạm được bật cho từng camera qua
`get_violations` = `smart_vms/ai_config/state/{camera_id}/{ai_modules}/violations`
(khoá là `camera_id` UUID, **không** phải `camera_code`):

```json
{"schema_version": 1, "camera_id": "<uuid>", "camera_code": "vanninh",
 "module_code": "PLATE", "module_enabled": true,
 "allowed_codes": ["NO_HELMET", "RED_LIGHT", "…"], "revision": 0}
```

`VmsClient` subscribe wildcard `+` cho `{camera_id}` và nạp vào
`business::violation::ConfigStore`. Vi phạm chỉ được bắn khi
`module_enabled = true` **và** mã nằm trong `allowed_codes` của camera đó.

Khi một track xe máy có ≥ `violation.helmet.min_hits` frame detect được người không
đội mũ **và** camera bật `NO_HELMET`, app **chỉ** bắn event vi phạm trên `pub_event`
(không gửi kèm event xe — khớp `phatnguoi_mbf` VehicleEventPublisher). Không có vi
phạm thì chỉ bắn event xe thường. Schema khớp
`tests/test_proto/test_pub_mqtt_violation.py`:

```json
"payload": {
  "direction": "IN",
  "vehicle": { "…": "như event xe" },
  "violation_type_code": "NO_HELMET",
  "violation_evidence": { "road_type": "highway" }
}
```

Điều kiện bắn (khớp `phatnguoi_mbf/gsan/controller/thread/vehicle/general_thread.py`):
`violation.helmet.enabled` **và** class sau vote = `Xe máy` **và** đủ `min_hits`
**và** camera bật `NO_HELMET`.

---

## 7. `src/common` — config

File runtime (ARCHITECTURE) — chỉ `resources/config/`:

- `config.yaml` — `AI_MODULE`, `plate.*`
- `violations.yaml` — `violation.*` (helmet)
- `mqtt.yaml` — broker + topic keys trên
- `restful.yaml` — `snapshot_api.base_url` / `endpoint`

| Knob | Mặc định tham chiếu | Module dùng |
|------|---------------------|-------------|
| `AI_MODULE` / `ai_modules` | `PLATE` | communication filter + `pub_event` |
| `plate.max_recognize_times` | `5` | business |
| `plate.send_mode` | `2` | business |
| `plate.plate_style` | list patterns | business |
| `snapshot_api.base_url` / `endpoint` | restful.yaml | communication upload |
| Vehicle / plate / digit conf | `0.55` / `0.40` / `0.25` | `resources/ds/infer/*.txt` |
| `pipeline.streammux.*` | batch_size / timeout / sync / config_file | nvstreammux v2 (không width/height; frame meta giữ resolution nguồn) |
| `pipeline.pgie.vehicle_class_map` | `[3, 0, 1, 2]` | probes (đổi class id model → chuẩn nghiệp vụ) |
| Plate expand | `0.30` | ⚠️ knob còn trong config nhưng **chưa dùng** — nvinfer crop đúng bbox xe |
| Anchor bottom ratio | `0.12` | probes / utils |
| Char IoU dedup | `0.4` | probes / utils |
| Square plate ratio | `0.5` | probes / utils |

---

## 8. Sơ đồ trạng thái (`business`, per track_id)

```text
            pipeline + probes
                   │
                   ▼
          anchor in polygon?
                │ yes
                ▼
         create/update track state
                   │
                   ▼
         SGIE1 plate → SGIE2 OCR
                   │
        count < n ──► tiếp tục
                   │ count >= n
                   ▼
            fuse + validate
                   │
          send_mode / dedup OK?
                   │ yes
                   ▼
     communication: upload → pub_event
                   │
                   ▼
              cleanup state
```

---

## 9. TDD — `tests/business`

Reference Python (`tests/business/plate_rules/`) khóa hành vi `src/business/plate/` trước khi port C++:

| Module Python | Test | File C++ tương ứng |
|---------------|------|--------------------|
| `constants`/`validate`/`fuse`/`normalize`/`send_mode` | style, vote, normalize, send_mode | `src/business/plate/rules.cpp` |
| `track_state` | OCR trong polygon đến leave/max, chọn biển+snapshot đẹp nhất, miss/force-delete, dedup | `src/business/plate/track.cpp` |
| `event` | payload MQTT, map loại xe; vi phạm thêm `violation_type_code` | `src/business/plate/event.cpp` |

`tests/business_cpp/test_business.cpp` chạy lại đúng bộ case này trên bản C++ (thêm
`plate_recognizer` — cổng emit, và `char_assembler` — ghép ký tự §4.3).

```bash
bash tests/business/run.sh        # Python reference
bash tests/business_cpp/run.sh    # C++ parity (cần vehicle.sh build trước)
```

---

## 10. Verify bằng golden fixture + `test_proto`

### 10.1 Golden media (đã kiểm chứng)

| File | Vai trò |
|------|---------|
| `tests/data/test.mp4` | Video nguồn |
| `tests/data/test_result.mp4` | Video kết quả (tham chiếu OSD) |
| `tests/data/test_plates.json` | Ground truth plates / readings / per-frame |

Ground truth chính trong JSON:

| track_id | class | plate | valid |
|----------|-------|-------|-------|
| 1 | motorbike | `14H05545` | true |
| 2 | car | `14K21493` (readings gồm `14A84263` → chốt đúng) | true |
| 3 | car | `1A` | false → normalize `_unk` |
| 6 | car | `29C70454` | true |

```bash
bash tests/verify/run.sh                        # business rules vs golden JSON
python3 tests/verify/eval_pipeline.py           # pipeline C++ thật vs golden JSON
```

`eval_pipeline.py` chạy binary với `--dry-run` (không upload/publish ra VMS).
nvstreammux v2 giữ resolution nguồn (`test.mp4` = 2960x1668) nên biển nhỏ không
bị scale mất nét. Kết quả OSD ghi ra `tests/output/test_result.mp4` để so trực
quan với `tests/data/test_result.mp4`.

**Kết quả đo (3 frame fixture, engine fp16, `--max-recognize 1`):**

| Ground truth | Pipeline đọc được | Nhận xét |
|--------------|-------------------|----------|
| `14K21493` (car) | `14K21493` ✅ | khớp tuyệt đối |
| `14A84263` (readings của cùng track 2) | `14A04263` | sai 1 ký tự (`8`→`0`) |
| `29C70454` (car) | `29K10454` | sai 2 ký tự, biển ở xa |
| `14H05545` (motorbike) | `H5915` | biển 2 dòng nhỏ, rơi ký tự |
| `1A` (invalid) | — | xe ngoài rìa khung, không đọc được |

Fixture chỉ có **3 frame** nên multi-frame vote (`max_recognize_times = 5`) gần như
không có tác dụng — trên stream thật, vote nhiều frame sẽ lọc bớt các lỗi 1–2 ký tự
như trên. Detect xe + track + phân tầng meta + snapshot JPEG + payload event đều
đúng; sai số còn lại nằm ở chất lượng OCR trên biển nhỏ.

### 10.2 `tests/test_proto` (MQTT/API)

Chạy trong `docker-compose.yaml` → container `vehicle_test`.

| Script | Xác nhận |
|--------|----------|
| `test_sub_mqtt_cameras.py` | `camera_list` + filter `PLATE` |
| `test_sub_mqtt_polygon.py` | `get_polygon` — zone ROI |
| `test_pub_mqtt_bbox.py` | `pub_bbox` |
| `test_api_upload.py` | upload `vehicle` / `plate` → object_key |
| `test_pub_mqtt_event.py` | `pub_event` schema |

Ảnh mẫu proto: `tests/data/vehicle/xe.jpg`, `tests/data/plate/bien.jpg`.

---

## 10.5. Đo hiệu năng & độ trễ

Ba mức đo, từ thô tới chi tiết:

| Mức | Cách chạy | Cho biết |
|-----|-----------|----------|
| FPS gốc của camera | `ffprobe -rtsp_transport tcp <uri>` / `ffmpeg -t 10 -f null -` | Trần lý thuyết; app không thể nhanh hơn con số này |
| FPS từng chặng | `bash scripts/bench_stages.sh 40 [uri…]` | Dựng lại pipeline bằng `gst-launch` và cộng dần PGIE → tracker → SGIE1 → SGIE2 → OSD; chặng nào tụt fps là chặng nghẽn |
| Độ trễ từng element | `VEHICLE_LATENCY=1 NVDS_ENABLE_LATENCY_MEASUREMENT=1 NVDS_ENABLE_COMPONENT_LATENCY_MEASUREMENT=1 docker compose up -d` | DeepStream in `Comp name = … component latency`; app in `ds_tracker` / `ds_full` (decoder → probe) |

Đo ngày 04/08/2026 (RTX 3090 dùng chung với ~10 container khác, 2 camera:
1920×1080@25 và 2688×1520@20):

| Chặng | p50 |
|-------|-----|
| PGIE xe (640, b8) | 3 ms |
| tracker NvDCF_perf | 2 ms |
| SGIE1 biển (256, b16) | 2 ms |
| SGIE2 ký tự (320, b16) | 0–3 ms |
| **Tổng suy luận** | **~10 ms/batch** |
| nvstreammux (trước sửa / sau sửa) | 205 ms → 8 ms |
| decoder + nguồn (cam 25fps) | 588 ms → 60 ms |
| decoder + nguồn (cam 20fps) | ~370 ms (nằm ngoài app) |

Kết luận: cụm nvinfer **không** phải nút thắt (~10 ms trong ngân sách ~600 ms).
Nghẽn nằm ở `max-same-source-frames=1` của nvstreammux v2 — mỗi batch chỉ nhận 1
frame/source nên camera 25 fps bị kéo về nhịp camera 20 fps, hàng đợi dồn dần.
Đặt `max-same-source-frames=4` (`resources/ds/streammux/config_mux.txt`) trả
camera nhanh về đúng 25 fps và cắt độ trễ 595 ms → 82 ms.

---

## 11. Trạng thái implement (khớp `src/`)

| # | Module | Trạng thái | Ghi chú |
|---|--------|-----------|---------|
| 1 | `common/` | ✅ | `Config::load` đọc `config/mqtt/restful.yaml` (+ block `pipeline:`); DTO camera/zone/detection/event |
| 2 | `communication/` | ✅ | MQTT qua `nvds_msgapi` + `libnvds_mqtt_proto.so`; sub camera_list/zones; pub bbox/event; upload multipart bằng libcurl |
| 3 | `pipeline/` | ✅ | `nvurisrcbin` → mux → PGIE → tracker → SGIE1 → SGIE2 → SGIE3 → `nvdsosd` (GPU) → sink (`fake`/`file`) |
| 4 | `probes/` | ✅ | Probe A: ROI polygon, phân tầng meta, pub bbox, enqueue JPEG; Probe B: đọc `NVDS_CROP_IMAGE_META` + cổng emit |
| 5 | SGIE1/warp/SGIE2 | ✅ | Pose + warp preprocess + digit tensor; custom `.so` build bởi CMake từ `resources/ds/` |
| 6 | `business/plate/` | ✅ | rules + track + event + `plate_recognizer` |
| 7 | `utils/` | ✅ | geometry (anchor, point-in-polygon, expand, IoU), time, string |

Phase 1: detect→track→plate→OCR→event. Không line crossing; bỏ brand/color/seat/lpcol.

**Chuẩn bị model:** `bash scripts/build_engines.sh` (xem `scripts/README.md`).
App kiểm tra file model có thật trước khi thêm stage nvinfer — thiếu thì log cảnh
báo và chạy passthrough (decode + sink).

**Khoảng trống đã biết (chưa làm ở phase 1):**

| Mục | Hiện trạng |
|-----|-----------|
| Expand ROI xe 30% trước SGIE1 (§4.2) | ✅ Probe expand trên sink `sgie_plate` + restore trên src (OSD/pub giữ bbox gốc). |
| Xoay crop biển nghiêng rồi infer lại (§4.3.4) | Chưa làm — thay bằng khử nghiêng khi ghép ký tự (đủ cho biển 1 dòng xiên). |
| Thêm/bớt/đổi URL camera lúc đang chạy | `Pipeline::applyCameraList` theo MQTT `camera_list` (idle trên GLib main loop). `source_id` ổn định theo slot; capacity = `pipeline.streammux.batch_size`. `--source` tắt dynamic. |
| Reconnect RTSP | `rtsp-reconnect-interval` + `rtsp-reconnect-attempts`; lỗi từ `source_*` không quit app. |
| Thứ tự ký tự của model digit | Checkpoint không kèm tên class (`names` = "0".."35"); đang giả định `0-9` rồi `A-Z` trong `labels_digit.txt` — đã kiểm chứng đúng với biển `14K21493` của fixture. |
