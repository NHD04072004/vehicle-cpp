# Logic nhận diện biển số

Reference: `LicensePlateReader.recognize` trong  
`/mnt/atin_t2/MANHDUY/API/vehicle/main_app/features/license_plate_recognition/services/reader.py`

Hai model YOLO: **keypoint** (detect biển + 4 góc) và **digit** (ký tự 0–9, A–Z).

---

## Luồng

```text
Ảnh BGR
  → 1. Detect biển (YOLO keypoint) → chọn conf cao nhất
  → 2. Warp 4 keypoints (fallback crop bbox)
  → 3. Detect màu biển (HSV)
  → 4. Detect ký tự (YOLO digit)
  → 5. Sắp xếp: vuông 2 dòng / ngang 1 dòng
  → 6. Validate style VN → plate_text (+ _unk nếu invalid)
```

---

## 1. Detect biển

- YOLO keypoint với `keypoint_confidence`
- Mỗi box → bbox + keypoints (4 góc)
- Không có detection → thất bại (`License plate not found`)
- Chọn biển có **confidence cao nhất**

## 2. Crop / warp

**Warp** (ưu tiên): cần ≥ 4 keypoints đủ `keypoint_min_confidence` → perspective transform `tl,tr,br,bl` → ảnh biển phẳng (clamp `max_width` / `max_height`).

**Fallback:** crop theo bbox nếu warp thất bại.

Crop rỗng → thất bại (`Unable to crop license plate`).

## 3. Màu biển

HSV trên ROI trung tâm:

| Điều kiện | Màu |
|-----------|-----|
| Sáng + sat thấp, hoặc tỷ lệ sat thấp | white |
| Hue 85–128 + sat cao | blue |
| Hue 15–38 | yellow |
| Hue ≤10 hoặc ≥160 | red |
| Khác | white / unknown |

## 4. Detect ký tự

- YOLO digit trên ảnh biển đã warp/crop
- Class `0–9` → số, `10–35` → `A–Z`
- Không có ký tự → text rỗng, invalid
- **Confidence** = `min(conf mọi ký tự)`

## 5. Sắp xếp ký tự

**Biển vuông** nếu:

- đủ ký tự và khoảng cách Y giữa 2 nhóm > `avg_height * square_y_gap_threshold`, hoặc
- `width/height < square_ratio_threshold`

Khi vuông: chia 2 dòng theo `avg(y1)`, mỗi dòng sort theo `x`, ghép dòng trên + dưới.  
Dòng trên match `NNNNN|NNC|CNN|NNCC` → car (`plate_type=1`), else motorbike (`0`).

**Biển ngang:** sort theo `x`, suy `plate_type` từ style car/moto của text.

## 6. Validate style VN

Cả hai điều kiện phải pass → `valid=True`; không pass → gắn `_unk` vào text.

**Pattern** (`N`=số, `C`=chữ) — match một trong:

```text
NNCNNNNN, NNCNNNNNN, CCNNNN, NNCNNNN,
NNCCNNNN, NNNNNCC, NNNNNCN, NNNNNCCNN,
NNCCNNNNN, NNCCNNNNNN
```

**Prefix 2 ký tự đầu:**

| Prefix | Rule |
|--------|------|
| Toàn số | Không nằm trong `DIGIT_CAR` (`00…10, 44,45,46,87,91,96`) |
| Toàn chữ | Phải nằm trong `ALPHA_ARMY` (mã quân sự) |
| Hỗn hợp | Pass |

**`plate_type`:** `1`=car, `0`=motorbike, `-1`=unknown (suy từ `PLATE_STYLE_CAR` / `PLATE_STYLE_MOTO` nếu chưa xác định).

---

## Output

| Field | Ý nghĩa |
|-------|---------|
| `plate_text` | Chuỗi OCR (có thể kèm `_unk`) |
| `confidence` | Min conf ký tự |
| `plate_type` / `plate_type_name` | car / motorbike / unknown |
| `license_plate_color` | blue / yellow / red / white / unknown |
| `valid` | Pass style + prefix |
| `plate_bbox` / `plate_confidence` / `keypoints` | Detection biển trên ảnh gốc |
