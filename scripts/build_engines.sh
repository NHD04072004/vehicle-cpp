#!/usr/bin/env bash
# Build TensorRT engine cho toàn bộ model trong resources/weights/.
#
#   .pt ──(scripts/export_onnx.py)──> .onnx ──(trtexec)──> .engine
#
# Chạy trong container có GPU:
#   docker exec vehicle_test bash -lc 'bash /app/scripts/build_engines.sh'
#   docker compose -f scripts/docker-compose.export.yaml run --rm vehicle_export
#
# Tuỳ chọn:
#   FORCE=1        build lại kể cả khi đã có file
#   PRECISION=fp32 (mặc định fp16)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WEIGHTS_DIR="$ROOT/resources/weights"
VENDOR_DIR="$ROOT/scripts/vendor"
YOLOV5_REPO="${YOLOV5_REPO:-$VENDOR_DIR/yolov5}"
PRECISION="${PRECISION:-fp16}"
FORCE="${FORCE:-0}"
TRTEXEC="${TRTEXEC:-$(command -v trtexec || echo /usr/src/tensorrt/bin/trtexec)}"

# name | file .pt | imgsz | max batch
# ONLY=<name> để build một model (vd. ONLY=digit FORCE=1 ...)
# Plate SGIE1 dùng YOLO-Pose (last_keypoint.*) — export/engine riêng
# (scripts/vendor/deepstream_yolo_pose), không đi qua export_onnx.py YOLOv5.
MODELS=(
  "vehicle|vehicle_n_best.pt|640|8"
  "digit_n_p3p4_256|digit_n_p3p4_256.pt|256|16"
  "helmet|helmet_ylv8_171125.pt|640|16"
)
ONLY="${ONLY:-}"

log() { printf '\n\033[1m[engines]\033[0m %s\n' "$*"; }

# --- Phụ thuộc ------------------------------------------------------------- #

ensure_python_deps() {
  # Checkpoint yolov5 nạp qua repo gốc → cần pandas/seaborn của repo đó.
  python3 - <<'PY' || pip install --no-cache-dir -q pandas seaborn
import importlib, sys
sys.exit(0 if all(importlib.util.find_spec(m) for m in ("pandas", "seaborn")) else 1)
PY
}

ensure_yolov5_repo() {
  if [[ -d "$YOLOV5_REPO/models" ]]; then
    return
  fi
  log "Chưa có repo yolov5 → clone vào $YOLOV5_REPO"
  mkdir -p "$VENDOR_DIR"
  git clone --depth 1 https://github.com/ultralytics/yolov5 "$YOLOV5_REPO"
}

# --- Build ----------------------------------------------------------------- #

export_onnx() {
  local name="$1" weights="$2" imgsz="$3" batch="$4"
  local onnx="$WEIGHTS_DIR/$name.onnx"
  if [[ -f "$onnx" && "$FORCE" != "1" ]]; then
    log "ONNX đã có, bỏ qua: $onnx (FORCE=1 để build lại)"
    return
  fi
  log "Export ONNX: $weights → $onnx (imgsz=$imgsz, max-batch=$batch)"
  python3 "$ROOT/scripts/export_onnx.py" \
    --weights "$WEIGHTS_DIR/$weights" \
    --output "$onnx" \
    --labels "$WEIGHTS_DIR/$name.labels.txt" \
    --imgsz "$imgsz" \
    --max-batch "$batch" \
    --yolov5-repo "$YOLOV5_REPO"
}

build_engine() {
  local name="$1" imgsz="$2" batch="$3"
  local onnx="$WEIGHTS_DIR/$name.onnx"
  local engine="$WEIGHTS_DIR/${name}_b${batch}_${PRECISION}.engine"
  if [[ -f "$engine" && "$FORCE" != "1" ]]; then
    log "Engine đã có, bỏ qua: $engine"
    return
  fi
  local flags=("--fp16")
  [[ "$PRECISION" == "fp32" ]] && flags=()
  log "Build engine: $engine (có thể mất vài phút)"
  "$TRTEXEC" \
    --onnx="$onnx" \
    --saveEngine="$engine" \
    "${flags[@]}" \
    --minShapes=input:1x3x${imgsz}x${imgsz} \
    --optShapes=input:${batch}x3x${imgsz}x${imgsz} \
    --maxShapes=input:${batch}x3x${imgsz}x${imgsz} \
    > "$WEIGHTS_DIR/${name}_trtexec.log" 2>&1 \
    || { echo "trtexec lỗi — xem $WEIGHTS_DIR/${name}_trtexec.log" >&2; exit 1; }
  grep -E "Engine built in|Throughput:" "$WEIGHTS_DIR/${name}_trtexec.log" || true
}

main() {
  [[ -x "$TRTEXEC" ]] || { echo "Không thấy trtexec: $TRTEXEC" >&2; exit 1; }
  ensure_python_deps
  ensure_yolov5_repo

  for spec in "${MODELS[@]}"; do
    IFS='|' read -r name weights imgsz batch <<< "$spec"
    if [[ -n "$ONLY" && "$name" != "$ONLY" ]]; then
      continue
    fi
    [[ -f "$WEIGHTS_DIR/$weights" ]] || { echo "Thiếu weights: $weights" >&2; exit 1; }
    export_onnx "$name" "$weights" "$imgsz" "$batch"
    build_engine "$name" "$imgsz" "$batch"
  done

  log "Xong. Nội dung resources/weights/:"
  ls -la "$WEIGHTS_DIR"
  cat <<'EOF'

Kiểm tra lại resources/ds/infer/*.txt phải trỏ đúng:
  pgie_vehicle.txt      → vehicle.onnx       / vehicle_b8_fp16.engine        (4 class)
  sgie1_plate_pose.txt  → last_keypoint.onnx / last_keypoint_b8_fp16.engine  (pose, khớp PGIE b8)
  sgie2_digit.txt       → digit_n_p3p4_256.onnx / digit_n_p3p4_256_b16_fp16.engine (36 class)
  sgie3_helmet.txt      → helmet.onnx        / helmet_b16_fp16.engine        (3 class, chỉ xe máy)
  config_preprocess_warp_plate.txt → build/libs/libnvdspreprocess_custom_warp_perspective.so

Nhãn model xuất ra <name>.labels.txt chỉ để tham chiếu — file nhãn dùng thật là
resources/ds/infer/labels_*.txt (đặc biệt labels_digit.txt: model trả class id
0..35, thứ tự ký tự do dataset quy định).
EOF
}

main "$@"
