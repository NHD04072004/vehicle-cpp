#!/usr/bin/env bash
# Build TensorRT engine cho model trong resources/weights/.
#
#   .pt ──(export)──> .onnx ──(trtexec)──> *_b{N}_{fp16|fp32}.engine
#
# Chạy trong container có GPU:
#   docker exec vehicle bash -lc 'bash /app/scripts/build_engines.sh'
#   docker compose -f scripts/docker-compose.export.yaml run --rm vehicle_export
#
# Tuỳ chọn:
#   FORCE=1        export ONNX + build engine lại kể cả khi đã có file
#   PRECISION=fp32 (mặc định fp16)
#   ONLY=<name>    chỉ một model (vehicle | last_keypoint | digit_n_p3p4_256 | helmet)
#   WEIGHTS_DIR=   thư mục weights (mặc định <repo>/resources/weights)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WEIGHTS_DIR="${WEIGHTS_DIR:-$ROOT/resources/weights}"
VENDOR_DIR="$ROOT/scripts/vendor"
YOLOV5_REPO="${YOLOV5_REPO:-$VENDOR_DIR/yolov5}"
PRECISION="${PRECISION:-fp16}"
FORCE="${FORCE:-0}"
TRTEXEC="${TRTEXEC:-$(command -v trtexec || echo /usr/src/tensorrt/bin/trtexec)}"
POSE_EXPORT="$ROOT/scripts/vendor/deepstream_yolo_pose/export_yoloV8_pose.py"

# name | file .pt | imgsz | max batch | kind (yolo | pose)
MODELS=(
  "vehicle|vehicle_n_best.pt|640|8|yolo"
  "last_keypoint|last_keypoint.pt|640|8|pose"
  "digit_n_p3p4_256|digit_n_p3p4_256.pt|256|16|yolo"
  "helmet|helmet_ylv8_171125.pt|640|16|yolo"
)
ONLY="${ONLY:-}"

log() { printf '\n\033[1m[engines]\033[0m %s\n' "$*"; }

is_required() {
  case "$1" in
    vehicle|last_keypoint|digit_n_p3p4_256) return 0 ;;
    *) return 1 ;;
  esac
}

skip_or_fail() {
  local name="$1" weights="$2"
  if is_required "$name"; then
    echo "Thiếu weights bắt buộc: $weights (hoặc $name.onnx)" >&2
    exit 1
  fi
  log "Bỏ qua $name — không có $weights / $name.onnx"
}

ensure_python_deps() {
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

export_onnx() {
  local name="$1" weights="$2" imgsz="$3" batch="$4"
  local onnx="$WEIGHTS_DIR/$name.onnx"
  local pt="$WEIGHTS_DIR/$weights"
  if [[ -f "$onnx" && "$FORCE" != "1" ]]; then
    log "ONNX đã có, bỏ qua: $onnx (FORCE=1 để build lại)"
    return
  fi
  [[ -f "$pt" ]] || { echo "Thiếu weights: $weights (cần để export $onnx)" >&2; exit 1; }
  log "Export ONNX: $weights → $onnx (imgsz=$imgsz, max-batch=$batch)"
  python3 "$ROOT/scripts/export_onnx.py" \
    --weights "$pt" \
    --output "$onnx" \
    --labels "$WEIGHTS_DIR/$name.labels.txt" \
    --imgsz "$imgsz" \
    --max-batch "$batch" \
    --yolov5-repo "$YOLOV5_REPO"
}

export_pose() {
  local name="$1" weights="$2" imgsz="$3"
  local onnx="$WEIGHTS_DIR/$name.onnx"
  local pt="$WEIGHTS_DIR/$weights"
  if [[ -f "$onnx" && "$FORCE" != "1" ]]; then
    log "ONNX đã có, bỏ qua: $onnx (FORCE=1 để build lại)"
    return
  fi
  [[ -f "$pt" ]] || { echo "Thiếu weights pose: $weights (cần để export $onnx)" >&2; exit 1; }
  [[ -f "$POSE_EXPORT" ]] || { echo "Thiếu exporter pose: $POSE_EXPORT" >&2; exit 1; }
  log "Export ONNX pose: $weights → $onnx (imgsz=$imgsz, dynamic batch)"
  (cd "$WEIGHTS_DIR" && python3 "$POSE_EXPORT" -w "$pt" -s "$imgsz" --dynamic)
  local produced="${pt%.pt}.onnx"
  if [[ "$produced" != "$onnx" && -f "$produced" ]]; then
    mv -f "$produced" "$onnx"
  fi
}

build_engine() {
  local name="$1" imgsz="$2" batch="$3"
  local onnx="$WEIGHTS_DIR/$name.onnx"
  local engine="$WEIGHTS_DIR/${name}_b${batch}_${PRECISION}.engine"
  if [[ -f "$engine" && "$FORCE" != "1" ]]; then
    log "Engine đã có, bỏ qua: $engine"
    return
  fi
  [[ -f "$onnx" ]] || { echo "Thiếu ONNX: $onnx" >&2; exit 1; }
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
  mkdir -p "$WEIGHTS_DIR"

  ensure_python_deps
  ensure_yolov5_repo

  local spec name weights imgsz batch kind pt onnx engine
  for spec in "${MODELS[@]}"; do
    IFS='|' read -r name weights imgsz batch kind <<< "$spec"
    kind="${kind:-yolo}"
    if [[ -n "$ONLY" && "$name" != "$ONLY" ]]; then
      continue
    fi

    pt="$WEIGHTS_DIR/$weights"
    onnx="$WEIGHTS_DIR/$name.onnx"
    engine="$WEIGHTS_DIR/${name}_b${batch}_${PRECISION}.engine"

    if [[ -f "$engine" && "$FORCE" != "1" ]]; then
      log "Engine đã có, bỏ qua: $engine"
      continue
    fi

    if [[ ! -f "$onnx" || "$FORCE" == "1" ]]; then
      if [[ -f "$pt" ]]; then
        if [[ "$kind" == "pose" ]]; then
          export_pose "$name" "$weights" "$imgsz"
        else
          export_onnx "$name" "$weights" "$imgsz" "$batch"
        fi
      elif [[ -f "$onnx" ]]; then
        log "Không có .pt, dùng ONNX có sẵn: $onnx"
      else
        skip_or_fail "$name" "$weights"
        continue
      fi
    else
      log "ONNX đã có, bỏ qua export: $onnx"
    fi

    build_engine "$name" "$imgsz" "$batch"
  done

  log "Xong. Nội dung $WEIGHTS_DIR:"
  ls -la "$WEIGHTS_DIR"
  cat <<'EOF'

Kiểm tra resources/ds/infer/*.txt phải trỏ đúng:
  pgie_vehicle.txt      → vehicle.onnx       / vehicle_b8_fp16.engine
  sgie1_plate_pose.txt  → last_keypoint.onnx / last_keypoint_b8_fp16.engine
  sgie2_digit.txt       → digit_n_p3p4_256.onnx / digit_n_p3p4_256_b16_fp16.engine
  sgie3_helmet.txt      → helmet.onnx        / helmet_b16_fp16.engine

Nhãn model xuất ra <name>.labels.txt chỉ để tham chiếu — file nhãn dùng thật là
resources/ds/infer/labels_*.txt.
EOF
}

main "$@"
