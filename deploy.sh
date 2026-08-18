#!/usr/bin/env bash
# Build image + TensorRT engine rồi đóng gói sang thư mục production tách
# khỏi source. Sau bước này có thể xóa source; target chỉ cần:
#
#   cd /mnt/atin/nhd/c12/vehicle_prod && docker compose up -d
#
#   ./deploy.sh [PROD_DIR]
#
# Mặc định PROD_DIR = ../vehicle_prod (cạnh thư mục source).
#
# Việc làm:
#   1. docker build image runtime (nhd04072004/vehicle-app:1.0.0)
#   2. docker build image export  (nhd04072004/vehicle-export:1.0.0) nếu cần .pt/.onnx → engine
#   3. Đồng bộ resources/config, resources/ds, resources/weights sang PROD_DIR
#   4. Nếu có .pt / .onnx (và chưa có engine, hoặc FORCE=1): build engine trên GPU máy này
#   5. Copy docker-compose.prod.yaml → docker-compose.yaml (không có build:)
#
# Biến môi trường:
#   SKIP_BUILD=1     không build lại image (dùng image local sẵn có)
#   SKIP_ENGINES=1   không chạy trtexec (giữ engine đã copy)
#   FORCE=1          export ONNX + build engine lại
#   PRECISION=fp16   (hoặc fp32)
#   TARGETARCH=amd64|arm64   (mặc định: uname -m của máy đang chạy)
#   BASE_IMAGE=...           (mặc định: nhd04072004/ds_app:8.0-$TARGETARCH)
set -euo pipefail

detect_targetarch() {
  local machine="${TARGETARCH:-$(uname -m)}"
  case "$machine" in
    x86_64|amd64) echo amd64 ;;
    aarch64|arm64) echo arm64 ;;
    *)
      echo "ERROR: kiến trúc không hỗ trợ: $machine (cần amd64/x86_64 hoặc arm64/aarch64)" >&2
      return 1
      ;;
  esac
}

ROOT="$(cd "$(dirname "$0")" && pwd)"
PROD_DIR="${1:-$ROOT/../vehicle_prod}"
TARGETARCH="$(detect_targetarch)"
BASE_IMAGE="${BASE_IMAGE:-nhd04072004/ds_app:8.0-${TARGETARCH}}"
IMAGE="${IMAGE:-nhd04072004/vehicle-app:1.0.0}"
EXPORT_IMAGE="${EXPORT_IMAGE:-nhd04072004/vehicle-export:1.0.0}"
SKIP_BUILD="${SKIP_BUILD:-0}"
SKIP_ENGINES="${SKIP_ENGINES:-0}"
FORCE="${FORCE:-0}"
PRECISION="${PRECISION:-fp16}"

command -v docker >/dev/null 2>&1 || { echo "ERROR: cần docker" >&2; exit 1; }
docker compose version >/dev/null 2>&1 || { echo "ERROR: cần docker compose plugin" >&2; exit 1; }
[[ -f "$ROOT/docker-compose.prod.yaml" ]] || { echo "ERROR: thiếu docker-compose.prod.yaml" >&2; exit 1; }
[[ -d "$ROOT/resources" ]] || { echo "ERROR: thiếu $ROOT/resources" >&2; exit 1; }

mkdir -p "$PROD_DIR"
PROD_DIR="$(cd "$PROD_DIR" && pwd)"
if [[ "$PROD_DIR" == "$ROOT" || "$PROD_DIR" == "$ROOT"/* ]]; then
  echo "ERROR: PROD_DIR không được nằm trong source ($PROD_DIR)" >&2
  exit 1
fi

# Compose/BuildKit v0.10+ gắn provenance/SBOM → manifest list, docker run lỗi.
export BUILDX_NO_DEFAULT_ATTESTATIONS=1

echo "==> Kiến trúc $TARGETARCH — base $BASE_IMAGE"

if [[ "$SKIP_BUILD" != "1" ]]; then
  echo "==> Build image $IMAGE"
  docker build --network host --provenance=false --sbom=false \
    --build-arg TARGETARCH="$TARGETARCH" \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    -t "$IMAGE" "$ROOT"
else
  echo "==> SKIP_BUILD=1 — dùng lại $IMAGE"
fi
docker image inspect "$IMAGE" >/dev/null 2>&1 \
  || { echo "ERROR: chưa có image $IMAGE" >&2; exit 1; }

echo "==> Đồng bộ resources sang $PROD_DIR"
mkdir -p "$PROD_DIR/resources/config" "$PROD_DIR/resources/ds" "$PROD_DIR/resources/weights"

if command -v rsync >/dev/null 2>&1; then
  rsync -a --delete "$ROOT/resources/config/" "$PROD_DIR/resources/config/"
  rsync -a --delete \
    --exclude 'nvdsinfer_custom_impl_Yolo/' \
    --exclude 'nvdsinfer_custom_impl_Yolo_pose/' \
    --exclude 'nvdspreprocess_custom_warp_perspective/' \
    "$ROOT/resources/ds/" "$PROD_DIR/resources/ds/"
  rsync -a \
    --exclude '*.log' \
    "$ROOT/resources/weights/" "$PROD_DIR/resources/weights/"
else
  rm -rf "$PROD_DIR/resources/config" "$PROD_DIR/resources/ds"
  cp -a "$ROOT/resources/config" "$PROD_DIR/resources/config"
  cp -a "$ROOT/resources/ds" "$PROD_DIR/resources/ds"
  rm -rf \
    "$PROD_DIR/resources/ds/nvdsinfer_custom_impl_Yolo" \
    "$PROD_DIR/resources/ds/nvdsinfer_custom_impl_Yolo_pose" \
    "$PROD_DIR/resources/ds/nvdspreprocess_custom_warp_perspective"
  mkdir -p "$PROD_DIR/resources/weights"
  cp -a "$ROOT/resources/weights/." "$PROD_DIR/resources/weights/"
  rm -f "$PROD_DIR/resources/weights/"*.log
fi

need_engines=0
if [[ "$SKIP_ENGINES" != "1" ]]; then
  if [[ "$FORCE" == "1" ]]; then
    need_engines=1
  else
    for eng in \
      "vehicle_b8_${PRECISION}.engine" \
      "last_keypoint_b8_${PRECISION}.engine" \
      "digit_n_p3p4_256_b16_${PRECISION}.engine"
    do
      if [[ ! -f "$PROD_DIR/resources/weights/$eng" ]]; then
        need_engines=1
      fi
    done
    if [[ -f "$PROD_DIR/resources/weights/helmet.onnx" || -f "$PROD_DIR/resources/weights/helmet_ylv8_171125.pt" ]]; then
      if [[ ! -f "$PROD_DIR/resources/weights/helmet_b16_${PRECISION}.engine" ]]; then
        need_engines=1
      fi
    fi
  fi
fi

if [[ "$need_engines" == "1" ]]; then
  echo "==> Build image export $EXPORT_IMAGE (torch + trtexec)"
  if [[ "$SKIP_BUILD" != "1" ]]; then
    docker build --network host --provenance=false --sbom=false \
      --build-arg TARGETARCH="$TARGETARCH" \
      --build-arg BASE_IMAGE="$BASE_IMAGE" \
      -t "$EXPORT_IMAGE" \
      -f "$ROOT/scripts/Dockerfile.export" \
      "$ROOT/scripts"
  fi
  docker image inspect "$EXPORT_IMAGE" >/dev/null 2>&1 \
    || { echo "ERROR: chưa có image $EXPORT_IMAGE" >&2; exit 1; }

  echo "==> Build TensorRT engine trên GPU máy này → $PROD_DIR/resources/weights"
  docker run --rm --gpus all \
    --runtime nvidia \
    -v "$PROD_DIR:/app" \
    -v "$ROOT/scripts:/app/scripts:ro" \
    -e YOLOV5_REPO=/opt/yolov5 \
    -e PRECISION="$PRECISION" \
    -e FORCE="$FORCE" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video \
    "$EXPORT_IMAGE"
else
  echo "==> Bỏ qua build engine (SKIP_ENGINES=$SKIP_ENGINES)"
fi

required=(
  "vehicle_b8_${PRECISION}.engine"
  "last_keypoint_b8_${PRECISION}.engine"
  "digit_n_p3p4_256_b16_${PRECISION}.engine"
)
missing=0
for eng in "${required[@]}"; do
  if [[ ! -f "$PROD_DIR/resources/weights/$eng" ]]; then
    echo "ERROR: thiếu engine $eng trong $PROD_DIR/resources/weights" >&2
    echo "       Cần file .pt hoặc .onnx tương ứng rồi chạy lại (FORCE=1 nếu cần)." >&2
    missing=1
  fi
done
if [[ "$missing" -ne 0 ]]; then
  echo "Nội dung weights:" >&2
  ls -la "$PROD_DIR/resources/weights" >&2 || true
  exit 1
fi

if [[ -f "$PROD_DIR/resources/weights/helmet.onnx" || -f "$PROD_DIR/resources/weights/helmet_ylv8_171125.pt" ]]; then
  if [[ ! -f "$PROD_DIR/resources/weights/helmet_b16_${PRECISION}.engine" ]]; then
    echo "WARNING: có model helmet nhưng chưa có helmet_b16_${PRECISION}.engine" >&2
  fi
fi

cp "$ROOT/docker-compose.prod.yaml" "$PROD_DIR/docker-compose.yaml"
cp "$ROOT/scripts/restart.sh" "$PROD_DIR/restart.sh"
chmod +x "$PROD_DIR/restart.sh"
printf '%s\n' "$IMAGE" > "$PROD_DIR/IMAGE"
printf '%s\n' "$TARGETARCH" > "$PROD_DIR/ARCH"
cat > "$PROD_DIR/.env" <<EOF
TARGETARCH=${TARGETARCH}
BASE_IMAGE=${BASE_IMAGE}
EOF

if docker ps -a --format '{{.Names}}' | grep -qx vehicle; then
  echo
  echo "WARNING: container tên 'vehicle' đã tồn tại."
  echo "         docker compose down ở source trước khi up ở target."
fi

echo
echo "Production: $PROD_DIR"
echo "Arch:       $TARGETARCH"
echo "Image:      $IMAGE  (giữ trong Docker daemon — không docker rmi / prune image này)"
echo
echo "Source có thể xóa. Chạy production:"
echo "  cd \"$PROD_DIR\" && docker compose up -d"
echo "  hoặc: \"$PROD_DIR/restart.sh\""
echo
docker image ls --format '{{.Repository}}:{{.Tag}}  {{.ID}}  {{.Size}}' "$IMAGE"
