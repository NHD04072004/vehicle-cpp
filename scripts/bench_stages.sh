#!/usr/bin/env bash
# Đo FPS từng chặng của pipeline PLATE bằng gst-launch (cùng model/config với app).
# Chạy trong container DeepStream:
#   docker exec vehicle_test bash -lc 'bash /app/scripts/bench_stages.sh 30'
# Tham số: [số giây đo mỗi chặng] [uri1] [uri2 ...]
set -uo pipefail

DUR="${1:-30}"
shift || true
if [[ $# -gt 0 ]]; then
  URIS=("$@")
else
  URIS=("rtsp://192.168.1.196:18554/cambienso_1" "rtsp://192.168.1.196:18554/camcaotoc_1")
fi

ROOT="${VEHICLE_ROOT:-/app}"
export USE_NEW_NVSTREAMMUX=yes
export GST_DEBUG=1

MUX_CFG="$ROOT/resources/ds/streammux/config_mux.txt"
# PGIE_CFG/MUX_BATCH: đổi khi test số camera > batch của engine mặc định (b8).
PGIE="${PGIE_CFG:-$ROOT/resources/ds/infer/pgie_vehicle.txt}"
MUX_BATCH="${MUX_BATCH:-8}"
SGIE1="$ROOT/resources/ds/infer/sgie1_plate_pose.txt"
PRE="$ROOT/resources/ds/infer/config_preprocess_warp_plate.txt"
SGIE2="$ROOT/resources/ds/infer/sgie2_digit.txt"
TRK_CFG="$ROOT/resources/ds/tracker/config_tracker_NvDCF_perf.yml"
TRK_LIB="/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so"

SRC=""
i=0
for u in "${URIS[@]}"; do
  SRC+="nvurisrcbin uri=$u gpu-id=0 latency=200 drop-on-latency=1 select-rtp-protocol=4 ! m.sink_$i "
  i=$((i + 1))
done
NSRC=$i

MUX="nvstreammux name=m batch-size=$MUX_BATCH batched-push-timeout=40000 attach-sys-ts=1 config-file-path=$MUX_CFG"
SINK="fpsdisplaysink name=fps video-sink=fakesink text-overlay=false sync=false"

run_stage() {
  local label="$1"
  local chain="$2"
  local pipe="$SRC $MUX"
  [[ -n "$chain" ]] && pipe+=" ! $chain"
  pipe+=" ! $SINK"

  # Bỏ giai đoạn khởi động (kết nối RTSP + nạp engine): chỉ lấy 8 mẫu `current` cuối.
  local out
  out=$(timeout -s INT "$DUR" gst-launch-1.0 -v $pipe 2>/dev/null |
        grep -o 'current: [0-9.]*' | tail -8 | awk '{s+=$2; n++} END {if(n) printf "%.1f", s/n}')
  printf '%-34s | %6s batch/s\n' "$label" "${out:-n/a}"
}

echo "== Bench $DUR s/chặng, $NSRC nguồn =="
run_stage "A decode+mux" ""
run_stage "B +PGIE(xe 640)" "nvinfer config-file-path=$PGIE unique-id=1 batch-size=$MUX_BATCH"
run_stage "C +tracker" "nvinfer config-file-path=$PGIE unique-id=1 batch-size=$MUX_BATCH ! nvtracker ll-lib-file=$TRK_LIB ll-config-file=$TRK_CFG tracker-width=640 tracker-height=384"
BASE="nvinfer config-file-path=$PGIE unique-id=1 batch-size=$MUX_BATCH ! nvtracker ll-lib-file=$TRK_LIB ll-config-file=$TRK_CFG tracker-width=640 tracker-height=384"
run_stage "D +SGIE1(plate pose)" "$BASE ! nvinfer config-file-path=$SGIE1 unique-id=2 process-mode=2"
run_stage "E +warp preprocess" "$BASE ! nvinfer config-file-path=$SGIE1 unique-id=2 process-mode=2 ! nvdspreprocess config-file=$PRE"
run_stage "F +SGIE2(digit tensor)" "$BASE ! nvinfer config-file-path=$SGIE1 unique-id=2 process-mode=2 ! nvdspreprocess config-file=$PRE ! nvinfer config-file-path=$SGIE2 unique-id=3 process-mode=2 input-tensor-meta=1"
run_stage "G +convert+OSD (full)" "$BASE ! nvinfer config-file-path=$SGIE1 unique-id=2 process-mode=2 ! nvdspreprocess config-file=$PRE ! nvinfer config-file-path=$SGIE2 unique-id=3 process-mode=2 input-tensor-meta=1 ! nvvideoconvert ! nvdsosd display-text=0"
