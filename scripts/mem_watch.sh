#!/usr/bin/env bash
# Đo RAM/GPU của container vehicle theo thời gian — bắt memory leak / growth.
#
# Chạy trên host (cạnh docker-compose.yaml):
#   bash scripts/mem_watch.sh                         # sample 10 phút, không restart
#   bash scripts/mem_watch.sh --restart               # restart → đợi sẵn sàng → đo 10 phút
#   bash scripts/mem_watch.sh --duration 43200 --interval 60    # 12h, không restart
#
# ĐO DÀI 12–24h (khuyến nghị — rò rỉ chậm ~150 MiB/giờ cần cửa sổ >= 12h mới lộ):
#   nohup bash scripts/mem_watch.sh --duration 86400 --interval 60 \
#     --csv logs/mem_24h.csv > logs/mem_24h.out 2>&1 &
#   (container phải đang chạy với VEHICLE_MEM_STATS=1 + VEHICLE_LOG_LEVEL=debug)
#
# Cột quan trọng: heap_mb tách khỏi rss_mb.
#   heap tăng đều          → rò rỉ thật trong code app
#   heap phẳng, rss tăng   → CUDA/driver pool (uvm/anon) hoặc phân mảnh allocator
#
# A/B tcmalloc (glibc malloc phân mảnh nặng với JPEG buffer):
#   bash scripts/mem_watch.sh --duration 43200 --csv logs/mem_tcmalloc.csv   # mặc định có tcmalloc
#   LD_PRELOAD= docker compose up -d && bash scripts/mem_watch.sh --duration 43200 --csv logs/mem_glibc.csv
#
# A/B (khoanh vùng upload queue vs pipeline DS):
#   VEHICLE_EXTRA_ARGS=--dry-run VEHICLE_MEM_STATS=1 bash scripts/mem_watch.sh --restart --csv logs/mem_dryrun.csv
#   (dry-run bỏ HTTP upload — nếu RAM hết tăng → nghi queue JPEG khi upload timeout)
#
# Biến môi trường:
#   CONTAINER     tên container (mặc định vehicle)
#   COMPOSE_FILE  docker compose file (mặc định docker-compose.yaml)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

CONTAINER="${CONTAINER:-vehicle}"
COMPOSE_FILE="${COMPOSE_FILE:-docker-compose.yaml}"
DURATION_S=600
INTERVAL_S=15
DO_RESTART=0
CSV=""
READY_TIMEOUT_S=180

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \?//'
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --restart) DO_RESTART=1; shift ;;
    --duration) DURATION_S="$2"; shift 2 ;;
    --interval) INTERVAL_S="$2"; shift 2 ;;
    --csv) CSV="$2"; shift 2 ;;
    --container) CONTAINER="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "$ROOT/logs"
TS="$(date +%Y%m%d_%H%M%S)"
[[ -n "$CSV" ]] || CSV="$ROOT/logs/mem_watch_${TS}.csv"

# Một dòng CSV sạch: dùng Python để tránh lệch cột / regex jpeg_kb vs queue_jpeg_kb.
sample_once() {
  local elapsed="$1"
  CONTAINER="$CONTAINER" ELAPSED="$elapsed" python3 - <<'PY'
import os, re, subprocess, time

container = os.environ["CONTAINER"]
elapsed = os.environ["ELAPSED"]
ts = time.strftime("%Y-%m-%dT%H:%M:%S%z")
# +0700 → +07:00
if len(ts) >= 5 and ts[-5] in "+-" and ts[-3] != ":":
    ts = ts[:-2] + ":" + ts[-2:]

def sh(cmd, timeout=8):
    try:
        return subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL, timeout=timeout, text=True).strip()
    except Exception:
        return ""

docker_mem = sh(f'docker stats --no-stream --format "{{{{.MemUsage}}}}" {container}').split()[0] if True else ""
try:
    docker_mem = sh(f'docker stats --no-stream --format "{{{{.MemUsage}}}}" {container}').split("/")[0].strip()
except Exception:
    docker_mem = ""

rss_kb = vsz_kb = ""
top = sh(f'docker top {container} -eo pid,cmd')
host_pid = ""
for line in top.splitlines():
    if "/app/build/vehicle" in line:
        host_pid = line.split()[0]
        break
if host_pid and os.path.isfile(f"/proc/{host_pid}/status"):
    with open(f"/proc/{host_pid}/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                rss_kb = line.split()[1]
            elif line.startswith("VmSize:"):
                vsz_kb = line.split()[1]
if not rss_kb:
    rss_kb = sh(
        f'docker exec {container} bash -lc '
        f'"ps -eo rss,cmd --no-headers | awk \'/\\/app\\/build\\/vehicle/{{print \\$1; exit}}\'"'
    )

rss_mb = f"{int(rss_kb)/1024:.1f}" if rss_kb.isdigit() else ""

gpu_used = gpu_total = ""
smi = sh("nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits")
if smi:
    # Cộng mọi GPU (hoặc chỉ lấy GPU 0 nếu muốn: parts[0]).
    used_sum = total_sum = 0
    for row in smi.splitlines():
        parts = [p.strip() for p in row.split(",")]
        if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
            used_sum += int(parts[0])
            total_sum += int(parts[1])
    if total_sum:
        gpu_used, gpu_total = str(used_sum), str(total_sum)

# Tách RSS theo loại mapping — phân biệt rò rỉ heap (code app) với CUDA/driver pool.
# Đọc smaps TRONG container: /proc/<host_pid>/smaps trên host thường bị permission denied.
heap_mb = anon_mb = uvm_mb = ""
smaps = sh(f"docker exec {container} cat /proc/1/smaps", timeout=20)
if smaps:
    agg = {}
    cur = None
    for line in smaps.splitlines():
        if re.match(r"^[0-9a-f]+-[0-9a-f]+ ", line):
            parts = line.split()
            cur = parts[5] if len(parts) > 5 else "[anon]"
        elif line.startswith("Rss:") and cur:
            try:
                agg[cur] = agg.get(cur, 0) + int(line.split()[1])
            except (ValueError, IndexError):
                pass
    if agg:
        heap_mb = f"{agg.get('[heap]', 0)/1024:.1f}"
        anon_mb = f"{agg.get('[anon]', 0)/1024:.1f}"
        uvm_mb = f"{agg.get('/dev/nvidia-uvm', 0)/1024:.1f}"

tracks = queue = pending = samples = jpeg_kb = qjpeg_kb = zones = app_rss = ""
# Log debug rất dày → --tail nhỏ dễ trượt mem_stats. Lọc trước, giữ vài dòng cuối.
logs = sh(f"docker logs --tail 4000 {container} 2>&1 | grep 'mem_stats:' | tail -3", timeout=25)
# Lấy dòng mem_stats cuối.
for line in reversed(logs.splitlines()):
    if "mem_stats:" not in line:
        continue
    def grab(key):
        m = re.search(rf"(?<![A-Za-z0-9_]){key}=([0-9]+)", line)
        return m.group(1) if m else ""
    tracks = grab("tracks")
    queue = grab("queue")
    pending = grab("pending")
    samples = grab("samples")
    jpeg_kb = grab("jpeg_kb")
    qjpeg_kb = grab("queue_jpeg_kb")
    zones = grab("zones")
    app_rss = grab("rss_mb")
    break

fields = [
    ts, elapsed, docker_mem, rss_kb, rss_mb, vsz_kb, heap_mb, anon_mb, uvm_mb,
    gpu_used, gpu_total, tracks, queue, pending, samples, jpeg_kb, qjpeg_kb, zones, app_rss,
]
print(",".join(fields))
PY
}

wait_ready() {
  local deadline=$((SECONDS + READY_TIMEOUT_S))
  echo "== Đợi $CONTAINER sẵn sàng (timeout ${READY_TIMEOUT_S}s) =="
  while (( SECONDS < deadline )); do
    if docker exec "$CONTAINER" bash -lc 'pgrep -f "/app/build/vehicle" >/dev/null' 2>/dev/null; then
      sleep 8
      echo "OK: vehicle process đang chạy"
      return 0
    fi
    sleep 2
  done
  echo "TIMEOUT: không thấy /app/build/vehicle trong $CONTAINER" >&2
  return 1
}

if [[ "$DO_RESTART" -eq 1 ]]; then
  echo "== Restart $CONTAINER (compose: $COMPOSE_FILE) =="
  echo "   VEHICLE_MEM_STATS=${VEHICLE_MEM_STATS:-0}  VEHICLE_EXTRA_ARGS=${VEHICLE_EXTRA_ARGS:-}"
  docker compose -f "$COMPOSE_FILE" down --remove-orphans || true
  docker compose -f "$COMPOSE_FILE" up -d
  wait_ready
fi

if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "Container $CONTAINER không tồn tại — chạy compose hoặc --restart" >&2
  exit 1
fi

{
  echo "# mem_watch container=$CONTAINER duration=${DURATION_S}s interval=${INTERVAL_S}s"
  echo "# VEHICLE_MEM_STATS=${VEHICLE_MEM_STATS:-0} VEHICLE_EXTRA_ARGS=${VEHICLE_EXTRA_ARGS:-}"
  echo "timestamp,elapsed_s,docker_mem,rss_kb,rss_mb,vsz_kb,heap_mb,anon_mb,uvm_mb,gpu_used_mib,gpu_total_mib,tracks,queue,pending,samples,jpeg_kb,queue_jpeg_kb,zones,app_rss_mb"
} > "$CSV"

echo "== Sample mỗi ${INTERVAL_S}s trong ${DURATION_S}s → $CSV =="
printf '%-10s %7s %9s %8s %8s %8s %7s %6s %8s\n' \
  "time" "elaps" "rss_mb" "heap_mb" "anon_mb" "uvm_mb" "tracks" "queue" "jpeg_kb"

START=$SECONDS
FIRST_RSS=""
LAST_RSS=""
while (( SECONDS - START <= DURATION_S )); do
  elapsed=$((SECONDS - START))
  line=$(sample_once "$elapsed")
  echo "$line" >> "$CSV"
  IFS=',' read -r ts el dmem rkb rmb vsz hmb amb umb gu gt tr qu pe sa jk qjk zo arss <<< "$line"
  printf '%-10s %7s %9s %8s %8s %8s %7s %6s %8s\n' \
    "${ts:11:8}" "$el" "${rmb:-?}" "${hmb:-?}" "${amb:-?}" "${umb:-?}" "${tr:-}" "${qu:-}" "${jk:-}"
  if [[ -n "${rkb:-}" && "$rkb" =~ ^[0-9]+$ ]]; then
    [[ -z "$FIRST_RSS" ]] && FIRST_RSS="$rkb"
    LAST_RSS="$rkb"
  fi
  next=$((START + ((elapsed / INTERVAL_S) + 1) * INTERVAL_S))
  now=$SECONDS
  if (( now < next && next - START <= DURATION_S )); then
    sleep $((next - now))
  elif (( SECONDS - START >= DURATION_S )); then
    break
  else
    sleep "$INTERVAL_S"
  fi
done

echo
echo "== Kết quả =="
echo "CSV: $CSV"
if [[ -n "$FIRST_RSS" && -n "$LAST_RSS" ]]; then
  # Bỏ 20% mẫu đầu (warm-up: CUDA/TRT pool phình rồi mới plateau) trước khi tính xu hướng.
  # Hồi quy tuyến tính trên phần còn lại — chịu nhiễu tốt hơn so sánh 2 điểm đầu/cuối.
  # Không dùng interval regex {4} — mawk mặc định không hỗ trợ. Lọc theo cột số.
  LC_ALL=C awk -F',' -v dur="$DURATION_S" '
    $1 ~ /^20/ && $2 ~ /^[0-9]+$/ && $5 ~ /^[0-9.]+$/ {
      n++; e[n]=$2+0; r[n]=$5+0; h[n]=$7+0; u[n]=$9+0;
    }
    END{
      if (n < 4) { print "Quá ít mẫu để kết luận xu hướng."; exit }
      start = int(n*0.2)+1;
      cnt=0; sx=sy=sxx=sxy=0; shy=0; sxh=0; sxxh=0;
      hs=0; he=0; us=0; ue=0;
      for (i=start; i<=n; i++) {
        x=e[i]/60.0; y=r[i];
        cnt++; sx+=x; sy+=y; sxx+=x*x; sxy+=x*y;
        shy+=h[i]; sxh+=x*h[i]; sxxh+=x*x;
        if (cnt==1) { hs=h[i]; us=u[i] }
        he=h[i]; ue=u[i];
      }
      den = cnt*sxx - sx*sx;
      slope = (den!=0) ? (cnt*sxy - sx*sy)/den : 0;
      denh = cnt*sxxh - sx*sx;
      slopeh = (denh!=0) ? (cnt*sxh - sx*shy)/denh : 0;
      printf "Cửa sổ ổn định: %d mẫu (bỏ %d mẫu warm-up đầu)\n", cnt, start-1;
      printf "RSS  : %.1f → %.1f MiB   xu hướng %+.2f MiB/phút (%+.0f MiB/giờ)\n",
             r[start], r[n], slope, slope*60;
      printf "heap : %.1f → %.1f MiB   xu hướng %+.2f MiB/phút (%+.0f MiB/giờ)\n",
             hs, he, slopeh, slopeh*60;
      printf "uvm  : %.1f → %.1f MiB\n", us, ue;
      print "";
      # Ngưỡng theo MiB/giờ — rò rỉ thật ở app thường lộ rõ trên heap.
      hph = slopeh*60; rph = slope*60;
      if (hph > 50)
        print "→ HEAP TĂNG ĐỀU: rò rỉ trong code app (container C++ / allocator). Xem cột tracks/queue/samples/jpeg_kb cùng thời điểm.";
      else if (rph > 100 && hph <= 50)
        print "→ RSS tăng nhưng heap phẳng: nghi CUDA/driver pool (uvm/anon) hoặc phân mảnh allocator, KHÔNG phải container app.";
      else if (rph > 30)
        print "→ Tăng nhẹ: cần cửa sổ dài hơn (>=6h) để phân biệt xu hướng thật với nhiễu.";
      else
        print "→ Ổn định: không thấy xu hướng tăng có ý nghĩa trong cửa sổ đo.";
      if (dur < 21600)
        print "LƯU Ý: cửa sổ < 6h — rò rỉ chậm (~150 MiB/giờ) có thể chưa lộ. Nên chạy >= 12h.";
    }' "$CSV"
else
  echo "Không lấy được RSS — kiểm tra container/process."
fi

if docker logs --tail 400 "$CONTAINER" 2>&1 | grep -q 'mem_stats:'; then
  echo
  echo "App mem_stats (tail):"
  docker logs --tail 400 "$CONTAINER" 2>&1 | grep 'mem_stats:' | tail -5
else
  echo
  echo "Gợi ý: bật VEHICLE_MEM_STATS=1 để thấy tracks/queue/jpeg_kb theo phần."
fi
