#!/usr/bin/env bash
# Một script cho build + chạy (dev & production ENTRYPOINT).
#
#   bash vehicle.sh build [-DBUILD_TESTS=OFF ...]
#   bash vehicle.sh run [--source …] [--dry-run] …
#   bash vehicle.sh [--source …]          # = run (Docker ENTRYPOINT)
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
BIN="$BUILD_DIR/vehicle"

do_build() {
  local build_type="${BUILD_TYPE:-Release}"
  local jobs="${JOBS:-$(nproc)}"
  cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$build_type" "$@"
  cmake --build "$BUILD_DIR" -j "$jobs"
  echo
  echo "Binary : $BUILD_DIR/vehicle"
  echo "Tests  : $BUILD_DIR/vehicle_business_tests"
  echo "Clangd : $ROOT/compile_commands.json"
}

do_run() {
  if [[ ! -x "$BIN" ]]; then
    echo "Chưa có binary — chạy: bash vehicle.sh build" >&2
    exit 1
  fi

  export GST_DEBUG="${GST_DEBUG:-1}"
  export VEHICLE_ROOT="${VEHICLE_ROOT:-$ROOT}"
  export USE_NEW_NVSTREAMMUX="${USE_NEW_NVSTREAMMUX:-yes}"

  local extra=()
  if [[ -n "${VEHICLE_EXTRA_ARGS:-}" ]]; then
    # shellcheck disable=SC2206
    extra=(${VEHICLE_EXTRA_ARGS})
  fi
  exec "$BIN" --root "${VEHICLE_ROOT:-$ROOT}" "$@" "${extra[@]}"
}

case "${1:-}" in
  build)
    shift
    do_build "$@"
    ;;
  run)
    shift
    do_run "$@"
    ;;
  -h|--help)
    sed -n '2,8p' "$0" | sed 's/^# \?//'
    ;;
  *)
    # Không có subcommand: Docker ENTRYPOINT hoặc gọi tắt → chạy app.
    do_run "$@"
    ;;
esac
