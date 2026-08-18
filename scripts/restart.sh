#!/usr/bin/env bash
# Restart production tại thư mục deploy (vehicle_prod).
# Luôn chạy detached (-d).
#
#   cd /mnt/atin/nhd/c12/vehicle_prod
#   ./restart.sh
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
COMPOSE="${DIR}/docker-compose.yaml"

if [[ ! -f "$COMPOSE" ]]; then
  echo "Không thấy docker-compose.yaml tại $DIR" >&2
  echo "Chạy deploy.sh từ source trước." >&2
  exit 1
fi

echo "==> docker compose down"
docker compose -f "$COMPOSE" --project-directory "$DIR" down

echo "==> docker compose up -d"
docker compose -f "$COMPOSE" --project-directory "$DIR" up -d

echo
echo "Production: $DIR"
docker compose -f "$COMPOSE" --project-directory "$DIR" ps
