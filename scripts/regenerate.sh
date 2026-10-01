#!/usr/bin/env bash
# Regenerate every reported number from raw captures.
set -euo pipefail
cd "$(dirname "$0")/.."
PY="${PY:-python}"
DATA="${DATA:-../data}"

echo "[1/3] LiDAR cold run"
PYTHONPATH=. "$PY" -m floorplan.cli "$DATA/single_scan_with_ceiling" --tier lidar --out runs/lidar

echo "[2/3] Photo tier"
PYTHONPATH=. "$PY" -m floorplan.cli runs/photo_input --tier photo --out runs/photo

echo "[3/3] Video tier"
PYTHONPATH=. "$PY" -m floorplan.cli "$DATA/single_scan_with_ceiling/rgb.mp4" --tier video --out runs/video

echo "[verify] contract tests"
PYTHONPATH=. "$PY" -m pytest tests -q
