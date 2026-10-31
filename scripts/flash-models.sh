#!/bin/sh
# Flash the esp-sr srmodels.bin (WakeNet model data) to the "model"
# partition at 0x10000 (partitions/16m.csv: model, spiffs, 0x10000, 0xF0000).
# The file is auto-packed by the esp-sr build from the SR_WN_* selections
# in sdkconfig.defaults (currently wn9_alexa) into build/srmodels/.
# Note: plain `idf.py flash` already flashes this partition too (esp-sr
# hooks esptool_py_flash_to_partition); this script is the standalone
# re-flash path when only the model changes.
# Usage: sh scripts/flash-models.sh [/dev/ttyACM0] [build-dir]
set -eu
PORT="${1:-/dev/ttyACM0}"
BUILD="${2:-build}"
BIN="$BUILD/srmodels/srmodels.bin"
if [ ! -f "$BIN" ]; then
  echo "flash-models: FAIL: $BIN not found (run idf.py build first)" >&2
  exit 1
fi
# shellcheck disable=SC1091
if [ -z "${IDF_PATH:-}" ]; then
  # shellcheck disable=SC1090
  . ~/esp-idf/v5.5/export.sh
fi
python -m esptool --chip esp32s3 -p "$PORT" -b 460800 \
  write_flash 0x10000 "$BIN"
echo "flash-models: OK ($BIN -> $PORT @ 0x10000)"
