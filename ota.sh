#!/bin/sh
# Build and push firmware over wifi. Usage: ./ota.sh [host]
# Requires a build already carrying the /update endpoint - the first install of
# that endpoint has to go over USB.
set -e
HOST="${1:-haze.local}"
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi"
OUT=$(mktemp -d)
arduino-cli compile --warnings all -b "$FQBN" --output-dir "$OUT" haze_regulator
BIN="$OUT/haze_regulator.ino.bin"
echo "uploading $(wc -c < "$BIN") bytes to $HOST"
curl -f --max-time 180 -F "firmware=@$BIN" "http://$HOST/update"
echo
rm -rf "$OUT"
