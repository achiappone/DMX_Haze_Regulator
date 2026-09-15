#!/bin/sh
# Build and push firmware over wifi. Usage: ./ota.sh [host]
# Requires a build already carrying the /update endpoint - the first install of
# that endpoint has to go over USB.
set -e
HOST="${1:-haze.local}"
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
arduino-cli compile --warnings all -b "$FQBN" --output-dir "$OUT" haze_regulator
BIN="$OUT/haze_regulator.ino.bin"
echo "uploading $(wc -c < "$BIN") bytes to $HOST"
curl -f --netrc --no-progress-meter --max-time 300 -F "firmware=@$BIN" "http://$HOST/update"
echo

# A build that dies in setup() never reaches WiFi, so it cannot be reflashed
# over the air - recovery means walking to the rig with a USB cable. Never let
# a push end without proof the board came back.
echo "waiting for $HOST to come back"
i=0
while [ $i -lt 30 ]; do
  sleep 2
  if curl -fs --netrc --max-time 3 "http://$HOST/api/state" > /dev/null 2>&1; then
    echo "back up"
    curl -fs --netrc --max-time 5 "http://$HOST/api/events" 2>/dev/null \
      | grep -i selftest && echo "^^ self-test failures - fix before trusting this build" || true
    exit 0
  fi
  i=$((i + 1))
done
echo "FAILED: $HOST did not answer within 60s - it may be boot looping." >&2
echo "Recover over USB: arduino-cli compile -b '$FQBN' -u -p /dev/cu.usbmodem* haze_regulator" >&2
exit 1
