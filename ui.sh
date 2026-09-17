#!/bin/sh
# Push only the web interface. No firmware flash, no reboot, no reset integral.
# Usage: ./ui.sh [host]
#
# The page still lives in the .ino as a PROGMEM literal, which stays the
# fallback: if the filesystem copy is missing or a push is interrupted, the
# board serves the compiled one. ./ui.sh reset puts it back.
set -e
HOST="${1:-haze.local}"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

if [ "$HOST" = reset ]; then
  curl -f --netrc --max-time 20 "http://${2:-haze.local}/ui/reset"; echo; exit 0
fi

cp ui/index.html "$OUT/index.html"

SIZE=$(wc -c < "$OUT/index.html" | tr -d ' ')
echo "uploading $SIZE bytes of ui to $HOST"
RESP=$(curl -f --netrc --no-progress-meter --max-time 120 -F "ui=@$OUT/index.html" "http://$HOST/ui")
case "$RESP" in
  OK*) ;;
  *) echo "FAILED: board rejected the ui: $RESP" >&2; exit 1;;
esac
SERVED=$(curl -fs --netrc --max-time 20 "http://$HOST/" | wc -c | tr -d ' ')
echo "ok - board is serving $SERVED bytes from the filesystem"
