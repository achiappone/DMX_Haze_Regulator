#!/bin/sh
# Push only the web interface. No firmware flash, no reboot, no reset integral.
# Usage: ./ui.sh [host]
#
# The page still lives in the .ino as a PROGMEM literal, which stays the
# fallback: if the filesystem copy is missing or a push is interrupted, the
# board serves the compiled one. ./ui.sh reset puts it back.
set -e
HOST="${1:-haze.local}"
SRC=haze_regulator/haze_regulator.ino
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

if [ "$HOST" = reset ]; then
  curl -f --netrc --max-time 20 "http://${2:-haze.local}/ui/reset"; echo; exit 0
fi

python3 - "$SRC" > "$OUT/index.html" <<'PY'
import re, sys
s = open(sys.argv[1]).read()
m = re.search(r'const char PAGE\[\] PROGMEM = R"HTML\((.*?)\)HTML";', s, re.S)
if not m:
    sys.exit("PAGE literal not found in " + sys.argv[1])
html = m.group(1)
# A compile-time splice would extract as literal C source and ship a broken
# page, so refuse rather than upload something that cannot work.
if ')HTML"' in html or 'R"HTML(' in html:
    sys.exit("PAGE still contains a compile-time splice; it cannot be served as a file")
sys.stdout.write(html)
PY

SIZE=$(wc -c < "$OUT/index.html" | tr -d ' ')
echo "uploading $SIZE bytes of ui to $HOST"
RESP=$(curl -f --netrc --no-progress-meter --max-time 120 -F "ui=@$OUT/index.html" "http://$HOST/ui")
case "$RESP" in
  OK*) ;;
  *) echo "FAILED: board rejected the ui: $RESP" >&2; exit 1;;
esac
SERVED=$(curl -fs --netrc --max-time 20 "http://$HOST/" | wc -c | tr -d ' ')
echo "ok - board is serving $SERVED bytes from the filesystem"
