#!/bin/sh
# Build and push firmware over wifi. Usage: ./ota.sh [host]
# Works across the Cloudflare tunnel: the image goes up in chunks, each its own
# request, and a chunk whose reply is lost is resumed rather than restarted.
# Requires a build already carrying /update/chunk - the first install of that
# endpoint has to go over USB or the LAN.
set -e
HOST="${1:-haze.local}"
FQBN="esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi"
SRC=haze_regulator/haze_regulator.ino
# 128 KiB. Cloudflare gives an origin 100s per request and a whole 1.1MB image
# over wifi at -76dBm does not fit; a chunk this size does, with room to spare.
CHUNK=131072
MAXFAIL=3
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

VER=$(sed -n 's/^#define FW_VERSION "\(.*\)"/\1/p' "$SRC")
[ -n "$VER" ] || { echo "cannot read FW_VERSION from $SRC" >&2; exit 1; }

arduino-cli compile --warnings all -b "$FQBN" --output-dir "$OUT" haze_regulator
BIN="$OUT/haze_regulator.ino.bin"
SIZE=$(wc -c < "$BIN" | tr -d ' ')
echo "uploading $SIZE bytes to $HOST as $VER"

# Connection: close on every chunk. ESP32's WebServer serves one client at a
# time and cloudflared keeps origin connections alive between requests, so a
# run of chunks otherwise leaves idle sockets the board is still nursing.
post_chunk() {
  curl -f --netrc --no-progress-meter --max-time 90 --http1.1 \
    -H "Connection: close" -F "firmware=@$OUT/chunk" \
    "http://$HOST/update/chunk?off=$1&total=$SIZE&last=$2" 2>/dev/null
}

OFF=0
FAILS=0
LASTLOST=0
RESTARTED=0
while [ "$OFF" -lt "$SIZE" ]; do
  dd if="$BIN" of="$OUT/chunk" bs="$CHUNK" skip=$((OFF / CHUNK)) count=1 2>/dev/null
  LEN=$(wc -c < "$OUT/chunk" | tr -d ' ')
  if [ $((OFF + LEN)) -ge "$SIZE" ]; then LAST=1; else LAST=0; fi

  if RESP=$(post_chunk "$OFF" "$LAST"); then
    case "$RESP" in
      FAILED*) echo "FAILED: board rejected the image at $OFF: $RESP" >&2; exit 1;;
    esac
    OFF=$((OFF + LEN))
    FAILS=0
    printf '\r  %s/%s bytes' "$OFF" "$SIZE"
    continue
  fi

  # No usable reply. On the final chunk that proves nothing - the board
  # finishes the image and reboots there, so the reply is expected to die.
  if [ "$LAST" = 1 ]; then LASTLOST=1; break; fi

  FAILS=$((FAILS + 1))
  if [ "$FAILS" -gt "$MAXFAIL" ]; then
    echo >&2; echo "FAILED: $MAXFAIL retries at offset $OFF, giving up" >&2; exit 1
  fi
  # Back off before touching it again. A chunk at offset 0 makes the board call
  # Update.begin(), which erases the target partition and blocks the web server
  # while it does - so an immediate retry lands on a board that cannot answer,
  # fails, and erases again. Eight of those in a row took the board off the
  # network entirely; the retry loop was causing the failures it was retrying.
  sleep $((FAILS * 5))
  # The chunk may have landed with only the reply lost, so ask the board where
  # it actually is rather than assuming either way.
  ST=$(curl -fs --netrc --max-time 20 "http://$HOST/update/chunk" 2>/dev/null)
  BOFF=$(echo "$ST" | cut -d' ' -f1)
  BRUN=$(echo "$ST" | cut -d' ' -f2)
  # A chunk cut off partway leaves the board holding more bytes than we sent and
  # the run aborted. Its offset is then neither "landed" nor "behind us", so the
  # old logic retried the same chunk forever. The running flag is the honest
  # signal: no run, no resume.
  if [ "$BRUN" = "0" ]; then
    if [ "$RESTARTED" -ge 3 ]; then
      echo >&2; echo "FAILED: board dropped the run 3 times; link is too lossy" >&2; exit 1
    fi
    RESTARTED=$((RESTARTED + 1))
    echo >&2; echo "  board aborted at $BOFF, restarting the image ($RESTARTED/3)" >&2
    OFF=0
    continue
  fi
  case "$BOFF" in
    ''|*[!0-9]*) sleep 2 ;;
    *)
      if [ "$BOFF" -ge $((OFF + LEN)) ]; then OFF=$((OFF + LEN))
      elif [ "$BOFF" -lt "$OFF" ]; then
        # Restarting means another full erase, so only ever do it once.
        RESTARTED=$((RESTARTED + 1))
        echo >&2; echo "  board dropped the run at $BOFF, restarting once" >&2
        OFF=0
      fi ;;
  esac
done
echo
[ "$LASTLOST" = 1 ] && echo "last chunk reply lost, which is normal - the version check decides"

# Confirm the board is running the build we just sent. Polling /api/state only
# proves something answers, and a failed update leaves the OLD firmware
# answering instantly - which is exactly how a failed flash once reported
# success. The version string is the only honest check.
echo "waiting for $HOST to come back as $VER"
i=0
while [ $i -lt 30 ]; do
  sleep 2
  # From the API, not by scraping the page. The page is 20KB and the version
  # moved out of it when the UI became replaceable; /api/state is a few hundred
  # bytes and still answers when the link is too poor to deliver the page.
  RUNNING=$(curl -fs --netrc --max-time 5 "http://$HOST/api/state" 2>/dev/null \
            | sed -n 's/.*"fw":"\([0-9.]*\)".*/\1/p') || true
  if [ "$RUNNING" = "$VER" ]; then
    echo "back up on $VER"
    # Only this boot's log. Grepping the whole file reports a failure that was
    # fixed two flashes ago, which is worse than not checking at all.
    curl -fs --netrc --max-time 5 "http://$HOST/api/events" 2>/dev/null \
      | awk '/ boot: /{out=""} {out = out $0 ORS} END{printf "%s", out}' \
      | grep -i selftest && echo "^^ self-test failures - fix before trusting this build" || true
    exit 0
  fi
  i=$((i + 1))
done
echo "FAILED: $HOST is running '${RUNNING:-nothing}', expected $VER." >&2
echo "curl -X POST --netrc http://$HOST/api/reboot clears a wedged updater." >&2
exit 1
