#!/bin/sh
# Poll the regulator and shout when it changes state. Runs from launchd every
# minute - see haze-watchdog.plist. Alerts on transitions only: a board that
# stays down does not need a notification every minute for a week.
# ponytail: only sees the board while this Mac is awake and on the rig's LAN.
# Move it to whatever always-on box ends up hosting cloudflared if that matters.
HOST="${1:-10.20.5.52}"
STATE="${TMPDIR:-/tmp}/haze-watchdog.state"
LOG="$HOME/Library/Logs/haze-watchdog.log"

if curl -fs --max-time 5 "http://$HOST/api/state" > /dev/null 2>&1; then
  now=up
else
  now=down
fi

was=$(cat "$STATE" 2>/dev/null || echo unknown)
[ "$now" = "$was" ] && exit 0
printf '%s' "$now" > "$STATE"
printf '%s %s %s -> %s\n' "$(date '+%F %T')" "$HOST" "$was" "$now" >> "$LOG"
osascript -e "display notification \"Haze regulator is $now ($HOST)\" with title \"Haze watchdog\"" 2>/dev/null || true
