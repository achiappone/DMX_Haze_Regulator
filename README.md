# DMX Haze Regulator

Regulates DMX haze machine output from a PMSA003I PM2.5 sensor. ESP32-S3 N16R8.

## Status
- [x] PMSA003I over I2C
- [x] Web UI for live readings + tuning
- [x] DMX output (CTC-DRA-10-R2 shield)
- [x] Live chart, selectable window (1 min to 7 days)
- [x] Purge - haze off, fan wide open, for a set duration
- [ ] BLE wifi provisioning
- [ ] Local display + buttons

## Wiring

STEMMA QT / Qwiic cable from the PMSA003I breakout:

| Wire   | Signal | ESP32-S3 |
|--------|--------|----------|
| black  | GND    | GND      |
| red    | VIN    | 3V3      |
| blue   | SDA    | GPIO8    |
| yellow | SCL    | GPIO9    |

Sensor is at I2C address 0x12, fixed. Pullups are on the breakout.

### DMX - CTC-DRA-10-R2 shield

Do not stack it; it is an Uno footprint. Four jumper wires:

| Shield  | ESP32-S3 |
|---------|----------|
| 5V      | 5V       |
| GND     | GND      |
| TX (1)  | GPIO17   |
| 2       | GPIO16   |

Shield jumpers: `EN`=on, `SLAVE`=DE, `TX-io`=TX-uart, `RX-io`=either.

**Leave header pins 0 and 3 disconnected.** That is the MAX485's receiver
output - a 5V push-pull driver into a 3.3V GPIO. Transmit-only costs nothing
here and is what removes the need for level shifting.

The shield is NOT isolated. For venue use consider the CTC-DRA-13-R2, which is
the isolated version of the same board - a ground potential difference between
the ESP32 and a hazer on stage power has nowhere to go but through this.

XLR out is the male connector: pin 1 ground, pin 2 cold, pin 3 hot.

## Fixtures

Selectable in the web UI. Set the machine's start address to match `dmxaddr`.

**Chauvet Amhaze Stadium 2X IP** - 2 channels, max start address 511

| Ch | Function    | 000-010     | 011-255            |
|----|-------------|-------------|--------------------|
| 1  | Fan Speed   | no function | 1-100% slow..fast  |
| 2  | Haze Output | no function | 1-100% thin..heavy |

Note the order: **fan is channel 1, haze is channel 2.**

**Chauvet Hurricane Haze 1DX** - 1 channel, max start address 512

| Ch | Function    | 000-010     | 011-255           |
|----|-------------|-------------|-------------------|
| 1  | Haze Volume | no function | 1-100% low..high  |

No fan channel; the fan slider hides when this profile is selected. Needs
2 minutes to heat up before it will output.

**Set the fixture profile to match the machine actually connected.** The
profiles put different functions on channel 1 - fan on the Amhaze, haze on the
1DX. Selecting Amhaze while a 1DX is plugged in means Purge drives ch1 to 255
expecting a fan, and the 1DX reads that as full haze output.

Both machines share the same dead band: 000-010 does nothing, and 011-255 is
1-100%. Control values are therefore percent throughout, and `toDmx()` maps
0% to DMX 0 and 1-100% onto 11-255, so the number in the UI is the number the
machine reports. Without it the bottom 4% of travel would be a silent no-op.

Sources: Amhaze Stadium 2X IP User Manual Rev. 5 and Hurricane Haze 1DX User
Manual Rev. 3, "DMX Values".

**Avoid GPIO33-37** - wired to the module's 8MB octal PSRAM. GPIO26-32 are flash.

## Control loop

The room is modelled as one equation:

    d(level)/dt = rise * duty - level / decayTau

`rise` is what the machine adds at 100% demand, `duty` is the demand as a
fraction, and `decayTau` is how fast the room clears on its own. Everything the
loop does is a consequence of this plus a dead time - the delay between
commanding haze and the sensor seeing it.

Measured on the rig in this repo, September 2026:

| constant | value | how |
|---|---|---|
| dead time | 64s | calibration; visible as 60s of full output before the sensor moved at all |
| rise | 15-18 /s per 100% demand | calibration, then maintained by the loop |
| decayTau | 89s | fitting 300s of undisturbed decay, R2 0.945 |
| integralTi | 250s | tuning, about four dead times |

**The dead time is the whole problem.** Sixty-four seconds of no feedback means
the loop can commit an enormous dose before seeing any of it arrive, and a
controller tuned as if feedback were immediate will oscillate. `doseCap` exists
for this: it refuses to commit more than closes the gap plus what the room will
leak while the dose is in flight.

### Control signal and target band

Three signals can be regulated: PM2.5 mass, PM1.0 mass, or the 0.3um particle
count / 100. The count has far more resolution at low haze, where PM2.5 mass
quantises into uselessness - clean air reads 0 or 1.

**The band is expressed in whatever units the selected signal uses.** The same
air is roughly 175 of count/100, 205 of PM2.5 and 75 of PM1.0. The chart shades
the band on the mass axis, so with the count signal selected the shaded region
is drawn at numbers that mean something else entirely and no PM line can sit in
it. Select the signal you actually want to hold.

Each signal keeps its own setpoint, deadband and rise rate, so switching between
them is lossless. The first switch to a signal converts the target by the
measured ratio; after that it is remembered. Converting every time lost up to 8%
of the target per round trip, because the ratio between two noisy readings is
itself noisy.

`setpoint` and `deadband` are stored; min and max are derived - `computePI`
zeroes the error anywhere inside +/-deadband, so the pair has always described a
band and min/max is the same band said out loud.

### What adapts, and on what timescale

Three loops, deliberately separated - stacking adaptive layers on one process is
how you get a limit cycle:

- **the PI**, seconds
- **cap relaxation**, minutes - if the loop is capped, below band and not
  rising, the cap is too tight whichever constant is wrong. Widens 15%/min,
  bounded at 4x, given back on reaching the band.
- **rise estimation**, tens of minutes - solves the model for `rise` from ten
  minutes of the loop's own output. Discards windows whose level moved more than
  a third of its average, since those measure the swing rather than the machine.

`decayTau` does **not** adapt, and cannot be measured during normal operation:
with a 64s dead time, "output off" never means "nothing arriving", so the decay
fits come back anywhere from 58s to 351s. Measuring it needs minutes of a
genuinely quiet room. This is the open weakness - two unknowns and one equation,
with the estimator solving for `rise` given a `decayTau` that nothing maintains.

### What pulse mode actually delivers

    delivered = demand * pminon / (pminon + tail)

Burst level does not appear in that. `pulsePeriodNow` stretches the period in
proportion to the burst level, so a gentler burst runs correspondingly longer
and the average is unchanged. Measured over 19 minutes each: `eff` 0.35 at burst
60 and 0.37 at burst 40, with the peak on the wire moving 60% to 40% as
expected. Burst level changes the shape, not the amount - it is a smoothness
control, not a delivery control.

`machineTail` is subtracted from every burst on the assumption the machine keeps
emitting that long after the command drops. Two attempts to measure it directly
failed - the first because `slew` at 1%/s turns a step into a triangle, the
second because auto-purge silently overrides manual output - so it was settled
by consequence instead:

| | delivery | burst | cv | in band | demand |
|---|---|---|---|---|---|
| pminon 2s, tail 4s | 0.35 | ~2s | 0.24 | 93% | 14.3% |
| pminon 8s, tail 4s | 0.66 | ~8s | 0.31-0.59 | 30-48% | 5-9.5% |
| **pminon 2s, tail 1s** | **0.64** | **~2s** | **0.19** | **86%** | **6.6%** |

The same delivery is reachable by lengthening the burst or by subtracting less
tail, and the two are not equivalent: longer bursts deliver the haze in slugs
four times larger and the level swings accordingly. Burst *level* changes shape
and not amount; burst *duration* changes both.

If the machine really coasted 4s, cutting the tail to 1s would over-command
every burst by three seconds and the room would run high. It does not - it sits
mid-band on 6.6% demand with the cap not binding. **The real coast is at most a
second, and the 4s default was discarding two thirds of every dose for
nothing.** Settings that work on this rig: `pminon` 2s, `tail` 1s, burst level
40.

### Pulse mode

Below roughly 20% demand a hazer cannot deliver a proportional trickle, so the
output is pulsed. `plevel` caps the burst amplitude and the on-time stretches to
compensate, so the average delivered is the demand either way: at 14% demand,
level 100 is a 2s full-power slug every 43s, level 20 is 20% for 70% of a 9s
cycle. Same haze, spread out - and the period shortens on its own, because a
gentler burst is already longer and needs less stretching to clear `pminon`.

Set it too low and the machine stops emitting altogether: at burst level 20 this
rig logged six NO RESPONSE events at 18-26% demand with the level falling
throughout.

## Everyday commands

    ./ui.sh <host>    # push the web interface only - no reboot, ~1s
    npm run ota       # build and push firmware over wifi - reboots the board
    npm run bin       # build a .bin into build/ for the web uploader
    npm run build     # compile only
    npm run usb       # compile and flash over USB
    npm run monitor   # serial monitor
    npm run state     # current readings and settings
    npm run logs      # event log
    npm run csv       # download the last hour as CSV
    npm run stop      # engage STOP
    npm run go        # release STOP
    npm run open      # open the control page

Host, serial port and FQBN live in the `config` block of package.json.

In WebStorm these also appear in the Run dropdown - "Firmware - OTA update over
wifi" is the usual one - and in the npm tool window.

The Setup section of the web page has a firmware picker that posts to the same
endpoint, so a phone can update the board at the rig. Use `npm run bin` to get a
.bin somewhere findable first - arduino-cli otherwise leaves it in a cache
directory.

Wifi updates need a build that already contains the /update endpoint, so the
first install of it has to go over USB. After that USB is only needed if an
update leaves the board unable to join wifi.

### Interface and firmware are separate

The page lives in `ui/index.html` and is served from the filesystem; `./ui.sh`
uploads it in about a second with no reboot. A label change does not deserve a
1.1MB flash, a reset integral and a restarted adaptation window - and those
reboots were themselves a problem, since twenty in an afternoon meant the rise
estimator never completed a ten minute window.

The `.ino` still carries a 1KB **recovery page**: a warning, a file picker that
posts to `/ui`, and a dump of `/api/state`. It serves whenever the filesystem
copy is missing, so a bad upload cannot leave the board without a way in. An
upload writes to a temp file and renames over the live page only once complete,
the previous page is kept as a backup the recovery page can restore, and
`./ui.sh reset` forces the built-in page deliberately.

### OTA over a poor link

`ota.sh` sends the image in 128KB chunks, each its own request. Cloudflare gives
an origin 100 seconds and a whole 1.1MB image over wifi at -76dBm does not fit -
six attempts in a row returned 502 with the upload still in flight. A chunk
whose reply is lost resumes rather than restarting: `GET /update/chunk` reports
how far the board got, and whether a run is still live.

Verification reads the version from `/api/state`, not by scraping the page. A
failed update leaves the **old** firmware answering instantly, which is exactly
how a failed flash once reported success.

## Build

    cp haze_regulator/secrets.h.example haze_regulator/secrets.h   # then fill in
    npm run usb

The FQBN is `esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi`.
PSRAM holds the high-resolution trend buffer.

`CDCOnBoot=cdc` is required: the board's native USB-Serial/JTAG port is the only
one connected, so without it `Serial` goes to UART0 (GPIO43/44) and you see nothing.

Then open http://haze.local/

`haze_sensor_test/` is a standalone I2C scan + sensor readout for wiring problems.

## Purge

Purge clears the air using the *machine's* fan: haze snaps to 0 (bypassing the
slew limit, since cutting output is always the safe direction) and the fan goes
to 100% for the chosen duration. On the 1DX there is no fan channel, so purge
just stops output.

The PMSA003I has no purge function of its own. Its fan is either running or
asleep, controlled by the SET pad on the breakout - not over I2C, and not
reachable through the STEMMA QT cable, which carries only SDA, SCL, VIN, GND.

### Three adaptive layers, and how each has misbehaved

They run on deliberately separated timescales - the PI in seconds, cap
relaxation in minutes, rise estimation in tens of minutes - because every time
two of them have interacted, the result looked like a control problem and was
not.

* **The rise estimator learned from its own transients.** A higher rise tightens
  the cap, which drops output and the level, which the next window reads as a
  weaker machine, which widens the cap again. That oscillates at twice the
  window length, and a 10-15 minute hunt against a 10 minute window is the
  fingerprint. It now discards any window whose level moved more than a third of
  its own average.
* **The decay fitter ratcheted.** Its bias was one-directional, which is the
  dangerous kind: a window where late arrival offsets decay fits a shallow
  slope, a shallow slope reads as an enormous tau, and every such window pushed
  the estimate up while none pushed it back. It walked 89s to 567s while the
  room was visibly draining at a rate implying 160s, cutting the cap's leak term
  from 142 to 15 and halving the dose cap. Now: two dead times of quiet, the
  level must fall 20%, and the fit must reach r2 0.8.
* **The cap relaxer is the backstop for both.** Capped, below band and not
  rising can only mean the cap is too tight, whichever constant is wrong, and
  that judgement needs no constants at all. When it sits above 1.0x for long,
  something upstream is lying.

Measured either side of the decay fix, same band, same room:

| | before | after |
|---|---|---|
| in band | 17-53% | 70-99% |
| cv (sd/mean) | 0.46-0.77 | 0.11-0.34 |
| rise estimate | sliding 11.2 -> 9.1 | steady ~8.1 |
| capRelax | up to 2.66x | 1.0x |

## Things that cost a day

Recorded because each one looked like a different problem than it was.

**A self-test that aborts cannot be recovered over the air.** `selfTest()` runs
before `WiFi.begin`, so an assert that fired left the board in a panic-reboot
loop with no route in but USB. It sat like that for four days while the fix
existed in source four minutes later. Checks now report and continue.

**An interrupted OTA used to wedge the updater.** `Update.begin()` was called on
every upload start and nothing aborted a run that stopped partway, so after the
first interrupted upload every later flash answered FAILED until a reboot. There
was no remote reboot either; there is now, at `/api/reboot`.

**None of the four hardware watchdogs catch a starved loop.** Both timer group
watchdogs, the RTC one and the super watchdog all detect stoppage. Twice this
board served the 20KB page at 14 and then 71 bytes a second while answering
pings in 20ms and feeding the task watchdog on every pass. Healthy to a
watchdog, down to anyone opening the page. `/api/state` now reports loop passes
per second - about 986 when healthy - and the board restarts itself if that
collapses, or reassociates wifi if responses crawl past 8 seconds.

**Reassociating fixed the slow link, not rebooting.** At -81dBm the page served
71 B/s; a reboot reassociated at -70 and it served 102KB/s. The reboot was
incidental. Trigger on response time rather than RSSI - it read -81 while
degraded, and an RSSI threshold at -85 would have watched it fail.

**Calibration measures the peak of a plume, not what the room does.** It
reported `rise` 16.71/s from an 82s full-power burst. It also measured the decay
at 403s where a clean fit gives 89s, and that number sets the integral time and
the leak model. Re-running calibration will overwrite a good `decayTau` with a
bad one.

**Chart decimation has to aggregate, not sample.** Taking every Nth sample made
pulse bursts blink in and out as the window scrolled, because whether a burst
was caught depended on where the stride landed. It takes the maximum per bucket
now, which reads high for a pulsed signal on wide windows but never hides an
excursion.

**"Did not rise" is not "did not respond."** The no-response check required the
level to climb, so it called the machine dead whenever the room could empty
faster than the machine filled it - which is most of the time here. Every such
entry in a day's log re-judged as responding once compared against what doing
nothing would have done: the room draining 115-200 per window while the machine
held the loss to 13-97. Same block, second bug: the window re-armed with
`pm25_env` while the comparison used `ctrlValue()`, so regulating on anything
else compared one signal against another and fired regardless of what the
machine did.

**Three numbers had two names.** "Haze output" was the controller's demand, not
what the machine was doing, and the on-wire percentage was in `/api/state` but
never shown. So 8% demand beside DMX 0 looked like a fault rather than two ends
of a duty cycle - at 15% demand with a 14s period the wire is live for about 2s
of it. They are now Haze demand, On wire now, and DMX value.

**A scrolled chart window drifts.** `off` is an age measured from now, so a
fixed offset slides forward as time passes and a window parked over history
walks off it. The pan is anchored when set and the offset grows with elapsed
time.

## Note on DMX libraries

`esp_dmx` 4.1.0 does not compile against Arduino core 3.x - it reaches into
`uart_signal_conn_t.module`, removed in newer ESP-IDF, and 4.1.0 is the latest
release. DMX transmit is generated directly on UART1 instead (250kbaud 8N2,
100us break, 12us mark-after-break, 513 slots). No library needed.

## Sketches

- `haze_regulator/` - sensor + web UI + control loop
- `haze_sensor_test/` - wiring diagnostic
