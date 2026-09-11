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

## Everyday commands

    npm run ota       # build and push over wifi - the normal path
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

The Setup section of the web page has a firmware picker that posts to the same
endpoint, so a phone can update the board at the rig. Use `npm run bin` to get a
.bin somewhere findable first - arduino-cli otherwise leaves it in a cache
directory.

Wifi updates need a build that already contains the /update endpoint, so the
first install of it has to go over USB. After that USB is only needed if an
update leaves the board unable to join wifi.

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

## Note on DMX libraries

`esp_dmx` 4.1.0 does not compile against Arduino core 3.x - it reaches into
`uart_signal_conn_t.module`, removed in newer ESP-IDF, and 4.1.0 is the latest
release. DMX transmit is generated directly on UART1 instead (250kbaud 8N2,
100us break, 12us mark-after-break, 513 slots). No library needed.

## Sketches

- `haze_regulator/` - sensor + web UI + control loop
- `haze_sensor_test/` - wiring diagnostic
