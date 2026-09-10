# DMX Haze Regulator

Regulates DMX haze machine output from a PMSA003I PM2.5 sensor. ESP32-S3 N16R8.

## Status
- [x] PMSA003I over I2C
- [x] Web UI for live readings + tuning
- [x] DMX output (CTC-DRA-10-R2 shield)
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

Both machines share the same dead band: 000-010 does nothing, and 011-255 is
1-100%. Control values are therefore percent throughout, and `toDmx()` maps
0% to DMX 0 and 1-100% onto 11-255, so the number in the UI is the number the
machine reports. Without it the bottom 4% of travel would be a silent no-op.

Sources: Amhaze Stadium 2X IP User Manual Rev. 5 and Hurricane Haze 1DX User
Manual Rev. 3, "DMX Values".

**Avoid GPIO33-37** - wired to the module's 8MB octal PSRAM. GPIO26-32 are flash.

## Build

    cp haze_regulator/secrets.h.example haze_regulator/secrets.h   # then fill in
    arduino-cli compile -b esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M \
      -u -p /dev/cu.usbmodem2121301 haze_regulator

`CDCOnBoot=cdc` is required: the board's native USB-Serial/JTAG port is the only
one connected, so without it `Serial` goes to UART0 (GPIO43/44) and you see nothing.

Then open http://haze.local/

`haze_sensor_test/` is a standalone I2C scan + sensor readout for wiring problems.

## Note on DMX libraries

`esp_dmx` 4.1.0 does not compile against Arduino core 3.x - it reaches into
`uart_signal_conn_t.module`, removed in newer ESP-IDF, and 4.1.0 is the latest
release. DMX transmit is generated directly on UART1 instead (250kbaud 8N2,
100us break, 12us mark-after-break, 513 slots). No library needed.

## Sketches

- `haze_regulator/` - sensor + web UI + control loop
- `haze_sensor_test/` - wiring diagnostic
