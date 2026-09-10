# DMX Haze Regulator

Regulates DMX haze machine output from a PMSA003I PM2.5 sensor. ESP32-S3 N16R8.

## Status
- [x] PMSA003I over I2C
- [x] Web UI for live readings + tuning
- [ ] DMX output (RS-485)
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

**Avoid GPIO33-37** - wired to the module's 8MB octal PSRAM. GPIO26-32 are flash.

## Build

    cp haze_regulator/secrets.h.example haze_regulator/secrets.h   # then fill in
    arduino-cli compile -b esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M \
      -u -p /dev/cu.usbmodem2121301 haze_regulator

`CDCOnBoot=cdc` is required: the board's native USB-Serial/JTAG port is the only
one connected, so without it `Serial` goes to UART0 (GPIO43/44) and you see nothing.

Then open http://haze.local/

`haze_sensor_test/` is a standalone I2C scan + sensor readout for wiring problems.

## Sketches

- `haze_regulator/` - sensor + web UI + control loop
- `haze_sensor_test/` - wiring diagnostic
