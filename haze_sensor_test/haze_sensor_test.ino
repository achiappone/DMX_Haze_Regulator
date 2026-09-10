// Wiring check for PMSA003I on ESP32-S3 N16R8.
// STEMMA QT: black=GND, red=3V3, blue=SDA->GPIO8, yellow=SCL->GPIO9
#include <Adafruit_PM25AQI.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_PM25AQI aqi;
bool sensorReady = false;

void scanBus() {
  Serial.print("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  Serial.println(found ? "" : " nothing found - check power and SDA/SCL");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}  // USB CDC needs a moment to attach
  Wire.begin(SDA_PIN, SCL_PIN);
  scanBus();
  sensorReady = aqi.begin_I2C(&Wire);
  Serial.println(sensorReady ? "PMSA003I ready" : "PMSA003I not responding at 0x12");
}

void loop() {
  if (!sensorReady) {
    scanBus();  // hotplug: rescan so you can wire it with the monitor open
    sensorReady = aqi.begin_I2C(&Wire);
    if (sensorReady) Serial.println("PMSA003I ready");
    delay(1000);
    return;
  }
  PM25_AQI_Data d;
  if (aqi.read(&d)) {
    Serial.printf("PM1.0=%u PM2.5=%u PM10=%u ug/m3  AQI=%u | counts 0.3um=%u 0.5um=%u 1.0um=%u | ver/err=0x%04X\n",
                  d.pm10_env, d.pm25_env, d.pm100_env, d.aqi_pm25_us,
                  d.particles_03um, d.particles_05um, d.particles_10um, d.unused);
  } else {
    Serial.println("read failed (sensor emits a frame ~1/sec, this is normal occasionally)");
  }
  delay(1000);
}
