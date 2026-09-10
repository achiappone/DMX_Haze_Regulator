// Haze regulator: PMSA003I PM2.5 sensor -> DMX haze machine output.
// Sensor  I2C   SDA=GPIO8  SCL=GPIO9  3V3  GND
// DMX     CTC-DRA-10-R2 shield, jumpers EN=on SLAVE=DE TX-io=TX-uart
//         shield TX->GPIO17, shield 2->GPIO16, 5V, GND. Leave 0 and 3 open.
#include <Adafruit_PM25AQI.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <assert.h>
#include <driver/uart.h>
#include "secrets.h"

#define SDA_PIN 8
#define SCL_PIN 9
#define DMX_TX_PIN 17
#define DMX_EN_PIN 16
#define DMX_UART UART_NUM_1
#define DMX_PACKET_SIZE 513  // start code + 512 slots
#define SENSOR_POLL_MS 250   // faster than the sensor's ~1s frame rate
#define SENSOR_STALE_MS 5000
#define DMX_INTERVAL_MS 30  // ~33Hz; full 513-slot packet takes ~23ms

Adafruit_PM25AQI aqi;
WebServer server(80);
uint8_t dmxData[DMX_PACKET_SIZE];

struct {
  bool automatic = true;
  uint8_t manual = 0;
  int setpoint = 150;    // target PM2.5 ug/m3
  int deadband = 10;     // no output until this far below setpoint
  float gain = 2.0f;     // output counts per ug/m3 of error
  int slew = 8;          // max output change per second
  int fan = 128;         // fan speed, independent of haze regulation
  int dmxAddress = 1;    // start address; +0 = fan, +1 = haze
} cfg;

PM25_AQI_Data data;
bool sensorOk = false, everRead = false;
uint8_t output = 0, target = 0;
unsigned long lastRead = 0, lastGoodRead = 0, lastDmx = 0, lastControl = 0;

// DMX512 straight onto the ESP32's UART: 250kbaud 8N2, a >=92us break, a
// >=12us mark-after-break, then the slots. That is the entire protocol for a
// transmitter, and esp_dmx 4.1.0 does not build against Arduino core 3.x.
void dmxBegin() {
  uart_config_t uc = {};
  uc.baud_rate = 250000;
  uc.data_bits = UART_DATA_8_BITS;
  uc.parity = UART_PARITY_DISABLE;
  uc.stop_bits = UART_STOP_BITS_2;
  uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uc.source_clk = UART_SCLK_DEFAULT;
  uart_driver_install(DMX_UART, 256, 2048, 0, NULL, 0);
  uart_param_config(DMX_UART, &uc);
  uart_set_pin(DMX_UART, DMX_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  pinMode(DMX_EN_PIN, OUTPUT);
  digitalWrite(DMX_EN_PIN, HIGH);  // transmit-only, so DE stays asserted
}

void dmxSend() {
  uart_wait_tx_done(DMX_UART, pdMS_TO_TICKS(DMX_INTERVAL_MS));
  uart_set_line_inverse(DMX_UART, UART_SIGNAL_TXD_INV);  // TX low = break
  delayMicroseconds(100);
  uart_set_line_inverse(DMX_UART, UART_SIGNAL_INV_DISABLE);
  delayMicroseconds(12);  // mark after break
  uart_write_bytes(DMX_UART, dmxData, DMX_PACKET_SIZE);
}

// Amhaze Stadium 2X IP treats DMX 0-10 as "no function" on both channels, so a
// raw level wastes the bottom 4% of the range. 0 stays off; 1-255 maps onto the
// live band 11-255.
uint8_t toDmx(uint8_t level) {
  if (level == 0) return 0;
  return 11 + (uint16_t)(level - 1) * 244 / 254;
}

// Proportional with deadband. Pure, so selfTest() can check it.
uint8_t computeOutput(int setpoint, int pm25, int deadband, float gain) {
  int err = setpoint - pm25;
  if (err <= deadband) return 0;
  long out = lroundf(gain * (err - deadband));
  return out > 255 ? 255 : (uint8_t)out;
}

// Rate-limits output movement. Doubles as the fix for two problems: haze takes
// ~30s to disperse so an unlimited proportional term hunts, and an unlimited
// step means the machine blasts full output the instant it boots.
uint8_t applySlew(uint8_t current, uint8_t target, int maxStep) {
  int d = (int)target - (int)current;
  if (d > maxStep) d = maxStep;
  if (d < -maxStep) d = -maxStep;
  return (uint8_t)(current + d);
}

void selfTest() {
  assert(computeOutput(150, 200, 10, 2.0f) == 0);   // too hazy -> off
  assert(computeOutput(150, 150, 10, 2.0f) == 0);   // at target -> off
  assert(computeOutput(150, 130, 10, 2.0f) == 20);  // err 20, less deadband, x2
  assert(computeOutput(150, 0, 10, 2.0f) == 255);   // clamps
  assert(applySlew(0, 255, 8) == 8);                // ramps up, no blast
  assert(applySlew(255, 0, 8) == 247);              // ramps down
  assert(applySlew(100, 102, 8) == 102);            // small step lands exactly
  assert(applySlew(0, 0, 8) == 0);
  assert(toDmx(0) == 0);      // off stays off
  assert(toDmx(1) == 11);     // lowest live value, skips the dead band
  assert(toDmx(255) == 255);  // full
  assert(toDmx(128) == 133);  // midpoint lands inside the live band
  Serial.println("selfTest ok");
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>Haze Regulator</title><style>
body{font:15px system-ui;margin:0;padding:16px;background:#111;color:#eee}
h1{font-size:17px;margin:0 0 12px}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(105px,1fr));gap:8px;margin-bottom:14px}
.c{background:#1c1c1c;border-radius:8px;padding:10px}
.c b{display:block;font-size:22px}.c span{color:#999;font-size:12px}
.bar{height:22px;background:#1c1c1c;border-radius:8px;overflow:hidden;margin:4px 0 6px;position:relative}
.bar i{display:block;height:100%;background:#4a9;width:0}
.bar u{position:absolute;top:0;height:100%;width:2px;background:#fff;opacity:.6}
label{display:block;margin:10px 0 2px;color:#999;font-size:12px}
input[type=range]{width:100%}
input[type=number]{background:#1c1c1c;color:#eee;border:1px solid #333;border-radius:6px;padding:6px;width:80px;font:inherit}
button{background:#333;color:#eee;border:0;border-radius:6px;padding:8px 14px;font:inherit}
button.on{background:#4a9;color:#000}
#warn{color:#e94;font-size:12px;min-height:16px;margin-bottom:8px}
</style>
<h1>Haze Regulator</h1>
<div class=g>
<div class=c><span>PM2.5 ug/m3</span><b id=pm25>-</b></div>
<div class=c><span>PM1.0</span><b id=pm10>-</b></div>
<div class=c><span>PM10</span><b id=pm100>-</b></div>
<div class=c><span>AQI US</span><b id=aqi>-</b></div>
<div class=c><span>0.3um count</span><b id=c03>-</b></div>
<div class=c><span>Level</span><b id=out>-</b></div>
<div class=c><span>DMX ch2 haze</span><b id=dmxh>-</b></div>
</div>
<div class=bar><i id=obar></i><u id=tmark></u></div>
<div id=warn></div>
<button id=mode onclick="post('automatic',this.dataset.v==1?0:1)">-</button>
<label>Manual level <span id=vman></span></label><input type=range id=manual min=0 max=255 oninput="post('manual',this.value)">
<label>Setpoint PM2.5 <span id=vsp></span></label><input type=range id=setpoint min=0 max=1000 oninput="post('setpoint',this.value)">
<label>Deadband <span id=vdb></span></label><input type=range id=deadband min=0 max=100 oninput="post('deadband',this.value)">
<label>Gain <span id=vg></span></label><input type=range id=gain min=1 max=100 oninput="post('gain',this.value/10)">
<label>Slew limit (counts/sec) <span id=vsl></span></label><input type=range id=slew min=1 max=255 oninput="post('slew',this.value)">
<label>Fan speed (ch1) <span id=vfan></span></label><input type=range id=fan min=0 max=255 oninput="post('fan',this.value)">
<label>DMX start address (ch1 fan, ch2 haze)</label><input type=number id=dmxaddr min=1 max=511 onchange="post('dmxaddr',this.value)">
<script>
let touching=0;
document.querySelectorAll('input[type=range]').forEach(e=>{
  e.onpointerdown=()=>touching=1; e.onpointerup=()=>touching=0;});
function post(k,v){fetch('/api/set?'+k+'='+v)}
async function tick(){
  let s=await(await fetch('/api/state')).json();
  pm25.textContent=s.pm25; pm10.textContent=s.pm10; pm100.textContent=s.pm100;
  aqi.textContent=s.aqi; c03.textContent=s.c03; out.textContent=s.output;
  dmxh.textContent=s.dmxhaze;
  obar.style.width=(s.output/255*100)+'%';
  tmark.style.left=(s.target/255*100)+'%';
  mode.textContent=s.automatic?'AUTO':'MANUAL';
  mode.className=s.automatic?'on':''; mode.dataset.v=s.automatic?1:0;
  warn.textContent=!s.sensorOk?'SENSOR LOST - output ramping to zero':
    (s.pm25>=990?'sensor near saturation - readings unreliable':'');
  vman.textContent=s.manual; vsp.textContent=s.setpoint; vdb.textContent=s.deadband;
  vg.textContent=s.gain.toFixed(1); vsl.textContent=s.slew;
  vfan.textContent=s.fan;
  if(!touching){manual.value=s.manual;setpoint.value=s.setpoint;
    deadband.value=s.deadband;gain.value=s.gain*10;slew.value=s.slew;fan.value=s.fan;}
  if(document.activeElement!=dmxaddr)dmxaddr.value=s.dmxaddr;
}
tick();setInterval(tick,1000);
</script>)HTML";

void handleState() {
  char buf[440];
  snprintf(buf, sizeof(buf),
           "{\"pm25\":%u,\"pm10\":%u,\"pm100\":%u,\"aqi\":%u,\"c03\":%u,"
           "\"output\":%u,\"target\":%u,\"automatic\":%s,\"manual\":%u,"
           "\"setpoint\":%d,\"deadband\":%d,\"gain\":%.1f,\"slew\":%d,"
           "\"fan\":%d,\"dmxaddr\":%d,\"dmxhaze\":%u,\"sensorOk\":%s}",
           everRead ? data.pm25_env : 0, everRead ? data.pm10_env : 0,
           everRead ? data.pm100_env : 0, everRead ? data.aqi_pm25_us : 0,
           everRead ? data.particles_03um : 0, output, target,
           cfg.automatic ? "true" : "false", cfg.manual, cfg.setpoint,
           cfg.deadband, cfg.gain, cfg.slew, cfg.fan, cfg.dmxAddress,
           toDmx(output), sensorOk ? "true" : "false");
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("automatic")) cfg.automatic = server.arg("automatic").toInt();
  if (server.hasArg("manual")) cfg.manual = constrain(server.arg("manual").toInt(), 0, 255);
  if (server.hasArg("setpoint")) cfg.setpoint = constrain(server.arg("setpoint").toInt(), 0, 1000);
  if (server.hasArg("deadband")) cfg.deadband = constrain(server.arg("deadband").toInt(), 0, 100);
  if (server.hasArg("gain")) cfg.gain = constrain(server.arg("gain").toFloat(), 0.1f, 10.0f);
  if (server.hasArg("slew")) cfg.slew = constrain(server.arg("slew").toInt(), 1, 255);
  if (server.hasArg("fan")) cfg.fan = constrain(server.arg("fan").toInt(), 0, 255);
  if (server.hasArg("dmxaddr")) {
    int a = constrain(server.arg("dmxaddr").toInt(), 1, 511);  // 2ch personality
    if (a != cfg.dmxAddress) {
      dmxData[cfg.dmxAddress] = 0;  // release both old channels
      dmxData[cfg.dmxAddress + 1] = 0;
      cfg.dmxAddress = a;
    }
  }
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  selfTest();

  Wire.begin(SDA_PIN, SCL_PIN);
  sensorOk = aqi.begin_I2C(&Wire);
  if (sensorOk) lastGoodRead = millis();
  Serial.println(sensorOk ? "PMSA003I ready" : "PMSA003I not responding at 0x12");

  dmxBegin();
  Serial.printf("DMX transmitting on GPIO%d, DE on GPIO%d\n", DMX_TX_PIN, DMX_EN_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("wifi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }
  // ponytail: no AP fallback yet. Regulation still runs headless without wifi;
  // add SoftAP or BLE provisioning when creds need changing at a venue.
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nhttp://%s/  (or http://haze.local/)\n",
                  WiFi.localIP().toString().c_str());
    MDNS.begin("haze");
  } else {
    Serial.println("\nno wifi - regulating headless");
  }

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", handleState);
  server.on("/api/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
  unsigned long now = millis();

  // The PMSA003I emits a frame about once a second on its own schedule, so a
  // failed read is routine, not a fault. Poll faster than that and judge health
  // only by how long it has been since a *good* read.
  if (now - lastRead >= SENSOR_POLL_MS) {
    lastRead = now;
    if (aqi.read(&data)) {
      everRead = true;
      lastGoodRead = now;
    } else if (now - lastGoodRead > SENSOR_STALE_MS) {
      aqi.begin_I2C(&Wire);  // genuinely gone; try to bring it back
    }
  }
  sensorOk = (now - lastGoodRead <= SENSOR_STALE_MS);

  if (now - lastControl >= 1000) {
    lastControl = now;
    if (!cfg.automatic) {
      target = cfg.manual;
    } else if (!everRead || !sensorOk) {
      target = 0;  // fail safe: never keep hazing on a stale reading
    } else {
      target = computeOutput(cfg.setpoint, data.pm25_env, cfg.deadband, cfg.gain);
    }
    output = applySlew(output, target, cfg.slew);
  }

  // Unconditional: frames keep going out at zero as well, so a receiver never
  // sees signal loss just because the haze is off.
  if (now - lastDmx >= DMX_INTERVAL_MS) {
    lastDmx = now;
    dmxData[0] = 0;                                   // DMX start code
    dmxData[cfg.dmxAddress] = toDmx(cfg.fan);         // ch1 fan speed
    dmxData[cfg.dmxAddress + 1] = toDmx(output);      // ch2 haze output
    dmxSend();
  }
}
