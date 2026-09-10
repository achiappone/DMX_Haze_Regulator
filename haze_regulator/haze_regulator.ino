// Haze regulator - dev webserver stage.
// PMSA003I on I2C: SDA=GPIO8 SCL=GPIO9, 3V3, GND.
// DMX output not wired yet; the computed level is displayed only.
#include <Adafruit_PM25AQI.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <assert.h>
#include "secrets.h"

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_PM25AQI aqi;
WebServer server(80);

struct {
  bool automatic = true;
  uint8_t manual = 0;     // 0-255 level used in manual mode
  int setpoint = 150;     // target PM2.5 ug/m3
  int deadband = 10;      // no output until this far below setpoint
  float gain = 2.0f;      // output counts per ug/m3 of error
} cfg;

PM25_AQI_Data data;
bool sensorOk = false, everRead = false;
uint8_t output = 0;
unsigned long lastRead = 0;

// Proportional with deadband. Pure so it can be self-tested.
// ponytail: ignores the ~30s lag between output and measured haze. If it hunts,
// add a slew limit or an integral term rather than lowering the gain.
uint8_t computeOutput(int setpoint, int pm25, int deadband, float gain) {
  int err = setpoint - pm25;
  if (err <= deadband) return 0;
  long out = lroundf(gain * (err - deadband));
  return out > 255 ? 255 : (uint8_t)out;
}

void selfTest() {
  assert(computeOutput(150, 200, 10, 2.0f) == 0);    // too hazy -> off
  assert(computeOutput(150, 150, 10, 2.0f) == 0);    // at target -> off
  assert(computeOutput(150, 130, 10, 2.0f) == 20);   // err 20, minus deadband, x2
  assert(computeOutput(150, 0, 10, 2.0f) == 255);    // clamps
  Serial.println("selfTest ok");
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>Haze Regulator</title><style>
body{font:15px system-ui;margin:0;padding:16px;background:#111;color:#eee}
h1{font-size:17px;margin:0 0 12px}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:8px;margin-bottom:16px}
.c{background:#1c1c1c;border-radius:8px;padding:10px}
.c b{display:block;font-size:22px}.c span{color:#999;font-size:12px}
.bar{height:22px;background:#1c1c1c;border-radius:8px;overflow:hidden;margin:4px 0 16px}
.bar i{display:block;height:100%;background:#4a9;width:0}
label{display:block;margin:10px 0 2px;color:#999;font-size:12px}
input[type=range]{width:100%}
button{background:#333;color:#eee;border:0;border-radius:6px;padding:8px 14px;font:inherit}
button.on{background:#4a9;color:#000}
#warn{color:#e94;font-size:12px}
</style>
<h1>Haze Regulator</h1>
<div class=g>
<div class=c><span>PM2.5 ug/m3</span><b id=pm25>-</b></div>
<div class=c><span>PM1.0</span><b id=pm10>-</b></div>
<div class=c><span>PM10</span><b id=pm100>-</b></div>
<div class=c><span>AQI US</span><b id=aqi>-</b></div>
<div class=c><span>0.3um count</span><b id=c03>-</b></div>
<div class=c><span>Output</span><b id=out>-</b></div>
</div>
<div class=bar><i id=obar></i></div>
<div id=warn></div>
<button id=mode onclick="post('automatic',this.dataset.v==1?0:1)">-</button>
<label>Manual level <span id=vman></span></label><input type=range id=manual min=0 max=255 oninput="post('manual',this.value)">
<label>Setpoint PM2.5 <span id=vsp></span></label><input type=range id=setpoint min=0 max=1000 oninput="post('setpoint',this.value)">
<label>Deadband <span id=vdb></span></label><input type=range id=deadband min=0 max=100 oninput="post('deadband',this.value)">
<label>Gain <span id=vg></span></label><input type=range id=gain min=1 max=100 oninput="post('gain',this.value/10)">
<script>
let touching=0;
document.querySelectorAll('input').forEach(e=>{
  e.onpointerdown=()=>touching=1; e.onpointerup=()=>touching=0;});
function post(k,v){fetch('/api/set?'+k+'='+v)}
async function tick(){
  let s=await(await fetch('/api/state')).json();
  pm25.textContent=s.pm25; pm10.textContent=s.pm10; pm100.textContent=s.pm100;
  aqi.textContent=s.aqi; c03.textContent=s.c03; out.textContent=s.output;
  obar.style.width=(s.output/255*100)+'%';
  mode.textContent=s.automatic?'AUTO':'MANUAL';
  mode.className=s.automatic?'on':''; mode.dataset.v=s.automatic?1:0;
  warn.textContent=!s.sensorOk?'sensor not responding':(s.pm25>=990?'sensor near saturation - readings unreliable':'');
  vman.textContent=s.manual; vsp.textContent=s.setpoint;
  vdb.textContent=s.deadband; vg.textContent=s.gain.toFixed(1);
  if(!touching){manual.value=s.manual;setpoint.value=s.setpoint;
    deadband.value=s.deadband;gain.value=s.gain*10;}
}
tick();setInterval(tick,1000);
</script>)HTML";

void handleState() {
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"pm25\":%u,\"pm10\":%u,\"pm100\":%u,\"aqi\":%u,\"c03\":%u,"
           "\"output\":%u,\"automatic\":%s,\"manual\":%u,\"setpoint\":%d,"
           "\"deadband\":%d,\"gain\":%.1f,\"sensorOk\":%s}",
           everRead ? data.pm25_env : 0, everRead ? data.pm10_env : 0,
           everRead ? data.pm100_env : 0, everRead ? data.aqi_pm25_us : 0,
           everRead ? data.particles_03um : 0, output,
           cfg.automatic ? "true" : "false", cfg.manual, cfg.setpoint,
           cfg.deadband, cfg.gain, sensorOk ? "true" : "false");
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("automatic")) cfg.automatic = server.arg("automatic").toInt();
  if (server.hasArg("manual")) cfg.manual = constrain(server.arg("manual").toInt(), 0, 255);
  if (server.hasArg("setpoint")) cfg.setpoint = constrain(server.arg("setpoint").toInt(), 0, 1000);
  if (server.hasArg("deadband")) cfg.deadband = constrain(server.arg("deadband").toInt(), 0, 100);
  if (server.hasArg("gain")) cfg.gain = constrain(server.arg("gain").toFloat(), 0.1f, 10.0f);
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  selfTest();

  Wire.begin(SDA_PIN, SCL_PIN);
  sensorOk = aqi.begin_I2C(&Wire);
  Serial.println(sensorOk ? "PMSA003I ready" : "PMSA003I not responding at 0x12");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("wifi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nhttp://%s/  (or http://haze.local/)\n", WiFi.localIP().toString().c_str());
  MDNS.begin("haze");

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", handleState);
  server.on("/api/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();

  if (millis() - lastRead >= 1000) {
    lastRead = millis();
    if (!sensorOk) {
      sensorOk = aqi.begin_I2C(&Wire);
    } else if (aqi.read(&data)) {
      everRead = true;
    } else {
      sensorOk = false;  // dropped off the bus; loop above re-inits
    }
    output = cfg.automatic
                 ? (everRead ? computeOutput(cfg.setpoint, data.pm25_env, cfg.deadband, cfg.gain) : 0)
                 : cfg.manual;
    // TODO: write `output` to DMX here once the RS-485 driver is wired.
  }
}
