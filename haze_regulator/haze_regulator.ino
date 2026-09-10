// Haze regulator: PMSA003I PM2.5 sensor -> DMX haze machine output.
// Sensor  I2C   SDA=GPIO8  SCL=GPIO9  3V3  GND
// DMX     CTC-DRA-10-R2 shield, jumpers EN=on SLAVE=DE TX-io=TX-uart
//         shield TX->GPIO17, shield 2->GPIO16, 5V, GND. Leave 0 and 3 open.
#include <Adafruit_PM25AQI.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <assert.h>
#include <string.h>
#include <driver/uart.h>
#include "secrets.h"

#define SDA_PIN 8
#define SCL_PIN 9
#define DMX_TX_PIN 17
#define DMX_EN_PIN 16
#define DMX_UART UART_NUM_1
#define DMX_PACKET_SIZE 513  // start code + 512 slots
#define SLOPE_WIN 30       // seconds of PM history used for the trend
#define SENSOR_POLL_MS 250   // faster than the sensor's ~1s frame rate
#define SENSOR_STALE_MS 5000
#define DMX_INTERVAL_MS 30  // ~33Hz; full 513-slot packet takes ~23ms

Adafruit_PM25AQI aqi;
WebServer server(80);
uint8_t dmxData[DMX_PACKET_SIZE];
Preferences prefs;

// Channel layout differs per machine. Offsets are from the start address; -1
// means the fixture has no such channel. Both machines use the same 011-255
// live band with 000-010 dead, so toDmx() is shared.
struct Fixture {
  const char *name;
  uint8_t chans;
  int8_t fanOff;
  int8_t hazeOff;
};
const Fixture FIXTURES[] = {
    {"Amhaze Stadium 2X IP", 2, 0, 1},  // ch1 fan, ch2 haze
    {"Hurricane Haze 1DX", 1, -1, 0},   // ch1 haze only, no fan
};
#define FIXTURE_COUNT (sizeof(FIXTURES) / sizeof(FIXTURES[0]))

struct {
  bool automatic = true;
  uint8_t manual = 0;    // 0-100%
  int setpoint = 150;    // target PM2.5 ug/m3
  int deadband = 10;     // no output until this far below setpoint
  float gain = 1.0f;     // output % per ug/m3 of error
  int slew = 3;          // max % change per second
  int fan = 50;          // 0-100%, independent of haze regulation
  int dmxAddress = 1;    // start address; channel layout depends on fixture
  int fixture = 0;       // index into FIXTURES
  int lookahead = 0;     // seconds to extrapolate the PM trend; 0 = off
  bool stopped = false;  // latched stop; survives reboot on purpose
  bool pulseMode = false;  // time-proportional output instead of continuous
  int pulsePeriod = 10;    // seconds per pulse cycle
  int pulseLevel = 100;    // output % during the on part of the cycle
  int calPulse = 30;       // calibration pulse length, seconds
  int pulseMinOn = 2;      // auto mode: shortest burst worth firing, seconds
} cfg;

PM25_AQI_Data data;
bool sensorOk = false, everRead = false;
uint8_t output = 0, target = 0;
unsigned long lastRead = 0, lastGoodRead = 0, lastDmx = 0, lastControl = 0;
unsigned long purgeUntil = 0;
unsigned long saveAt = 0;  // debounce NVS writes; a slider drag is many changes
// Calibration measures the three numbers that are properties of the room, not
// the equipment: how long haze takes to arrive, how fast it accumulates at full
// output, and how slowly it clears. Guessing these is what causes overshoot.
enum { CAL_OFF = 0, CAL_PURGE, CAL_SETTLE, CAL_PULSE, CAL_DECAY, CAL_DONE, CAL_FAIL };
#define CAL_SETTLE_S 10
int calState = CAL_OFF;
unsigned long calT0 = 0, calPeakT = 0;
int calBaseline = 0, calPeak = 0, calDead = 0, calTau = 0, calPulseUsed = 0;
bool calBaseHigh = false;
float calRise = 0;
char calMsg[128] = "";
bool calibrating() { return calState >= CAL_PURGE && calState <= CAL_DECAY; }

uint16_t pmHist[SLOPE_WIN];
int pmCount = 0, pmIdx = 0;
float pmSlope = 0;   // ug/m3 per second, negative when haze is clearing
int predicted = 0;

// Purge clears the air with the machine's own fan: haze off, fan wide open.
// Overflow-safe compare, so it cannot latch on at the millis() rollover.
bool purging() { return purgeUntil && (int32_t)(millis() - purgeUntil) < 0; }

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

// The Amhaze documents both channels as "1-100%" spread over DMX 011-255, with
// 000-010 a dead "no function" band. Taking percent in means the number on the
// UI is the same number the machine reports.
uint8_t toDmx(uint8_t pct) {
  if (pct == 0) return 0;  // off is off, not "lowest live value"
  return 11 + (uint16_t)(pct - 1) * 244 / 99;
}

// Trend over the whole window rather than between consecutive samples: the
// PMSA003I is noisy enough that a two-point derivative is mostly noise.
float slopeOf(int newest, int oldest, int samples) {
  return samples < 2 ? 0.0f : (float)(newest - oldest) / (samples - 1);
}

// Where PM2.5 will be once haze commanded now actually reaches the sensor.
// Controlling on this instead of the present reading is what buys back the
// dead time between opening the machine and seeing the result.
int predict(int pm, float slopePerSec, int lookaheadSec) {
  long p = pm + lroundf(slopePerSec * lookaheadSec);
  return p < 0 ? 0 : (p > 2000 ? 2000 : (int)p);
}

// Time-proportional output: instead of holding a low continuous level, run the
// machine at a strong level for a fraction of each cycle. Hazers atomise poorly
// at low duty, so 20% as 2s-on/8s-off often disperses better than a steady 20%.
uint8_t dutyLevel(unsigned long ms, int demand, int period, int level) {
  if (demand <= 0) return 0;
  if (demand >= 100) return 100;
  unsigned long per = (unsigned long)period * 1000;
  return (ms % per) < per * demand / 100 ? (uint8_t)level : 0;
}

// A fixture occupying n channels cannot start later than 513-n.
int maxAddress(uint8_t chans) { return DMX_PACKET_SIZE - chans; }

void calFail(const char *m) {
  snprintf(calMsg, sizeof(calMsg), "%s", m);
  calState = CAL_FAIL;
  target = output = 0;
}

// Ziegler-Nichols for a dead-time process gives Kp = 100/(rise*dead); halved,
// because this plant is violently asymmetric - haze arrives in seconds and
// clears over many minutes, so overshoot is far more expensive than droop.
void calFinish() {
  if (calDead < 1) calDead = 1;
  if (calPulseUsed < 1) calPulseUsed = 1;
  calRise = (float)(calPeak - calBaseline) / calPulseUsed;
  cfg.lookahead = constrain(calDead, 0, 120);
  cfg.slew = constrain(100 / calDead, 1, 100);
  if (calRise > 0.01f)
    cfg.gain = constrain(50.0f / (calRise * calDead), 0.1f, 10.0f);
  snprintf(calMsg, sizeof(calMsg),
           "pulse %ds, dead %ds, rise %.1f ug/s, decay %ds -> look %d slew %d gain %.1f",
           calPulseUsed, calDead, calRise, calTau, cfg.lookahead, cfg.slew,
           cfg.gain);
  if (calBaseHigh)
    strncat(calMsg, " (hazy baseline: rise may be understated)",
            sizeof(calMsg) - strlen(calMsg) - 1);
  calState = CAL_DONE;
  target = output = 0;
  saveAt = millis() + 500;
}

void runCalibration(unsigned long now, int pm) {
  unsigned long el = (now - calT0) / 1000;
  int trig = calBaseline + max(10, calBaseline / 6);  // clear of sensor noise
  switch (calState) {
    case CAL_PURGE:
      // Clear the room first. Measuring from a hazy baseline squeezes the
      // excursion against the sensor ceiling and gives a useless rise rate.
      target = output = 0;
      purgeUntil = now + 2000;  // rolling, so the fan stays open via purge
      // A room with no forced ventilation clears slowly, so "stopped falling"
      // needs minutes of evidence, not one minute. Only refuse when the sensor
      // ceiling leaves no room for a measurable excursion at all.
      if (pm < 60 || el > 1800 || (el > 300 && pmSlope > -0.1f)) {
        purgeUntil = 0;
        if (pm > 850) {
          calFail("still above 850 ug/m3 - ventilate the room and retry");
          break;
        }
        calBaseHigh = pm > 400;
        calT0 = now;
        calState = CAL_SETTLE;
      }
      break;
    case CAL_SETTLE:
      target = output = 0;
      if (el >= CAL_SETTLE_S) {
        calBaseline = pm; calPeak = pm; calDead = 0; calPeakT = 0;
        calPulseUsed = 0;
        calT0 = now; calState = CAL_PULSE;
      }
      break;
    case CAL_PULSE:
      target = output = 100;
      if (pm > calPeak) { calPeak = pm; calPeakT = now; }
      if (!calDead && pm > trig) calDead = el < 1 ? 1 : el;
      // Stop early if the sensor nears its ~1000 ug/m3 ceiling: a saturated
      // reading flattens the peak and would understate the rise rate.
      if (el >= (unsigned long)cfg.calPulse || pm >= 900) {
        calPulseUsed = el < 1 ? 1 : el;
        calT0 = now;
        calState = CAL_DECAY;
      }
      break;
    case CAL_DECAY:
      target = output = 0;
      if (pm > calPeak) { calPeak = pm; calPeakT = now; }
      if (!calDead && pm > trig) calDead = calPulseUsed + el;
      if (!calDead && el > 45) {
        calFail("no PM rise - machine off, not hazing, or sensor too far");
        break;
      }
      // Decay constant: time from the peak to 37% of the excursion.
      if (calDead && calPeakT && calPeak > calBaseline + 10 &&
          pm <= calBaseline + (calPeak - calBaseline) * 37 / 100) {
        calTau = (now - calPeakT) / 1000;
        calFinish();
      } else if (el > (unsigned long)max(900, cfg.calPulse * 4)) {
        calTau = el;  // never fell to 37%; record what we waited
        calFinish();
      }
      break;
  }
}

// Settings survive a reboot or a reflash. Losing them silently was actively
// dangerous: the fixture profile would revert to a 2-channel machine, and Purge
// would then drive ch1 to full expecting a fan, which a 1DX reads as full haze.
// One key per setting. A blob keyed on a version number meant that adding any
// new field invalidated every stored value, which silently reset the fixture
// profile - the one setting that is genuinely unsafe to get wrong.
void saveCfg() {
  prefs.putBool("auto", cfg.automatic);
  prefs.putUChar("manual", cfg.manual);
  prefs.putInt("setpoint", cfg.setpoint);
  prefs.putInt("deadband", cfg.deadband);
  prefs.putFloat("gain", cfg.gain);
  prefs.putInt("slew", cfg.slew);
  prefs.putInt("fan", cfg.fan);
  prefs.putInt("addr", cfg.dmxAddress);
  prefs.putInt("fixture", cfg.fixture);
  prefs.putInt("look", cfg.lookahead);
  prefs.putBool("stopped", cfg.stopped);
  prefs.putBool("pulse", cfg.pulseMode);
  prefs.putInt("pperiod", cfg.pulsePeriod);
  prefs.putInt("plevel", cfg.pulseLevel);
  prefs.putInt("calpulse", cfg.calPulse);
  prefs.putInt("pminon", cfg.pulseMinOn);
}

void loadCfg() {
  cfg.automatic = prefs.getBool("auto", cfg.automatic);
  cfg.manual = prefs.getUChar("manual", cfg.manual);
  cfg.setpoint = prefs.getInt("setpoint", cfg.setpoint);
  cfg.deadband = prefs.getInt("deadband", cfg.deadband);
  cfg.gain = prefs.getFloat("gain", cfg.gain);
  cfg.slew = prefs.getInt("slew", cfg.slew);
  cfg.fan = prefs.getInt("fan", cfg.fan);
  cfg.dmxAddress = prefs.getInt("addr", cfg.dmxAddress);
  cfg.fixture = prefs.getInt("fixture", cfg.fixture);
  cfg.lookahead = prefs.getInt("look", cfg.lookahead);
  cfg.stopped = prefs.getBool("stopped", cfg.stopped);
  cfg.pulseMode = prefs.getBool("pulse", cfg.pulseMode);
  cfg.pulsePeriod = prefs.getInt("pperiod", cfg.pulsePeriod);
  cfg.pulseLevel = prefs.getInt("plevel", cfg.pulseLevel);
  cfg.calPulse = prefs.getInt("calpulse", cfg.calPulse);
  cfg.pulseMinOn = prefs.getInt("pminon", cfg.pulseMinOn);
  // Clamp everything: stored bytes are not trustworthy input.
  cfg.fixture = constrain(cfg.fixture, 0, (int)FIXTURE_COUNT - 1);
  cfg.dmxAddress =
      constrain(cfg.dmxAddress, 1, maxAddress(FIXTURES[cfg.fixture].chans));
  cfg.manual = constrain(cfg.manual, 0, 100);
  cfg.fan = constrain(cfg.fan, 0, 100);
  cfg.setpoint = constrain(cfg.setpoint, 0, 1000);
  cfg.deadband = constrain(cfg.deadband, 0, 100);
  cfg.gain = constrain(cfg.gain, 0.1f, 10.0f);
  cfg.slew = constrain(cfg.slew, 1, 100);
  cfg.lookahead = constrain(cfg.lookahead, 0, 120);
  cfg.pulsePeriod = constrain(cfg.pulsePeriod, 5, 60);
  cfg.pulseLevel = constrain(cfg.pulseLevel, 10, 100);
  cfg.calPulse = constrain(cfg.calPulse, 30, 600);
  cfg.pulseMinOn = constrain(cfg.pulseMinOn, 1, 10);
}

// Proportional with deadband. Pure, so selfTest() can check it.
uint8_t computeOutput(int setpoint, int pm25, int deadband, float gain) {
  int err = setpoint - pm25;
  if (err <= deadband) return 0;
  long out = lroundf(gain * (err - deadband));
  return out > 100 ? 100 : (uint8_t)out;
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
  assert(computeOutput(150, 200, 10, 1.0f) == 0);   // too hazy -> off
  assert(computeOutput(150, 150, 10, 1.0f) == 0);   // at target -> off
  assert(computeOutput(150, 130, 10, 1.0f) == 10);  // err 20, less deadband
  assert(computeOutput(150, 0, 10, 1.0f) == 100);   // clamps at 100%
  assert(applySlew(0, 100, 3) == 3);                // ramps up, no blast
  assert(applySlew(100, 0, 3) == 97);               // ramps down
  assert(applySlew(40, 41, 3) == 41);               // small step lands exactly
  assert(applySlew(0, 0, 3) == 0);
  assert(toDmx(0) == 0);      // off stays off
  assert(toDmx(1) == 11);     // 1% = lowest live value, skips the dead band
  assert(toDmx(100) == 255);  // 100%
  assert(toDmx(50) == 131);   // 50% lands mid-band
  assert(maxAddress(2) == 511);  // Amhaze, 2ch
  assert(maxAddress(1) == 512);  // Hurricane Haze 1DX, 1ch
  assert(slopeOf(100, 10, 31) == 3.0f);      // +90 over 30s
  assert(slopeOf(10, 100, 31) == -3.0f);     // falling
  assert(slopeOf(50, 50, 1) == 0.0f);        // too few samples
  assert(predict(100, -2.0f, 30) == 40);     // falling fast, act early
  assert(predict(10, -2.0f, 30) == 0);       // clamps at zero
  assert(predict(100, 0.0f, 30) == 100);     // flat trend changes nothing
  assert(dutyLevel(0, 20, 10, 100) == 100);      // 20% of 10s: on at t=0
  assert(dutyLevel(1999, 20, 10, 100) == 100);   // still on just before 2s
  assert(dutyLevel(2001, 20, 10, 100) == 0);     // off after 2s
  assert(dutyLevel(9999, 20, 10, 100) == 0);     // off until the cycle repeats
  assert(dutyLevel(5000, 0, 10, 100) == 0);      // zero demand never fires
  assert(dutyLevel(5000, 100, 10, 100) == 100);  // full demand is continuous
  Serial.println("selfTest ok");
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>Haze Regulator</title><style>
html{background:#0a0a0a}
body{font:15px system-ui;margin:22px auto;max-width:1600px;padding:22px 28px;
background:#141414;color:#eee;border:1px solid #2e2e2e;border-radius:14px;
box-shadow:0 2px 18px #0008}
.c{background:#1e1e1e}
h1{font-size:17px;margin:0 0 12px}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(105px,1fr));gap:8px;margin-bottom:14px}
.c{background:#1c1c1c;border-radius:8px;padding:10px}
.c b{display:block;font-size:22px}.c span{color:#999;font-size:12px}
.bar{height:22px;background:#1c1c1c;border-radius:8px;overflow:hidden;margin:4px 0 6px;position:relative}
.bar i{display:block;height:100%;background:#4a9;width:0}
.bar u{position:absolute;top:0;height:100%;width:2px;background:#fff;opacity:.6}
label{display:block;margin:10px 0 2px;color:#999;font-size:12px}
input[type=range]{width:100%}
input[type=number],select{background:#1c1c1c;color:#eee;border:1px solid #333;border-radius:6px;padding:6px;font:inherit}
input[type=number]{width:80px}select{width:auto;max-width:100%;margin-bottom:4px}
button{background:#333;color:#eee;border:0;border-radius:6px;padding:8px 14px;font:inherit}
button.on{background:#4a9;color:#000}
button.stop{background:#c0392b;color:#fff;font-weight:600;letter-spacing:.5px}
button.stop.armed{background:#e74c3c;box-shadow:0 0 0 2px #e74c3c55}
#warn.halt{color:#e74c3c;font-weight:600}
#warn{color:#e94;font-size:12px;min-height:16px;margin-bottom:8px}
canvas{width:100%;height:360px;display:block;background:#1c1c1c;border-radius:8px}
#win,#pdur{width:auto;margin:0 0 12px}
details{margin:12px 0;border-top:1px solid #262626;padding-top:6px}
summary{cursor:pointer;color:#888;font-size:12px;padding:4px 0}
.row{display:flex;gap:8px;align-items:center;margin-bottom:6px;flex-wrap:wrap}
#savebtn{margin-left:auto}
.leg{font-size:11px;color:#777;margin:4px 0 10px;display:flex;gap:12px}
.leg i{font-style:normal}
</style>
<h1>Haze Regulator</h1>
<div class=g>
<div class=c><span>PM2.5 ug/m3</span><b id=pm25>-</b></div>
<div class=c><span>PM1.0</span><b id=pm10>-</b></div>
<div class=c><span>PM10</span><b id=pm100>-</b></div>
<div class=c><span>AQI US</span><b id=aqi>-</b></div>
<div class=c><span>0.3um count</span><b id=c03>-</b></div>
<div class=c><span>Haze output</span><b id=out>-</b></div>
<div class=c><span>DMX haze</span><b id=dmxh>-</b></div>
<div class=c><span>DMX fan</span><b id=dmxf>-</b></div>
<div class=c><span>trend ug/m3/s</span><b id=slope>-</b></div>
<div class=c><span>predicted PM2.5</span><b id=pred>-</b></div>
</div>
<div class=bar><i id=obar></i><u id=tmark></u></div>
<div id=warn></div>
<canvas id=chart></canvas>
<div class=leg><i style=color:#4a9>PM2.5</i><i style=color:#e94>haze demand %</i><i style=color:#e9944490>actual on wire</i>
<i style=color:#888>setpoint</i><i id=peakLbl style=color:#4a9></i><i id=span></i></div>
<select id=win onchange="setWin(this.value)">
<option value=30>30 seconds</option><option value=60>1 minute</option><option value=300>5 minutes</option>
<option value=600 selected>10 minutes</option><option value=1800>30 minutes</option>
<option value=3600>1 hour</option><option value=7200>2 hours</option>
<option value=21600>6 hours</option><option value=43200>12 hours</option>
<option value=86400>24 hours</option><option value=604800>7 days</option></select>
<div class=row><button id=stopbtn class=stop onclick="this.classList.toggle('armed');post('stop',stopped?0:1)">STOP</button>
<button id=mode onclick="var n=this.dataset.v==1?0:1;this.dataset.v=n;this.textContent=n?'AUTO':'MANUAL';this.className=n?'on':'';post('automatic',n)">-</button>
<button id=purge onclick="this.textContent=purging?'Purge':'Purging...';post('purge',purging?0:pdur.value)">Purge</button>
<select id=pdur><option value=30>30s</option><option value=60 selected>1 min</option>
<option value=120>2 min</option><option value=300>5 min</option></select>
<button id=savebtn onclick="post('save',1);this.textContent='Saved';setTimeout(()=>{this.textContent='Save'},1500)">Save</button></div>
<label>Manual haze <span id=vman></span>%</label><input type=range id=manual min=0 max=100 oninput="post('manual',this.value)">
<div id=fanrow><label>Fan speed <span id=vfan></span>%</label><input type=range id=fan min=0 max=100 oninput="post('fan',this.value)"></div>
<label>Target haze <span id=vsp></span> ug/m3</label><input type=range id=setpoint min=0 max=1000 oninput="post('setpoint',this.value)">
<details><summary>Tuning</summary>
<div class=row><button id=cal onclick="post('calibrate',s_cal&&s_cal<5?0:1)">Calibrate</button>
<select id=calpulse onchange="post('calpulse',this.value)">
<option value=30>30s pulse</option><option value=60>1 min</option>
<option value=120>2 min</option><option value=300>5 min</option>
<option value=600>10 min</option></select>
<i id=calstat style=color:#888;font-size:12px></i></div>
<label>Deadband <span id=vdb></span></label><input type=range id=deadband min=0 max=100 oninput="post('deadband',this.value)">
<label>Gain <span id=vg></span></label><input type=range id=gain min=1 max=100 oninput="post('gain',this.value/10)">
<label>Slew limit <span id=vsl></span>%/sec</label><input type=range id=slew min=1 max=100 oninput="post('slew',this.value)">
<div class=row><button id=pulsebtn onclick="post('pulse',pulseOn?0:1)">Pulse mode</button>
<i id=pulsestat style=color:#888;font-size:12px></i></div>
<label>Pulse period, manual <span id=vpp></span>s</label><input type=range id=pperiod min=5 max=60 oninput="post('pperiod',this.value)">
<label>Min burst, auto <span id=vpm></span>s</label><input type=range id=pminon min=1 max=10 oninput="post('pminon',this.value)">
<label>Lookahead <span id=vla></span>s <i style=color:#666>(0 = react only)</i></label><input type=range id=lookahead min=0 max=120 oninput="post('lookahead',this.value)">
</details>
<details><summary>Setup</summary>
<label>Fixture</label><select id=fixture onchange="post('fixture',this.value)">
<option value=0>Amhaze Stadium 2X IP (2ch: fan, haze)</option>
<option value=1>Hurricane Haze 1DX (1ch: haze)</option></select>
<label>DMX start address</label>
<input type=number id=dmxaddr value=1 min=1 max=511 onchange="post('dmxaddr',this.value)">
</details>
<script>
let touching=0,purging=false,stopped=false,s_cal=0,pulseOn=false;
document.querySelectorAll('input[type=range]').forEach(e=>{
  e.onpointerdown=()=>touching=1; e.onpointerup=()=>touching=0;});
// Coalesce changes into one request and refresh straight after, so a button
// reflects its new state immediately instead of waiting for the 1s poll. Also
// stops a slider drag firing a separate request per pixel.
let pend={},flushT=null;
function post(k,v){pend[k]=v;if(!flushT)flushT=setTimeout(flush,60);}
function flush(){
  flushT=null;
  const q=new URLSearchParams(pend).toString();pend={};
  if(q)fetch('/api/set?'+q).then(tick).catch(()=>{});
}
// Two resolutions: 1s detail for the last hour, 1-minute averages beyond it.
// 7 days at 1Hz would be 604800 samples - too many to hold or to draw.
const FINE=[],COARSE=[];let acc=null,winSec=600;
function setWin(v){winSec=+v;draw();}
function record(pm,out,act,sp){
  const now=Date.now();
  FINE.push({t:now,pm:pm,out:out,act:act,sp:sp});
  while(FINE.length>7200)FINE.shift();   // 2Hz, so an hour of detail
  if(!acc||now-acc.t>=60000){
    if(acc)COARSE.push({t:acc.t,pm:acc.pm/acc.n,out:acc.out/acc.n,
                        act:acc.act/acc.n,sp:acc.sp});
    while(COARSE.length>10080)COARSE.shift();   // 7 days of minutes
    acc={t:now,pm:0,out:0,act:0,sp:sp,n:0};
  }
  acc.pm+=pm;acc.out+=out;acc.act+=act;acc.sp=sp;acc.n++;
}
function series(){
  if(winSec<=3600)return FINE;
  const c=COARSE.slice();
  if(acc&&acc.n)c.push({t:acc.t,pm:acc.pm/acc.n,out:acc.out/acc.n,act:acc.act/acc.n,sp:acc.sp});
  return c.length>1?c:FINE;   // nothing aggregated yet, show what we have
}
function draw(){
  const c=chart,ctx=c.getContext('2d'),dpr=devicePixelRatio||1;
  const w=c.clientWidth,h=c.clientHeight;
  c.width=w*dpr;c.height=h*dpr;ctx.scale(dpr,dpr);
  ctx.clearRect(0,0,w,h);
  const t1=Date.now(),t0=t1-winSec*1000;
  let pts=series().filter(p=>p.t>=t0);
  const step=Math.max(1,Math.ceil(pts.length/(w*2)));
  if(step>1)pts=pts.filter((_,i)=>i%step==0);
  span.textContent=pts.length<2?'collecting...':'';
  if(pts.length<2)return;
  const ML=56,MR=50,MT=30,MB=22,pw=w-ML-MR,ph=h-MT-MB;
  const sp=pts[pts.length-1].sp;
  const peak=Math.max(...pts.map(p=>p.pm));
  // Left axis follows the data but never hides the target line.
  const top=Math.max(20,sp*1.25,peak)*1.08;
  const X=p=>ML+(p.t-t0)/(t1-t0)*pw;
  const Ypm=v=>MT+ph-v/top*ph;
  const Ypc=v=>MT+ph-v/100*ph;
  peakLbl.textContent='peak '+Math.round(peak);

  ctx.font='12px system-ui';ctx.textBaseline='middle';  // fixed, never scaled
  for(let i=0;i<=4;i++){
    const y=MT+ph-i/4*ph;
    ctx.strokeStyle='#ffffff12';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(ML,y);ctx.lineTo(ML+pw,y);ctx.stroke();
    ctx.fillStyle='#4a9';ctx.textAlign='right';
    ctx.fillText(Math.round(top*i/4),ML-8,y);
    ctx.fillStyle='#e94';ctx.textAlign='left';
    ctx.fillText(i*25+'%',ML+pw+8,y);
  }
  ctx.textAlign='left';

  const spy=Ypm(sp);
  ctx.strokeStyle='#fff';ctx.setLineDash([6,4]);ctx.lineWidth=1.5;
  ctx.beginPath();ctx.moveTo(ML,spy);ctx.lineTo(ML+pw,spy);ctx.stroke();
  ctx.setLineDash([]);
  const lab='target '+Math.round(sp);
  ctx.font='bold 17px system-ui';   // fixed: only the number changes, not the size
  const tw=ctx.measureText(lab).width;
  const by=spy<MT+24?spy+3:spy-24;
  ctx.fillStyle='#000d';ctx.fillRect(ML+pw-tw-13,by,tw+11,23);
  ctx.fillStyle='#fff';ctx.fillText(lab,ML+pw-tw-7,by+16);

  // Shaded area is what is actually on the wire (pulses included); the solid
  // line is what the controller is asking for.
  ctx.fillStyle='#e9944440';ctx.beginPath();
  ctx.moveTo(X(pts[0]),MT+ph);
  pts.forEach(p=>ctx.lineTo(X(p),Ypc(p.act||0)));
  ctx.lineTo(X(pts[pts.length-1]),MT+ph);ctx.closePath();ctx.fill();

  const line=(f,col,lw)=>{
    ctx.strokeStyle=col;ctx.lineWidth=lw;ctx.beginPath();
    pts.forEach((p,i)=>{const y=f(p);i?ctx.lineTo(X(p),y):ctx.moveTo(X(p),y)});
    ctx.stroke();};
  line(p=>Ypc(p.out),'#e94',1.5);
  line(p=>Ypm(p.pm),'#4a9',2);

  ctx.font='11px system-ui';ctx.fillStyle='#777';
  ctx.fillText(winSec<=3600?'1s samples':'1 min averages',ML,h-8);
  ctx.textAlign='right';
  ctx.fillStyle='#4a9';ctx.fillText('ug/m3',ML-8,12);
  ctx.textAlign='left';
  ctx.fillStyle='#e94';ctx.fillText('haze',ML+pw+8,12);
  ctx.textAlign='left';
}
addEventListener('resize',draw);
async function tick(){
  let s=await(await fetch('/api/state')).json();
  pm25.textContent=s.pm25; pm10.textContent=s.pm10; pm100.textContent=s.pm100;
  aqi.textContent=s.aqi; c03.textContent=s.c03; out.textContent=s.output+'%';
  dmxh.textContent=s.dmxhaze; dmxf.textContent=s.hasfan?s.dmxfan:'-';
  slope.textContent=(s.slope>0?'+':'')+s.slope.toFixed(2);
  pred.textContent=s.lookahead?s.predicted:'off';
  obar.style.width=s.output+'%';
  tmark.style.left=s.target+'%';
  mode.textContent=s.automatic?'AUTO':'MANUAL';
  mode.className=s.automatic?'on':''; mode.dataset.v=s.automatic?1:0;
  warn.className=stopped?'halt':'';
  warn.textContent=stopped?'OUTPUT STOPPED':
    !s.sensorOk?'SENSOR LOST - output ramping to zero':
    (s.pm25>=990?'sensor near saturation - readings unreliable':'');
  vman.textContent=s.manual; vsp.textContent=s.setpoint; vdb.textContent=s.deadband;
  vg.textContent=s.gain.toFixed(1); vsl.textContent=s.slew;
  vfan.textContent=s.fan; fanrow.hidden=!s.hasfan; vla.textContent=s.lookahead;
  s_cal=s.cal; pulseOn=s.pulse; vpp.textContent=s.pperiod; vpm.textContent=s.pminon;
  pulsebtn.className=s.pulse?'on':'';
  const on=(s.pnow*s.output/100);
  pulsestat.textContent=s.pulse?(s.output>0&&s.output<100?
    on.toFixed(1)+'s on / '+(s.pnow-on).toFixed(1)+'s off'+
      (s.automatic?' (auto '+s.pnow+'s cycle)':''):
    (s.output?'continuous':'off')):'';
  cal.textContent=s.cal&&s.cal<5?'Cancel':'Calibrate';
  cal.className=s.cal&&s.cal<5?'on':'';
  if(document.activeElement!=calpulse)calpulse.value=s.calpulse;
  const lt=s.calleft?' ('+s.calleft+'s)':'';
  calstat.textContent=['','purging the room'+lt,'settling'+lt,'pulsing 100%'+lt,
    'watching decay...',s.calmsg,'failed: '+s.calmsg][s.cal]||'';
  stopped=s.stopped;
  // NB: id must not be "stop" - window.stop() already owns that name, so the
  // element never shadows it and every property access here throws.
  stopbtn.textContent=stopped?'STOPPED - resume':'STOP';
  stopbtn.classList.toggle('armed',stopped);
  purging=s.purge>0;
  purge.textContent=purging?'Purging '+s.purge+'s (stop)':'Purge';
  purge.className=purging?'on':'';
  dmxaddr.max=s.maxaddr;
  if(document.activeElement!=fixture)fixture.value=s.fixture;
  if(!touching){manual.value=s.manual;setpoint.value=s.setpoint;
    deadband.value=s.deadband;gain.value=s.gain*10;slew.value=s.slew;fan.value=s.fan;
    lookahead.value=s.lookahead;pperiod.value=s.pperiod;pminon.value=s.pminon;}
  if(document.activeElement!=dmxaddr)dmxaddr.value=s.dmxaddr;
  record(s.pm25,s.output,s.haze,s.setpoint);
  draw();
}
tick();setInterval(tick,500);
</script>)HTML";

// Seconds left in the current calibration phase; 0 when it cannot be known.
int calLeft() {
  unsigned long el = (millis() - calT0) / 1000;
  if (calState == CAL_PURGE) return (int)el;  // elapsed, not remaining
  if (calState == CAL_SETTLE) return max(0, (int)(CAL_SETTLE_S - el));
  if (calState == CAL_PULSE) return max(0, (int)(cfg.calPulse - el));
  return 0;
}

// In automatic the duty already tracks predicted PM2.5, because duty is the
// controller's demand. The period has to adapt too: at 5% demand a fixed 10s
// cycle asks for a half-second burst, which a hazer cannot deliver. Stretch the
// cycle instead so every burst is at least pulseMinOn long. Manual keeps the
// period the user dialled in.
int pulsePeriodNow() {
  if (!cfg.automatic || output <= 0) return cfg.pulsePeriod;
  int per = (cfg.pulseMinOn * 100 + output - 1) / output;  // round up
  return constrain(per, 5, 60);
}

// What is actually on the wire right now, pulsing included.
uint8_t hazeLevel() {
  if (cfg.pulseMode && !cfg.stopped && !purging() && !calibrating())
    return dutyLevel(millis(), output, pulsePeriodNow(), cfg.pulseLevel);
  return output;
}

void handleState() {
  char buf[900];
  snprintf(buf, sizeof(buf),
           "{\"pm25\":%u,\"pm10\":%u,\"pm100\":%u,\"aqi\":%u,\"c03\":%u,"
           "\"output\":%u,\"target\":%u,\"automatic\":%s,\"manual\":%u,"
           "\"setpoint\":%d,\"deadband\":%d,\"gain\":%.1f,\"slew\":%d,"
           "\"fan\":%d,\"dmxaddr\":%d,\"dmxhaze\":%u,\"fixture\":%d,"
           "\"hasfan\":%s,\"maxaddr\":%d,\"purge\":%d,\"dmxfan\":%u,\"lookahead\":%d,"
           "\"slope\":%.2f,\"predicted\":%d,\"stopped\":%s,\"cal\":%d,\"calmsg\":\"%s\",\"pulse\":%s,\"pperiod\":%d,"
           "\"calpulse\":%d,\"calleft\":%d,\"haze\":%u,\"pnow\":%d,"
           "\"pminon\":%d,\"sensorOk\":%s}",
           everRead ? data.pm25_env : 0, everRead ? data.pm10_env : 0,
           everRead ? data.pm100_env : 0, everRead ? data.aqi_pm25_us : 0,
           everRead ? data.particles_03um : 0, output, target,
           cfg.automatic ? "true" : "false", cfg.manual, cfg.setpoint,
           cfg.deadband, cfg.gain, cfg.slew, cfg.fan, cfg.dmxAddress,
           toDmx(hazeLevel()), cfg.fixture,
           FIXTURES[cfg.fixture].fanOff >= 0 ? "true" : "false",
           maxAddress(FIXTURES[cfg.fixture].chans),
           purging() ? (int)((purgeUntil - millis()) / 1000) : 0,
           FIXTURES[cfg.fixture].fanOff >= 0 ? toDmx(purging() ? 100 : cfg.fan) : 0,
           cfg.lookahead, pmSlope, predicted, cfg.stopped ? "true" : "false",
           calState, calMsg, cfg.pulseMode ? "true" : "false", cfg.pulsePeriod,
           cfg.calPulse, calLeft(), hazeLevel(), pulsePeriodNow(),
           cfg.pulseMinOn, sensorOk ? "true" : "false");
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("automatic")) cfg.automatic = server.arg("automatic").toInt();
  if (server.hasArg("manual")) cfg.manual = constrain(server.arg("manual").toInt(), 0, 100);
  if (server.hasArg("setpoint")) cfg.setpoint = constrain(server.arg("setpoint").toInt(), 0, 1000);
  if (server.hasArg("deadband")) cfg.deadband = constrain(server.arg("deadband").toInt(), 0, 100);
  if (server.hasArg("gain")) cfg.gain = constrain(server.arg("gain").toFloat(), 0.1f, 10.0f);
  if (server.hasArg("slew")) cfg.slew = constrain(server.arg("slew").toInt(), 1, 100);
  if (server.hasArg("lookahead"))
    cfg.lookahead = constrain(server.arg("lookahead").toInt(), 0, 120);
  if (server.hasArg("fan")) cfg.fan = constrain(server.arg("fan").toInt(), 0, 100);
  if (server.hasArg("save")) {
    saveAt = 0;  // explicit save: write now rather than on the debounce
    saveCfg();
  }
  if (server.hasArg("stop")) cfg.stopped = server.arg("stop").toInt();
  if (server.hasArg("pulse")) cfg.pulseMode = server.arg("pulse").toInt();
  if (server.hasArg("pminon"))
    cfg.pulseMinOn = constrain(server.arg("pminon").toInt(), 1, 10);
  if (server.hasArg("calpulse"))
    cfg.calPulse = constrain(server.arg("calpulse").toInt(), 30, 600);
  if (server.hasArg("pperiod"))
    cfg.pulsePeriod = constrain(server.arg("pperiod").toInt(), 5, 60);
  if (server.hasArg("calibrate")) {
    if (server.arg("calibrate").toInt()) {
      calState = CAL_PURGE; calT0 = millis(); calMsg[0] = 0;
    } else {
      calState = CAL_OFF; calMsg[0] = 0; purgeUntil = 0;
    }
  }
  if (server.hasArg("purge")) {
    int secs = constrain(server.arg("purge").toInt(), 0, 600);
    purgeUntil = secs ? millis() + (unsigned long)secs * 1000 : 0;
  }
  if (server.hasArg("fixture")) {
    int f = constrain(server.arg("fixture").toInt(), 0, (int)FIXTURE_COUNT - 1);
    if (f != cfg.fixture) {
      cfg.fixture = f;
      memset(dmxData + 1, 0, DMX_PACKET_SIZE - 1);  // release old channels
    }
  }
  if (server.hasArg("dmxaddr")) {
    int a = constrain(server.arg("dmxaddr").toInt(), 1,
                      maxAddress(FIXTURES[cfg.fixture].chans));
    if (a != cfg.dmxAddress) {
      cfg.dmxAddress = a;
      memset(dmxData + 1, 0, DMX_PACKET_SIZE - 1);
    }
  }
  // A fixture switch can leave the address past the new limit.
  cfg.dmxAddress = constrain(cfg.dmxAddress, 1,
                             maxAddress(FIXTURES[cfg.fixture].chans));
  saveAt = millis() + 2000;  // write once the user stops fiddling
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  selfTest();

  prefs.begin("haze", false);
  loadCfg();
  Serial.printf("fixture: %s, addr %d\n", FIXTURES[cfg.fixture].name,
                cfg.dmxAddress);

  Wire.begin(SDA_PIN, SCL_PIN);
  sensorOk = aqi.begin_I2C(&Wire);
  if (sensorOk) lastGoodRead = millis();
  Serial.println(sensorOk ? "PMSA003I ready" : "PMSA003I not responding at 0x12");

  dmxBegin();
  Serial.printf("DMX transmitting on GPIO%d, DE on GPIO%d\n", DMX_TX_PIN, DMX_EN_PIN);

  WiFi.mode(WIFI_STA);
  // Modem sleep is on by default and adds ~100ms to every request, which makes
  // the UI feel laggy. This box is mains powered; the tradeoff is not worth it.
  WiFi.setSleep(false);
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

  if (saveAt && (int32_t)(now - saveAt) >= 0) {
    saveAt = 0;
    saveCfg();
  }

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

    // Trend and prediction are readouts as much as control inputs, so they must
    // update in every mode. Computing them only on the automatic path left both
    // stuck at zero in manual, purge and calibration.
    if (everRead && sensorOk) {
      pmHist[pmIdx] = data.pm25_env;
      pmIdx = (pmIdx + 1) % SLOPE_WIN;
      if (pmCount < SLOPE_WIN) pmCount++;
      pmSlope = pmCount < 5 ? 0
                            : slopeOf(pmHist[(pmIdx - 1 + SLOPE_WIN) % SLOPE_WIN],
                                      pmHist[(pmIdx - pmCount + SLOPE_WIN) % SLOPE_WIN],
                                      pmCount);
      predicted = predict(data.pm25_env, pmSlope, cfg.lookahead);
    }
    if (cfg.stopped) {
      if (calibrating()) purgeUntil = 0;
      calState = CAL_OFF;
      // Latched, and snaps rather than slewing. The fan is deliberately left
      // alone: it makes no haze, and running it helps clear what is already up.
      target = 0;
      output = 0;
    } else if (calibrating()) {
      runCalibration(now, data.pm25_env);
    } else if (purging()) {
      // Snap off rather than slewing down. Slew exists to stop haze coming on
      // too fast; a purge is asking for it gone now, and cutting output is
      // always the safe direction.
      target = 0;
      output = 0;
    } else if (!cfg.automatic) {
      target = cfg.manual;
    } else if (!everRead || !sensorOk) {
      target = 0;  // fail safe: never keep hazing on a stale reading
    } else {
      target = computeOutput(cfg.setpoint, predicted, cfg.deadband, cfg.gain);
    }
    if (!purging() && !cfg.stopped && !calibrating())
      output = applySlew(output, target, cfg.slew);
  }

  // Unconditional: frames keep going out at zero as well, so a receiver never
  // sees signal loss just because the haze is off.
  if (now - lastDmx >= DMX_INTERVAL_MS) {
    lastDmx = now;
    const Fixture &f = FIXTURES[cfg.fixture];
    uint8_t haze = hazeLevel();
    dmxData[0] = 0;  // DMX start code
    if (f.fanOff >= 0)
      dmxData[cfg.dmxAddress + f.fanOff] = toDmx(purging() ? 100 : cfg.fan);
    dmxData[cfg.dmxAddress + f.hazeOff] = toDmx(haze);
    dmxSend();
  }
}
