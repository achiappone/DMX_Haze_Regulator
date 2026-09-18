// Haze regulator: PMSA003I PM2.5 sensor -> DMX haze machine output.
// Sensor  I2C   SDA=GPIO8  SCL=GPIO9  3V3  GND
// DMX     CTC-DRA-10-R2 shield, jumpers EN=on SLAVE=DE TX-io=TX-uart
//         shield TX->GPIO17, shield 2->GPIO16, 5V, GND. Leave 0 and 3 open.
#include <Adafruit_PM25AQI.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Update.h>
#include <time.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <assert.h>
#include <string.h>
#include <driver/uart.h>
#include "secrets.h"

#define SDA_PIN 8
#define SCL_PIN 9
#define DMX_TX_PIN 17
#define DMX_EN_PIN 16
// Onboard addressable LED. The generic S3 variant puts it on GPIO48; some
// clones have a board bug and wire it to 38 instead - change this if it stays
// dark while DMX is clearly running.
#define RGB_LED_PIN RGB_BUILTIN
#define DMX_UART UART_NUM_1
#define DMX_PACKET_SIZE 513  // start code + 512 slots
#define SLOPE_WIN 30       // seconds of PM history used for the trend
#define SENSOR_POLL_MS 250   // faster than the sensor's ~1s frame rate
#define SENSOR_STALE_MS 5000
#define NORESP_PCT 10      // below this no measurable rise is expected anyway
#define NORESP_WIN_S 90    // commanded this long before judging
#define NORESP_RISE 15     // ug/m3 that counts as the machine responding
// Bumped by hand in the same commit as the change, matching the scheme
// pve-stack uses. The build stamp comes from the compiler rather than a
// constant anyone has to remember: the question this footer answers is "is the
// board running the push I just made", and a version alone cannot answer it
// when a flash silently fails and leaves the old binary in place.
#define FW_VERSION "1.31.000"
#define FW_BUILT __DATE__ " " __TIME__

#define SAT_PM 990        // PMSA003I mass tops out near 1000
#define SAT_COUNT 60000   // 0.3um count field is 16-bit and saturates near here
#define SAT_PURGE_MIN_MS 15000   // first response to saturation
#define SAT_PURGE_MAX_MS 120000
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
  int source = 0;           // 0 = PM2.5 mass, 1 = 0.3um particle count / 100
  float setpoint = 150.0f;  // target, in whichever unit source selects
  float deadband = 10.0f;   // no output until this far below setpoint
  float gain = 1.0f;     // output % per ug/m3 of error
  int slew = 3;          // max % change per second
  int fan = 50;          // 0-100%, independent of haze regulation
  int dmxAddress = 1;    // start address; channel layout depends on fixture
  int fixture = 0;       // index into FIXTURES
  int leadFall = 20;     // s of anticipation while levels are falling (on early)
  int leadRise = 45;     // s while rising (off early) - overshoot costs more
  int filterTau = 30;    // s, low-pass on the control input; 0 = raw
  int machineTail = 4;   // s the hazer keeps producing after DMX goes to zero
  int floorPct = 10;     // never command less than this when well below target
  int riseCut = 3;       // ug/m3/s climb that latches output off; 0 = disabled
  int integralTi = 300;  // integral time, seconds; 0 = proportional only
  // How fast the room clears, which is a property of the room, not a tuning
  // knob. It was sharing integralTi - a reading of "the integral should track
  // the room" that is defensible until you need to detune the integral for
  // stability and find you have moved the leak model and the rise estimator
  // with it. Two different things that happened to start with one number.
  int decayTau = 300;
  // Room baseline, so a new install starts from something rather than nothing.
  int roomSize = 1;   // 0 small, 1 medium, 2 large
  int airMode = 0;    // 0 air conditioned / cycling, 1 static
  int roomTempF = 70;
  // Whether these constants have ever been measured, as opposed to guessed from
  // the room. A room size is a starting point; a measurement outranks it. The
  // 1200 cu ft baseline replaced a rise rate the board had converged on over
  // hours - 16/s measured, 48/s inferred - which made doseCap three times too
  // tight and left capRelax clawing back to 2.66x while the level swung from 21
  // to 293. One button should not be able to discard that silently.
  bool riseLearned = false;
  bool decayLearned = false;
  float riseRate = 0;    // signal units per second at 100% output, from calibration
  // What was asked for on each control signal, kept per signal so switching
  // between them is lossless. Converting by the measured ratio instead lost up
  // to 8% of the target per round trip, in a random direction, because the
  // PM1.0/PM2.5 ratio is noisy and k * (1/k) only equals 1 if both readings
  // land on the same sensor frame. An operator picks a level once per signal;
  // remembering it beats deriving it every time. 0 means never set.
  float spBy[3] = {0, 0, 0};
  float dbBy[3] = {0, 0, 0};
  float riseBy[3] = {0, 0, 0};
  int deadTime = 0;      // seconds from commanding output to seeing it
  int dosePct = 100;     // how much of the computed deficit to commit, percent
  bool stopped = false;  // latched stop; survives reboot on purpose
  bool pulseMode = false;  // time-proportional output instead of continuous
  int pulsePeriod = 10;    // seconds per pulse cycle
  int pulseLevel = 100;    // output % during the on part of the cycle
  int calPulse = 30;       // calibration pulse length, seconds
  int pulseMinOn = 2;      // auto mode: shortest burst worth firing, seconds
  bool autoPurge = true;   // clear the air automatically when the sensor pegs
} cfg;

PM25_AQI_Data data;
bool sensorOk = false, everRead = false;
uint8_t output = 0, target = 0;
unsigned long lastRead = 0, lastGoodRead = 0, lastDmx = 0, lastControl = 0;
unsigned long purgeUntil = 0;
unsigned long wifiTry = 0, wifiLostAt = 0;
unsigned long satPurgeMs = SAT_PURGE_MIN_MS;  // escalates while it stays pegged
unsigned long saveAt = 0;  // debounce NVS writes; a slider drag is many changes
// The dose cap clipping the controller is invisible from outside: output just
// sits low while the room stays below the band, which reads as a machine that
// cannot keep up. Working out that the PI wanted 95% and the cap allowed 5%
// took a CSV export and arithmetic. The board knows both numbers - it should
// say so.
unsigned long capSince = 0, capLogged = 0;

// doseCap is only as good as riseRate and the decay constant, and those are two
// unknowns against one equation: the adaptive estimator solves for the rise
// rate given the decay constant, and nothing solves for the decay constant. Get
// it wrong and the cap refuses output the room genuinely needs - measured here,
// the controller asking for 100% while the cap allowed 12%, the room 70 below
// its band, and 18% demand losing 51 units over ninety seconds.
//
// Evidence beats the model. Capped, below the band and not rising can only mean
// the cap is too tight, whichever constant is wrong, and that judgement needs no
// constants at all. So relax it - slowly, bounded, and give it all back once the
// band is reached, because this is a correction for a bad model rather than a
// second controller.
#define CAP_RELAX_MAX 4.0f
float capRelax = 1.0f;
float relaxRef = 0;
unsigned long relaxAt = 0, relaxLogged = 0;
#define UI_PATH "/index.html"
#define UI_TMP "/index.new"
#define UI_BAK "/index.bak"
File uiTmp;
bool uiUploadOk = false;
// Bumped whenever the served page changes. An open browser watches it and
// reloads itself, so pushing a UI no longer means walking round telling people
// to hit refresh - and nobody reads a stale page while wondering why the fix
// they were promised is not there. Persisted, so a reboot does not look like
// a new page and reload every tab on the rig.
uint32_t uiGen = 0;
bool uiFromFs();

// Health the hardware watchdogs cannot see. All four on this chip - both timer
// group watchdogs, the RTC one and the super watchdog - detect stoppage: a task
// that never yields, an interrupt that never returns. Twice today the board did
// neither. The loop ran, fed the task watchdog on every pass and answered pings
// in 20ms, while serving the 20KB page at 14 and then 71 bytes a second. To a
// watchdog that is a perfectly healthy board; to anyone trying to open the page
// it is down. So measure the two things that actually went wrong.
unsigned long loopTicks = 0, healthAt = 0;
uint32_t loopRate = 0;       // loop passes per second
uint32_t servedReqs = 0;     // requests handleClient actually did work for
uint32_t slowestReqMs = 0;   // worst single response since boot
uint32_t reqMsTotal = 0;     // summed response time, for an average
unsigned long starvedSince = 0, reassocAt = 0;
// RF history, because the radio is where two of this board's outages actually
// lived. RSSI alone said -81 while it was failing and -85 would not have
// tripped; which AP, which channel and which PHY rate it settled on is what
// distinguishes a weak link from a bad association.
uint16_t reassocCount = 0;
unsigned long assocAt = 0;
uint8_t slowReqs = 0;  // consecutive responses that crawled
// Calibration measures the three numbers that are properties of the room, not
// the equipment: how long haze takes to arrive, how fast it accumulates at full
// output, and how slowly it clears. Guessing these is what causes overshoot.
enum { CAL_OFF = 0, CAL_PURGE, CAL_SETTLE, CAL_PULSE, CAL_DECAY, CAL_DONE, CAL_FAIL };
#define CAL_SETTLE_S 30   // long enough to measure the noise band
int calState = CAL_OFF;
unsigned long calT0 = 0, calPeakT = 0;
int calBaseline = 0, calPeak = 0, calDead = 0, calTau = 0, calPulseUsed = 0;
bool calBaseHigh = false;
int calNoiseLo = 0, calNoiseHi = 0, calNoise = 0;
float calRise = 0;
char calMsg[128] = "";
bool calibrating() { return calState >= CAL_PURGE && calState <= CAL_DECAY; }

// On-board trend history, so a browser reload is just a new view of the same
// data rather than a fresh start. Two tiers: 2Hz detail (fast enough that pulse
// bursts are not aliased away) and 1-minute averages for the long windows.
struct Sample {
  uint16_t pm;    // PM2.5 mass, always, so the chart keeps one stable unit
  uint16_t ctrl;  // what the loop actually regulated on, in ctrlValue() units
  uint16_t pm1;   // PM1.0 and PM10, for the chart. Deliberately not persisted:
  uint16_t pm10;  // the regulated signal is worth surviving a reboot, two
                  // diagnostic series are not worth a third format migration,
                  // and restored samples simply read 0 for these.
  uint8_t out;
  uint8_t act;
};
#define HIST_FINE_MS 500
#define HIST_COARSE_MS 60000
Sample *fineBuf = nullptr, *coarseBuf = nullptr;
int fineCap = 7200, coarseCap = 10080;  // 1 hour at 2Hz, 7 days of minutes
int fineN = 0, fineHead = 0, coarseN = 0, coarseHead = 0;
// Samples restored from flash were written before this boot, but nothing
// records how long the board was off. Counting how many minute samples this
// session wrote lets the rest be placed before boot instead of being folded
// into the current session's timeline, which put them out of order.
int coarsePostBoot = 0;
unsigned long histLast = 0, coarseLast = 0;
uint32_t cAccPm = 0, cAccCtrl = 0, cAccOut = 0, cAccAct = 0;
uint32_t cAccPm1 = 0, cAccPm10 = 0;
int cAccN = 0;

// Only built-in types in the signature: the Arduino preprocessor inserts
// generated prototypes above this file's struct definitions, so a Sample in the
// parameter list fails to compile.
// The minute tier is mirrored to flash so the trend survives a reboot, a
// reflash, or the venue killing power. Appending 4 bytes a minute keeps the
// write load trivial; the file is rotated only once it holds twice the ring.
// v3: 10-byte records, adding PM1.0 and PM10 so every charted series survives
// a reboot. A new path per version rather than a version byte - a file written
// at one record size parses as plausible garbage at another, and silently wrong
// history is worse than none. Each version imports its predecessor once.
#define TREND_PATH "/trend3.bin"
#define TREND_PATH_V2 "/trend2.bin"
#define TREND_PATH_V1 "/trend.bin"
#define TREND_REC 10
#define EVT_PATH "/events.log"
#define EVT_MAX 196608  // 192KB, months of transitions
bool fsOk = false;
uint32_t bootId = 0;

// No RTC, so events are stamped with a boot counter plus uptime. That is enough
// to say "third boot, 41 minutes in" and to line an event up against the trend.
void logEvent(const char *fmt, ...) {
  char msg[110];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  // No RTC, so the browser hands us its wall clock on connect. Until that
  // happens, fall back to boot number and uptime.
  char ts[26];
  time_t tnow = time(nullptr);
  if (tnow > 1700000000) {
    struct tm tmv;
    localtime_r(&tnow, &tmv);
    strftime(ts, sizeof(ts), "%m-%d %H:%M:%S", &tmv);
  } else {
    snprintf(ts, sizeof(ts), "b%lu %lus", (unsigned long)bootId,
             (unsigned long)(millis() / 1000));
  }
  char line[190];
  snprintf(line, sizeof(line), "%-15s  %s\n", ts, msg);
  Serial.print(line);
  if (!fsOk) return;
  File f = LittleFS.open(EVT_PATH, "a");
  if (!f) return;
  f.print(line);
  size_t sz = f.size();
  f.close();
  if (sz <= EVT_MAX) return;
  File in = LittleFS.open(EVT_PATH, "r");        // keep the newer half
  if (!in) return;
  in.seek(sz / 2);
  File out = LittleFS.open("/events.tmp", "w");
  if (!out) { in.close(); return; }
  in.readStringUntil('\n');                      // drop the partial line
  uint8_t b[256];
  int r;
  while ((r = in.read(b, sizeof(b))) > 0) out.write(b, r);
  in.close();
  out.close();
  LittleFS.remove(EVT_PATH);
  LittleFS.rename("/events.tmp", EVT_PATH);
}


void trendAppend(uint16_t pm, uint16_t ctrl, uint16_t pm1, uint16_t pm10,
                 uint8_t out, uint8_t act) {
  if (!fsOk) return;
  File f = LittleFS.open(TREND_PATH, "a");
  if (!f) return;
  uint8_t rec[TREND_REC] = {(uint8_t)(pm & 0xFF),   (uint8_t)(pm >> 8),
                            (uint8_t)(ctrl & 0xFF), (uint8_t)(ctrl >> 8),
                            (uint8_t)(pm1 & 0xFF),  (uint8_t)(pm1 >> 8),
                            (uint8_t)(pm10 & 0xFF), (uint8_t)(pm10 >> 8),
                            out, act};
  f.write(rec, TREND_REC);
  size_t sz = f.size();
  f.close();
  if (sz / TREND_REC <= (size_t)coarseCap * 2) return;

  File in = LittleFS.open(TREND_PATH, "r");
  if (!in) return;
  in.seek((in.size() / TREND_REC - coarseCap) * TREND_REC);
  File out2 = LittleFS.open("/trend.tmp", "w");
  if (!out2) { in.close(); return; }
  uint8_t buf[256];
  int r;
  while ((r = in.read(buf, sizeof(buf))) > 0) out2.write(buf, r);
  in.close();
  out2.close();
  LittleFS.remove(TREND_PATH);
  LittleFS.rename("/trend.tmp", TREND_PATH);
}

// Read a trend file written by an older firmware. Layout is identical up to
// the fields that version knew about, so the only difference is the record
// size and which trailing columns exist.
void importOld(const char *path, int rec);

void histPush(bool coarse, uint16_t pm, uint16_t ctrl, uint16_t pm1,
              uint16_t pm10, uint8_t out, uint8_t act) {
  if (coarse) {
    if (!coarseBuf) return;
    coarseBuf[coarseHead] = {pm, ctrl, pm1, pm10, out, act};
    coarseHead = (coarseHead + 1) % coarseCap;
    if (coarseN < coarseCap) coarseN++;
  } else {
    if (!fineBuf) return;
    fineBuf[fineHead] = {pm, ctrl, pm1, pm10, out, act};
    fineHead = (fineHead + 1) % fineCap;
    if (fineN < fineCap) fineN++;
  }
}

void importOld(const char *path, int rec) {
  if (!LittleFS.exists(path)) return;
  File f = LittleFS.open(path, "r");
  if (!f) return;
  size_t recs = f.size() / rec;
  size_t skip = recs > (size_t)coarseCap ? recs - coarseCap : 0;
  f.seek(skip * rec);
  uint8_t b[10];
  while (f.read(b, rec) == rec) {
    uint16_t pm = b[0] | (b[1] << 8);
    uint16_t ctrl = rec >= 6 ? (uint16_t)(b[2] | (b[3] << 8)) : 0;
    uint8_t out = rec >= 6 ? b[4] : b[2];
    uint8_t act = rec >= 6 ? b[5] : b[3];
    histPush(true, pm, ctrl, 0, 0, out, act);
  }
  f.close();
  Serial.printf("imported %d minutes from %s\n", coarseN, path);
}

bool uiFromFs() { return fsOk && LittleFS.exists(UI_PATH); }

// The negotiated PHY, which is the first thing to fall back on a marginal link:
// 11n gives tens of megabits, 11b gives one, and the page is 30KB.
const char *phyMode() {
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return "?";
  if (ap.phy_11n) return "11n";
  if (ap.phy_11g) return "11g";
  if (ap.phy_11b) return "11b";
  return "?";
}

double txPowerDbm() {
  int8_t p = 0;
  if (esp_wifi_get_max_tx_power(&p) != ESP_OK) return 0;
  return p / 4.0;  // reported in quarter dBm
}

uint16_t pmHist[SLOPE_WIN];
int pmCount = 0, pmIdx = 0;
float pmSlope = 0;   // ug/m3 per second, negative when haze is clearing
float pmFilt = 0;
float integ = 0;  // integral contribution, in output percent

// A planned reboot throws the integral away, and on a process with a 64s dead
// time and an 89s integral time that costs minutes of recovery every update -
// output restarts at zero and climbs at the slew limit while the room drains.
// It is worth carrying across a reboot we chose to make, but only briefly: an
// integral is a statement about a room that no longer holds once the board has
// been off long enough for the haze to clear.
#define INTEG_KEEP_S 300
void saveInteg() {
  prefs.putFloat("integ", integ);
  prefs.putUInt("integAt", (uint32_t)time(nullptr));
}
void restoreInteg() {
  uint32_t at = prefs.getUInt("integAt", 0);
  uint32_t now = (uint32_t)time(nullptr);
  if (!at || now < 1700000000 || now - at > INTEG_KEEP_S) return;
  integ = constrain(prefs.getFloat("integ", 0.0f), 0.0f, 100.0f);
  logEvent("integral %.1f restored after a %us restart", (double)integ,
           (unsigned)(now - at));
}
bool pmFiltInit = false;
// Peak tracking. Session peaks reset on boot; the all-time peak is kept in NVS
// with the boot and uptime it happened at, so it can be found in the event log.
uint16_t pkSes = 0, pkAll = 0, pkOut = 0;
uint32_t pkSesAt = 0, pkAllBoot = 0, pkAllAt = 0;
unsigned long pkLogged = 0;
float predicted = 0;
unsigned long outSince = 0;
float pmAtOutStart = 0;
bool noResponse = false;
bool riseLock = false;
unsigned long zeroSince = 0, zeroLogged = 0;

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

// Least squares across the whole window instead of a line through its two end
// points. The end points are exactly where sensor noise does the most damage:
// one plume drifting past as the window opens or closes swings the slope by
// several units a second, and that slope is then multiplied by a lead of up to
// 120s before it reaches the output. Measured on this rig the raw signal moved
// 142 -> 319 in twelve seconds with the output at zero, which no room with a
// 403s decay constant can actually do. Fitting every sample costs a few dozen
// multiplies once a second and ignores what the ends happen to be doing.
float slopeFit(const uint16_t *buf, int idx, int count, int cap) {
  if (count < 5) return 0.0f;
  float n = count;
  float sx = (n - 1) * n / 2.0f;
  float sxx = (n - 1) * n * (2 * n - 1) / 6.0f;
  float sy = 0, sxy = 0;
  for (int j = 0; j < count; j++) {
    float y = buf[(idx - count + j + cap) % cap];
    sy += y;
    sxy += j * y;
  }
  float den = sxx - sx * sx / n;
  if (den <= 0) return 0.0f;
  return (sxy - sx * sy / n) / den;  // units per sample, and samples are 1Hz
}

// Falling and rising want different amounts of anticipation. Haze arrives in
// seconds but clears over many minutes, so being late to stop costs far more
// than being late to start.
int leadFor(float slope, int fall, int rise) { return slope < 0 ? fall : rise; }

// Dead time means output commanded now is invisible for tens of seconds, so a
// feedback loop keeps asking for more until the reading finally moves - by which
// point far too much is already on its way. That is what produces a 300 reading
// against a 150 target, and no amount of gain tuning removes it.
//
// So cap output at the most that could be useful: the rate that would exactly
// close the remaining deficit over one dead time. Committing more than the
// deficit guarantees overshoot regardless of what feedback decides afterwards.
// Both constants come from calibration; without them the cap does nothing.
// The deficit that will exist when haze commanded now actually lands, not the
// one visible today. A decaying room keeps falling throughout the dead time, so
// dosing for the present gap always arrives short - and on a long dead time it
// arrives badly short, which reads as the loop refusing to act while levels sink.
float deficitAtArrival(float setpoint, float measured, float slopePerSec,
                       int deadTime) {
  float atArrival = measured + slopePerSec * deadTime;
  float deficit = setpoint - atArrival;
  // A briefly steep slope would otherwise project an absurd shortfall and throw
  // the dose cap wide open. Filling from an empty room needs the target itself
  // and never more, so that is the ceiling.
  return deficit > setpoint ? setpoint : deficit;
}

// How hard to chase, as one choice instead of five sliders. Deliberately does
// not touch riseRate, deadTime, gain or the lead times: those describe the
// machine and the room, they come from calibration, and a preset that
// overwrote them would quietly discard a measurement in favour of a guess.
// These five are all judgement - how much noise to swallow before reacting.
struct Preset {
  const char *name;
  int tau, plevel, dosePct, slew, riseCut;
};
const Preset PRESETS[] = {
    // dose 100, not 60: doseCap already shrinks with the gap, so the loop backs
    // off on its own as it approaches the band. A second handicap on top of it
    // held the level ~30 below the minimum and could never be integrated away.
    {"Stable", 60, 20, 100, 1, 3},
    {"Balanced", 30, 40, 100, 1, 3},
    {"Responsive", 10, 100, 100, 3, 5},
};
const size_t PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

// Solve the model the loop already assumes for the one constant it cannot
// trust. A calibration measures one plume on one afternoon: it read 16.71/s
// here from the peak of a full-power burst crossing the sensor, while the
// steady state said 5.5 - and doseCap, sized on the first number, held output
// at 8% while the room sat 100 below the band for half an hour. Doors open,
// the HVAC changes, fluid runs low; a number measured once stops describing
// the room. Given a window of its own output and the level that output
// produced, the loop can work out the truth.
//   d(level)/dt = rise * duty - level / tau
// Pure, so the arithmetic can be tested without waiting ten minutes for it.
float riseEstimate(float lvlStart, float lvlEnd, float lvlAvg, float dutyAvg,
                   float dt, int tau) {
  if (dutyAvg < 0.01f || tau <= 0 || dt <= 0) return 0.0f;
  return ((lvlEnd - lvlStart) / dt + lvlAvg / tau) / dutyAvg;
}

// Ten minutes of evidence per update, and only 15% of each estimate believed.
// This corrects drift over hours; anything faster would be a second control
// loop fighting the first one.
#define ADAPT_WIN 600
#define ADAPT_BLEND 0.10f
float adaptDuty = 0, adaptLvl = 0, adaptStart = 0;
int adaptN = 0;
unsigned long adaptT0 = 0;

void adaptReset(unsigned long now, float lvl) {
  adaptDuty = adaptLvl = 0;
  adaptN = 0;
  adaptStart = lvl;
  adaptT0 = now;
}

void adaptTick(unsigned long now, float lvl, int duty) {
  if (!adaptT0) { adaptReset(now, lvl); return; }
  adaptDuty += duty;
  adaptLvl += lvl;
  adaptN++;
  if (adaptN < 5 || now - adaptT0 < (unsigned long)ADAPT_WIN * 1000) return;
  float lvlAvg = adaptLvl / adaptN;
  // Learn from steady windows only. Feeding transients back into riseRate
  // closes a loop: a higher rise tightens the cap, which drops the output and
  // the level, which the next window reads as a weaker machine, which widens
  // the cap again - a limit cycle at twice the window length. Thirty minutes on
  // PM2.5 hunted 174 -> 227 -> 148 with a 10-15 minute period, against a 10
  // minute window. A window that moved more than a third of its own average was
  // measuring the swing, not the machine.
  float moved = fabsf(lvl - adaptStart);
  if (lvlAvg > 1.0f && moved > 0.35f * lvlAvg) {
    adaptReset(now, lvl);
    return;
  }
  float est = riseEstimate(adaptStart, lvl, lvlAvg,
                           adaptDuty / adaptN / 100.0f,
                           (now - adaptT0) / 1000.0f, cfg.decayTau);
  // A window that saw almost no output, or that implies something absurd, is
  // evidence of nothing. Drop it rather than letting it move the constant.
  if (est > 0.2f && est < 200.0f) {
    float was = cfg.riseRate;
    cfg.riseRate += (est - cfg.riseRate) * ADAPT_BLEND;
    cfg.riseLearned = true;
    logEvent("rise %.2f -> %.2f/s (10 min window said %.2f)", (double)was,
             (double)cfg.riseRate, (double)est);
    saveAt = now + 2000;
  }
  adaptReset(now, lvl);
}

// What the room loses while a dose is in flight. A dose sized to exactly close
// the gap arrives into a room that has been leaking the whole time, and at the
// target - gap closed, nothing left to fill - the loop still has to replace
// level/tau every second or the level falls straight back out of the band.
// Holding a level is a rate, not a one-shot, and doseCap had no term for it:
// measured here it allowed 5% while holding the band needed 12.1%, and the
// integral could not rescue that because the cap clips output before the
// integral ever reaches it.
float holdLoss(float level, int tau, int deadTime) {
  if (tau <= 0 || deadTime <= 0 || level <= 0) return 0.0f;
  return level / tau * deadTime;
}

// Starting estimates for a room nobody has calibrated. Anchored on the one rig
// with real measurements - 3600 cubic feet, air cycling, rise 16/s, decay 89s -
// and scaled from there rather than invented from an air-change table. A place
// to start, not an answer: calibration measures the machine, and the rise
// estimator keeps it honest afterwards.
//
// rise scales as 1/volume, which is just dilution: the same machine into twice
// the air gives half the concentration per second.
//
// decayTau is the weaker half. Its anchor implies about 40 air changes an hour,
// which no ordinary room does, so what it really captures is a plume dispersing
// near the sensor rather than the room clearing. Scaled gently with volume and
// tripled for a static room: directionally right, and no more than that.
const long ROOM_FT3[3] = {1200, 3600, 7000};
#define ROOM_REF_FT3 3600.0f
#define RISE_REF 16.0f
#define TAU_REF 89.0f

float baselineRise(int room) {
  return RISE_REF * ROOM_REF_FT3 / (float)ROOM_FT3[constrain(room, 0, 2)];
}

int baselineTau(int room, int airStatic, int tempF) {
  float v = (float)ROOM_FT3[constrain(room, 0, 2)] / ROOM_REF_FT3;
  float tau = TAU_REF * cbrtf(v);
  if (airStatic) tau *= 3.0f;
  // Warmer air convects, mixing and dispersing faster. Small, and linear is as
  // much as this deserves.
  tau *= constrain(1.0f - 0.01f * (tempF - 70), 0.7f, 1.3f);
  return (int)constrain(tau, 5.0f, 1800.0f);
}

// decayTau is the one constant nothing maintained: it cannot be measured while
// the loop is dosing, because with a 64s dead time "output off" does not mean
// "nothing arriving" - haze commanded a minute ago is still landing. Fitting
// the quiet stretches naively gave 58s, 128s, 294s and 351s from one half hour.
//
// The fix is to wait out the dead time first. After a full dead time with no
// output commanded and none on the wire, whatever is still arriving has
// arrived, and what remains is the room clearing on its own. Those windows are
// rarer but they are the real thing.
double dcN = 0, dcSx = 0, dcSy = 0, dcSxx = 0, dcSxy = 0, dcSyy = 0;
float adaptStartLvl = 0;
unsigned long dcStart = 0, lastOutputAt = 0;

void decayReset() {
  dcN = dcSx = dcSy = dcSxx = dcSxy = dcSyy = 0;
  dcStart = 0;
  adaptStartLvl = 0;
}

void decayTick(unsigned long now, float lvl, int demand, int wire) {
  if (demand > 0 || wire > 0) {
    lastOutputAt = now;
    decayReset();
    return;
  }
  // Two dead times, not one. A dead time is the mean transport delay and
  // arrival has a long tail behind it: with bursts every 14s there is nearly
  // always something still landing, and a window where late arrival offsets
  // decay fits a shallow slope, which reads as an enormous tau. That bias is
  // one-directional, so it ratchets - this fitter walked decayTau from 89s to
  // 567s while the room was visibly draining at a rate implying about 160s,
  // and halved the dose cap on the way.
  if (!lastOutputAt ||
      now - lastOutputAt < (unsigned long)cfg.deadTime * 2000 || lvl < 5.0f) {
    decayReset();
    return;
  }
  if (!dcStart) {
    dcStart = now;
    adaptStartLvl = lvl;
  }
  double t = (now - dcStart) / 1000.0;
  double y = log(lvl);
  dcN++; dcSx += t; dcSy += y; dcSxx += t * t; dcSxy += t * y; dcSyy += y * y;
  if (dcN < 60 || t < 90) return;  // 90s of quiet before believing anything

  // A window has to show real clearing to be evidence of clearing. Without
  // this, a level that barely moved fit a near-zero slope and produced a tau
  // of hundreds of seconds from a room that was not doing anything.
  if (lvl > adaptStartLvl * 0.8f) {
    decayReset();
    return;
  }
  double den = dcN * dcSxx - dcSx * dcSx;
  double b = den > 0 ? (dcN * dcSxy - dcSx * dcSy) / den : 0;
  // And it has to be a decay, not a wander: without a fit quality check a noisy
  // flat window passes on the strength of one end point, which is the same
  // mistake the two-point slope made.
  double sst = dcSyy - dcSy * dcSy / dcN;
  double ssr = sst - (b * (dcSxy - dcSx * dcSy / dcN));
  double r2 = sst > 0 ? 1.0 - ssr / sst : 0;
  if (b < -1e-6 && r2 > 0.8) {
    double tau = -1.0 / b;
    if (tau > 5 && tau < 1800) {
      int was = cfg.decayTau;
      // A quarter of each measurement, like the rise rate: this is drift, and
      // one quiet stretch is one sample of a room that changes.
      cfg.decayTau = (int)(cfg.decayTau + 0.25 * (tau - cfg.decayTau));
      cfg.decayLearned = true;
      logEvent("decay fitted %ds over %ds quiet, %.0f->%.0f, r2 %.2f, tau %d -> %d",
               (int)tau, (int)t, (double)adaptStartLvl, (double)lvl, r2, was,
               cfg.decayTau);
      saveAt = now + 2000;
    }
  }
  decayReset();
}

uint8_t doseCap(float gap, float riseRate, int deadTime, int pct) {
  if (riseRate <= 0 || deadTime <= 0) return 100;  // uncalibrated: no opinion
  if (gap <= 0) return 0;
  float u = 100.0f * gap / (riseRate * deadTime) * (pct / 100.0f);
  return u >= 100.0f ? 100 : (uint8_t)lroundf(u);
}

// Once levels are climbing quickly, the haze already in flight will keep them
// climbing for tens of seconds. Adding more only buys overshoot, so latch output
// off and hold it there until the rise actually stops. Latching matters: release
// on "not rising fast" would chatter, release on "falling" does not.
bool riseLockNext(bool cur, float slope, int cut) {
  if (cut <= 0) return false;
  if (slope >= cut) return true;
  if (slope <= 0) return false;
  return cur;  // between the thresholds: hold whatever we were doing
}

// Saturation has to be judged on whichever signal the loop is actually using.
// The mass reading pegs at about 1000 ug/m3 in any real haze, but the 0.3um
// count is still climbing freely there - treating a pegged mass reading as
// blindness while regulating on counts made auto-purge fire forever and hold
// output at zero, so the room could never reach target.
bool sensorSaturated() {
  if (!everRead) return false;
  if (cfg.source == 1) return data.particles_03um >= SAT_COUNT;
  if (cfg.source == 2) return data.pm10_env >= SAT_PM;
  return data.pm25_env >= SAT_PM;
}

// The signal the loop regulates on. PM2.5 mass is reported in whole ug/m3, so
// below about 5 it is quantised into uselessness - clean air reads 0 or 1. The
// 0.3um count reads in the thousands over the same span, which is where the
// resolution for very light haze actually lives. Scaled by 100 so one set of
// tuning constants, ranges and calibration numbers covers both sources.
// What each control signal reads from the current sample. Lets a baseline seed
// every signal at once, and a ratio be taken without switching sources.
float signalValue(int src) {
  if (!everRead) return 0.0f;
  if (src == 1) return data.particles_03um / 100.0f;
  if (src == 2) return (float)data.pm10_env;
  return (float)data.pm25_env;
}

float ctrlValue() {
  if (!everRead) return 0.0f;
  if (cfg.source == 1) return data.particles_03um / 100.0f;
  if (cfg.source == 2) return (float)data.pm10_env;  // PM1.0
  return (float)data.pm25_env;
}

// Prediction may hold output back while levels are still above the target band,
// which is the point of leading on the rising side. It must never do so once the
// measurement itself is already below the band: a lagging filter can show a
// rising trend while the room is in fact emptying, and acting on that starves
// the room exactly when it needs haze.
float controlPm(float predicted, float measured, float setpoint, float deadband) {
  // Below the band, anticipation may still ease output off as levels climb back
  // toward target - that is how you stop before overshooting. What it must not
  // do is claim the room is already past target when the sensor says it is not.
  if (measured < setpoint - deadband && predicted > setpoint) return setpoint;
  return predicted;
}

// Where PM2.5 will be once haze commanded now actually reaches the sensor.
// Controlling on this instead of the present reading is what buys back the
// dead time between opening the machine and seeing the result.
float predict(float pm, float slopePerSec, int lookaheadSec) {
  float p = pm + slopePerSec * lookaheadSec;
  return p < 0 ? 0 : (p > 2000 ? 2000 : p);
}

// Time-proportional output: instead of holding a low continuous level, run the
// machine at a strong level for a fraction of each cycle. Hazers atomise poorly
// at low duty, so 20% as 2s-on/8s-off often disperses better than a steady 20%.
// The machine keeps hazing for a few seconds after DMX drops to zero, so a
// commanded burst delivers its own length plus that tail. Shorten the command
// by the tail, or every short pulse overshoots its intended duty.
// The burst amplitude, given what is being asked for. level is a ceiling on
// how hard a burst hits, not a fixed height: asking for more than the ceiling
// means running continuously at the demand rather than diluting it.
int burstLevel(int demand, int level) { return level < demand ? demand : level; }

// Pulse with the on-time compensating for the amplitude, so the average
// delivered is the demand either way. At level 100 a 14% demand is a 100%
// slug for 14% of the cycle - which is what produces the spikes: the machine
// dumps a full-power burst and the room sees it arrive all at once. At level
// 20 the same 14% runs at 20% for 70% of the cycle. Same haze, spread out.
// level 100 reduces exactly to the original behaviour.
uint8_t dutyLevel(unsigned long ms, int demand, int period, int level, int tail) {
  if (demand <= 0) return 0;
  if (demand >= 100) return 100;
  int lvl = burstLevel(demand, level);
  unsigned long per = (unsigned long)period * 1000;
  long on = (long)(per * demand / lvl) - (long)tail * 1000;
  if (on <= 0) return 0;  // cannot deliver this little; period must be longer
  if ((unsigned long)on > per) on = (long)per;
  return (ms % per) < (unsigned long)on ? (uint8_t)lvl : 0;
}

// A fixture occupying n channels cannot start later than 513-n.
int maxAddress(uint8_t chans) { return DMX_PACKET_SIZE - chans; }

void calFail(const char *m) {
  snprintf(calMsg, sizeof(calMsg), "%s", m);
  logEvent("calibration failed: %s", m);
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
  cfg.leadFall = constrain(calDead, 0, 120);
  cfg.leadRise = constrain(calDead * 2, 0, 120);
  cfg.slew = constrain(100 / calDead, 1, 100);
  if (calNoise > 0) cfg.deadband = constrain((float)calNoise, 2.0f, 100.0f);
  // Integral time tracks the room's own decay constant: integrate no faster
  // than the process can actually respond, or the loop winds itself up.
  if (calTau > 0) {
    cfg.decayTau = constrain(calTau, 5, 1800);
    cfg.decayLearned = true;
    cfg.integralTi = constrain(calTau, 30, 1800);
  }
  // Feed the dose limiter: these are exactly the numbers it needs.
  cfg.riseRate = constrain(calRise, 0.0f, 500.0f);
  cfg.riseLearned = true;
  cfg.deadTime = constrain(calDead, 0, 300);
  if (calRise > 0.01f) {
    // Lambda (IMC) tuning rather than Ziegler-Nichols. ZN assumes you can
    // afford to oscillate around setpoint; here you cannot, because haze
    // arrives in seconds and clears over many minutes.
    float lambda = 3.0f * calDead;
    cfg.gain = constrain(100.0f / (calRise * (lambda + calDead)), 0.05f, 10.0f);
    // A huge measured rise rate - which is what a sensor sitting in the plume
    // reports - drives this formula to a gain so low the proportional band is
    // wider than the setpoint, and output rounds to nothing until levels are
    // far below target. Insist on reaching full output by half the setpoint.
    float minGain = 100.0f / max(20.0f, cfg.setpoint / 2.0f);
    if (cfg.gain < minGain) cfg.gain = minGain;
  }
  snprintf(calMsg, sizeof(calMsg),
           "pulse %ds dead %ds rise %.1f decay %ds -> lead %d/%d slew %d gain %.1f band %.2f",
           calPulseUsed, calDead, calRise, calTau, cfg.leadFall, cfg.leadRise,
           cfg.slew, cfg.gain, (double)cfg.deadband);
  if (calBaseHigh)
    strncat(calMsg, " (hazy baseline: rise may be understated)",
            sizeof(calMsg) - strlen(calMsg) - 1);
  logEvent("calibration done: %s", calMsg);
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
      // Measure how much the control signal moves with the machine off. A
      // deadband narrower than that just makes the loop chase its own noise.
      if (el < 2) { calNoiseLo = calNoiseHi = pm; }
      else { calNoiseLo = min(calNoiseLo, pm); calNoiseHi = max(calNoiseHi, pm); }
      if (el >= CAL_SETTLE_S) {
        calNoise = calNoiseHi - calNoiseLo;
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
      if (el >= (unsigned long)cfg.calPulse || sensorSaturated()) {
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
  prefs.putInt("source", cfg.source);
  prefs.putFloat("setpoint", cfg.setpoint);
  for (int i = 0; i < 3; i++) {
    char k[8];
    snprintf(k, sizeof(k), "sp%d", i);  prefs.putFloat(k, cfg.spBy[i]);
    snprintf(k, sizeof(k), "db%d", i);  prefs.putFloat(k, cfg.dbBy[i]);
    snprintf(k, sizeof(k), "ri%d", i);  prefs.putFloat(k, cfg.riseBy[i]);
  }
  prefs.putFloat("deadband", cfg.deadband);
  prefs.putFloat("gain", cfg.gain);
  prefs.putInt("slew", cfg.slew);
  prefs.putInt("fan", cfg.fan);
  prefs.putInt("addr", cfg.dmxAddress);
  prefs.putInt("fixture", cfg.fixture);
  prefs.putInt("look", cfg.leadFall);
  prefs.putInt("leadup", cfg.leadRise);
  prefs.putInt("tau", cfg.filterTau);
  prefs.putInt("tail", cfg.machineTail);
  prefs.putInt("floor", cfg.floorPct);
  prefs.putInt("risecut", cfg.riseCut);
  prefs.putInt("ti", cfg.integralTi);
  prefs.putInt("dtau", cfg.decayTau);
  prefs.putInt("room", cfg.roomSize);
  prefs.putInt("air", cfg.airMode);
  prefs.putInt("tempf", cfg.roomTempF);
  prefs.putBool("rlrn", cfg.riseLearned);
  prefs.putBool("dlrn", cfg.decayLearned);
  prefs.putFloat("rise", cfg.riseRate);
  prefs.putInt("dead", cfg.deadTime);
  prefs.putInt("dosepct", cfg.dosePct);
  prefs.putBool("stopped", cfg.stopped);
  prefs.putBool("pulse", cfg.pulseMode);
  prefs.putInt("pperiod", cfg.pulsePeriod);
  prefs.putInt("plevel", cfg.pulseLevel);
  prefs.putInt("calpulse", cfg.calPulse);
  prefs.putInt("pminon", cfg.pulseMinOn);
  prefs.putBool("autopurge", cfg.autoPurge);
}

void loadCfg() {
  cfg.automatic = prefs.getBool("auto", cfg.automatic);
  cfg.manual = prefs.getUChar("manual", cfg.manual);
  cfg.source = prefs.getInt("source", cfg.source);
  cfg.setpoint = prefs.getFloat("setpoint", cfg.setpoint);
  for (int i = 0; i < 3; i++) {
    char k[8];
    snprintf(k, sizeof(k), "sp%d", i);  cfg.spBy[i] = prefs.getFloat(k, 0.0f);
    snprintf(k, sizeof(k), "db%d", i);  cfg.dbBy[i] = prefs.getFloat(k, 0.0f);
    snprintf(k, sizeof(k), "ri%d", i);  cfg.riseBy[i] = prefs.getFloat(k, 0.0f);
  }
  cfg.deadband = prefs.getFloat("deadband", cfg.deadband);
  cfg.gain = prefs.getFloat("gain", cfg.gain);
  cfg.slew = prefs.getInt("slew", cfg.slew);
  cfg.fan = prefs.getInt("fan", cfg.fan);
  cfg.dmxAddress = prefs.getInt("addr", cfg.dmxAddress);
  cfg.fixture = prefs.getInt("fixture", cfg.fixture);
  cfg.leadFall = prefs.getInt("look", cfg.leadFall);
  cfg.leadRise = prefs.getInt("leadup", cfg.leadRise);
  cfg.filterTau = prefs.getInt("tau", cfg.filterTau);
  cfg.machineTail = prefs.getInt("tail", cfg.machineTail);
  cfg.floorPct = prefs.getInt("floor", cfg.floorPct);
  cfg.riseCut = prefs.getInt("risecut", cfg.riseCut);
  cfg.integralTi = prefs.getInt("ti", cfg.integralTi);
  // Upgrades inherit the value the two used to share.
  cfg.decayTau = prefs.getInt("dtau", cfg.integralTi);
  cfg.roomSize = prefs.getInt("room", cfg.roomSize);
  cfg.airMode = prefs.getInt("air", cfg.airMode);
  cfg.roomTempF = prefs.getInt("tempf", cfg.roomTempF);
  cfg.riseLearned = prefs.getBool("rlrn", false);
  cfg.decayLearned = prefs.getBool("dlrn", false);
  cfg.riseRate = prefs.getFloat("rise", cfg.riseRate);
  cfg.deadTime = prefs.getInt("dead", cfg.deadTime);
  cfg.dosePct = prefs.getInt("dosepct", cfg.dosePct);
  cfg.stopped = prefs.getBool("stopped", cfg.stopped);
  cfg.pulseMode = prefs.getBool("pulse", cfg.pulseMode);
  cfg.pulsePeriod = prefs.getInt("pperiod", cfg.pulsePeriod);
  cfg.pulseLevel = prefs.getInt("plevel", cfg.pulseLevel);
  cfg.calPulse = prefs.getInt("calpulse", cfg.calPulse);
  cfg.pulseMinOn = prefs.getInt("pminon", cfg.pulseMinOn);
  cfg.autoPurge = prefs.getBool("autopurge", cfg.autoPurge);
  // Clamp everything: stored bytes are not trustworthy input.
  cfg.fixture = constrain(cfg.fixture, 0, (int)FIXTURE_COUNT - 1);
  cfg.dmxAddress =
      constrain(cfg.dmxAddress, 1, maxAddress(FIXTURES[cfg.fixture].chans));
  cfg.manual = constrain(cfg.manual, 0, 100);
  cfg.fan = constrain(cfg.fan, 0, 100);
  cfg.source = constrain(cfg.source, 0, 2);
  cfg.setpoint = constrain(cfg.setpoint, 0.0f, 1000.0f);
  cfg.deadband = constrain(cfg.deadband, 0.0f, 100.0f);
  cfg.gain = constrain(cfg.gain, 0.1f, 10.0f);
  cfg.slew = constrain(cfg.slew, 1, 100);
  cfg.leadFall = constrain(cfg.leadFall, 0, 120);
  cfg.leadRise = constrain(cfg.leadRise, 0, 120);
  cfg.filterTau = constrain(cfg.filterTau, 0, 120);
  cfg.machineTail = constrain(cfg.machineTail, 0, 15);
  cfg.floorPct = constrain(cfg.floorPct, 0, 50);
  cfg.riseCut = constrain(cfg.riseCut, 0, 50);
  cfg.integralTi = constrain(cfg.integralTi, 0, 1800);
  cfg.decayTau = constrain(cfg.decayTau, 5, 1800);
  cfg.riseRate = constrain(cfg.riseRate, 0.0f, 500.0f);
  cfg.deadTime = constrain(cfg.deadTime, 0, 300);
  cfg.dosePct = constrain(cfg.dosePct, 10, 200);
  cfg.pulsePeriod = constrain(cfg.pulsePeriod, 5, 60);
  cfg.pulseLevel = constrain(cfg.pulseLevel, 10, 100);
  cfg.calPulse = constrain(cfg.calPulse, 30, 600);
  cfg.pulseMinOn = constrain(cfg.pulseMinOn, 1, 10);
}

// PI with a deadband. Proportional alone always settles short of target - it
// needs a standing error to produce the output that balances the room's decay -
// and that droop is what the integral removes. No derivative term: the lead
// times already anticipate, and differentiating this sensor amplifies noise.
//
// Anti-windup is conditional integration: stop accumulating whenever the output
// is already pinned at a limit and the error pushes it further that way. Without
// it the integral charges through a long saturation and then will not let go.
uint8_t computePI(float setpoint, float pm25, float deadband, float gain, int ti,
                  float &acc) {
  float err = setpoint - pm25;
  if (err > deadband) err -= deadband;
  else if (err < -deadband) err += deadband;
  else err = 0;
  float p = gain * err;
  float out = p + acc;
  if (ti > 0) {
    bool pinned = (out >= 100.0f && err > 0) || (out <= 0.0f && err < 0);
    if (!pinned) acc += gain * err / ti;  // control tick is 1s
    acc = constrain(acc, 0.0f, 100.0f);
    out = p + acc;
  } else {
    acc = 0;
  }
  long r = lroundf(out);
  return r < 0 ? 0 : (r > 100 ? 100 : (uint8_t)r);
}

// Proportional-only wrapper, used by the tests.
// computePI zeroes the error anywhere inside +/-deadband, so setpoint and
// deadband have always described a band - a centre and a half-width. Saying it
// as min/max changes no behaviour, only which two numbers the operator types.
// Kept as derived accessors rather than stored fields so there is exactly one
// representation of the band and the two can never disagree.
float targetMin() { return cfg.setpoint - cfg.deadband; }
float targetMax() { return cfg.setpoint + cfg.deadband; }

void setTargetBand(float lo, float hi) {
  if (hi < lo) { float t = lo; lo = hi; hi = t; }  // typed backwards: do not invert the loop
  float was = cfg.setpoint;
  cfg.setpoint = constrain((lo + hi) * 0.5f, 0.0f, 1000.0f);
  cfg.deadband = constrain((hi - lo) * 0.5f, 0.0f, 100.0f);
  // Same rule the old setpoint setter used: an integral charged to hold a
  // different level keeps commanding it for minutes after the band moves.
  if (fabsf(cfg.setpoint - was) > fmaxf(2.0f, was * 0.1f)) integ = 0;
}

uint8_t computeOutput(float setpoint, float pm25, float deadband, float gain) {
  float ignore = 0;
  return computePI(setpoint, pm25, deadband, gain, 0, ignore);
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

// Checks report rather than abort. assert() panics inside setup(), before wifi
// starts, so a single wrong expectation takes the board off the network with no
// way in but a cable - which it has now done twice. A failure here is a bug to
// fix, but it must not cost physical access to the machine.
int checkFails = 0;
#define CHECK(c)                                                    \
  do {                                                              \
    if (!(c)) {                                                     \
      checkFails++;                                                 \
      Serial.printf("SELFTEST FAILED line %d: %s\n", __LINE__, #c); \
    }                                                               \
  } while (0)

// Defined further down, and the only thing selfTest calls that is. The .ino
// auto-prototype pass normally covers this, but it is sensitive to edits
// elsewhere in the file - it stopped emitting this one after an unrelated
// change to the page markup. Declaring it explicitly costs a line and does not
// depend on that pass behaving.
uint8_t ledLevel(uint8_t pct);

void selfTest() {
  CHECK(computeOutput(150, 200, 10, 1.0f) == 0);   // too hazy -> off
  CHECK(computeOutput(150, 150, 10, 1.0f) == 0);   // at target -> off
  CHECK(computeOutput(150, 130, 10, 1.0f) == 10);  // err 20, less deadband
  CHECK(computeOutput(150, 0, 10, 1.0f) == 100);   // clamps at 100%
  CHECK(applySlew(0, 100, 3) == 3);                // ramps up, no blast
  CHECK(applySlew(100, 0, 3) == 97);               // ramps down
  CHECK(applySlew(40, 41, 3) == 41);               // small step lands exactly
  CHECK(applySlew(0, 0, 3) == 0);
  CHECK(toDmx(0) == 0);      // off stays off
  CHECK(toDmx(1) == 11);     // 1% = lowest live value, skips the dead band
  CHECK(toDmx(100) == 255);  // 100%
  CHECK(toDmx(50) == 131);   // 50% lands mid-band
  CHECK(ledLevel(0) == 0);          // off is dark
  CHECK(ledLevel(1) >= 12);         // lowest output is still visible
  CHECK(ledLevel(100) == 130);      // full output, capped brightness
  CHECK(ledLevel(50) > ledLevel(25) && ledLevel(50) < ledLevel(100));
  CHECK(maxAddress(2) == 511);  // Amhaze, 2ch
  CHECK(maxAddress(1) == 512);  // Hurricane Haze 1DX, 1ch
  CHECK(slopeOf(100, 10, 31) == 3.0f);      // +90 over 30s
  CHECK(slopeOf(10, 100, 31) == -3.0f);     // falling
  CHECK(slopeOf(50, 50, 1) == 0.0f);        // too few samples
  {
    // A clean ramp fits exactly, and one wild sample barely moves the fit -
    // the two-point version would have followed it straight off the rail.
    uint16_t ramp[SLOPE_WIN];
    for (int i = 0; i < SLOPE_WIN; i++) ramp[i] = 100 + 2 * i;
    CHECK(fabsf(slopeFit(ramp, 0, SLOPE_WIN, SLOPE_WIN) - 2.0f) < 0.01f);
    // A plume across the newest sample. The fit is not immune to it - it moves
    // by about 4.8 - but the two-point slope moves by 25.6, and it is that
    // number which gets multiplied by a 120s lead. Assert the ratio, since the
    // ratio is the actual claim.
    ramp[SLOPE_WIN - 1] = 900;
    float fit = fabsf(slopeFit(ramp, 0, SLOPE_WIN, SLOPE_WIN) - 2.0f);
    float two = fabsf(slopeOf(ramp[SLOPE_WIN - 1], ramp[0], SLOPE_WIN) - 2.0f);
    CHECK(fit < two / 4.0f);
    CHECK(slopeFit(ramp, 0, 4, SLOPE_WIN) == 0.0f);  // too few to fit
  }
  CHECK(predict(100, -2.0f, 30) == 40);     // falling fast, act early
  CHECK(predict(10, -2.0f, 30) == 0);       // clamps at zero
  CHECK(predict(100, 0.0f, 30) == 100);     // flat trend changes nothing
  CHECK(leadFor(-1.0f, 20, 45) == 20);      // falling -> start early
  CHECK(leadFor(1.0f, 20, 45) == 45);       // rising  -> stop early
  CHECK(leadFor(0.0f, 20, 45) == 45);
  CHECK(controlPm(265, 157, 200, 10) == 200);  // below band: capped at target
  CHECK(controlPm(195, 157, 200, 10) == 195);  // easing off early is allowed
  CHECK(controlPm(265, 300, 200, 10) == 265);  // above band: lead is allowed
  CHECK(controlPm(100, 157, 200, 10) == 100);  // leading on early is untouched
  // A usable gain must reach full output before the room is empty.
  CHECK(computeOutput(200, 100, 10, 1.0f) == 90);   // half target -> 90%
  CHECK(computeOutput(200, 100, 10, 0.1f) == 9);    // same error at 0.1 -> 9%
  CHECK(riseLockNext(false, 5.0f, 3) == true);      // climbing fast -> cut
  CHECK(riseLockNext(true, 1.0f, 3) == true);       // still rising -> stay cut
  CHECK(riseLockNext(true, -1.0f, 3) == false);     // falling again -> release
  CHECK(riseLockNext(true, 0.0f, 3) == false);      // stopped rising -> release
  CHECK(riseLockNext(true, 9.0f, 0) == false);      // disabled
  // 150 deficit, 20 units/s at full, 20s dead time: 37.5% exactly closes it
  CHECK(doseCap(150, 20.0f, 20, 100) == 38);
  CHECK(doseCap(0, 20.0f, 20, 100) == 0);        // at target, commit nothing
  CHECK(doseCap(1000, 20.0f, 20, 100) == 100);   // huge deficit, still capped
  CHECK(doseCap(150, 0.0f, 20, 100) == 100);     // uncalibrated: no cap
  CHECK(doseCap(150, 20.0f, 20, 50) == 19);      // half-dose setting
  // Replacing the leak. At the target with the gap closed the cap must still
  // allow the maintenance rate, or the level cannot be held at all.
  CHECK(fabsf(holdLoss(175, 89, 64) - 125.8f) < 0.5f);
  CHECK(holdLoss(175, 0, 64) == 0.0f);   // no decay constant, no claim
  CHECK(holdLoss(0, 89, 64) == 0.0f);    // empty room loses nothing
  CHECK(doseCap(holdLoss(175, 89, 64), 16.22f, 64, 100) == 12);  // the 12.1% it needed
  // And the old behaviour is unchanged where there is no decay to replace.
  CHECK(doseCap(150 + holdLoss(0, 89, 20), 20.0f, 20, 100) == 38);
  // Adaptive rise. Holding level at 100 with 5% duty and a 400s decay means
  // the machine is replacing 0.25/s with 0.05 of its full output: 5.0/s.
  CHECK(fabsf(riseEstimate(100, 100, 100, 0.05f, 600, 400) - 5.0f) < 0.01f);
  // Same duty but still climbing 0.1/s means it is stronger than that.
  CHECK(fabsf(riseEstimate(100, 160, 130, 0.05f, 600, 400) - 8.5f) < 0.01f);
  CHECK(riseEstimate(100, 100, 100, 0.0f, 600, 400) == 0.0f);    // no output, no evidence
  CHECK(riseEstimate(100, 100, 100, 0.05f, 600, 0) == 0.0f);     // no decay constant
  // Falling 0.62/s for 90s adds ~56 to a 135 deficit
  // 190.8 projected, but capped at the setpoint by the clause above
  // setTargetBand writes live config, so put it back afterwards. Today this
  // runs before prefs load and the values would be overwritten anyway, but a
  // self-test that quietly rewrites the operator's band if anyone reorders
  // setup() is not a trade worth leaving in place.
  const float keepSp = cfg.setpoint, keepDb = cfg.deadband;
  // The band round-trips: what you type is what comes back.
  setTargetBand(160, 190);
  CHECK(fabsf(cfg.setpoint - 175.0f) < 0.01f);
  CHECK(fabsf(cfg.deadband - 15.0f) < 0.01f);
  CHECK(fabsf(targetMin() - 160.0f) < 0.01f);
  CHECK(fabsf(targetMax() - 190.0f) < 0.01f);
  setTargetBand(190, 160);  // reversed input must not invert the band
  CHECK(fabsf(targetMin() - 160.0f) < 0.01f);
  // Inside the band the loop does nothing; below min it acts; above max it is
  // off. This is the behaviour min/max is only renaming.
  CHECK(computeOutput(175, 175, 15, 1.0f) == 0);   // centre
  CHECK(computeOutput(175, 161, 15, 1.0f) == 0);   // just inside min
  CHECK(computeOutput(175, 189, 15, 1.0f) == 0);   // just inside max
  CHECK(computeOutput(175, 200, 15, 1.0f) == 0);   // above max
  CHECK(computeOutput(175, 150, 15, 1.0f) == 10);  // below min: err 25 less 15
  cfg.setpoint = keepSp;
  cfg.deadband = keepDb;
  CHECK(deficitAtArrival(150, 15, -0.62f, 90) == 150.0f);
  CHECK(fabsf(deficitAtArrival(150, 120, -0.2f, 90) - 48.0f) < 0.5f);
  CHECK(deficitAtArrival(150, 150, 0.0f, 90) == 0.0f);
  CHECK(deficitAtArrival(150, 100, 1.0f, 50) == 0.0f);   // rising: arrives on target
  CHECK(deficitAtArrival(150, 20, -9.0f, 90) == 150.0f);  // steep slope, clamped
  float acc = 0;
  CHECK(computePI(200, 100, 10, 1.0f, 0, acc) == 90);   // Ti=0 is plain P
  CHECK(acc == 0.0f);
  acc = 0;
  computePI(200, 190, 10, 1.0f, 100, acc);               // err 0 inside band
  CHECK(acc == 0.0f);                                   // band does not wind up
  acc = 0;
  for (int i = 0; i < 50; i++) computePI(200, 180, 10, 0.1f, 100, acc);
  CHECK(acc > 0.0f && acc <= 100.0f);  // droop outside the band gets integrated
  acc = 90.0f;
  for (int i = 0; i < 50; i++) computePI(200, 0, 10, 1.0f, 100, acc);
  CHECK(acc <= 100.0f);                                 // anti-windup caps it
  CHECK(dutyLevel(0, 20, 10, 100, 0) == 100);      // 20% of 10s: on at t=0
  CHECK(dutyLevel(1999, 20, 10, 100, 0) == 100);   // still on just before 2s
  CHECK(dutyLevel(2001, 20, 10, 100, 0) == 0);     // off after 2s
  CHECK(dutyLevel(9999, 20, 10, 100, 0) == 0);     // off until the cycle repeats
  CHECK(dutyLevel(5000, 0, 10, 100, 0) == 0);      // zero demand never fires
  CHECK(dutyLevel(5000, 100, 10, 100, 0) == 100);  // full demand is continuous
  CHECK(dutyLevel(999, 20, 10, 100, 1) == 100);    // 1s tail: command only 1s
  CHECK(dutyLevel(1001, 20, 10, 100, 1) == 0);
  CHECK(dutyLevel(500, 20, 10, 100, 3) == 0);      // tail alone exceeds the duty
  // Amplitude-compensated pulsing: same average, gentler burst, longer on-time.
  CHECK(burstLevel(14, 20) == 20);   // ceiling applies
  CHECK(burstLevel(50, 20) == 50);   // asking for more than the ceiling wins
  CHECK(dutyLevel(6999, 14, 10, 20, 0) == 20);   // 14/20 = 70% of a 10s cycle
  CHECK(dutyLevel(7001, 14, 10, 20, 0) == 0);    // off after 7s
  CHECK(dutyLevel(1399, 14, 10, 100, 0) == 100); // level 100: the old 1.4s slug
  CHECK(dutyLevel(1401, 14, 10, 100, 0) == 0);
  CHECK(dutyLevel(5000, 50, 10, 20, 0) == 50);   // demand above ceiling: continuous
  CHECK(dutyLevel(9999, 50, 10, 20, 0) == 50);
  if (checkFails) Serial.printf("selfTest: %d CHECK(s) FAILED\n", checkFails);
  else Serial.println("selfTest ok");
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>HAZE B STDY - recovery</title>
<style>body{font:15px system-ui;background:#141414;color:#eee;margin:0;padding:20px}
h1{font-size:17px;margin:0 0 10px}
pre{background:#1c1c1c;padding:10px;border-radius:8px;overflow:auto;font-size:12px;line-height:1.5}
button,input{font:inherit;background:#2e2e2e;color:#eee;border:1px solid #444;border-radius:6px;padding:6px 10px}
.w{color:#c9b458}</style>
<h1>Haze Regulator - recovery page</h1>
<p class=w>The full interface is not on the filesystem. The controller is
running normally - only the page is missing. Upload one, or run ./ui.sh</p>
<p><button onclick="location.reload()">Retry</button>
<button onclick="r()">Restore previous interface</button>
<i id=m style=color:#888></i></p>
<form method=POST action=/ui enctype=multipart/form-data>
<input type=file name=ui accept=text/html><button>Upload interface</button></form>
<pre id=s>loading...</pre>
<script>
// Restore first, upload second: putting back a page that worked an hour ago
// needs no laptop, no toolchain and no network beyond this one request.
function r(){m.textContent='restoring...';
  fetch('/ui/restore').then(x=>x.text()).then(t=>{m.textContent=t;
    if(t=='restored')setTimeout(()=>location.reload(),600)})
  .catch(e=>m.textContent='failed')}
const f=()=>fetch('/api/state').then(r=>r.json()).then(d=>{
  s.textContent=Object.entries(d).map(([k,v])=>k+': '+v).join('\n')}).catch(e=>0);
f();setInterval(f,2000);
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
  // Long enough that a burst still lasts pulseMinOn after the tail is removed.
  // Against the burst amplitude, not 100: a gentler burst is already longer for
  // the same demand, so it needs less stretching to clear the minimum.
  int lvl = burstLevel(output, cfg.pulseLevel);
  int per = ((cfg.pulseMinOn + cfg.machineTail) * lvl + output - 1) / output;
  return constrain(per, 5, 120);
}

// Green while haze is actually being commanded, dark otherwise. In pulse mode
// this blinks with the bursts, which is a useful at-a-glance duty indicator.
// Written only on change: driving the LED briefly masks interrupts.
uint8_t hazeLevel();  // forward decl
int16_t ledGreen = -1;

// Green channel scaled by the haze percentage actually on the wire. Gamma
// curves the response, since LED output is linear in value but the eye is not.
// A floor of 12 matters more than strict perceptual accuracy: at gamma 2.2 a
// typical 19% output landed on 7 of 255, which is invisible in a lit room, and
// an indicator you cannot see is worse than a slightly non-linear one.
uint8_t ledLevel(uint8_t pct) {
  if (pct == 0) return 0;
  return 12 + (uint8_t)(powf(pct / 100.0f, 1.6f) * 118.0f);
}

void updateLed() {
  // Red beats green: a pegged sensor means the regulator is blind, which is
  // worth seeing across the room whatever the output happens to be.
  bool sat = sensorOk && sensorSaturated();
  int16_t g = sat ? -1 : ledLevel(hazeLevel());
  // Brightness follows the wire, which in pulse mode is off for most of the
  // cycle - at 2% demand that is a ~2s burst every 120s, so the board looks
  // dead to anyone glancing at it. Hold a dim floor while the regulator is
  // running so the LED answers "is this alive" first and "how hard is it
  // working" second. Dark now means stopped, which is the one state worth
  // being unambiguous about.
  if (!sat && !cfg.stopped && g < 12) g = 12;
  if (g == ledGreen) return;  // written only on change: this masks interrupts
  ledGreen = g;
  if (sat) rgbLedWrite(RGB_LED_PIN, 110, 0, 0);
  else rgbLedWrite(RGB_LED_PIN, 0, (uint8_t)g, 0);
}

// What is actually on the wire right now, pulsing included.
uint8_t hazeLevel() {
  if (cfg.pulseMode && !cfg.stopped && !purging() && !calibrating())
    return dutyLevel(millis(), output, pulsePeriodNow(), cfg.pulseLevel,
                     cfg.machineTail);
  return output;
}

// Every endpoint, not just the writing ones. This board is published at
// haze.anthonychiappone.com now, so "anyone who can reach the control page can
// already drive the hazer" stopped being a statement about the office LAN.
// Basic auth over plain HTTP on the LAN is weak, but the tunnel terminates TLS
// at the edge and this is what stops an unauthenticated /update from the
// public internet. ponytail: swap for Cloudflare Access + a service token if
// the credentials ever need rotating without a flash.
bool authOk() {
  if (server.authenticate(WEB_USER, WEB_PASS)) return true;
  server.requestAuthentication();
  return false;
}

void handleState() {
  if (!authOk()) return;
  char buf[1560];
  snprintf(buf, sizeof(buf),
           "{\"pm25\":%u,\"pm10\":%u,\"pm100\":%u,\"c03\":%u,\"rssi\":%d,\"up\":%lu,"
           "\"output\":%u,\"target\":%u,\"automatic\":%s,\"manual\":%u,"
           "\"source\":%d,\"ctrl\":%.2f,\"setpoint\":%.2f,\"deadband\":%.2f,"
           "\"tgtmin\":%.2f,\"tgtmax\":%.2f,\"gain\":%.1f,\"slew\":%d,"
           "\"fan\":%d,\"dmxaddr\":%d,\"dmxhaze\":%u,\"fixture\":%d,"
           "\"hasfan\":%s,\"maxaddr\":%d,\"purge\":%d,\"dmxfan\":%u,\"leadfall\":%d,\"leadrise\":%d,\"tau\":%d,\"tail\":%d,\"floor\":%d,\"risecut\":%d,\"riselock\":%s,\"ti\":%d,\"integ\":%.1f,\"rise\":%.2f,\"dead\":%d,\"dosepct\":%d,\"clock\":%s,\"pmf\":%d,\"pkses\":%u,\"pksesat\":%lu,"
           "\"pkall\":%u,\"pkallb\":%lu,\"pkallat\":%lu,\"pkout\":%u,"
           "\"slope\":%.2f,\"predicted\":%.2f,\"stopped\":%s,\"cal\":%d,\"calmsg\":\"%s\",\"pulse\":%s,\"pperiod\":%d,"
           "\"calpulse\":%d,\"calleft\":%d,\"haze\":%u,\"pnow\":%d,"
           "\"pminon\":%d,\"plevel\":%d,\"lps\":%u,\"reqs\":%u,\"slowreq\":%u,"
           "\"fw\":\"%s\",\"built\":\"%s\",\"ui\":%d,\"uigen\":%u,\"caprelax\":%.2f,\"dtau\":%d,\"room\":%d,\"air\":%d,\"tempf\":%d,\"baserise\":%.2f,\"rlearn\":%d,\"dlearn\":%d,"
           // Board health. minheap rather than heap alone: free heap at the
           // moment you looked says little, the low water mark since boot says
           // whether anything ever came close.
           "\"heap\":%u,\"minheap\":%u,\"heaptotal\":%u,\"avgreq\":%u,"
           "\"psfree\":%u,\"pstotal\":%u,"
           "\"sketch\":%u,\"flashfree\":%u,\"fsused\":%u,\"fstotal\":%u,"
           "\"chiptemp\":%.1f,"
           "\"ssid\":\"%s\",\"ip\":\"%s\",\"bssid\":\"%s\",\"chan\":%d,\"phy\":\"%s\",\"txp\":%.1f,"
           "\"reassoc\":%u,\"assoc\":%lu,"
           "\"autopurge\":%s,\"noresp\":%s,\"sensorOk\":%s}",
           everRead ? data.pm25_env : 0, everRead ? data.pm10_env : 0,
           everRead ? data.pm100_env : 0, everRead ? data.particles_03um : 0,
           (int)WiFi.RSSI(), (unsigned long)(millis() / 1000), output, target,
           cfg.automatic ? "true" : "false", cfg.manual, cfg.source,
           (double)ctrlValue(), (double)cfg.setpoint, (double)cfg.deadband,
           (double)targetMin(), (double)targetMax(),
           cfg.gain, cfg.slew, cfg.fan, cfg.dmxAddress,
           toDmx(hazeLevel()), cfg.fixture,
           FIXTURES[cfg.fixture].fanOff >= 0 ? "true" : "false",
           maxAddress(FIXTURES[cfg.fixture].chans),
           purging() ? (int)((purgeUntil - millis()) / 1000) : 0,
           FIXTURES[cfg.fixture].fanOff >= 0 ? toDmx(purging() ? 100 : cfg.fan) : 0,
           cfg.leadFall, cfg.leadRise, cfg.filterTau, cfg.machineTail,
           cfg.floorPct, cfg.riseCut, riseLock ? "true" : "false",
           cfg.integralTi, integ, (double)cfg.riseRate, cfg.deadTime,
           cfg.dosePct,
           time(nullptr) > 1700000000 ? "true" : "false",
           (int)lroundf(pmFilt), pkSes, (unsigned long)pkSesAt, pkAll,
           (unsigned long)pkAllBoot, (unsigned long)pkAllAt, pkOut,
           pmSlope, predicted, cfg.stopped ? "true" : "false",
           calState, calMsg, cfg.pulseMode ? "true" : "false", cfg.pulsePeriod,
           cfg.calPulse, calLeft(), hazeLevel(), pulsePeriodNow(),
           cfg.pulseMinOn, cfg.pulseLevel, loopRate, servedReqs, slowestReqMs,
           FW_VERSION, FW_BUILT, uiFromFs() ? 1 : 0, uiGen, (double)capRelax, cfg.decayTau, cfg.roomSize,
           cfg.airMode, cfg.roomTempF, (double)baselineRise(cfg.roomSize),
           cfg.riseLearned ? 1 : 0, cfg.decayLearned ? 1 : 0,
           (unsigned)ESP.getFreeHeap(),
           (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getHeapSize(),
           (unsigned)(servedReqs ? reqMsTotal / servedReqs : 0),
           (unsigned)ESP.getFreePsram(),
           (unsigned)ESP.getPsramSize(), (unsigned)ESP.getSketchSize(),
           (unsigned)ESP.getFreeSketchSpace(),
           (unsigned)(fsOk ? LittleFS.usedBytes() : 0),
           (unsigned)(fsOk ? LittleFS.totalBytes() : 0),
           (double)temperatureRead(), WiFi.SSID().c_str(),
           WiFi.localIP().toString().c_str(), WiFi.BSSIDstr().c_str(),
           WiFi.channel(),
           phyMode(), txPowerDbm(), (unsigned)reassocCount,
           (unsigned long)(assocAt ? (millis() - assocAt) / 1000 : 0),
           cfg.autoPurge ? "true" : "false",
           noResponse ? "true" : "false", sensorOk ? "true" : "false");
  server.send(200, "application/json", buf);
}

// The 2Hz tier lives only in RAM, so after a reboot it is empty while the
// minute tier has been restored from flash. Prefer whichever actually covers
// more of the asked-for window, rather than always trusting resolution.
bool pickFine(int win) {
  if (win > 3600) return false;
  // Under two minutes the minute tier yields one point or none, so however
  // sparse the fast tier is, it is the only one that can draw anything.
  if (win < 120) return true;
  long fineMs = (long)fineN * HIST_FINE_MS;
  long coarseMs = (long)coarseN * HIST_COARSE_MS;
  // Half a window of real detail beats a full window of minute averages.
  // Demanding full coverage meant every reboot gave ten chunky points for the
  // first ten minutes, even though 2Hz data was already piling up.
  return fineMs * 2 >= (long)win * 1000 || fineMs >= coarseMs;
}

// Decimates server-side to at most MAXPTS points: a 7 day window holds 10080
// samples, and sending them all would be a megabyte of JSON to draw on a strip
// a few hundred pixels wide.
#define MAXPTS 420
// Serve one merged series instead of picking a tier: minute averages for the
// older part of the window, 2Hz detail for however recent a stretch the RAM
// buffer still holds. Switching between tiers made the chart jump resolution
// as the fast buffer filled after a reboot. Each point carries its own age, so
// the two resolutions can sit side by side on the same axis.
// Age of minute sample k (0 = newest). Ones this session wrote sit on the
// current timeline; restored ones are pushed back behind boot.
long coarseAgeMs(int k) {
  if (k < coarsePostBoot) return (long)k * HIST_COARSE_MS;
  return (long)millis() + (long)(k - coarsePostBoot) * HIST_COARSE_MS;
}

void emitRange(bool coarse, int from, int cnt, int step, int field, bool &first) {
  Sample *buf = coarse ? coarseBuf : fineBuf;
  int cap = coarse ? coarseCap : fineCap;
  int head = coarse ? coarseHead : fineHead;
  int per = coarse ? HIST_COARSE_MS : HIST_FINE_MS;
  int newest = (head - 1 + cap) % cap;
  String chunk;
  chunk.reserve(1200);
  for (int k = from + (cnt - 1) * step; k >= from; k -= step) {
    long v;
    if (field == 0) {
      // The timestamp still comes from the bucket's own edge, so x positions
      // are unchanged and the axis does not shift.
      v = (coarse ? coarseAgeMs(k) : (long)k * per) / 100;  // tenths of a second
    } else {
      // Worst value in the bucket, not whichever single sample the stride
      // happens to land on. In pulse mode the wire is a square wave - a ~6s
      // burst every ~29s at 21% demand - while an hour-wide window strides
      // ~8.5s, so plain subsampling hit a burst or missed it depending on
      // where the stride fell. Scrolling by one sample changed the answer and
      // bursts blinked in and out. Taking the max makes a spike that exists
      // stay drawn, at the cost of reading high for a pulsed signal on wide
      // windows: one pixel covers several bursts and shows the burst level
      // rather than the duty. For a chart whose job is finding excursions,
      // never hiding one is the right side to err on.
      long best = 0;
      for (int j = 0; j < step; j++) {
        int kk = k - j;  // toward newer; the bucket is (k-step, k]
        if (kk < from) break;
        int idx = ((newest - kk) % cap + cap) % cap;
        long vv = field == 1   ? buf[idx].pm
                  : field == 2 ? buf[idx].out
                  : field == 3 ? buf[idx].act
                  : field == 4 ? buf[idx].ctrl
                  : field == 5 ? buf[idx].pm1
                               : buf[idx].pm10;
        if (vv > best) best = vv;
      }
      v = best;
    }
    if (!first) chunk += ',';
    first = false;
    chunk += v;
    if (chunk.length() > 1000) { server.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) server.sendContent(chunk);
}

void handleHistory() {
  if (!authOk()) return;
  if (server.hasArg("clearpeaks")) {
    pkSes = pkOut = pkAll = 0;
    pkSesAt = pkAllBoot = pkAllAt = 0;
    prefs.putUShort("pkall", 0);
    prefs.putUInt("pkallb", 0);
    prefs.putUInt("pkallat", 0);
    logEvent("peaks cleared");
    server.send(200, "application/json", "{\"n\":0}");
    return;
  }
  if (server.hasArg("clear")) {
    fineN = fineHead = coarseN = coarseHead = coarsePostBoot = 0;
    cAccPm = cAccCtrl = cAccOut = cAccAct = 0;
    cAccPm1 = cAccPm10 = 0;
    cAccN = 0;
    if (fsOk) LittleFS.remove(TREND_PATH);
    logEvent("trend history cleared");
    server.send(200, "application/json", "{\"n\":0}");
    return;
  }
  int win = server.hasArg("win") ? server.arg("win").toInt() : 600;
  win = constrain(win, 30, 604800);
  long winMs = (long)win * 1000;

  // Right edge of the window, as an age. 0 is live; anything else is the chart
  // scrolled back into history, which is the only way to zoom in on something
  // that has already happened - the buffers hold the detail, the old API just
  // had no way to ask for it.
  long offMs = (long)constrain(server.hasArg("off") ? server.arg("off").toInt() : 0,
                               0, 604800) * 1000;
  long farMs = offMs + winMs;  // left edge

  // The fine tier only reaches back as far as it holds; beyond that the window
  // is served entirely from minute samples.
  long fineHave = (long)fineN * HIST_FINE_MS;
  long fineFromMs = offMs;
  long fineToMs = farMs < fineHave ? farMs : fineHave;
  int fineFrom = fineFromMs / HIST_FINE_MS;
  int fineWant = fineToMs > fineFromMs ? (fineToMs - fineFromMs) / HIST_FINE_MS : 0;
  long fineMs = fineWant ? fineToMs - fineFromMs : 0;

  // Skip minute samples that the fine part already covers, then take only the
  // ones whose real age still falls inside the window.
  int coarseSkip = fineToMs / HIST_COARSE_MS;
  if (coarseSkip > coarsePostBoot) coarseSkip = coarsePostBoot;
  int coarseWant = 0;
  for (int k = coarseSkip; k < coarseN; k++) {
    if (coarseAgeMs(k) <= fineToMs) { coarseSkip = k + 1; continue; }
    if (coarseAgeMs(k) > farMs) break;
    coarseWant++;
  }

  if (fineWant + coarseWant < 2) {
    server.send(200, "application/json", "{\"n\":0}");
    return;
  }
  // Budget points by how much of the axis each part covers, but always keep a
  // little detail at the right-hand edge.
  int budF = winMs ? (int)(fineMs * MAXPTS / winMs) : MAXPTS;
  if (fineWant && budF < 40) budF = 40;
  if (budF > MAXPTS) budF = MAXPTS;
  int budC = MAXPTS - budF;
  int stepF = fineWant && budF ? (fineWant + budF - 1) / budF : 1;
  int stepC = coarseWant && budC ? (coarseWant + budC - 1) / budC : 1;
  if (stepF < 1) stepF = 1;
  if (stepC < 1) stepC = 1;
  int nF = stepF ? fineWant / stepF : 0;
  int nC = stepC ? coarseWant / stepC : 0;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  char hd[176];
  long spanMs = coarseN ? coarseAgeMs(coarseN - 1) : (long)fineN * HIST_FINE_MS;
  snprintf(hd, sizeof(hd),
           "{\"n\":%d,\"sp\":%.2f,\"lo\":%.2f,\"hi\":%.2f,\"span\":%ld,\"off\":%ld,\"t\":[",
           nF + nC, (double)cfg.setpoint, (double)targetMin(),
           (double)targetMax(), spanMs / 1000, offMs / 1000);
  server.sendContent(hd);
  // ctrl is appended last so existing keys keep their meaning and a page
  // served from an older cache still finds pm/out/act where it expects them.
  for (int field = 0; field < 7; field++) {
    if (field) server.sendContent(field == 1   ? "],\"pm\":["
                                  : field == 2 ? "],\"out\":["
                                  : field == 3 ? "],\"act\":["
                                  : field == 4 ? "],\"ctrl\":["
                                  : field == 5 ? "],\"pm1\":["
                                               : "],\"pm10\":[");
    bool first = true;
    if (nC) emitRange(true, coarseSkip, nC, stepC, field, first);
    if (nF) emitRange(false, fineFrom, nF, stepF, field, first);
  }
  server.sendContent("]}");
  server.sendContent("");
}

void handleEvents() {
  if (!authOk()) return;
  if (server.hasArg("clear")) {
    if (fsOk) LittleFS.remove(EVT_PATH);
    logEvent("log cleared");
    server.send(200, "text/plain", "");
    return;
  }
  if (!fsOk) { server.send(200, "text/plain", "no filesystem"); return; }
  File f = LittleFS.open(EVT_PATH, "r");
  if (!f) { server.send(200, "text/plain", "(no events yet)"); return; }
  size_t sz = f.size();
  bool dl = server.hasArg("download");
  // The viewer only needs the tail; an export wants the whole file.
  if (!dl && sz > 12288) { f.seek(sz - 12288); f.readStringUntil('\n'); }
  if (dl)
    server.sendHeader("Content-Disposition",
                      "attachment; filename=haze_events.log");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain", "");
  uint8_t b[256];
  int r;
  while ((r = f.read(b, sizeof(b))) > 0) server.sendContent((const char *)b, r);
  f.close();
  server.sendContent("");
}

void handleCsv() {
  if (!authOk()) return;
  int win = server.hasArg("win") ? server.arg("win").toInt() : 600;
  win = constrain(win, 30, 604800);
  bool fine = pickFine(win);
  Sample *buf = fine ? fineBuf : coarseBuf;
  int cap = fine ? fineCap : coarseCap;
  int n = fine ? fineN : coarseN;
  int head = fine ? fineHead : coarseHead;
  int perMs = fine ? HIST_FINE_MS : HIST_COARSE_MS;
  int want = (int)((long)win * 1000 / perMs);
  if (want > n) want = n;

  server.sendHeader("Content-Disposition", "attachment; filename=haze_trend.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("seconds_ago,pm25_ugm3,pm1_ugm3,pm10_ugm3,ctrl,demand_pct,wire_pct\n");
  int newest = (head - 1 + cap) % cap;
  String chunk;
  chunk.reserve(1400);
  for (int i = want - 1; i >= 0; i--) {
    int idx = ((newest - i) % cap + cap) % cap;
    long ms = (long)i * perMs;  // 2Hz samples need a decimal, or rows collide
    char row[72];
    snprintf(row, sizeof(row), "%ld.%01ld,%u,%u,%u,%u,%u,%u\n", ms / 1000,
             (ms % 1000) / 100, buf[idx].pm, buf[idx].pm1, buf[idx].pm10,
             buf[idx].ctrl, buf[idx].out, buf[idx].act);
    chunk += row;
    if (chunk.length() > 1200) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("");
}

// Plain settings: a name, where it goes, and the range it must land in. As a
// chain of hasArg/arg/constrain this was 15.7KB of compiled code for one
// function - the largest in the firmware - because every entry built a String,
// compared it, constrained it and destructed it, inlined forty times over. The
// table is the same forty facts with one copy of the machinery.
//
// Anything with a consequence beyond assignment - resetting the integral,
// rescaling a constant, writing NVS, starting a calibration - stays written out
// below, where the consequence is visible next to the cause.
struct IntSetting {
  const char *name;
  int *target;
  int lo, hi;
};
struct FloatSetting {
  const char *name;
  float *target;
  float lo, hi;
};

const IntSetting INT_SETTINGS[] = {
    {"slew", &cfg.slew, 1, 100},
    {"leadfall", &cfg.leadFall, 0, 120},
    {"leadrise", &cfg.leadRise, 0, 120},
    {"dosepct", &cfg.dosePct, 10, 200},
    {"dtau", &cfg.decayTau, 5, 1800},
    {"ti", &cfg.integralTi, 0, 1800},
    {"risecut", &cfg.riseCut, 0, 50},
    {"floor", &cfg.floorPct, 0, 50},
    {"tail", &cfg.machineTail, 0, 15},
    {"tau", &cfg.filterTau, 0, 120},
    {"fan", &cfg.fan, 0, 100},
    {"plevel", &cfg.pulseLevel, 10, 100},
    {"pminon", &cfg.pulseMinOn, 1, 10},
    {"calpulse", &cfg.calPulse, 30, 600},
    {"pperiod", &cfg.pulsePeriod, 5, 60},
};
const FloatSetting FLOAT_SETTINGS[] = {
    {"gain", &cfg.gain, 0.1f, 10.0f},
    {"deadband", &cfg.deadband, 0.0f, 100.0f},
};

void handleSet() {
  if (!authOk()) return;
  if (server.hasArg("automatic")) {
    bool was = cfg.automatic;
    cfg.automatic = server.arg("automatic").toInt();
    if (was != cfg.automatic) logEvent("mode -> %s", cfg.automatic ? "AUTO" : "MANUAL");
  }
  // riseRate is measured per unit of demand, through whatever fraction of that
  // demand pulse mode actually delivers - pminon/(pminon+tail). Change either
  // and the same demand produces a different effect, so a constant measured
  // before the change describes a path that no longer exists.
  //
  // Measured: pminon 2s -> 8s took delivery from 0.35 to 0.66 and the room
  // overshot to 575 against a band topping out at 225, because doseCap was
  // still sizing against a rise rate learned through the old two-thirds loss.
  // The estimator would have caught up in half an hour of over-dosing. The
  // ratio is known exactly at the moment of the change, so apply it then.
  float deliverBefore =
      cfg.pulseMode
          ? cfg.pulseMinOn / (float)(cfg.pulseMinOn + cfg.machineTail)
          : 1.0f;

  // manual is a uint8_t, so it does not fit the int table and is not worth
  // widening the field for.
  if (server.hasArg("manual"))
    cfg.manual = constrain((int)server.arg("manual").toInt(), 0, 100);
  for (const IntSetting &a : INT_SETTINGS)
    if (server.hasArg(a.name))
      *a.target = constrain((int)server.arg(a.name).toInt(), a.lo, a.hi);
  for (const FloatSetting &a : FLOAT_SETTINGS)
    if (server.hasArg(a.name))
      *a.target = constrain(server.arg(a.name).toFloat(), a.lo, a.hi);
  // dtau goes through the table, but setting it by hand is still a claim that
  // somebody measured the room rather than guessed at it.
  if (server.hasArg("dtau")) cfg.decayLearned = true;

  if ((server.hasArg("pminon") || server.hasArg("tail")) && cfg.pulseMode) {
    float after = cfg.pulseMinOn / (float)(cfg.pulseMinOn + cfg.machineTail);
    if (deliverBefore > 0.01f && after > 0.01f && cfg.riseRate > 0 &&
        fabsf(after - deliverBefore) > 0.005f) {
      float was = cfg.riseRate;
      cfg.riseRate =
          constrain(cfg.riseRate * (after / deliverBefore), 0.0f, 500.0f);
      cfg.riseBy[cfg.source] = cfg.riseRate;
      adaptT0 = 0;  // a window spanning the change averages two delivery paths
      logEvent("delivery %.2f -> %.2f of demand, rise %.1f -> %.1f/s",
               (double)deliverBefore, (double)after, (double)was,
               (double)cfg.riseRate);
    }
  }
  if (server.hasArg("source")) {
    int sc = constrain(server.arg("source").toInt(), 0, 2);
    if (sc != cfg.source) {
      // riseRate is in units of the control signal per second, so switching
      // signals leaves it describing a scale that no longer exists and doseCap
      // caps against nothing real - PM1.0 runs about half the 0.3um count, so
      // the cap would be out by 2x until the estimator relearned it over the
      // next half hour. Both readings come from the same sensor sample, so
      // their ratio converts it exactly.
      // Put down what this signal was holding before picking up the next.
      int was = cfg.source;
      cfg.spBy[was] = cfg.setpoint;
      cfg.dbBy[was] = cfg.deadband;
      cfg.riseBy[was] = cfg.riseRate;

      float before = ctrlValue();
      cfg.source = sc;
      float after = ctrlValue();
      bool known = cfg.spBy[sc] > 0;
      bool usable = before > 1.0f && after > 1.0f;
      if (known) {
        // Been here before: give back exactly what was left behind.
        cfg.riseRate = cfg.riseBy[sc];
      } else if (usable && cfg.riseRate > 0) {
        cfg.riseRate = constrain(cfg.riseRate * (after / before), 0.0f, 500.0f);
      }
      // A window spanning the switch would average two different units.
      adaptT0 = 0;
      // The signals share tuning constants but not magnitudes, so a switch
      // starts from that source's own sensible target with nothing carried
      // over. PM1.0 runs about a third of PM2.5 on haze - measured 0.34 here -
      // so its default target is scaled to match rather than copied.
      // Convert the band rather than resetting it. The operator picked a haze
      // level, not a number: the same air is 175 of 0.3um count/100, 205 of
      // PM2.5 and 75 of PM1.0, and only the label changed. Falling back to a
      // hardcoded default threw that choice away and made switching signals
      // cost a retune. Both readings are the same sensor sample, so the ratio
      // converts the band exactly the way it converts the rise rate.
      if (known) {
        cfg.setpoint = cfg.spBy[sc];
        cfg.deadband = cfg.dbBy[sc];
      } else if (usable) {
        // First visit to this signal: carry the intent across by the measured
        // ratio, which is the best guess available, and remember it from here.
        float k = after / before;
        cfg.setpoint = constrain(cfg.setpoint * k, 0.0f, 1000.0f);
        cfg.deadband = constrain(cfg.deadband * k, 0.0f, 100.0f);
      } else {
        cfg.setpoint = sc == 1 ? 20.0f : sc == 2 ? 50.0f : 150.0f;
      }
      cfg.spBy[sc] = cfg.setpoint;
      cfg.dbBy[sc] = cfg.deadband;
      cfg.riseBy[sc] = cfg.riseRate;
      capRelax = 1.0f;  // a different signal is a different model error
      logEvent("control signal -> %s, band %.0f-%.0f, rise %.1f/s (%s)",
               sc == 1 ? "0.3um count/100" : sc == 2 ? "PM1.0" : "PM2.5",
               (double)targetMin(), (double)targetMax(), (double)cfg.riseRate,
               known ? "remembered" : "converted");
      integ = 0;
      pmFiltInit = false;
      pmCount = pmIdx = 0;
      logEvent("control signal -> %s", sc ? "0.3um count/100" : "PM2.5 ug/m3");
    }
  }
  if (server.hasArg("setpoint")) {
    float was = cfg.setpoint;
    cfg.setpoint = constrain(server.arg("setpoint").toFloat(), 0.0f, 1000.0f);
    // A material target change invalidates the accumulated integral: it was
    // charged to hold a different level, and at a several-minute integral time
    // it would otherwise keep commanding the old one long after the change.
    if (fabsf(cfg.setpoint - was) > fmaxf(2.0f, was * 0.1f)) integ = 0;
  }
  // After the raw pair, so a client sending both wins with the band. The page
  // coalesces edits into one request, so min and max can arrive together.
  if (server.hasArg("tgtmin") || server.hasArg("tgtmax")) {
    float lo = server.hasArg("tgtmin") ? server.arg("tgtmin").toFloat() : targetMin();
    float hi = server.hasArg("tgtmax") ? server.arg("tgtmax").toFloat() : targetMax();
    setTargetBand(lo, hi);
  }
  if (server.hasArg("epoch")) {
    time_t e = (time_t)server.arg("epoch").toInt();
    if (e > 1700000000 && time(nullptr) < 1700000000) {
      struct timeval tv = {.tv_sec = e, .tv_usec = 0};
      settimeofday(&tv, nullptr);
      logEvent("clock set from browser");
    }
  }
  if (server.hasArg("room") || server.hasArg("air") || server.hasArg("temp")) {
    if (server.hasArg("room"))
      cfg.roomSize = constrain(server.arg("room").toInt(), 0, 2);
    if (server.hasArg("air"))
      cfg.airMode = constrain(server.arg("air").toInt(), 0, 1);
    if (server.hasArg("temp"))
      cfg.roomTempF = constrain(server.arg("temp").toInt(), 32, 120);
    if (server.hasArg("apply")) {
      // A starting point fills in blanks; it does not overrule a measurement.
      // force=1 is the deliberate override, and says so in the log.
      bool force = server.hasArg("force") && server.arg("force").toInt();
      float bRise = baselineRise(cfg.roomSize);
      int bTau = baselineTau(cfg.roomSize, cfg.airMode, cfg.roomTempF);
      if (force || !cfg.riseLearned) {
        cfg.riseRate = bRise;
        cfg.riseLearned = false;
      } else {
        logEvent("room baseline: kept measured rise %.1f/s over baseline %.1f/s",
                 (double)cfg.riseRate, (double)bRise);
      }
      if (force || !cfg.decayLearned) {
        cfg.decayTau = bTau;
        cfg.decayLearned = false;
      }
      // Seed every signal, not just the one selected. riseBy is per signal, so
      // seeding only the current one left the others holding whatever stale
      // number they were last abandoned with - and switching to one of those
      // started cold on a board that had just been told about the room.
      // Seed signals that have nothing, always. A signal already holding a
      // value keeps it unless this is a forced replacement.
      float here = signalValue(cfg.source);
      for (int i = 0; i < 3; i++) {
        if (cfg.riseBy[i] > 0 && !force && i != cfg.source) continue;
        float there = signalValue(i);
        cfg.riseBy[i] = (here > 1.0f && there > 1.0f)
                            ? cfg.riseRate * (there / here)
                            : cfg.riseRate;
      }
      capRelax = 1.0f;
      adaptT0 = 0;
      logEvent("room baseline: %ld cu ft, %s, %dF -> rise %.1f/s, decay %ds",
               ROOM_FT3[cfg.roomSize], cfg.airMode ? "static" : "air cycling",
               cfg.roomTempF, (double)cfg.riseRate, cfg.decayTau);
    }
  }
  if (server.hasArg("save")) {
    saveAt = 0;  // explicit save: write now rather than on the debounce
    saveCfg();
  }
  if (server.hasArg("stop")) {
    bool was = cfg.stopped;
    cfg.stopped = server.arg("stop").toInt();
    if (was != cfg.stopped) logEvent(cfg.stopped ? "STOP engaged" : "STOP released");
  }
  if (server.hasArg("pulse")) cfg.pulseMode = server.arg("pulse").toInt();
  if (server.hasArg("autopurge")) cfg.autoPurge = server.arg("autopurge").toInt();
  // riseRate could only ever be written by calibration, so a calibration that
  // measured the wrong thing could not be corrected without running another
  // one. It measures the peak of a full-power plume crossing the sensor, which
  // is not what the room does over minutes: measured here, the plume said
  // 16.71/s while the steady state said 5.5/s, and doseCap sized on the former
  // held output at 8% while the room sat 100 below the band for half an hour.
  if (server.hasArg("rise")) {
    cfg.riseRate = constrain(server.arg("rise").toFloat(), 0.0f, 500.0f);
    cfg.riseLearned = true;  // typed in from a measurement, not from a room size
    logEvent("rise rate set to %.2f/s by hand", (double)cfg.riseRate);
  }
  if (server.hasArg("preset")) {
    const Preset &pr = PRESETS[constrain(server.arg("preset").toInt(), 0,
                                         (int)PRESET_COUNT - 1)];
    cfg.filterTau = pr.tau;
    cfg.pulseLevel = pr.plevel;
    cfg.dosePct = pr.dosePct;
    cfg.slew = pr.slew;
    cfg.riseCut = pr.riseCut;
    logEvent("preset %s: tau %d, burst %d%%, dose %d%%, slew %d, risecut %d",
             pr.name, pr.tau, pr.plevel, pr.dosePct, pr.slew, pr.riseCut);
  }
  if (server.hasArg("calibrate")) {
    if (server.arg("calibrate").toInt()) {
      // Calibration drives the hazer, and the stopped branch in loop() resets
      // calState every pass - so starting one while stopped logged "started"
      // and then cancelled it within milliseconds, with nothing said. Refuse
      // out loud instead. STOP is not overridden here on purpose: it is a
      // latch that survives reboot, and a routine that can drive the machine
      // through it is not a stop.
      if (cfg.stopped) {
        snprintf(calMsg, sizeof(calMsg), "release STOP first - calibration drives the hazer");
        logEvent("calibration refused: STOP is engaged");
      } else {
        calState = CAL_PURGE; calT0 = millis(); calMsg[0] = 0;
        logEvent("calibration started, %ds pulse", cfg.calPulse);
      }
    } else {
      calState = CAL_OFF; calMsg[0] = 0; purgeUntil = 0;
    }
  }
  if (server.hasArg("purge")) {
    int secs = constrain(server.arg("purge").toInt(), 0, 600);
    purgeUntil = secs ? millis() + (unsigned long)secs * 1000 : 0;
    logEvent(secs ? "purge started, %ds" : "purge cancelled", secs);
  }
  if (server.hasArg("fixture")) {
    int f = constrain(server.arg("fixture").toInt(), 0, (int)FIXTURE_COUNT - 1);
    if (f != cfg.fixture) {
      cfg.fixture = f;
      logEvent("fixture -> %s", FIXTURES[f].name);
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

  // 8MB of PSRAM is otherwise idle; fall back to a much shorter history in
  // internal RAM if it is not available.
  fineBuf = (Sample *)ps_malloc((size_t)fineCap * sizeof(Sample));
  if (!fineBuf) {
    fineCap = 1200;
    fineBuf = (Sample *)malloc((size_t)fineCap * sizeof(Sample));
  }
  coarseBuf = (Sample *)ps_malloc((size_t)coarseCap * sizeof(Sample));
  if (!coarseBuf) {
    coarseCap = 1440;
    coarseBuf = (Sample *)malloc((size_t)coarseCap * sizeof(Sample));
  }
  Serial.printf("history: fine %d, coarse %d (psram %s)\n", fineCap, coarseCap,
                ESP.getPsramSize() ? "yes" : "no");

  fsOk = LittleFS.begin(true);
  if (fsOk) {
    File f = LittleFS.open(TREND_PATH, "r");
    if (f) {
      size_t recs = f.size() / TREND_REC;
      size_t skip = recs > (size_t)coarseCap ? recs - coarseCap : 0;
      f.seek(skip * TREND_REC);
      uint8_t rec[TREND_REC];
      while (f.read(rec, TREND_REC) == TREND_REC)
        histPush(true, (uint16_t)(rec[0] | (rec[1] << 8)),
                 (uint16_t)(rec[2] | (rec[3] << 8)),
                 (uint16_t)(rec[4] | (rec[5] << 8)),
                 (uint16_t)(rec[6] | (rec[7] << 8)), rec[8], rec[9]);
      f.close();
    }
    // Carry older files across rather than dropping them. Their missing
    // columns read 0, which the chart draws as a gap rather than a value -
    // throwing away days of trend to add a column is a bad trade.
    if (!coarseN) importOld(TREND_PATH_V2, 6);
    if (!coarseN) importOld(TREND_PATH_V1, 4);
    LittleFS.remove(TREND_PATH_V2);
    LittleFS.remove(TREND_PATH_V1);
    Serial.printf("trend restored from flash: %d minutes\n", coarseN);
  } else {
    Serial.println("LittleFS mount failed - trend will not survive a reboot");
  }

  prefs.begin("haze", false);
  loadCfg();
  bootId = prefs.getUInt("boot", 0) + 1;  // must follow prefs.begin()
  pkAll = prefs.getUShort("pkall", 0);
  pkAllBoot = prefs.getUInt("pkallb", 0);
  pkAllAt = prefs.getUInt("pkallat", 0);
  uiGen = prefs.getUInt("uigen", 0);
  // After loadCfg, since it needs nothing from the config, but before the loop
  // starts so the first control tick already has it.
  restoreInteg();
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
  // Regulation still runs headless without wifi, but loop() keeps retrying:
  // giving up permanently at boot means one missed association leaves the board
  // unreachable with no way back except a power cycle, which is exactly how an
  // over-the-air update stranded it.
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nhttp://%s/  (or http://haze.local/)\n",
                  WiFi.localIP().toString().c_str());
    MDNS.begin("haze");
    assocAt = millis();  // first association, not only reconnections
  } else {
    Serial.println("\nno wifi - regulating headless, will keep retrying");
    wifiLostAt = millis();
  }

  // A UI-only change - a label, a colour, a chart tweak - used to cost a 1.1MB
  // flash, a reboot, a reset integral and a restarted adaptation window, for
  // bytes the controller never executes. Uploading the page to the filesystem
  // costs 20KB and nothing else. The compiled copy stays as the fallback, so a
  // truncated or broken upload can never leave the board without an interface:
  // delete the file and the built-in page is back.
  server.on("/ui", HTTP_POST,
            []() {
              if (!authOk()) return;
              bool ok = uiUploadOk && uiFromFs();
              server.send(200, "text/plain", ok ? "OK" : "FAILED");
              logEvent(ok ? "web ui replaced from upload"
                          : "web ui upload failed, built-in page still serving");
            },
            []() {
              if (!server.authenticate(WEB_USER, WEB_PASS)) return;
              HTTPUpload &up = server.upload();
              if (up.status == UPLOAD_FILE_START) {
                uiUploadOk = false;
                uiTmp = LittleFS.open(UI_TMP, "w");
              } else if (up.status == UPLOAD_FILE_WRITE) {
                if (uiTmp) uiTmp.write(up.buf, up.currentSize);
              } else if (up.status == UPLOAD_FILE_END) {
                if (uiTmp) {
                  uiTmp.close();
                  // Swap in only once the whole thing has arrived, so a dropped
                  // connection leaves the previous page untouched rather than
                  // half of the new one.
                  // Keep the page that was working before replacing it. The
                  // recovery page can only put something back if something was
                  // kept, and the moment a replacement is about to happen is
                  // the only moment the old one is still known good.
                  LittleFS.remove(UI_BAK);
                  if (LittleFS.exists(UI_PATH)) LittleFS.rename(UI_PATH, UI_BAK);
                  uiUploadOk = LittleFS.rename(UI_TMP, UI_PATH);
                  if (uiUploadOk) {
                    uiGen++;
                    prefs.putUInt("uigen", uiGen);
                  }
                }
              } else if (up.status == UPLOAD_FILE_ABORTED) {
                if (uiTmp) uiTmp.close();
                LittleFS.remove(UI_TMP);
              }
            });
  server.on("/ui/restore", []() {
    if (!authOk()) return;
    if (!LittleFS.exists(UI_BAK)) {
      server.send(200, "text/plain", "no backup on the filesystem");
      return;
    }
    LittleFS.remove(UI_PATH);
    bool ok = LittleFS.rename(UI_BAK, UI_PATH);
    if (ok) {
      uiGen++;
      prefs.putUInt("uigen", uiGen);
    }
    server.send(200, "text/plain", ok ? "restored" : "restore failed");
    logEvent(ok ? "web ui restored from the backup copy"
                : "web ui restore from backup failed");
  });
  server.on("/ui/reset", []() {
    if (!authOk()) return;
    LittleFS.remove(UI_PATH);
    uiGen++;
    prefs.putUInt("uigen", uiGen);
    server.send(200, "text/plain", "reverted to built-in ui");
    logEvent("web ui reverted to the built-in page");
  });
  server.on("/", []() {
    // The page carries no version in its URL and changes whenever ui.sh runs,
    // so a browser must never keep it. With no Cache-Control, ETag or
    // Last-Modified at all, browsers fall back to heuristic caching and were
    // serving the old page - old title included - while the board served the
    // new one. no-store rather than no-cache: without an ETag there is nothing
    // to revalidate against, so a conditional request would refetch anyway.
    server.sendHeader("Cache-Control", "no-store");
    if (!authOk()) return;
    if (uiFromFs()) {
      File f = LittleFS.open(UI_PATH, "r");
      if (f) {
        server.streamFile(f, "text/html");
        f.close();
        return;
      }
    }
    server.send_P(200, "text/html", PAGE);
  });
  server.on("/api/state", handleState);
  server.on("/api/history", handleHistory);
  server.on("/api/csv", handleCsv);
  server.on("/api/events", handleEvents);

  // Firmware update over wifi. The board lives on a DMX rig, not a desk, so
  // needing USB for every change stops being practical quickly. No auth: this
  // is a LAN appliance, and anyone who can reach the control page can already
  // drive the hazer. Do not expose it beyond the local network.
  //
  // An interrupted upload used to wedge the updater until the next reboot:
  // Update.begin() was called on every UPLOAD_FILE_START and nothing ever
  // aborted a run that stopped partway, so once one upload died every later
  // begin() returned false and every flash answered FAILED. Six uploads cut off
  // by a tunnel timeout left the board unflashable with no remote way out. Now
  // a stale run is aborted before starting a new one, an aborted body aborts
  // the run, and begin() failures are logged rather than discovered at the end.
  server.on(
      "/update", HTTP_POST,
      []() {
        if (!authOk()) return;
        bool ok = !Update.hasError();
        server.sendHeader("Connection", "close");
        server.send(200, "text/plain", ok ? "OK - rebooting" : "FAILED");
        if (ok) {
          logEvent("firmware updated over wifi, rebooting");
          saveInteg();
          delay(300);
          ESP.restart();
        }
      },
      []() {
        // Runs while the body streams in, before the handler above. Without
        // this check the flash write happens and only the response is refused.
        if (!server.authenticate(WEB_USER, WEB_PASS)) return;
        HTTPUpload &up = server.upload();
        if (up.status == UPLOAD_FILE_START) {
          if (Update.isRunning()) {
            Update.abort();
            logEvent("aborted a stale firmware upload before starting");
          }
          logEvent("firmware upload started: %s", up.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
            logEvent("update begin failed: %s", Update.errorString());
        } else if (up.status == UPLOAD_FILE_WRITE) {
          Update.write(up.buf, up.currentSize);
        } else if (up.status == UPLOAD_FILE_END) {
          if (!Update.end(true))
            logEvent("firmware update failed: %s", Update.errorString());
        } else if (up.status == UPLOAD_FILE_ABORTED) {
          Update.abort();
          logEvent("firmware upload aborted after %u bytes", up.totalSize);
        }
      });
  // Bytes handed to Update so far in this chunked run. Not Update.progress():
  // that counts what has reached flash, and the library holds up to a 4096
  // byte sector in its own buffer, so after a 131072 byte chunk it reports
  // 126976 and every chunk after the first looks out of order.
  static size_t otaChunkOff = 0;

  // Chunked OTA, for flashing across the Cloudflare tunnel. The edge gives an
  // origin 100 seconds to finish a request, and a 1.1MB image over wifi at
  // -76dBm does not make it - six attempts in a row returned 502 with the
  // upload still in flight. Each chunk is its own request and its own budget,
  // so the link speed stops mattering. off must equal what has already been
  // written: a chunk arriving out of order would corrupt the image silently,
  // and that is the one failure this must never produce.
  server.on(
      "/update/chunk", HTTP_POST,
      []() {
        if (!authOk()) return;
        server.sendHeader("Connection", "close");
        if (Update.hasError()) {
          String e = String("FAILED ") + Update.errorString();
          Update.abort();
          server.send(200, "text/plain", e);
          return;
        }
        if (server.arg("last") != "1") {
          server.send(200, "text/plain", String("OK ") + Update.progress());
          return;
        }
        if (Update.end(true)) {
          server.send(200, "text/plain", "OK - rebooting");
          logEvent("firmware updated over wifi (chunked), rebooting");
          saveInteg();
          delay(300);
          ESP.restart();
        } else {
          logEvent("chunked update failed: %s", Update.errorString());
          server.send(200, "text/plain", "FAILED");
        }
      },
      []() {
        if (!server.authenticate(WEB_USER, WEB_PASS)) return;
        HTTPUpload &up = server.upload();
        size_t off = (size_t)server.arg("off").toInt();
        if (up.status == UPLOAD_FILE_START) {
          if (off == 0) {
            if (Update.isRunning()) Update.abort();
            size_t total = (size_t)server.arg("total").toInt();
            if (!Update.begin(total ? total : UPDATE_SIZE_UNKNOWN)) {
              logEvent("chunked begin failed: %s", Update.errorString());
            } else {
              otaChunkOff = 0;
              logEvent("chunked upload started, %u bytes", (unsigned)total);
            }
          } else if (!Update.isRunning() || otaChunkOff != off) {
            logEvent("chunk at %u rejected, have %u", (unsigned)off,
                     (unsigned)otaChunkOff);
            Update.abort();
          }
        } else if (up.status == UPLOAD_FILE_WRITE) {
          if (Update.isRunning()) {
            size_t w = Update.write(up.buf, up.currentSize);
            otaChunkOff += w;
            // A short write means the image is already wrong; say so here
            // rather than letting end() report a vague checksum failure.
            if (w != up.currentSize)
              logEvent("short write at %u: %u of %u", (unsigned)otaChunkOff,
                       (unsigned)w, (unsigned)up.currentSize);
          }
        } else if (up.status == UPLOAD_FILE_ABORTED) {
          Update.abort();
          logEvent("chunked upload aborted at %u", (unsigned)otaChunkOff);
          // The run is dead, so the count belongs to nothing. Leaving it set
          // made the next chunk look out of order against an offset no longer
          // attached to an update, and the push retried that chunk forever.
          otaChunkOff = 0;
        }
      });
  // Where the board thinks it is, so a push whose reply was lost can work out
  // whether the chunk landed and resume instead of starting the megabyte again.
  // A flaky link is the normal case here, not the exception.
  server.on("/update/chunk", HTTP_GET, []() {
    if (!authOk()) return;
    server.send(200, "text/plain",
                String(otaChunkOff) + " " + (Update.isRunning() ? "1" : "0"));
  });
  server.on("/api/reboot", []() {
    if (!authOk()) return;
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "rebooting");
    logEvent("reboot requested over http");
    saveInteg();
    delay(300);
    ESP.restart();
  });
  server.on("/api/set", handleSet);
  server.begin();
  prefs.putUInt("boot", bootId);
  // Everything worth seeing at boot goes to the event log as well as Serial:
  // with no USB attached the serial monitor is unreachable, so the event log has
  // to be the console.
  logEvent("boot: %s addr %d, trend %d min, ip %s", FIXTURES[cfg.fixture].name,
           cfg.dmxAddress, coarseN, WiFi.localIP().toString().c_str());
  if (checkFails) logEvent("SELFTEST: %d check(s) failed", checkFails);
  logEvent("  sensor %s, psram %s, history %d/%d, fs %s",
           sensorOk ? "ready" : "NOT RESPONDING at 0x12",
           ESP.getPsramSize() ? "yes" : "no", fineCap, coarseCap,
           fsOk ? "ok" : "MOUNT FAILED");
  logEvent("  dmx on GPIO%d, DE GPIO%d, wifi %s rssi %d", DMX_TX_PIN, DMX_EN_PIN,
           WiFi.status() == WL_CONNECTED ? WiFi.SSID().c_str() : "DISCONNECTED",
           (int)WiFi.RSSI());
}

void loop() {
  {
    // handleClient returns immediately with no client, so only time the passes
    // where it did something - otherwise the average is diluted to nothing by
    // thousands of idle calls.
    unsigned long t0 = millis();
    server.handleClient();
    unsigned long took = millis() - t0;
    if (took > 1) {
      servedReqs++;
      reqMsTotal += took;
      if (took > slowestReqMs) slowestReqMs = took;
      // A healthy board serves the whole page in about 265ms. Degraded, the
      // same request took 30 seconds and returned a quarter of it. Anything
      // past 8s is not slow, it is broken, and two in a row rules out one
      // unlucky retransmit.
      if (took > 8000) {
        if (slowReqs < 255) slowReqs++;
      } else {
        slowReqs = 0;
      }
    }
  }
  loopTicks++;
  unsigned long now = millis();

  if (now - healthAt >= 1000) {
    loopRate = loopTicks * 1000 / (now - healthAt);
    loopTicks = 0;
    healthAt = now;

    // A collapsed loop rate is this morning's fault: something blocking long
    // enough that the web server is barely serviced. Deliberately a low bar -
    // a healthy board runs thousands of passes a second, so anything under 100
    // is already two orders of magnitude wrong - and it must persist, because
    // a flash erase or a filesystem trim briefly does this on purpose.
    if (loopRate < 100 && WiFi.status() == WL_CONNECTED) {
      if (!starvedSince) starvedSince = now;
      if (now - starvedSince > 120000) {
        logEvent("WATCHDOG: %lu loop passes/s for 120s, restarting",
                 (unsigned long)loopRate);
        saveInteg();
        delay(200);
        ESP.restart();
      }
    } else {
      starvedSince = 0;
    }

    // Tonight's fault is different and a reboot does not fix it: the radio, not
    // the firmware. Reassociating can land on a better rate or a nearer AP, and
    // costs a few seconds of web UI rather than the whole control loop, so it
    // is tried long before anything more drastic.
    // Measured tonight: at -81dBm the page served 71 bytes a second; a reboot
    // reassociated at -70 and it served 102KB/s. The reboot was never the cure,
    // the reassociation was - so do that directly, and trigger on responses
    // actually crawling rather than on RSSI, which read -81 while a threshold
    // of -85 would have sat there watching.
    if (WiFi.status() == WL_CONNECTED &&
        (slowReqs >= 2 || WiFi.RSSI() < -85) &&
        (!reassocAt || now - reassocAt > 600000)) {
      reassocAt = now;
      logEvent("WATCHDOG: %u slow responses, rssi %d - reassociating",
               (unsigned)slowReqs, (int)WiFi.RSSI());
      reassocCount++;
      slowReqs = 0;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

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
  static bool lastOk = true;
  if (everRead && sensorOk != lastOk) {
    logEvent(sensorOk ? "sensor recovered" : "SENSOR LOST");
    lastOk = sensorOk;
  }

  if (now - lastControl >= 1000) {
    lastControl = now;

    // Trend and prediction are readouts as much as control inputs, so they must
    // update in every mode. Computing them only on the automatic path left both
    // stuck at zero in manual, purge and calibration.
    if (everRead && sensorOk) {
      // Low-pass the control input. The sensor sees plume turbulence, not room
      // average, and differentiating that raw signal produces noise, not trend.
      float raw = ctrlValue();
      if (cfg.filterTau <= 0 || !pmFiltInit) pmFilt = raw;
      else pmFilt += (raw - pmFilt) / (cfg.filterTau + 1.0f);
      pmFiltInit = true;

      uint16_t rawPm = data.pm25_env;  // peaks stay in ug/m3, whatever we regulate on
      if (rawPm > pkSes) { pkSes = rawPm; pkSesAt = millis() / 1000; }
      if (rawPm > pkAll) {
        pkAll = rawPm;
        pkAllBoot = bootId;
        pkAllAt = millis() / 1000;
        prefs.putUShort("pkall", pkAll);
        prefs.putUInt("pkallb", pkAllBoot);
        prefs.putUInt("pkallat", pkAllAt);
        if (now - pkLogged > 60000) {  // rate limited: a rising peak is noisy
          logEvent("new all-time peak %u ug/m3", pkAll);
          pkLogged = now;
        }
      }
      pmHist[pmIdx] = (uint16_t)lroundf(pmFilt);
      pmIdx = (pmIdx + 1) % SLOPE_WIN;
      if (pmCount < SLOPE_WIN) pmCount++;
      pmSlope = slopeFit(pmHist, pmIdx, pmCount, SLOPE_WIN);
      predicted = predict(pmFilt, pmSlope,
                          leadFor(pmSlope, cfg.leadFall, cfg.leadRise));
      predicted = controlPm(predicted, pmFilt, cfg.setpoint, cfg.deadband);

      // Only learn from ordinary regulating. A purge, a calibration pulse, a
      // stopped output or a pegged sensor are all the loop not being in charge
      // of what the room is doing, and a window containing one of them would
      // teach it something false.
      if (!cfg.stopped && cfg.automatic && !purging() && !calibrating() &&
          !sensorSaturated()) {
        // output, not hazeLevel(). doseCap applies riseRate to demand, and
        // pulsing makes demand and on-wire differ by about 3x - so feeding the
        // wire value estimated a constant in the wrong units and doseCap then
        // under-dosed by that factor. Measured here: 55/s per wire, 17/s per
        // demand, and 17 is what calibration's full-power pulse measured too,
        // because during calibration pulsing is bypassed and the two are equal.
        adaptTick(now, pmFilt, output);
        decayTick(now, pmFilt, output, hazeLevel());
      } else {
        adaptReset(now, pmFilt);
        decayReset();
      }
    }
    // A pegged sensor cannot report a trend, so the regulator is flying blind
    // and any output it commands is guesswork. Clear the air instead. Not during
    // calibration, which runs its own purge phase and aborts near the ceiling.
    if (cfg.autoPurge && !cfg.stopped && !calibrating() && everRead && sensorOk) {
      if (sensorSaturated() && !purging()) {
        // Start short. If it is still pegged after clearing, the room needs
        // more than a nudge, so each successive attempt doubles.
        purgeUntil = now + satPurgeMs;
        logEvent("%s saturated (%u), auto-purge %lus",
                 cfg.source ? "count" : "pm2.5",
                 cfg.source ? data.particles_03um : data.pm25_env,
                 satPurgeMs / 1000);
        satPurgeMs = min(satPurgeMs * 2, (unsigned long)SAT_PURGE_MAX_MS);
      } else if (!sensorSaturated()) {
        satPurgeMs = SAT_PURGE_MIN_MS;  // clear of the ceiling: reset escalation
      }
    }

    // Anything that overrides the loop invalidates the accumulated integral.
    if (cfg.stopped || !cfg.automatic || purging() || calibrating() || riseLock)
      integ = 0;

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
      target = computePI(cfg.setpoint, predicted, cfg.deadband, cfg.gain,
                         cfg.integralTi, integ);
      // Insurance against a wrong trend estimate: whatever the prediction says,
      // never command nothing while the sensor reports the room well below
      // target. Being wrong downward means an empty room mid-show.
      if (pmFilt < cfg.setpoint - 3 * cfg.deadband && target < cfg.floorPct)
        target = cfg.floorPct;
      bool wasLock = riseLock;
      // Applied after the floor: an anti-starvation minimum must not be able
      // to overshoot a target that is already nearly reached.
      uint8_t cap =
          doseCap(deficitAtArrival(cfg.setpoint, pmFilt, pmSlope, cfg.deadTime) +
                      holdLoss(pmFilt, cfg.decayTau, cfg.deadTime),
                  cfg.riseRate, cfg.deadTime, cfg.dosePct);
      // Once a minute, ask whether the cap is standing between the loop and its
      // target. Rising counts as working even if it has not arrived yet.
      if (now - relaxAt > 60000) {
        bool starved = target > cap && pmFilt < targetMin() &&
                       pmFilt <= relaxRef + 1.0f;
        if (relaxAt && starved && capRelax < CAP_RELAX_MAX) {
          capRelax = min(CAP_RELAX_MAX, capRelax * 1.15f);
          if (now - relaxLogged > 300000) {
            relaxLogged = now;
            logEvent("cap relaxed to %.2fx: capped at %u%% with %.0f below the "
                     "band and not rising", (double)capRelax, cap,
                     (double)(targetMin() - pmFilt));
          }
        } else if (pmFilt >= targetMin() && capRelax > 1.0f) {
          // Reached the band, so the model is no longer demonstrably wrong.
          capRelax = max(1.0f, capRelax * 0.9f);
        }
        relaxRef = pmFilt;
        relaxAt = now;
      }
      if (capRelax > 1.0f) {
        long widened = lroundf(cap * capRelax);
        cap = widened > 100 ? 100 : (uint8_t)widened;
      }
      if (target > cap) {
        // Only a clip big enough to matter, held long enough to not be a
        // transient, and at most once every five minutes.
        if (target - cap >= 15) {
          if (!capSince) capSince = now;
          if (now - capSince > 60000 && now - capLogged > 300000) {
            capLogged = now;
            logEvent("dose cap: controller wants %u%%, cap allows %u%% for %lus "
                     "(gap %.0f, leak %.0f over %ds dead, rise %.1f/s)",
                     target, cap, (unsigned long)((now - capSince) / 1000),
                     (double)deficitAtArrival(cfg.setpoint, pmFilt, pmSlope,
                                              cfg.deadTime),
                     (double)holdLoss(pmFilt, cfg.decayTau, cfg.deadTime),
                     cfg.deadTime, (double)cfg.riseRate);
          }
        } else {
          capSince = 0;
        }
        target = cap;
      } else {
        capSince = 0;
      }

      riseLock = riseLockNext(riseLock, pmSlope, cfg.riseCut);
      if (riseLock != wasLock)
        logEvent(riseLock ? "rise lock on, climbing %+.1f/s" : "rise lock off",
                 (double)pmSlope);
      if (riseLock) target = 0;  // beats the floor: do not feed a rising room
    }
    if (!purging() && !cfg.stopped && !calibrating()) {
      // Cutting is always the safe direction, so skip the slew on a rise lock.
      if (riseLock) output = 0;
      else output = applySlew(output, target, cfg.slew);
    }
    if (output > pkOut) pkOut = output;

    // Something holding output at zero while the room is below target is the
    // failure that is hardest to spot: everything looks healthy, the loop just
    // never acts. Whatever the cause, say so by name rather than leaving it to
    // be inferred from a flat chart.
    bool shouldHaze = cfg.automatic && !cfg.stopped && everRead && sensorOk &&
                      pmFilt < cfg.setpoint - cfg.deadband;
    if (shouldHaze && output == 0) {
      if (!zeroSince) zeroSince = now;
      if (now - zeroSince > 60000 && now - zeroLogged > 300000) {
        zeroLogged = now;
        const char *why = purging()          ? "purge active"
                          : riseLock         ? "rise lock"
                          : calibrating()    ? "calibrating"
                          : sensorSaturated()? "sensor saturated"
                                             : "controller commanding zero";
        logEvent("output held at 0%% for %lus, %.1f below target: %s",
                 (unsigned long)((now - zeroSince) / 1000),
                 (double)(cfg.setpoint - pmFilt), why);
      }
    } else {
      zeroSince = 0;
    }

    // "Commanding haze but nothing is happening" - out of fluid, heater
    // reheating, thermal cutout, DMX unplugged, wrong address. It cannot tell
    // which, but knowing it is happening is the part that matters mid-show.
    // Saturation is excluded: a pegged sensor cannot show a rise either way.
    bool judging = output >= NORESP_PCT && !cfg.stopped && !purging() &&
                   !calibrating() && everRead && sensorOk && !sensorSaturated();
    if (!judging) {
      outSince = 0;
      if (noResponse && output < NORESP_PCT) noResponse = false;
    } else if (!outSince) {
      outSince = now;
      pmAtOutStart = ctrlValue();
    } else if (now - outSince >= (unsigned long)NORESP_WIN_S * 1000) {
      float moved = ctrlValue() - pmAtOutStart;
      // Against what doing nothing would have done, not against zero. The room
      // leaks level/decayTau the whole time: at 150 with tau 61 that is 2.4/s,
      // so about 220 units drain across this window. Requiring an absolute rise
      // called the machine dead whenever the room could empty faster than it
      // filled - which is most of the time here, with the tail cancelling two
      // thirds of every dose. A level falling more slowly than pure decay is
      // the machine contributing, even while it loses.
      float decayOnly =
          cfg.decayTau > 0
              ? pmAtOutStart * (expf(-(float)NORESP_WIN_S / cfg.decayTau) - 1.0f)
              : 0.0f;
      bool responding = moved > decayOnly + NORESP_RISE;
      if (!responding) {
        if (!noResponse)
          logEvent("NO RESPONSE: %ds at %u%%, moved %+.1f, decay alone %+.1f",
                   NORESP_WIN_S, output, (double)moved, (double)decayOnly);
        noResponse = true;
      } else if (noResponse) {
        logEvent("machine responding again (moved %+.1f vs decay %+.1f)",
                 (double)moved, (double)decayOnly);
        noResponse = false;
      }
      outSince = now;
      pmAtOutStart = ctrlValue();  // was pm25_env, which is not the signal
                                   // being compared when regulating anything
                                   // else - the first window after a source
                                   // switch judged one signal against another
    }
  }

  // Unconditional: frames keep going out at zero as well, so a receiver never
  // sees signal loss just because the haze is off.
  // Reconnect if wifi drops or never came up, retrying every 10s. Without this
  // a single failed association at boot leaves the board headless until someone
  // power-cycles it.
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiLostAt) wifiLostAt = now;
    if (now - wifiTry > 10000) {
      wifiTry = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    // Still nothing after ten minutes: the radio or stack is wedged, and a
    // restart is the only remaining move. Regulation resumes on the way back.
    if (now - wifiLostAt > 600000) {
      logEvent("wifi down 10 min, restarting");
      delay(100);
      ESP.restart();
    }
  } else if (wifiLostAt) {
    wifiLostAt = 0;
    MDNS.end();
    MDNS.begin("haze");
    assocAt = millis();
    logEvent("wifi reconnected, ip %s rssi %d",
             WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  }

  updateLed();

  if (now - histLast >= HIST_FINE_MS) {
    histLast = now;
    uint16_t pm = everRead ? data.pm25_env : 0;
    // Rounded, not truncated: at source=1 this is a count/100, so a unit here
    // is 100 particles and dropping the fraction biases the whole trend low.
    uint16_t ctrl = (uint16_t)lroundf(ctrlValue());
    uint16_t pm1 = everRead ? data.pm10_env : 0;   // PM1.0
    uint16_t pmT = everRead ? data.pm100_env : 0;  // PM10
    uint8_t act = hazeLevel();
    histPush(false, pm, ctrl, pm1, pmT, output, act);
    cAccPm += pm;
    cAccCtrl += ctrl;
    cAccPm1 += pm1;
    cAccPm10 += pmT;
    cAccOut += output;
    cAccAct += act;
    cAccN++;
    if (now - coarseLast >= HIST_COARSE_MS && cAccN) {
      coarseLast = now;
      uint16_t cpm = cAccPm / cAccN, cctrl = cAccCtrl / cAccN;
      uint8_t co = cAccOut / cAccN, ca = cAccAct / cAccN;
      histPush(true, cpm, cctrl, cAccPm1 / cAccN, cAccPm10 / cAccN, co, ca);
      if (coarsePostBoot < coarseCap) coarsePostBoot++;
      trendAppend(cpm, cctrl, cAccPm1 / cAccN, cAccPm10 / cAccN, co, ca);
      cAccPm = cAccCtrl = cAccOut = cAccAct = 0;
      cAccPm1 = cAccPm10 = 0;
      cAccN = 0;
    }
  }

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
