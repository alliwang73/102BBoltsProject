// =====================================================================
//  BoltSorter.ino  -  1/4-20 bolt thread inspection + sorting machine
//
//  Board : Adafruit Feather ESP32 V2   (ESP32 Arduino core 3.x)
//  Tools > Partition Scheme : "Default (3MB APP/1.5MB SPIFFS)"
//          (Wi-Fi + Bluetooth do not fit in the 1.2MB default)
//  Libraries : none beyond the ESP32 core
//
//  Actuators (all in this one sketch):
//    feeder  = hobby servo, flips the bolt 180 deg so it drops head-first
//    tube    = Pololu #2215 + encoder, spins the inspection tube slowly
//    flap    = Pololu #2215 + encoder, opens to let the bolt out of the tube
//    hatch   = Pololu #2215 + encoder, steers the bolt to the GOOD or BAD bin
//              (optional endpoint limit switches)
//
//  Cycle:  WAIT_BOLT -> FEED -> SETTLE -> INSPECT (tube turns, camera looks)
//          -> AWAIT_VERDICT -> SORT (hatch) -> RELEASE (flap open) -> CLOSE
//
//  Control from any of:
//    Wi-Fi  : join "BoltSorter" (pw bolts1420) -> http://192.168.4.1
//    BLE    : bolt-control / bolt-inspector pages, "Connect Bluetooth"
//    Serial : 115200 baud, same text commands (type "help")
//  The on-board button (SW38) toggles Run/Stop, or clears a fault.
//
//  The control page embedded below is generated from
//  bolt-control/index.html by tools/embed_control_page.py. Edit the HTML
//  file, then re-run the script.
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <freertos/stream_buffer.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define ENABLE_BLE 1          // 0 = Wi-Fi + Serial only (smaller, faster boot)
#if ENABLE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#endif

// ---------------- NETWORK ----------------
const char *AP_SSID   = "BoltSorter";
const char *AP_PASS   = "bolts1420";     // 8+ characters
// Optional: also join an existing network (phone hotspot / lab router) so
// every device on that network can reach http://boltsorter.local
const char *STA_SSID  = "";
const char *STA_PASS  = "";
const char *HOST_NAME = "boltsorter";
const IPAddress AP_IP(192, 168, 4, 1);

// ---------------- PINS (Feather ESP32 V2 silkscreen in brackets) ----------------
// Two DRV8833 dual drivers. Every motor uses two PWM inputs (IN/IN mode).
constexpr int TUBE_IN1 = 26, TUBE_IN2 = 25;   // [A0] [A1]  -> DRV8833 #1 AIN1/AIN2
constexpr int TUBE_ENC_A = 27, TUBE_ENC_B = 33;// [27] [33]
constexpr int FLAP_IN1 = 4, FLAP_IN2 = 13;     // [A5] [13]  -> DRV8833 #1 BIN1/BIN2 (13 = red LED, flickers when flap drives)
constexpr int FLAP_ENC_A = 15, FLAP_ENC_B = 20;// [15] [SCL]
constexpr int HATCH_IN1 = 19, HATCH_IN2 = 21;  // [MO] [MI]  -> DRV8833 #2 AIN1/AIN2
constexpr int HATCH_ENC_A = 7, HATCH_ENC_B = 8;// [RX] [TX]
constexpr int SERVO_PIN = 32;                  // [32]  feeder servo signal
constexpr int LIMIT_GOOD_PIN = 14;             // [14]  optional, switch to GND
constexpr int LIMIT_BAD_PIN  = 22;             // [SDA] optional, switch to GND
constexpr int BOLT_SENSOR_PIN = 5;             // [SCK] optional, active LOW (switch / IR beam to GND)
constexpr int BUTTON_PIN = 38;                 // on-board SW38 (active LOW)

constexpr int MOTOR_PWM_HZ = 20000, MOTOR_PWM_BITS = 8;
constexpr int SERVO_HZ = 50, SERVO_BITS = 14;

// ---------------- FIXED CONTROL CONSTANTS ----------------
constexpr float    CTRL_DT      = 0.002f;  // 500 Hz control task
constexpr float    I_LIMIT      = 80.0f;   // max PWM from the integral term
constexpr float    SETTLE_VEL   = 8.0f;    // deg/s
constexpr uint32_t SETTLE_MS    = 150;
constexpr uint32_t STALL_MS     = 800;     // driven hard, encoder silent
constexpr uint32_t WRONGWAY_MS  = 250;
constexpr uint8_t  DEBOUNCE_TICKS = 10;    // 10 x 2 ms

// =====================================================================
//  SETTINGS  (editable live, saved to flash with "save")
// =====================================================================
struct AxisCfg {
  float cpr;     // encoder counts per OUTPUT-shaft revolution
  float kp;      // PWM per deg of error
  float ki;      // PWM per deg*s
  float kd;      // PWM per deg/s of velocity error
  float ff;      // friction PWM
  float vmax;    // deg/s
  float acc;     // deg/s^2
  float tol;     // arrival tolerance, deg
  float maxpwm;  // 0..255
  float inv;     // 1 = reverse motor direction
  float einv;    // 1 = reverse encoder direction
};

enum AxisId { TUBE = 0, FLAP = 1, HATCH = 2, NUM_AX = 3 };

struct Config {
  AxisCfg ax[NUM_AX];
  float ispd, ideg;            // inspection: tube speed deg/s, total turn deg
  float settle;                // ms after feeding before the tube turns
  float vto, vfb;              // verdict timeout ms; fallback 0=BAD 1=GOOD 2=wait 3=fault
  float fopen, fclose, fdwell; // flap angles (deg) and open dwell (ms)
  float hgood, hbad, hlim;     // hatch angles (deg); 1 = use limit switches
  float svload, svdrop, svspd, svhold, svret, svmin, svmax; // feeder servo
  float bsens, gap, mto;       // 1 = wait for bolt sensor; ms between auto feeds; move timeout ms
};
constexpr uint32_t CFG_VERSION = 3;

Config cfg;

void setDefaults() {
  const AxisCfg d = {12.0f * 75.81f, 8.0f, 25.0f, 0.15f, 45.0f, 360.0f, 1500.0f, 0.5f, 230.0f, 0, 0};
  for (int i = 0; i < NUM_AX; i++) cfg.ax[i] = d;
  cfg.ax[TUBE].tol = 1.0f;
  cfg.ispd = 40;   cfg.ideg = 360;  cfg.settle = 800;
  cfg.vto = 5000;  cfg.vfb = 0;
  cfg.fopen = 90;  cfg.fclose = 0;  cfg.fdwell = 600;
  cfg.hgood = 45;  cfg.hbad = -45;  cfg.hlim = 0;
  cfg.svload = 0;  cfg.svdrop = 180; cfg.svspd = 133; cfg.svhold = 700; cfg.svret = 0;
  cfg.svmin = 500; cfg.svmax = 2400;
  cfg.bsens = 0;   cfg.gap = 1500;  cfg.mto = 4000;
}

struct Setting { const char *k; float *v; float lo, hi; };
#define AXSET(P, i) \
  {P "cpr", &cfg.ax[i].cpr, 10, 100000}, {P "kp", &cfg.ax[i].kp, 0, 500}, \
  {P "ki", &cfg.ax[i].ki, 0, 500},       {P "kd", &cfg.ax[i].kd, 0, 50}, \
  {P "ff", &cfg.ax[i].ff, 0, 200},       {P "vmax", &cfg.ax[i].vmax, 5, 3000}, \
  {P "acc", &cfg.ax[i].acc, 10, 50000},  {P "tol", &cfg.ax[i].tol, 0.1f, 20}, \
  {P "maxpwm", &cfg.ax[i].maxpwm, 30, 255}, {P "inv", &cfg.ax[i].inv, 0, 1}, \
  {P "einv", &cfg.ax[i].einv, 0, 1}

Setting SETTINGS[] = {
  AXSET("t_", TUBE), AXSET("f_", FLAP), AXSET("h_", HATCH),
  {"ispd", &cfg.ispd, 1, 720},       {"ideg", &cfg.ideg, 10, 3600},
  {"settle", &cfg.settle, 0, 10000}, {"vto", &cfg.vto, 500, 60000}, {"vfb", &cfg.vfb, 0, 3},
  {"fopen", &cfg.fopen, -720, 720},  {"fclose", &cfg.fclose, -720, 720}, {"fdwell", &cfg.fdwell, 0, 10000},
  {"hgood", &cfg.hgood, -720, 720},  {"hbad", &cfg.hbad, -720, 720},     {"hlim", &cfg.hlim, 0, 1},
  {"svload", &cfg.svload, 0, 180},   {"svdrop", &cfg.svdrop, 0, 180},   {"svspd", &cfg.svspd, 5, 1000},
  {"svhold", &cfg.svhold, 0, 10000}, {"svret", &cfg.svret, 0, 1},
  {"svmin", &cfg.svmin, 400, 1500},  {"svmax", &cfg.svmax, 1500, 2600},
  {"bsens", &cfg.bsens, 0, 1},       {"gap", &cfg.gap, 0, 60000},       {"mto", &cfg.mto, 500, 60000},
};
constexpr int NUM_SETTINGS = sizeof(SETTINGS) / sizeof(SETTINGS[0]);
uint32_t cfgVer = 1;   // bumps on every change so pages know to re-read

Preferences prefs;
void saveConfig(const Config &c) {
  prefs.begin("boltsort", false);
  prefs.putUInt("ver", CFG_VERSION);
  prefs.putBytes("cfg", &c, sizeof(c));
  prefs.end();
}
bool loadConfig() {
  prefs.begin("boltsort", true);
  bool ok = prefs.getUInt("ver", 0) == CFG_VERSION && prefs.getBytesLength("cfg") == sizeof(Config);
  if (ok) prefs.getBytes("cfg", &cfg, sizeof(Config));
  prefs.end();
  return ok;
}

// =====================================================================
//  ENCODERS  (quadrature state table, one ISR per pin edge)
// =====================================================================
DRAM_ATTR static const int8_t QEM[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

struct EncState { int pinA, pinB; volatile long count; volatile uint8_t last; };
DRAM_ATTR EncState encs[NUM_AX] = {
  {TUBE_ENC_A, TUBE_ENC_B, 0, 0}, {FLAP_ENC_A, FLAP_ENC_B, 0, 0}, {HATCH_ENC_A, HATCH_ENC_B, 0, 0}};

static inline uint32_t IRAM_ATTR fastRead(int p) {
  return p < 32 ? (REG_READ(GPIO_IN_REG) >> p) & 1 : (REG_READ(GPIO_IN1_REG) >> (p - 32)) & 1;
}
void IRAM_ATTR encISR(void *arg) {
  EncState *e = (EncState *)arg;
  uint8_t s = (fastRead(e->pinA) << 1) | fastRead(e->pinB);
  portENTER_CRITICAL_ISR(&encMux);
  e->count += QEM[(e->last << 2) | s];
  e->last = s;
  portEXIT_CRITICAL_ISR(&encMux);
}
long encRead(int i)            { portENTER_CRITICAL(&encMux); long c = encs[i].count; portEXIT_CRITICAL(&encMux); return c; }
void encWrite(int i, long v)   { portENTER_CRITICAL(&encMux); encs[i].count = v;     portEXIT_CRITICAL(&encMux); }

// =====================================================================
//  DC MOTOR AXIS  (profile -> PID + friction comp -> settle -> active hold)
// =====================================================================
enum AxMode : uint8_t { AX_STOP, AX_HOLD, AX_MOVE, AX_SPIN, AX_RAW, AX_CAL, AX_FAULT };
const char *AX_MODE_NAME[] = {"STOP", "HOLD", "MOVE", "SPIN", "RAW", "CAL", "FAULT"};

struct Axis {
  const char *name;
  uint8_t id;
  int in1, in2;
  AxMode mode = AX_HOLD;
  float angle = 0, vel = 0, target = 0, sp = 0, spVel = 0, integ = 0;
  float moveV = 0, spinV = 0;
  int pwm = 0, rawPwm = 0;
  uint32_t rawUntil = 0, settledSince = 0, lastChange = 0, wrongSince = 0;
  long lastCount = 0;
  float lastAngle = 0;
  char fault[80] = "";

  AxisCfg &c() { return cfg.ax[id]; }
  float cpd() { return c().cpr / 360.0f; }
  long counts() { long v = encRead(id); return c().einv > 0.5f ? -v : v; }

  void begin() {
    ledcAttach(in1, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    ledcAttach(in2, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    brake();
    EncState &e = encs[id];
    pinMode(e.pinA, INPUT_PULLUP);
    pinMode(e.pinB, INPUT_PULLUP);
    e.last = (digitalRead(e.pinA) << 1) | digitalRead(e.pinB);
    attachInterruptArg(digitalPinToInterrupt(e.pinA), encISR, &e, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(e.pinB), encISR, &e, CHANGE);
  }

  // raw outputs; p in -255..255 (positive = positive encoder counts once tuned)
  void drive(int p) {
    if (c().inv > 0.5f) p = -p;
    if (p > 0)      { ledcWrite(in1, p); ledcWrite(in2, 0); }
    else if (p < 0) { ledcWrite(in1, 0); ledcWrite(in2, -p); }
    else            brake();
  }
  void brake() { ledcWrite(in1, 255); ledcWrite(in2, 255); }
  void coast() { ledcWrite(in1, 0);   ledcWrite(in2, 0); }

  void resetLoop() { sp = angle; spVel = 0; integ = 0; settledSince = 0; wrongSince = 0; lastCount = counts(); lastChange = millis(); }

  void moveTo(float deg, float v = 0) {
    if (mode != AX_MOVE) resetLoop();
    target = deg;
    moveV = v > 0 ? v : c().vmax;
    fault[0] = 0;
    mode = AX_MOVE;
  }
  void moveBy(float d, float v = 0) {
    float base = (mode == AX_HOLD || mode == AX_MOVE) ? target : angle;
    moveTo(base + d, v);
  }
  void spin(float v) {
    if (mode != AX_SPIN) resetLoop();
    spinV = v; fault[0] = 0; mode = AX_SPIN;
  }
  void raw(int p, uint32_t ms) { rawPwm = constrain(p, -255, 255); rawUntil = millis() + ms; fault[0] = 0; mode = AX_RAW; }
  void holdHere() { angle = counts() / cpd(); lastAngle = angle; target = angle; resetLoop(); fault[0] = 0; mode = AX_HOLD; }
  void stop() { mode = AX_STOP; pwm = 0; brake(); }
  void setFault(const char *m) { mode = AX_FAULT; pwm = 0; integ = 0; strlcpy(fault, m, sizeof(fault)); brake(); }
  // declare the current physical position to be `deg`
  void setAngle(float deg) {
    long cnt = lroundf(deg * cpd());
    encWrite(id, c().einv > 0.5f ? -cnt : cnt);
    angle = lastAngle = deg;
    if (mode == AX_HOLD || mode == AX_STOP || mode == AX_FAULT || mode == AX_CAL) target = deg;
    else holdHere();
    resetLoop();
  }
  bool done() const { return mode == AX_HOLD; }
  bool busy() const { return mode == AX_MOVE || mode == AX_SPIN || mode == AX_RAW; }
  // direction this axis is currently being driven (+1 / -1 / 0)
  int motionDir() const {
    if (mode == AX_MOVE) return target > angle + 0.05f ? 1 : (target < angle - 0.05f ? -1 : 0);
    if (mode == AX_SPIN) return spinV > 0 ? 1 : (spinV < 0 ? -1 : 0);
    if (mode == AX_RAW)  return rawPwm > 0 ? 1 : (rawPwm < 0 ? -1 : 0);
    return 0;
  }

  void update(float dt) {
    long cnt = counts();
    angle = cnt / cpd();
    float vRaw = (angle - lastAngle) / dt;
    lastAngle = angle;
    vel += 0.1f * (vRaw - vel);
    uint32_t now = millis();

    switch (mode) {
      case AX_STOP: case AX_FAULT: pwm = 0; brake(); return;
      case AX_CAL:  pwm = 0; coast(); return;
      case AX_RAW:
        if ((int32_t)(now - rawUntil) >= 0) { holdHere(); pwm = 0; brake(); return; }
        pwm = rawPwm; drive(pwm); return;
      case AX_HOLD:
        pwm = 0; brake();
        lastCount = cnt; lastChange = now;
        if (fabsf(target - angle) > fmaxf(1.5f, 3.0f * c().tol)) {   // pushed off: correct
          float t = target; resetLoop(); target = t; moveV = c().vmax; mode = AX_MOVE;
        }
        return;
      default: break;   // MOVE / SPIN
    }

    // 1) motion profile
    const float acc = c().acc, maxDv = acc * dt;
    bool profileDone = false;
    if (mode == AX_MOVE) {
      float dist = target - sp;
      float vDes = (dist >= 0 ? 1 : -1) * fminf(moveV, sqrtf(2.0f * acc * fabsf(dist)));
      spVel += constrain(vDes - spVel, -maxDv, maxDv);
      sp += spVel * dt;
      if (fabsf(target - sp) < 0.05f && fabsf(spVel) <= maxDv * 2) { sp = target; spVel = 0; profileDone = true; }
    } else {   // SPIN: constant-velocity setpoint
      spVel += constrain(spinV - spVel, -maxDv, maxDv);
      sp += spVel * dt;
      target = sp;
      if (spinV == 0 && fabsf(spVel) <= maxDv * 2) { spVel = 0; moveV = c().vmax; mode = AX_MOVE; }
    }

    // 2) PID on (setpoint - measured)
    float e = sp - angle, ve = spVel - vel;
    integ += e * dt;
    if (c().ki > 0) integ = constrain(integ, -I_LIMIT / c().ki, I_LIMIT / c().ki);
    float u = c().kp * e + c().ki * integ + c().kd * ve;

    // 3) friction compensation
    const float ff = c().ff;
    int p = 0;
    if (fabsf(u) > 1.0f) p = (int)(u + (u > 0 ? ff : -ff));
    int mp = (int)c().maxpwm;
    p = constrain(p, -mp, mp);

    // 4) arrived = inside tolerance AND slow for SETTLE_MS
    if (mode == AX_MOVE && profileDone && fabsf(target - angle) < c().tol && fabsf(vel) < SETTLE_VEL) {
      if (!settledSince) settledSince = now ? now : 1;
      if (now - settledSince > SETTLE_MS) { mode = AX_HOLD; integ = 0; settledSince = 0; pwm = 0; brake(); return; }
    } else settledSince = 0;

    // 5) safety
    if (cnt != lastCount || abs(p) < ff + 15) { lastCount = cnt; lastChange = now; }
    else if (now - lastChange > STALL_MS) { setFault("driven but encoder not counting: check encoder wiring/power or a jam"); return; }
    if (abs(p) > ff + 30 && fabsf(vel) > 30 && ((p > 0) != (vel > 0))) {
      if (!wrongSince) wrongSince = now ? now : 1;
      if (now - wrongSince > WRONGWAY_MS) { setFault("motor and encoder directions disagree: toggle 'invert encoder'"); return; }
    } else wrongSince = 0;

    pwm = p;
    drive(p);
  }
};

Axis axes[NUM_AX] = {
  {"tube", TUBE, TUBE_IN1, TUBE_IN2},
  {"flap", FLAP, FLAP_IN1, FLAP_IN2},
  {"hatch", HATCH, HATCH_IN1, HATCH_IN2},
};
Axis &tube = axes[TUBE], &flap = axes[FLAP], &hatch = axes[HATCH];

// =====================================================================
//  FEEDER SERVO  (non-blocking, rate-limited)
// =====================================================================
struct ServoAx {
  float angle = 0, target = 0;
  bool reached = true;
  uint32_t reachedAt = 0;
  void begin(float a) { ledcAttach(SERVO_PIN, SERVO_HZ, SERVO_BITS); angle = target = a; write(); reached = true; reachedAt = millis(); }
  void write() {
    float us = cfg.svmin + (cfg.svmax - cfg.svmin) * angle / 180.0f;
    ledcWrite(SERVO_PIN, (uint32_t)(us * ((1 << SERVO_BITS) - 1) / 20000.0f));
  }
  void moveTo(float a) { target = constrain(a, 0.0f, 180.0f); reached = false; }
  void update(float dt) {
    if (reached) return;
    float step = cfg.svspd * dt;
    if (fabsf(target - angle) <= step) { angle = target; reached = true; reachedAt = millis(); }
    else angle += (target > angle ? step : -step);
    write();
  }
  bool done(uint32_t holdMs) const { return reached && millis() - reachedAt >= holdMs; }
} feeder;

// =====================================================================
//  DEBOUNCED INPUTS (active LOW, polled by the control task)
// =====================================================================
struct Din {
  int pin; bool state = false, pressed = false; uint8_t n = 0;
  void begin() { pinMode(pin, pin >= 34 ? INPUT : INPUT_PULLUP); state = digitalRead(pin) == LOW; }
  void update() {
    bool raw = digitalRead(pin) == LOW;
    if (raw != state) { if (++n >= DEBOUNCE_TICKS) { state = raw; n = 0; if (state) pressed = true; } }
    else n = 0;
  }
  bool takePress() { bool p = pressed; pressed = false; return p; }
};
Din limGood{LIMIT_GOOD_PIN}, limBad{LIMIT_BAD_PIN}, boltSensor{BOLT_SENSOR_PIN}, button{BUTTON_PIN};

// =====================================================================
//  MACHINE STATE
// =====================================================================
enum MState : uint8_t { M_IDLE, M_WAIT, M_FEED, M_SETTLE, M_INSPECT, M_VERDICT, M_SORT, M_RELEASE, M_CLOSE, M_FAULT };
const char *M_NAME[] = {"IDLE", "WAIT_BOLT", "FEED", "SETTLE", "INSPECT", "AWAIT_VERDICT", "SORT", "RELEASE", "CLOSE", "FAULT"};

MState mstate = M_IDLE;
uint32_t stateAt = 0, subAt = 0;
uint8_t sub = 0;
uint8_t runMode = 0;          // 0 stopped, 1 continuous, 2 finish current cycle then stop
bool triggerReq = false;
bool feederAtDrop = false;
char verdict = 0;             // 'g' 'b' 'e' for the bolt in the tube
char lastVerdict = 0;
char lastReason[48] = "";
char pendingReason[48] = "";
char faultMsg[96] = "";
uint32_t cycleNo = 0, nGood = 0, nBad = 0, nEmpty = 0, nTimeout = 0;
char replyMsg[96] = "";
uint32_t msgSeq = 0;

SemaphoreHandle_t gLock;      // guards everything above + axes + cfg
#define LOCK()   xSemaphoreTake(gLock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(gLock)

// ---- step-response trace (for tuning) ----
constexpr int TRACE_N = 250, TRACE_DIV = 5;   // 250 samples x 10 ms = 2.5 s
struct TraceSample { int16_t sp, a, p; };
TraceSample trace[TRACE_N];
int traceLen = 0, traceAxis = -1, traceDivCnt = 0;
uint8_t traceState = 0;   // 0 none, 1 recording, 2 ready
uint32_t traceId = 0;
float traceBase = 0, traceTarget = 0;

void enter(MState s);

void faultMachine(const char *why) {
  strlcpy(faultMsg, why, sizeof(faultMsg));
  Serial.printf("[FAULT] %s (in %s)\n", why, M_NAME[mstate]);
  enter(M_FAULT);
}

float hatchTargetFor(char v) { return v == 'g' ? cfg.hgood : cfg.hbad; }   // empty / bad -> BAD bin

void enter(MState s) {
  MState from = mstate;
  mstate = s; stateAt = millis(); sub = 0;
  if (from != s) Serial.printf("[STATE] %s -> %s\n", M_NAME[from], M_NAME[s]);
  switch (s) {
    case M_IDLE: break;
    case M_WAIT: break;
    case M_FEED:
      cycleNo++; verdict = 0; pendingReason[0] = 0;
      if (cfg.svret > 0.5f) feeder.moveTo(cfg.svdrop);
      else { feederAtDrop = !feederAtDrop; feeder.moveTo(feederAtDrop ? cfg.svdrop : cfg.svload); }
      break;
    case M_SETTLE: flap.moveTo(cfg.fclose); break;
    case M_INSPECT: tube.setAngle(0); tube.moveTo(cfg.ideg, cfg.ispd); break;
    case M_VERDICT: break;
    case M_SORT: hatch.moveTo(hatchTargetFor(verdict)); break;
    case M_RELEASE: flap.moveTo(cfg.fopen); break;
    case M_CLOSE: flap.moveTo(cfg.fclose); break;
    case M_FAULT:
      runMode = 0; triggerReq = false;
      for (auto &a : axes) if (a.mode != AX_FAULT) a.stop();
      break;
  }
}

void tally(char v) {
  if (v == 'g') nGood++; else if (v == 'e') nEmpty++; else nBad++;
  lastVerdict = v;
  strlcpy(lastReason, pendingReason, sizeof(lastReason));
}

bool hatchArrived() { return hatch.done() && fabsf(hatch.target - hatchTargetFor(verdict)) < 0.01f; }

void machineTick() {
  uint32_t el = millis() - stateAt;

  if (mstate != M_IDLE && mstate != M_FAULT) {
    for (auto &a : axes) if (a.mode == AX_FAULT) {
      char b[96]; snprintf(b, sizeof(b), "%s: %s", a.name, a.fault); faultMachine(b); return;
    }
  }

  switch (mstate) {
    case M_IDLE: case M_FAULT: break;

    case M_WAIT:
      if (triggerReq) { triggerReq = false; enter(M_FEED); }
      else if (cfg.bsens > 0.5f) { if (boltSensor.state) enter(M_FEED); }
      else if (el >= cfg.gap) enter(M_FEED);
      break;

    case M_FEED:
      if (cfg.svret > 0.5f) {
        if (sub == 0 && feeder.done((uint32_t)cfg.svhold)) { feeder.moveTo(cfg.svload); sub = 1; }
        else if (sub == 1 && feeder.done(0)) enter(M_SETTLE);
      } else if (feeder.done((uint32_t)cfg.svhold)) enter(M_SETTLE);
      break;

    case M_SETTLE:
      if (el >= cfg.settle && flap.done()) enter(M_INSPECT);
      else if (el > cfg.settle + cfg.mto) faultMachine("flap did not close before inspection");
      break;

    case M_INSPECT:
      if (tube.done()) enter(M_VERDICT);
      else if (el > cfg.ideg / cfg.ispd * 1000.0f + cfg.mto) faultMachine("tube rotation timed out");
      break;

    case M_VERDICT:
      if (verdict) enter(M_SORT);
      else if ((int)cfg.vfb != 2 && el >= cfg.vto) {
        nTimeout++;
        if ((int)cfg.vfb == 3) { faultMachine("no verdict from the inspector"); break; }
        verdict = (int)cfg.vfb == 1 ? 'g' : 'b';
        strlcpy(pendingReason, "no verdict (timeout fallback)", sizeof(pendingReason));
        Serial.printf("[INSPECT] verdict timeout -> %c\n", verdict);
        enter(M_SORT);
      }
      break;

    case M_SORT:
      if (hatchArrived()) enter(M_RELEASE);
      else if (el > cfg.mto) faultMachine("hatch did not reach its bin position");
      break;

    case M_RELEASE:
      if (flap.done()) {
        if (!sub) { sub = 1; subAt = millis(); }
        if (millis() - subAt >= cfg.fdwell) enter(M_CLOSE);
      } else if (el > cfg.mto + cfg.fdwell) faultMachine("flap did not open");
      break;

    case M_CLOSE:
      if (flap.done()) {
        tally(verdict);
        Serial.printf("[CYCLE] bolt %lu -> %s\n", (unsigned long)cycleNo, verdict == 'g' ? "GOOD" : verdict == 'e' ? "EMPTY" : "BAD");
        if (runMode == 1) enter(M_WAIT); else { runMode = 0; enter(M_IDLE); }
      } else if (el > cfg.mto) faultMachine("flap did not close");
      break;
  }
}

// hatch limit switches: stop + re-anchor when driven into a switch
void hatchLimits() {
  if (cfg.hlim < 0.5f) return;
  int goodDir = cfg.hgood >= cfg.hbad ? 1 : -1, d = hatch.motionDir();
  if (d == goodDir && limGood.state)  hatch.setAngle(cfg.hgood);
  else if (d == -goodDir && limBad.state) hatch.setAngle(cfg.hbad);
}

void traceTick() {
  if (traceState != 1 || traceAxis < 0) return;
  if (++traceDivCnt < TRACE_DIV) return;
  traceDivCnt = 0;
  Axis &a = axes[traceAxis];
  trace[traceLen++] = {(int16_t)constrain(lroundf((a.sp - traceBase) * 10), -32000, 32000),
                       (int16_t)constrain(lroundf((a.angle - traceBase) * 10), -32000, 32000),
                       (int16_t)a.pwm};
  if (traceLen >= TRACE_N) { traceState = 2; traceId++; }
}

// =====================================================================
//  CONTROL TASK  (500 Hz, core 1, above loop())
// =====================================================================
void controlTask(void *) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastUs = micros();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
    uint32_t now = micros();
    float dt = constrain((now - lastUs) * 1e-6f, 0.0005f, 0.02f);
    lastUs = now;
    LOCK();
    limGood.update(); limBad.update(); boltSensor.update(); button.update();
    for (auto &a : axes) a.update(dt);
    hatchLimits();
    feeder.update(dt);
    if (button.takePress()) {
      if (mstate == M_FAULT) { for (auto &a : axes) a.holdHere(); faultMsg[0] = 0; enter(M_IDLE); }
      else if (mstate == M_IDLE) { runMode = 1; enter(M_WAIT); }
      else { runMode = 0; for (auto &a : axes) if (a.busy()) a.holdHere(); enter(M_IDLE); }
    }
    machineTick();
    traceTick();
    UNLOCK();
  }
}

// =====================================================================
//  JSON
// =====================================================================
// copy s into a JSON string body (no quotes/backslashes/control chars)
void jsonClean(char *dst, const char *s, size_t n) {
  size_t j = 0;
  for (; *s && j + 1 < n; s++) { char ch = *s; dst[j++] = (ch == '"' || ch == '\\' || (uint8_t)ch < 0x20) ? '\'' : ch; }
  dst[j] = 0;
}

String ipString() {
  if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
  return WiFi.softAPIP().toString();
}

volatile int bleConns = 0;

String statusJson() {   // call with LOCK held
  char buf[1700], fm[100], rs[50], rm[100], axb[3][240];
  jsonClean(fm, faultMsg, sizeof(fm));
  jsonClean(rs, lastReason, sizeof(rs));
  jsonClean(rm, replyMsg, sizeof(rm));
  for (int i = 0; i < NUM_AX; i++) {
    Axis &a = axes[i]; char af[82]; jsonClean(af, a.fault, sizeof(af));
    snprintf(axb[i], sizeof(axb[i]), "{\"a\":%.2f,\"t\":%.2f,\"sp\":%.2f,\"v\":%.1f,\"p\":%d,\"m\":\"%s\",\"c\":%ld,\"f\":\"%s\"}",
             a.angle, a.target, a.sp, a.vel, a.pwm, AX_MODE_NAME[a.mode], encRead(i), af);
  }
  float prog = 0;
  if (mstate == M_INSPECT) prog = constrain(tube.angle / fmaxf(cfg.ideg, 1), 0.0f, 1.0f);
  else if (mstate > M_INSPECT && mstate != M_FAULT) prog = 1;
  char vs[2] = {verdict, 0}, lv[2] = {lastVerdict, 0};
  snprintf(buf, sizeof(buf),
    "{\"t\":\"s\",\"up\":%lu,\"st\":\"%s\",\"run\":%d,\"cyc\":%lu,\"el\":%lu,\"prog\":%.3f,\"flt\":\"%s\","
    "\"v\":\"%s\",\"lv\":\"%s\",\"lr\":\"%s\",\"n\":[%lu,%lu,%lu,%lu],\"ax\":[%s,%s,%s],"
    "\"sv\":%.1f,\"in\":[%d,%d,%d,%d],\"cv\":%lu,\"tr\":%d,\"tid\":%lu,\"ms\":%lu,\"msg\":\"%s\","
    "\"ble\":%d,\"ip\":\"%s\"}",
    (unsigned long)(millis() / 1000), M_NAME[mstate], runMode, (unsigned long)cycleNo,
    (unsigned long)(millis() - stateAt), prog, fm, vs, lv, rs,
    (unsigned long)nGood, (unsigned long)nBad, (unsigned long)nEmpty, (unsigned long)nTimeout,
    axb[0], axb[1], axb[2], feeder.angle,
    boltSensor.state, limGood.state, limBad.state, button.state,
    (unsigned long)cfgVer, traceState, (unsigned long)traceId, (unsigned long)msgSeq, rm,
    (int)bleConns, ipString().c_str());
  return String(buf);
}

String configJson() {   // call with LOCK held
  String s; s.reserve(1400);
  s += "{\"t\":\"c\",\"cv\":"; s += cfgVer; s += ",\"v\":{";
  for (int i = 0; i < NUM_SETTINGS; i++) {
    char b[48]; snprintf(b, sizeof(b), "%s\"%s\":%g", i ? "," : "", SETTINGS[i].k, *SETTINGS[i].v);
    s += b;
  }
  s += "}}";
  return s;
}

String traceJson() {   // call with LOCK held
  String s; s.reserve(TRACE_N * 18 + 200);
  s += "{\"t\":\"tr\",\"id\":"; s += traceId;
  s += ",\"ax\":\""; s += traceAxis >= 0 ? axes[traceAxis].name : ""; s += "\"";
  s += ",\"dt\":"; s += TRACE_N > 0 ? TRACE_DIV * 2 : 0;
  s += ",\"tgt\":"; s += String((traceTarget - traceBase), 2);
  s += ",\"tol\":"; s += traceAxis >= 0 ? String(axes[traceAxis].c().tol, 2) : String("0");
  s += ",\"n\":"; s += traceState == 2 ? traceLen : 0;
  const char *names[3] = {"sp", "a", "p"};
  for (int k = 0; k < 3; k++) {
    s += ",\""; s += names[k]; s += "\":[";
    for (int i = 0; traceState == 2 && i < traceLen; i++) {
      if (i) s += ',';
      s += k == 0 ? trace[i].sp : k == 1 ? trace[i].a : trace[i].p;
    }
    s += "]";
  }
  s += "}";
  return s;
}

// =====================================================================
//  COMMANDS  (shared by Wi-Fi, BLE and Serial; call with LOCK held)
// =====================================================================
const char HELP[] =
  "run | finish | stop | estop | reset | single | trigger(k) | v g|b|e [reason] (or g/b/e)\n"
  "jog <ax> <deg> | goto <ax> <deg> | step <ax> <deg> | spin <ax> <deg/s> | pwm <ax> <-255..255> [ms]\n"
  "halt <ax> | zero <ax> | cal <ax> start|done   (ax = t tube, f flap, h hatch)\n"
  "servo <deg> | feed | flap open|close | hatch good|bad | inspect\n"
  "set key=val ... | get | save | load | defaults | tally reset | plot t|f|h|off | status";

int axisFrom(const String &s) {
  if (s == "t" || s == "tube") return TUBE;
  if (s == "f" || s == "flap") return FLAP;
  if (s == "h" || s == "hatch" || s == "s" || s == "sorter" || s == "funnel") return HATCH;
  return -1;
}

int plotAxis = -1;
bool saveRequested = false;

bool applySetting(const String &kv, String &err) {
  int eq = kv.indexOf('=');
  if (eq <= 0) { err = "bad pair '" + kv + "'"; return false; }
  String k = kv.substring(0, eq), v = kv.substring(eq + 1);
  for (int i = 0; i < NUM_SETTINGS; i++) {
    if (k != SETTINGS[i].k) continue;
    if (!v.length()) { err = "no value for " + k; return false; }
    float f = constrain(v.toFloat(), SETTINGS[i].lo, SETTINGS[i].hi);
    *SETTINGS[i].v = f;
    // keep axes where they physically are when their scale/sign changes
    if (k.endsWith("cpr") || k.endsWith("einv")) {
      int ax = k[0] == 't' ? TUBE : k[0] == 'f' ? FLAP : HATCH;
      if (k.endsWith("einv")) encWrite(ax, -encRead(ax));   // keep the same logical position
      axes[ax].holdHere();
    }
    if (k.startsWith("sv")) feeder.write();
    return true;
  }
  err = "unknown setting " + k;
  return false;
}

String exec(String line, bool &ok) {
  ok = true;
  line.trim();
  if (!line.length()) { ok = false; return ""; }
  String L = line; L.toLowerCase();
  int sp1 = L.indexOf(' ');
  String cmd = sp1 < 0 ? L : L.substring(0, sp1);
  String rest = sp1 < 0 ? "" : line.substring(sp1 + 1); rest.trim();
  String restL = rest; restL.toLowerCase();
  // second / third tokens
  int sp2 = restL.indexOf(' ');
  String a1 = sp2 < 0 ? restL : restL.substring(0, sp2);
  String a2 = sp2 < 0 ? "" : restL.substring(sp2 + 1); a2.trim();
  bool idle = mstate == M_IDLE;
  auto needIdle = [&]() -> bool { if (!idle) { ok = false; } return idle; };
  const char *NOT_IDLE = "stop the machine first (manual moves only in IDLE)";

  // ---- machine ----
  if (cmd == "help" || cmd == "?") return HELP;
  if (cmd == "status") return statusJson();
  if (cmd == "run") {
    if (mstate == M_FAULT) { ok = false; return "in FAULT: reset first"; }
    runMode = 1; if (idle) enter(M_WAIT); return "running";
  }
  if (cmd == "finish") { if (runMode) runMode = 2; return "will stop after this bolt"; }
  if (cmd == "single" || cmd == "cycle") {
    if (!idle) { ok = false; return "already running"; }
    runMode = 2; triggerReq = true; enter(M_WAIT); return "one cycle";
  }
  if (cmd == "stop") {
    runMode = 0; triggerReq = false;
    for (auto &a : axes) if (a.busy()) a.holdHere();
    feeder.moveTo(feeder.angle);
    if (mstate != M_FAULT) enter(M_IDLE);
    return "stopped";
  }
  if (cmd == "estop") {
    for (auto &a : axes) a.stop();
    feeder.moveTo(feeder.angle);
    faultMachine("E-STOP pressed");
    return "E-STOP";
  }
  if (cmd == "reset" || cmd == "r") {
    for (auto &a : axes) a.holdHere();
    faultMsg[0] = 0; runMode = 0;
    enter(M_IDLE); return "reset: IDLE";
  }
  if (cmd == "trigger" || cmd == "k") {
    if (mstate != M_WAIT) { ok = false; return "not waiting for a bolt"; }
    triggerReq = true; return "bolt trigger";
  }
  if (cmd == "v" || cmd == "g" || cmd == "b" || cmd == "e" || cmd == "good" || cmd == "bad" || cmd == "empty") {
    char v = cmd == "v" ? (a1.length() ? a1[0] : 0) : cmd[0];
    String reason = cmd == "v" ? (sp2 < 0 ? "" : rest.substring(sp2 + 1)) : rest;
    if (v != 'g' && v != 'b' && v != 'e') { ok = false; return "verdict must be g, b or e"; }
    if (mstate != M_INSPECT && mstate != M_VERDICT) { ok = false; return "no bolt is being inspected"; }
    if (verdict) { ok = false; return "verdict already given for this bolt"; }
    verdict = v;
    jsonClean(pendingReason, reason.c_str(), sizeof(pendingReason));
    Serial.printf("[INSPECT] verdict %c %s\n", v, pendingReason);
    return v == 'g' ? "GOOD" : v == 'b' ? "BAD" : "EMPTY";
  }
  if (cmd == "tally") { nGood = nBad = nEmpty = nTimeout = 0; cycleNo = 0; lastVerdict = 0; lastReason[0] = 0; return "tally cleared"; }

  // ---- settings ----
  if (cmd == "set") {
    int n = 0; String err; int from = 0;
    while (from < (int)rest.length()) {
      int to = rest.indexOf(' ', from); if (to < 0) to = rest.length();
      String kv = rest.substring(from, to); kv.trim(); from = to + 1;
      if (!kv.length()) continue;
      if (applySetting(kv, err)) n++; else { ok = false; break; }
    }
    if (n) cfgVer++;
    return ok ? String("set ") + n + (n == 1 ? " value" : " values") : err;
  }
  if (cmd == "get") return configJson();
  if (cmd == "save") { saveRequested = true; return "saved to flash"; }
  if (cmd == "load") { bool l = loadConfig(); cfgVer++; for (auto &a : axes) a.holdHere(); return l ? "loaded saved settings" : "nothing saved yet"; }
  if (cmd == "defaults") { setDefaults(); cfgVer++; for (auto &a : axes) a.holdHere(); return "defaults restored (not saved)"; }
  if (cmd == "plot") { plotAxis = axisFrom(a1); return plotAxis < 0 ? "plot off" : String("plotting ") + axes[plotAxis].name; }

  // ---- manual axis commands ----
  if (cmd == "halt") {
    int ax = axisFrom(a1);
    if (ax < 0) { for (auto &a : axes) if (a.busy() || a.mode == AX_CAL) a.holdHere(); return "all axes halted"; }
    axes[ax].holdHere(); return String(axes[ax].name) + " halted";
  }
  if (cmd == "jog" || cmd == "goto" || cmd == "step" || cmd == "spin" || cmd == "pwm" || cmd == "zero" || cmd == "cal") {
    int ax = axisFrom(a1);
    if (ax < 0) { ok = false; return "axis must be t, f or h"; }
    if (!needIdle()) return NOT_IDLE;
    Axis &a = axes[ax];
    if (a.mode == AX_FAULT && cmd != "zero") { ok = false; return String(a.name) + " is faulted: reset first"; }
    float v = a2.toFloat();
    if (cmd == "jog")  { a.moveBy(v); return String(a.name) + " jog " + String(v, 1); }
    if (cmd == "goto") { a.moveTo(v); return String(a.name) + " -> " + String(v, 1); }
    if (cmd == "step") {
      if (!a2.length() || v == 0) { ok = false; return "step needs a distance"; }
      a.holdHere();
      traceAxis = ax; traceLen = 0; traceDivCnt = TRACE_DIV - 1; traceBase = a.angle; traceTarget = a.angle + v; traceState = 1;
      a.moveTo(traceTarget); return String("step test ") + a.name + " " + String(v, 1);
    }
    if (cmd == "spin") { a.spin(v); return String(a.name) + " spin " + String(v, 1) + " deg/s"; }
    if (cmd == "pwm") {
      int sp3 = a2.indexOf(' ');
      int p = (sp3 < 0 ? a2 : a2.substring(0, sp3)).toInt();
      uint32_t ms = sp3 < 0 ? 1000 : constrain(a2.substring(sp3 + 1).toInt(), 50, 5000);
      a.raw(p, ms); return String(a.name) + " open-loop pwm " + p + " for " + ms + " ms";
    }
    if (cmd == "zero") { a.holdHere(); a.setAngle(0); return String(a.name) + " zeroed here"; }
    if (cmd == "cal") {
      if (a1.length() && a2 == "start") { a.mode = AX_CAL; encWrite(ax, 0); a.angle = 0; return "turn the output shaft exactly one turn by hand, then cal done"; }
      if (a2 == "done") {
        if (a.mode != AX_CAL) { ok = false; return "cal start first"; }
        long n = labs(encRead(ax));
        a.holdHere(); a.setAngle(0);
        if (n < 50) { ok = false; return "too few counts: is the encoder working?"; }
        a.c().cpr = n; cfgVer++; return String(a.name) + " cpr = " + n + " (save to keep)";
      }
      ok = false; return "cal <ax> start|done";
    }
  }
  if (cmd == "servo") { if (!needIdle()) return NOT_IDLE; feeder.moveTo(a1.toFloat()); return "servo -> " + String(feeder.target, 0); }
  if (cmd == "feed") {
    if (!needIdle()) return NOT_IDLE;
    feederAtDrop = !feederAtDrop; feeder.moveTo(feederAtDrop ? cfg.svdrop : cfg.svload);
    return feederAtDrop ? "feeder -> drop" : "feeder -> load";
  }
  if (cmd == "flap") {
    if (!needIdle()) return NOT_IDLE;
    if (a1 == "open")  { flap.moveTo(cfg.fopen);  return "flap opening"; }
    if (a1 == "close") { flap.moveTo(cfg.fclose); return "flap closing"; }
    ok = false; return "flap open|close";
  }
  if (cmd == "hatch") {
    if (!needIdle()) return NOT_IDLE;
    if (a1 == "good") { hatch.moveTo(cfg.hgood); return "hatch -> GOOD bin"; }
    if (a1 == "bad")  { hatch.moveTo(cfg.hbad);  return "hatch -> BAD bin"; }
    ok = false; return "hatch good|bad";
  }
  if (cmd == "inspect") {
    if (!needIdle()) return NOT_IDLE;
    tube.setAngle(0); tube.moveTo(cfg.ideg, cfg.ispd); return "tube inspection turn";
  }
  ok = false;
  return "unknown command '" + cmd + "' (try help)";
}

// run a command from any transport; records the reply for status
String runCommand(const String &line, bool &ok) {
  LOCK();
  String r = exec(line, ok);
  if (r.length() && r[0] != '{') { strlcpy(replyMsg, r.c_str(), sizeof(replyMsg)); msgSeq++; }
  bool doSave = saveRequested; saveRequested = false;
  Config copy = cfg;
  UNLOCK();
  if (doSave) saveConfig(copy);   // flash write outside the lock
  return r;
}

// =====================================================================
//  WI-FI / HTTP
// =====================================================================
WebServer server(80);
extern const char CONTROL_PAGE[];

void corsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Private-Network", "true");
  server.sendHeader("Cache-Control", "no-store");
}
void sendJson(const String &s) { corsHeaders(); server.send(200, "application/json", s); }

void setupWeb() {
  bool sta = strlen(STA_SSID) > 0;
  WiFi.setHostname(HOST_NAME);
  WiFi.mode(sta ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS, 6);
  if (sta) WiFi.begin(STA_SSID, STA_PASS);
#if !ENABLE_BLE
  WiFi.setSleep(false);   // Wi-Fi/BLE coexistence requires modem sleep, so only when BLE is off
#endif
  MDNS.begin(HOST_NAME);
  MDNS.addService("http", "tcp", 80);

  server.on("/", HTTP_GET, []() { server.sendHeader("Cache-Control", "no-store"); server.send_P(200, "text/html", CONTROL_PAGE); });
  server.on("/api/status", HTTP_GET, []() { LOCK(); String s = statusJson(); UNLOCK(); sendJson(s); });
  server.on("/api/config", HTTP_GET, []() { LOCK(); String s = configJson(); UNLOCK(); sendJson(s); });
  server.on("/api/trace", HTTP_GET, []() { LOCK(); String s = traceJson(); UNLOCK(); sendJson(s); });
  server.on("/api/cmd", HTTP_GET, []() {
    bool ok; String r = runCommand(server.arg("c"), ok);
    if (r.length() && r[0] == '{') { sendJson(r); return; }
    char m[200]; jsonClean(m, r.c_str(), sizeof(m));
    sendJson(String("{\"t\":\"r\",\"ok\":") + (ok ? 1 : 0) + ",\"msg\":\"" + m + "\"}");
  });
  server.on("/favicon.ico", []() { server.send(204); });
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {   // CORS / private-network preflight
      corsHeaders();
      server.sendHeader("Access-Control-Allow-Methods", "GET");
      server.sendHeader("Access-Control-Allow-Headers", "*");
      server.send(204); return;
    }
    server.send(404, "text/plain", "not found");
  });
  server.begin();
}

// =====================================================================
//  BLUETOOTH LE  (Nordic UART service: newline-delimited text/JSON)
// =====================================================================
#if ENABLE_BLE
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // page -> ESP32
#define NUS_TX      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // ESP32 -> page (notify)

BLEServer *bleServer = nullptr;
BLECharacteristic *bleTx = nullptr;
StreamBufferHandle_t bleIn;
constexpr int MAX_CONN = 9;
volatile uint16_t connMtu[MAX_CONN] = {0};
String bleOut, bleLine;
uint32_t bleLastSend = 0, bleLastStatus = 0;

class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *p) override {
    if (p->connect.conn_id < MAX_CONN) connMtu[p->connect.conn_id] = 23;
    bleConns = bleConns + 1;
    BLEDevice::startAdvertising();   // allow a second device (tuning + camera)
  }
  void onDisconnect(BLEServer *s, esp_ble_gatts_cb_param_t *p) override {
    if (p->disconnect.conn_id < MAX_CONN) connMtu[p->disconnect.conn_id] = 0;
    if (bleConns > 0) bleConns = bleConns - 1;
    BLEDevice::startAdvertising();
  }
  void onMtuChanged(BLEServer *s, esp_ble_gatts_cb_param_t *p) override {
    if (p->mtu.conn_id < MAX_CONN) connMtu[p->mtu.conn_id] = p->mtu.mtu;
  }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    if (v.length()) xStreamBufferSend(bleIn, v.c_str(), v.length(), 0);
  }
};

void setupBLE() {
  bleIn = xStreamBufferCreate(2048, 1);
  BLEDevice::init("BoltSorter");
  BLEDevice::setMTU(247);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new SrvCB());
  BLEService *svc = bleServer->createService(NUS_SERVICE);
  bleTx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void bleQueue(const String &s) { if (bleConns > 0 && bleOut.length() < 12000) { bleOut += s; bleOut += '\n'; } }

int bleChunk() {
  int m = 0;
  for (int i = 0; i < MAX_CONN; i++) if (connMtu[i] && (m == 0 || connMtu[i] < m)) m = connMtu[i];
  return constrain((m ? m : 23) - 3, 20, 240);
}

void bleService() {
  // incoming bytes -> lines -> commands
  char tmp[128]; size_t n;
  while ((n = xStreamBufferReceive(bleIn, tmp, sizeof(tmp), 0)) > 0) {
    for (size_t i = 0; i < n; i++) {
      char ch = tmp[i];
      if (ch == '\n' || ch == '\r') {
        if (bleLine.length()) {
          bool ok; String r;
          if (bleLine == "trace") { LOCK(); r = traceJson(); UNLOCK(); bleQueue(r); }
          else {
            r = runCommand(bleLine, ok);
            if (r.length() && r[0] == '{') bleQueue(r);
            else { char m[200]; jsonClean(m, r.c_str(), sizeof(m)); bleQueue(String("{\"t\":\"r\",\"ok\":") + (ok ? 1 : 0) + ",\"msg\":\"" + m + "\"}"); }
          }
          bleLine = "";
        }
      } else if (bleLine.length() < 400) bleLine += ch;
    }
  }
  if (bleConns <= 0) { bleOut = ""; return; }
  // periodic status, only if the pipe is not backed up
  if (millis() - bleLastStatus >= 250 && bleOut.length() < 600) {
    bleLastStatus = millis();
    LOCK(); String s = statusJson(); UNLOCK();
    bleQueue(s);
  }
  // drain one chunk every few ms so the BLE stack keeps up
  if (bleOut.length() && millis() - bleLastSend >= 4) {
    bleLastSend = millis();
    int c = min((int)bleOut.length(), bleChunk());
    bleTx->setValue((uint8_t *)bleOut.c_str(), c);
    bleTx->notify();
    bleOut.remove(0, c);
  }
}
#endif

// =====================================================================
//  SERIAL
// =====================================================================
String serialLine;
void serialService() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialLine.length()) {
        bool ok;
        String r = runCommand(serialLine, ok);
        Serial.println(r);
        serialLine = "";
      }
    } else if (serialLine.length() < 400) serialLine += ch;
  }
  static uint32_t lastPlot = 0;
  if (plotAxis >= 0 && millis() - lastPlot >= 20) {
    lastPlot = millis();
    LOCK(); Axis &a = axes[plotAxis]; float t = a.target, s = a.sp, an = a.angle; int p = a.pwm; UNLOCK();
    Serial.printf("target:%.1f,setpoint:%.1f,angle:%.1f,pwm:%d\n", t, s, an, p);
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  gLock = xSemaphoreCreateMutex();

  setDefaults();
  bool loaded = loadConfig();

  for (auto &a : axes) a.begin();
  feeder.begin(cfg.svload);
  limGood.begin(); limBad.begin(); boltSensor.begin(); button.begin();
  for (auto &a : axes) a.holdHere();
  enter(M_IDLE);

  setupWeb();
#if ENABLE_BLE
  setupBLE();
#endif
  xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 10, nullptr, 1);

  Serial.printf("\nBoltSorter ready (%s settings)\n", loaded ? "saved" : "default");
  Serial.printf("Wi-Fi \"%s\" / \"%s\" -> http://%s  (or http://%s.local)\n", AP_SSID, AP_PASS, AP_IP.toString().c_str(), HOST_NAME);
#if ENABLE_BLE
  Serial.println("Bluetooth: advertising as \"BoltSorter\"");
#endif
  Serial.println("Type 'help' for commands.");
}

void loop() {
  server.handleClient();
#if ENABLE_BLE
  bleService();
#endif
  serialService();
  static bool staReported = false;
  if (!staReported && WiFi.status() == WL_CONNECTED) {
    staReported = true;
    Serial.printf("Joined \"%s\": http://%s\n", STA_SSID, WiFi.localIP().toString().c_str());
  }
  delay(1);
}

// =====================================================================
//  EMBEDDED CONTROL PAGE  (generated - edit bolt-control/index.html and
//  run: python3 tools/embed_control_page.py)
// =====================================================================
// <<CONTROL_PAGE_BEGIN>>
const char CONTROL_PAGE[] PROGMEM = R"PAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>BoltSorter Control</title>
<style>
/* Shop-floor control panel. Same tokens as the inspector; system fonts so it works offline from the ESP32. */
:root{color-scheme:light;--bg:#fff;--ink:#16191d;--mut:#5b636c;--line:#dde1e5;--panel:#f2f4f6;--blue:#1f4fd1;--ok:#137a3a;--warn:#a35f00;--bad:#c4161c;
 --s1:#257de0;--s2:#f47230;--grid:#e6e9ec;--band:rgba(19,122,58,.12)}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){color-scheme:dark;--bg:#111418;--ink:#e8ebee;--mut:#9aa3ad;--line:#2a3037;--panel:#1a1f25;--blue:#6f93ff;--ok:#3fca73;--warn:#f0a53a;--bad:#ff5a5f;--s1:#338ced;--s2:#e76624;--grid:#232a31;--band:rgba(63,202,115,.14)}}
:root[data-theme=dark]{color-scheme:dark;--bg:#111418;--ink:#e8ebee;--mut:#9aa3ad;--line:#2a3037;--panel:#1a1f25;--blue:#6f93ff;--ok:#3fca73;--warn:#f0a53a;--bad:#ff5a5f;--s1:#338ced;--s2:#e76624;--grid:#232a31;--band:rgba(63,202,115,.14)}
*{box-sizing:border-box;min-width:0}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;padding-bottom:calc(76px + env(safe-area-inset-bottom,0px))}
.wrap{max-width:1180px;margin:0 auto;padding:16px max(16px,3vw)}
h1,h2,h3{font-family:"Barlow Semi Condensed","Arial Narrow",system-ui,sans-serif;font-weight:700;line-height:1.1;margin:0}
h1{font-size:26px}h2{font-size:19px;margin-bottom:10px}h3{font-size:16px;margin:14px 0 6px}
.top{display:flex;flex-wrap:wrap;gap:10px;align-items:center;justify-content:space-between}
.conn{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.chip{font-size:13px;font-weight:600;padding:3px 10px;border-radius:99px;background:var(--panel);border:1px solid var(--line);color:var(--mut)}
.chip.on{color:var(--ok);border-color:var(--ok)}
.grid{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.35fr);gap:16px;margin-top:14px}
@media(max-width:900px){.grid{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:16px}
button,select,input{font:inherit;color:var(--ink)}
button{min-height:42px;padding:0 14px;border-radius:9px;border:1px solid var(--line);background:var(--bg);cursor:pointer;font-weight:600}
button.pri{background:var(--blue);border-color:var(--blue);color:#fff}
button.okb{border-color:var(--ok);color:var(--ok)}button.badb{border-color:var(--bad);color:var(--bad)}
button.stop{background:var(--bad);border-color:var(--bad);color:#fff}
button:disabled{opacity:.45;cursor:not-allowed}
button:focus-visible,input:focus-visible,select:focus-visible{outline:3px solid var(--blue);outline-offset:2px}
input[type=number],input[type=text],select{min-height:40px;padding:0 10px;border-radius:8px;border:1px solid var(--line);background:var(--bg);width:100%}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-top:8px}
.row>input{flex:1 1 90px;width:auto}
.state{font-family:"Barlow Semi Condensed","Arial Narrow",system-ui,sans-serif;font-weight:700;font-size:40px;line-height:1;letter-spacing:.5px}
.state.IDLE{color:var(--mut)}.state.FAULT{color:var(--bad)}.state.INSPECT,.state.AWAIT_VERDICT{color:var(--blue)}
.sub{color:var(--mut);font-size:14px}
.bar{height:10px;border-radius:5px;background:var(--line);overflow:hidden;margin-top:10px}.bar i{display:block;height:100%;width:0;background:var(--blue);transition:width .25s}
.flt{margin-top:10px;padding:10px 12px;border-radius:8px;border-left:4px solid var(--bad);background:var(--bg);color:var(--bad);font-weight:600}
.tally{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px;text-align:center}
.tally div{background:var(--bg);border-radius:8px;padding:8px 4px;border:1px solid var(--line)}
.tally b{display:block;font-size:24px;font-variant-numeric:tabular-nums}
.tally span{font-size:12px;color:var(--mut)}
.leds{display:flex;flex-wrap:wrap;gap:12px;margin-top:12px;font-size:13px;color:var(--mut)}
.leds span::before{content:"";display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;vertical-align:-1px;background:var(--line);border:1px solid var(--mut)}
.leds span.on::before{background:var(--ok);border-color:var(--ok)}
.tabs{display:flex;gap:4px;flex-wrap:wrap;border-bottom:1px solid var(--line);margin-bottom:12px}
.tabs button{border:0;border-bottom:3px solid transparent;border-radius:0;background:none;color:var(--mut);min-height:40px}
.tabs button[aria-selected=true]{color:var(--ink);border-bottom-color:var(--blue)}
.read{display:grid;grid-template-columns:repeat(auto-fit,minmax(92px,1fr));gap:8px}
.read div{background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:6px 10px}
.read span{display:block;font-size:12px;color:var(--mut)}.read b{font-size:18px;font-variant-numeric:tabular-nums}
.fields{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px}
.fields label{font-size:12px;color:var(--mut);display:block}
.fields label.ck{display:flex;align-items:center;gap:8px;font-size:14px;color:var(--ink);min-height:40px;margin-top:14px}
.fields input[type=checkbox]{width:20px;height:20px}
details{margin-top:12px}summary{cursor:pointer;font-weight:600;color:var(--mut);min-height:32px}
.note{font-size:13px;color:var(--mut);margin:6px 0 0}
.chart{position:relative;margin-top:10px}
.chart svg{display:block;width:100%;height:auto;overflow:visible}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:13px;color:var(--mut);margin-top:6px}
.legend i{display:inline-block;width:18px;height:0;border-top:2px solid var(--c);vertical-align:middle;margin-right:6px}
.legend i.d{border-top-style:dashed}
.tip{position:absolute;pointer-events:none;background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:6px 9px;font-size:12px;box-shadow:0 2px 8px rgba(0,0,0,.15);white-space:nowrap}
.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:8px;margin-top:8px}
.dock{position:fixed;left:0;right:0;bottom:0;background:var(--bg);border-top:1px solid var(--line);padding:10px max(16px,3vw) calc(10px + env(safe-area-inset-bottom,0px));display:flex;gap:8px;justify-content:center;z-index:5}
.dock button{flex:1 1 0;max-width:220px;min-height:48px}
.toast{position:fixed;left:50%;transform:translateX(-50%);bottom:calc(84px + env(safe-area-inset-bottom,0px));background:var(--ink);color:var(--bg);padding:8px 14px;border-radius:8px;font-size:14px;opacity:0;transition:opacity .2s;pointer-events:none;max-width:90vw;z-index:6}
.toast.show{opacity:1}.toast.err{background:var(--bad);color:#fff}
.verd{display:none;margin-top:12px;padding:10px;border-radius:10px;border:2px dashed var(--blue)}
.verd.show{display:block}
.hide{display:none!important}
.dirty{color:var(--warn);font-weight:600;font-size:13px}
</style>
</head>
<body>
<div class="wrap">
<div class="top">
  <div><h1>BoltSorter control</h1><div class="sub">Tune, jog and run the four actuators</div></div>
  <div class="conn">
    <span class="chip" id="linkChip">Not connected</span>
    <button id="btBtn">Connect Bluetooth</button>
    <input type="text" id="addr" value="192.168.4.1" aria-label="ESP32 address" style="width:150px">
    <button id="wifiBtn">Connect Wi-Fi</button>
  </div>
</div>
<p class="note" id="connHint"></p>

<div class="grid">
<section>
  <div class="card">
    <div class="sub">Machine state</div>
    <div class="state IDLE" id="st">—</div>
    <div class="sub" id="stSub">Connect to see live status</div>
    <div class="bar" aria-hidden="true"><i id="prog"></i></div>
    <div class="flt hide" id="flt"></div>
    <div class="row">
      <button class="pri" data-cmd="run">Run</button>
      <button data-cmd="single">One bolt</button>
      <button data-cmd="finish">Finish &amp; stop</button>
      <button data-cmd="trigger" id="trigBtn">Bolt loaded</button>
      <button data-cmd="reset" id="resetBtn">Reset fault</button>
    </div>
    <div class="verd" id="verd">
      <b>Waiting for a verdict</b> <span class="sub">(the inspector page sends this automatically)</span>
      <div class="row"><button class="okb" data-cmd="v g manual">Good</button><button class="badb" data-cmd="v b manual">Bad</button><button data-cmd="v e manual">Empty</button></div>
    </div>
    <div class="tally">
      <div><b id="nG">0</b><span>Good</span></div>
      <div><b id="nB">0</b><span>Bad</span></div>
      <div><b id="nE">0</b><span>Empty</span></div>
      <div><b id="nT">0</b><span>No verdict</span></div>
    </div>
    <div class="sub" style="margin-top:8px" id="lastV">No bolts sorted yet</div>
    <div class="leds">
      <span id="inBolt">Bolt sensor</span><span id="inLG">Good limit</span><span id="inLB">Bad limit</span><span id="inBtn">SW38</span>
    </div>
    <div class="row"><button data-cmd="tally reset">Clear tally</button></div>
  </div>

  <div class="card">
    <h2>Cycle settings</h2>
    <div class="fields" id="seqFields"></div>
    <p class="note">No bolt sensor: in Run the feeder cycles every "feed gap" and the camera reports empty tubes. Fallback picks the bin when the inspector doesn't answer in time.</p>
  </div>

  <div class="card">
    <h2>Settings storage</h2>
    <div class="row">
      <button class="pri" data-cmd="save" id="saveBtn">Save to flash</button>
      <button data-cmd="load">Reload saved</button>
      <button data-cmd="defaults">Factory defaults</button>
      <span class="dirty hide" id="dirty">Unsaved changes</span>
    </div>
    <p class="note">Changes take effect immediately but are lost at power-off until saved.</p>
  </div>
</section>

<section>
  <div class="card">
    <div class="tabs" role="tablist" id="tabs">
      <button role="tab" data-tab="0" aria-selected="true">Tube</button>
      <button role="tab" data-tab="1" aria-selected="false">Flap</button>
      <button role="tab" data-tab="2" aria-selected="false">Hatch</button>
      <button role="tab" data-tab="3" aria-selected="false">Feeder servo</button>
    </div>

    <div id="axisPane">
      <div class="read">
        <div><span>Angle</span><b id="aA">—</b></div>
        <div><span>Target</span><b id="aT">—</b></div>
        <div><span>Error</span><b id="aE">—</b></div>
        <div><span>PWM</span><b id="aP">—</b></div>
        <div><span>Mode</span><b id="aM">—</b></div>
        <div><span>Counts</span><b id="aC">—</b></div>
      </div>
      <div class="flt hide" id="aF"></div>

      <h3>Move</h3>
      <div class="row">
        <input type="number" id="jogStep" value="30" step="any" aria-label="Step degrees">
        <button data-jog="-1">− step</button><button data-jog="1">+ step</button>
        <button data-ax="halt">Halt</button><button data-ax="zero">Set 0° here</button>
      </div>
      <div class="row">
        <input type="number" id="gotoDeg" placeholder="Go to angle (°)" step="any">
        <button data-ax="goto">Go</button>
      </div>
      <div class="row" id="presetRow"></div>
      <div class="row" id="spinRow">
        <input type="number" id="spinV" value="40" step="any" aria-label="Spin speed deg/s">
        <button data-ax="spin">Spin (°/s)</button><button data-ax="spin0">Stop spin</button>
      </div>

      <h3>Step response test</h3>
      <div class="row">
        <input type="number" id="stepDeg" value="90" step="any" aria-label="Step test distance">
        <button class="pri" data-ax="step">Run step test</button>
      </div>
      <div class="chart" id="chartBox">
        <svg id="posSvg" viewBox="0 0 640 220" role="img" aria-label="Step response: setpoint and measured angle over time"></svg>
        <svg id="pwmSvg" viewBox="0 0 640 110" role="img" aria-label="Motor PWM over time"></svg>
        <div class="tip hide" id="tip"></div>
      </div>
      <div class="legend"><span><i style="--c:var(--s1)"></i>Measured angle</span><span><i class="d" style="--c:var(--s2)"></i>Profiled setpoint</span><span><i style="--c:var(--mut)"></i>PWM</span><span><i style="--c:var(--ok);border-top-width:8px;opacity:.35"></i>Target ± tolerance</span></div>
      <div class="metrics read" id="metrics"></div>
      <p class="note" id="chartNote">Run a step test to see how the loop responds. Overshoot: lower Kp or raise Kd. Slow to arrive or stalls short: raise Kp, Ki or friction PWM.</p>

      <h3>Tuning</h3>
      <div class="fields" id="axFields"></div>

      <details>
        <summary>Wiring check (open-loop)</summary>
        <p class="note">Drives the motor with no feedback for 1 s. Positive PWM must make the counts go up. If the motor spins the wrong way, tick "Reverse motor". If the counts run backwards, tick "Reverse encoder".</p>
        <div class="row">
          <input type="number" id="rawPwm" value="120" min="0" max="255" aria-label="Open-loop PWM">
          <button data-raw="-1">Run −PWM 1 s</button><button data-raw="1">Run +PWM 1 s</button>
        </div>
      </details>
      <details>
        <summary>Calibrate counts per revolution</summary>
        <p class="note">1. Mark the output shaft. 2. Tap Start (the motor goes limp). 3. Turn the shaft exactly one full turn by hand. 4. Tap Done, then Save to flash.</p>
        <div class="row"><button data-ax="calstart">Start</button><button class="pri" data-ax="caldone">Done</button></div>
      </details>
    </div>

    <div id="servoPane" class="hide">
      <div class="read"><div><span>Commanded angle</span><b id="svA">—</b></div></div>
      <h3>Move</h3>
      <div class="row">
        <input type="range" id="svSlider" min="0" max="180" value="0" style="flex:1" aria-label="Servo angle">
        <b id="svVal" style="width:48px;text-align:right">0°</b>
      </div>
      <div class="row"><button data-svgo="load">Go to load</button><button data-svgo="drop">Go to drop</button><button data-cmd="feed">Feed step (toggle)</button></div>
      <h3>Settings</h3>
      <div class="fields" id="svFields"></div>
      <p class="note">Pulse range sets the 0° and 180° ends. Narrow it if the servo buzzes at an end stop.</p>
    </div>
  </div>
</section>
</div>
</div>

<div class="dock">
  <button class="pri" data-cmd="run">Run</button>
  <button data-cmd="stop">Stop</button>
  <button class="stop" data-cmd="estop">E-STOP</button>
</div>
<div class="toast" id="toast" role="status" aria-live="polite"></div>

<script>
// ---------------- SorterLink: one protocol over BLE (Nordic UART) or Wi-Fi (HTTP) ----------------
const NUS = '6e400001-b5a3-f393-e0a9-e50e24dcca9e', NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e', NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
class SorterLink {
  constructor() { this.kind = null; this.h = {}; this.pend = []; this.buf = ''; this.wq = Promise.resolve(); this.cv = -1; this.tid = -1; this.gen = 0; }
  on(ev, f) { (this.h[ev] ||= []).push(f); return this; }
  emit(ev, d) { (this.h[ev] || []).forEach(f => { try { f(d); } catch (e) { console.error(e); } }); }
  get connected() { return !!this.kind; }
  close(why) {
    const was = this.kind; this.kind = null; this.gen++;
    this.pend.splice(0).forEach(p => { clearTimeout(p.t); p.rej(new Error('disconnected')); });
    if (this.dev) { try { this.dev.gatt.disconnect(); } catch {} this.dev = null; }
    if (was) this.emit('close', why || 'Disconnected');
  }
  async connectBLE() {
    if (!navigator.bluetooth) throw new Error('This browser has no Web Bluetooth. Use Chrome or Edge (Android, Mac, Windows, ChromeOS) or the Bluefy app on iPhone, or connect over Wi-Fi.');
    this.close();
    const dev = await navigator.bluetooth.requestDevice({ filters: [{ services: [NUS] }] });
    dev.addEventListener('gattserverdisconnected', () => { if (this.dev === dev) { this.dev = null; this.close('Bluetooth link lost'); } });
    const gatt = await dev.gatt.connect(), svc = await gatt.getPrimaryService(NUS);
    this.rx = await svc.getCharacteristic(NUS_RX); this.tx = await svc.getCharacteristic(NUS_TX);
    this.tx.addEventListener('characteristicvaluechanged', e => this._chunk(new TextDecoder().decode(e.target.value)));
    await this.tx.startNotifications();
    this.dev = dev; this.buf = ''; this.kind = 'ble'; this.cv = -1; this.tid = -1; this.gen++;
    this.emit('open', 'Bluetooth: ' + (dev.name || 'BoltSorter'));
  }
  async connectHTTP(base) {
    this.close();
    base = String(base || '').trim().replace(/\/+$/, '');
    if (!/^https?:\/\//.test(base)) base = 'http://' + base;
    const s = await this._get(base + '/api/status');
    this.base = base; this.kind = 'wifi'; this.cv = -1; this.tid = -1; this.fails = 0; const g = ++this.gen;
    this.emit('open', 'Wi-Fi: ' + base.replace(/^https?:\/\//, ''));
    this._status(s); this._poll(g);
  }
  _get(url) {
    const ac = new AbortController(), t = setTimeout(() => ac.abort(), 3000), o = { cache: 'no-store', signal: ac.signal };
    if (location.protocol === 'https:') o.targetAddressSpace = 'local';   // Chrome local-network access from an https page
    return fetch(url, o).then(r => { if (!r.ok) throw new Error('HTTP ' + r.status); return r.json(); }).finally(() => clearTimeout(t));
  }
  async _poll(g) {
    while (this.kind === 'wifi' && g === this.gen) {
      await new Promise(r => setTimeout(r, 300));
      if (this.kind !== 'wifi' || g !== this.gen) return;
      try { this._status(await this._get(this.base + '/api/status')); this.fails = 0; }
      catch { if (++this.fails >= 5) { this.close('Wi-Fi link lost'); return; } }
    }
  }
  _chunk(s) {
    this.buf += s; let i;
    while ((i = this.buf.indexOf('\n')) >= 0) { const line = this.buf.slice(0, i); this.buf = this.buf.slice(i + 1); if (line) this._msg(line); }
    if (this.buf.length > 20000) this.buf = '';
  }
  _msg(line) {
    let m; try { m = JSON.parse(line); } catch { return; }
    if (m.t === 's') return this._status(m);
    if (m.t === 'c') this.emit('config', m);
    if (m.t === 'tr') this.emit('trace', m);
    const p = this.pend.shift(); if (p) { clearTimeout(p.t); p.res(m); }
  }
  _status(s) {
    this.last = s; this.emit('status', s);
    if (s.cv !== this.cv) { this.cv = s.cv; this.refreshConfig(); }
    if (s.tr === 2 && s.tid !== this.tid) { this.tid = s.tid; this.fetchTrace(); }
  }
  refreshConfig() { return this.send('get').catch(() => {}); }
  fetchTrace() {
    if (this.kind === 'wifi') return this._get(this.base + '/api/trace').then(m => this.emit('trace', m)).catch(() => {});
    return this.send('trace').catch(() => {});
  }
  send(cmd) {
    if (!this.kind) return Promise.reject(new Error('Not connected'));
    if (this.kind === 'wifi') return this._get(this.base + '/api/cmd?c=' + encodeURIComponent(cmd)).then(m => { if (m.t === 'c') this.emit('config', m); this.emit('reply', m); return m; });
    return new Promise((res, rej) => {
      const p = { res: m => { this.emit('reply', m); res(m); }, rej };
      p.t = setTimeout(() => { const i = this.pend.indexOf(p); if (i >= 0) this.pend.splice(i, 1); rej(new Error('No reply')); }, 5000);
      this.pend.push(p);
      const data = new TextEncoder().encode(cmd.replace(/\n/g, ' ') + '\n'), rx = this.rx;
      this.wq = this.wq.then(async () => {
        for (let i = 0; i < data.length; i += 20) { const c = data.slice(i, i + 20); rx.writeValueWithResponse ? await rx.writeValueWithResponse(c) : await rx.writeValue(c); }
      }).catch(e => { const i = this.pend.indexOf(p); if (i >= 0) { this.pend.splice(i, 1); clearTimeout(p.t); rej(e); } });
    });
  }
}

// ---------------- page ----------------
const $ = id => document.getElementById(id);
const link = new SorterLink();
const store = { get(k) { try { return JSON.parse(localStorage.getItem(k)); } catch { return null; } }, set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch {} } };
let S = null, CFG = {}, tab = store.get('bc-tab') || 0, dirty = false, lastSeq = -1;
const AXN = ['t', 'f', 'h'], AXNAME = ['Tube', 'Flap', 'Hatch'];

function toast(msg, err) { const t = $('toast'); t.textContent = msg; t.className = 'toast show' + (err ? ' err' : ''); clearTimeout(toast.t); toast.t = setTimeout(() => t.className = 'toast', 2600); }
async function cmd(c) {
  try { const r = await link.send(c); if (r && r.t === 'r') toast(r.msg, !r.ok); if (/^(set|defaults|load|cal \w+ done)/.test(c)) setDirty(true); if (c === 'save' && r.ok) setDirty(false); if (c === 'load') setDirty(false); return r; }
  catch (e) { toast(e.message, true); }
}
function setDirty(d) { dirty = d; $('dirty').classList.toggle('hide', !d); }

// ----- settings schema (labels live here; firmware has keys + limits) -----
const AXF = [['kp', 'Kp (PWM/°)'], ['ki', 'Ki (PWM/°·s)'], ['kd', 'Kd (PWM per °/s)'], ['ff', 'Friction PWM'], ['vmax', 'Max speed (°/s)'], ['acc', 'Accel (°/s²)'], ['tol', 'Arrive tolerance (°)'], ['maxpwm', 'Max PWM (0–255)'], ['cpr', 'Counts / output rev'], ['inv', 'Reverse motor', 'b'], ['einv', 'Reverse encoder', 'b']];
const AXEXTRA = [
  [['ispd', 'Inspection speed (°/s)'], ['ideg', 'Inspection turn (°)']],
  [['fopen', 'Open angle (°)'], ['fclose', 'Closed angle (°)'], ['fdwell', 'Open dwell (ms)']],
  [['hgood', 'GOOD bin angle (°)'], ['hbad', 'BAD bin angle (°)'], ['hlim', 'Use limit switches', 'b']]];
const SVF = [['svload', 'Load angle (°)'], ['svdrop', 'Drop angle (°)'], ['svspd', 'Speed (°/s)'], ['svhold', 'Hold at end (ms)'], ['svret', 'Return to load after each drop', 'b'], ['svmin', 'Pulse at 0° (µs)'], ['svmax', 'Pulse at 180° (µs)']];
const SEQF = [['settle', 'Settle before inspect (ms)'], ['vto', 'Verdict timeout (ms)'], ['vfb', 'If no verdict', 's', [[0, 'Send to BAD bin'], [1, 'Send to GOOD bin'], [2, 'Wait for manual'], [3, 'Fault']]], ['bsens', 'Wait for bolt sensor', 'b'], ['gap', 'Feed gap without sensor (ms)'], ['mto', 'Move timeout (ms)']];

function fieldHTML([k, label, type, opts]) {
  if (type === 'b') return `<label class="ck"><input type="checkbox" data-k="${k}"> ${label}</label>`;
  if (type === 's') return `<div><label for="f_${k}">${label}</label><select id="f_${k}" data-k="${k}">${opts.map(([v, l]) => `<option value="${v}">${l}</option>`).join('')}</select></div>`;
  return `<div><label for="f_${k}">${label}</label><input type="number" step="any" id="f_${k}" data-k="${k}"></div>`;
}
function renderAxisFields() {
  const p = AXN[tab] + '_';
  $('axFields').innerHTML = [...AXEXTRA[tab], ...AXF.map(([k, l, t]) => [p + k, l, t])].map(fieldHTML).join('');
  fillFields($('axFields'));
}
$('svFields').innerHTML = SVF.map(fieldHTML).join('');
$('seqFields').innerHTML = SEQF.map(fieldHTML).join('');
function fillFields(root) {
  root.querySelectorAll('[data-k]').forEach(el => {
    const v = CFG[el.dataset.k]; if (v === undefined || el === document.activeElement) return;
    if (el.type === 'checkbox') el.checked = v > 0.5; else if (el.tagName === 'SELECT') el.value = String(Math.round(v)); else el.value = +(+v).toPrecision(6);
  });
}
document.addEventListener('change', e => {
  const el = e.target, k = el.dataset && el.dataset.k; if (!k) return;
  const v = el.type === 'checkbox' ? (el.checked ? 1 : 0) : el.value;
  if (v === '' || isNaN(+v)) return;
  cmd(`set ${k}=${v}`);
});
link.on('config', m => { CFG = m.v || {}; document.querySelectorAll('.fields').forEach(fillFields); if (S) updateServoSlider(true); });

// ----- tabs -----
function setTab(i) {
  tab = i; store.set('bc-tab', i);
  document.querySelectorAll('#tabs button').forEach(b => b.setAttribute('aria-selected', +b.dataset.tab === i));
  $('axisPane').classList.toggle('hide', i === 3); $('servoPane').classList.toggle('hide', i !== 3);
  if (i < 3) {
    renderAxisFields();
    $('spinRow').classList.toggle('hide', i !== 0);
    $('presetRow').innerHTML = [
      '<button data-cmd="inspect">Inspection turn</button>',
      '<button data-cmd="flap open">Open</button><button data-cmd="flap close">Close</button>',
      '<button data-cmd="hatch good">GOOD bin</button><button data-cmd="hatch bad">BAD bin</button>'][i];
    drawTrace();
  }
  if (S) render(S);
}
document.querySelectorAll('#tabs button').forEach(b => b.onclick = () => setTab(+b.dataset.tab));

// ----- buttons -----
document.addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  const a = AXN[tab], d = b.dataset;
  if (d.cmd) return cmd(d.cmd);
  if (d.jog) return cmd(`jog ${a} ${(+$('jogStep').value || 0) * +d.jog}`);
  if (d.raw) return cmd(`pwm ${a} ${Math.abs(+$('rawPwm').value || 0) * +d.raw} 1000`);
  if (d.svgo) return cmd(`servo ${CFG[d.svgo === 'load' ? 'svload' : 'svdrop'] ?? 0}`);
  if (d.ax) {
    const m = { halt: `halt ${a}`, zero: `zero ${a}`, goto: `goto ${a} ${$('gotoDeg').value}`, spin: `spin ${a} ${$('spinV').value}`, spin0: `spin ${a} 0`, step: `step ${a} ${$('stepDeg').value}`, calstart: `cal ${a} start`, caldone: `cal ${a} done` }[d.ax];
    if (d.ax === 'goto' && $('gotoDeg').value === '') return;
    return cmd(m);
  }
});
$('svSlider').oninput = () => { $('svVal').textContent = $('svSlider').value + '°'; };
$('svSlider').onchange = () => cmd(`servo ${$('svSlider').value}`);
function updateServoSlider(force) { if (document.activeElement !== $('svSlider') || force) { $('svSlider').value = Math.round(S.sv); $('svVal').textContent = Math.round(S.sv) + '°'; } }

// ----- status render -----
const f1 = v => (v == null ? '—' : (+v).toFixed(1));
function setLed(id, on) { $(id).classList.toggle('on', !!on); }
function render(s) {
  S = s;
  const st = $('st'); st.textContent = s.st.replace('_', ' '); st.className = 'state ' + s.st;
  const runTxt = ['stopped', 'running', 'finishing this bolt'][s.run] || '';
  $('stSub').textContent = `Bolt #${s.cyc} · ${runTxt} · ${(s.el / 1000).toFixed(1)} s in state`;
  $('prog').style.width = (s.prog * 100).toFixed(1) + '%';
  $('flt').classList.toggle('hide', !s.flt); $('flt').textContent = s.flt;
  $('verd').classList.toggle('show', (s.st === 'INSPECT' || s.st === 'AWAIT_VERDICT') && !s.v);
  $('trigBtn').disabled = s.st !== 'WAIT_BOLT'; $('resetBtn').disabled = s.st !== 'FAULT';
  [['nG', 0], ['nB', 1], ['nE', 2], ['nT', 3]].forEach(([id, i]) => $(id).textContent = s.n[i]);
  $('lastV').textContent = s.lv ? `Last bolt: ${{ g: 'GOOD', b: 'BAD', e: 'EMPTY' }[s.lv]}${s.lr ? ' — ' + s.lr : ''}` : 'No bolts sorted yet';
  setLed('inBolt', s.in[0]); setLed('inLG', s.in[1]); setLed('inLB', s.in[2]); setLed('inBtn', s.in[3]);
  if (tab < 3) {
    const a = s.ax[tab];
    $('aA').textContent = f1(a.a) + '°'; $('aT').textContent = f1(a.t) + '°'; $('aE').textContent = f1(a.t - a.a) + '°';
    $('aP').textContent = a.p; $('aM').textContent = a.m; $('aC').textContent = a.c;
    $('aF').classList.toggle('hide', !a.f); $('aF').textContent = a.f;
  } else { $('svA').textContent = f1(s.sv) + '°'; updateServoSlider(); }
  if (s.ms !== lastSeq) { lastSeq = s.ms; }
}
link.on('status', render);
link.on('open', name => { $('linkChip').textContent = name; $('linkChip').classList.add('on'); $('connHint').textContent = ''; });
link.on('close', why => { $('linkChip').textContent = why; $('linkChip').classList.remove('on'); });

// ----- step response chart -----
let TR = store.get('bc-trace');
link.on('trace', m => { if (!m.n) return; TR = m; store.set('bc-trace', m); drawTrace(); });
const NS = 'http://www.w3.org/2000/svg';
function svgEl(tag, attrs, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); parent && parent.appendChild(e); return e; }
function niceTicks(lo, hi, n) { const span = hi - lo || 1, step0 = span / n, mag = 10 ** Math.floor(Math.log10(step0)), step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => s >= step0); const out = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(6)); return out; }
function drawTrace() {
  const pos = $('posSvg'), pw = $('pwmSvg'); pos.innerHTML = ''; pw.innerHTML = '';
  const tr = TR && TR.n && TR.ax === ['tube', 'flap', 'hatch'][tab] ? TR : null;
  $('metrics').innerHTML = '';
  if (!tr) { svgEl('text', { x: 320, y: 110, 'text-anchor': 'middle', fill: 'var(--mut)', 'font-size': 14 }, pos).textContent = `No step test for the ${AXNAME[tab].toLowerCase()} yet`; return; }
  const n = tr.n, dt = tr.dt / 1000, T = i => i * dt, sp = tr.sp.map(v => v / 10), a = tr.a.map(v => v / 10), p = tr.p, tgt = tr.tgt, tol = tr.tol;
  const L = 46, R = 630, W = R - L;
  const lo = Math.min(0, tgt, ...a, ...sp), hi = Math.max(0, tgt, ...a, ...sp), pad = (hi - lo) * 0.08 || 1;
  const yT = 12, yB = 196, Y = v => yB - (v - (lo - pad)) / (hi - lo + 2 * pad) * (yB - yT), X = t => L + t / T(n - 1) * W;
  const xt = niceTicks(0, T(n - 1), 6);
  const axes = (svg, yt, Yf, yb, fmt) => {
    yt.forEach(v => { svgEl('line', { x1: L, x2: R, y1: Yf(v), y2: Yf(v), stroke: 'var(--grid)', 'stroke-width': 1 }, svg); svgEl('text', { x: L - 6, y: Yf(v) + 4, 'text-anchor': 'end', fill: 'var(--mut)', 'font-size': 11 }, svg).textContent = fmt(v); });
    xt.forEach(t => svgEl('text', { x: X(t), y: yb + 16, 'text-anchor': 'middle', fill: 'var(--mut)', 'font-size': 11 }, svg).textContent = t.toFixed(1) + ' s');
  };
  axes(pos, niceTicks(lo - pad, hi + pad, 5), Y, yB, v => v + '°');
  svgEl('rect', { x: L, width: W, y: Y(tgt + tol), height: Math.max(2, Y(tgt - tol) - Y(tgt + tol)), fill: 'var(--band)' }, pos);
  const path = (vals, Yf) => vals.map((v, i) => (i ? 'L' : 'M') + X(T(i)).toFixed(1) + ' ' + Yf(v).toFixed(1)).join('');
  svgEl('path', { d: path(sp, Y), fill: 'none', stroke: 'var(--s2)', 'stroke-width': 2, 'stroke-dasharray': '6 4' }, pos);
  svgEl('path', { d: path(a, Y), fill: 'none', stroke: 'var(--s1)', 'stroke-width': 2 }, pos);
  const pmax = Math.max(60, ...p.map(Math.abs)), yT2 = 8, yB2 = 86, Y2 = v => yB2 - (v + pmax) / (2 * pmax) * (yB2 - yT2);
  axes(pw, niceTicks(-pmax, pmax, 2), Y2, yB2, v => String(v));
  svgEl('path', { d: path(p, Y2), fill: 'none', stroke: 'var(--mut)', 'stroke-width': 1.5 }, pw);
  // metrics
  const err = a.map(v => tgt - v), dir = Math.sign(tgt) || 1, peak = dir > 0 ? Math.max(...a) : Math.min(...a);
  const over = Math.max(0, (peak - tgt) * dir), overPct = Math.abs(tgt) > 0 ? over / Math.abs(tgt) * 100 : 0;
  const i10 = a.findIndex(v => v * dir >= 0.1 * tgt * dir), i90 = a.findIndex(v => v * dir >= 0.9 * tgt * dir);
  let settle = -1; for (let i = n - 1; i >= 0; i--) if (Math.abs(err[i]) > tol) { settle = i + 1 < n ? i + 1 : -1; break; } if (settle === -1 && Math.abs(err[0]) <= tol) settle = 0;
  const M = [['Overshoot', `${over.toFixed(2)}° (${overPct.toFixed(1)}%)`], ['Rise 10–90%', i10 >= 0 && i90 >= 0 ? `${((i90 - i10) * dt).toFixed(2)} s` : '—'], ['Settled (±tol)', settle >= 0 ? `${(settle * dt).toFixed(2)} s` : 'not in 2.5 s'], ['Final error', `${err[n - 1].toFixed(2)}°`], ['Peak PWM', String(Math.max(...p.map(Math.abs)))]];
  $('metrics').innerHTML = M.map(([k, v]) => `<div><span>${k}</span><b>${v}</b></div>`).join('');
  // hover crosshair
  const cross = [pos, pw].map(svg => svgEl('line', { y1: 0, y2: svg === pos ? yB : yB2, stroke: 'var(--mut)', 'stroke-width': 1, visibility: 'hidden' }, svg));
  const dot = svgEl('circle', { r: 4, fill: 'var(--s1)', stroke: 'var(--bg)', 'stroke-width': 2, visibility: 'hidden' }, pos);
  const tip = $('tip'), box = $('chartBox');
  const move = ev => {
    const r = pos.getBoundingClientRect(), sx = (ev.clientX - r.left) / r.width * 640;
    const i = Math.max(0, Math.min(n - 1, Math.round((sx - L) / W * (n - 1))));
    cross.forEach(c => { c.setAttribute('x1', X(T(i))); c.setAttribute('x2', X(T(i))); c.setAttribute('visibility', 'visible'); });
    dot.setAttribute('cx', X(T(i))); dot.setAttribute('cy', Y(a[i])); dot.setAttribute('visibility', 'visible');
    tip.innerHTML = `<b>${T(i).toFixed(2)} s</b><br>Angle ${a[i].toFixed(1)}° · Setpoint ${sp[i].toFixed(1)}°<br>PWM ${p[i]}`;
    tip.classList.remove('hide');
    const bx = box.getBoundingClientRect(), px = ev.clientX - bx.left;
    tip.style.left = Math.min(px + 12, bx.width - tip.offsetWidth - 4) + 'px'; tip.style.top = '8px';
  };
  const leave = () => { cross.forEach(c => c.setAttribute('visibility', 'hidden')); dot.setAttribute('visibility', 'hidden'); tip.classList.add('hide'); };
  [pos, pw].forEach(svg => { svg.onpointermove = move; svg.onpointerleave = leave; });
}

// ----- connecting -----
$('btBtn').onclick = async () => { try { $('btBtn').disabled = true; await link.connectBLE(); } catch (e) { if (e.name !== 'NotFoundError') toast(e.message, true); } finally { $('btBtn').disabled = false; } };
$('wifiBtn').onclick = async () => {
  const a = $('addr').value; store.set('bc-addr', a);
  try { $('wifiBtn').disabled = true; await link.connectHTTP(a); }
  catch (e) {
    toast('Could not reach ' + a, true);
    $('connHint').textContent = location.protocol === 'https:' ? 'This page is on https, so most browsers block plain-http Wi-Fi requests to the ESP32. Use Connect Bluetooth here, or open http://' + a + ' directly.' : 'Join the "BoltSorter" Wi-Fi (password bolts1420), or the shared network set in the sketch, then try again.';
  } finally { $('wifiBtn').disabled = false; }
};
$('addr').value = store.get('bc-addr') || '192.168.4.1';
setTab(tab);
// served by the ESP32 itself -> connect over Wi-Fi automatically
if (location.protocol === 'http:' && !/^(localhost|127\.)/.test(location.hostname) && location.hostname) {
  $('addr').value = location.host;
  link.connectHTTP(location.origin).catch(() => {});
}
</script>
</body>
</html>
)PAGE";
// <<CONTROL_PAGE_END>>
