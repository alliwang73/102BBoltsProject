// =====================================================================
//  MOTOR LAB  —  dual gearmotor controller with a phone/laptop web app
//  Board : Adafruit Feather ESP32 V2   (Tools > Board > "Adafruit Feather ESP32 V2")
//  Core  : esp32 by Espressif, version 3.x
//  Driver: one DRV8833 — channel A = Motor 1, channel B = Motor 2
//
//  Motor 1 "Indexer": closed-loop step moves (default +120 deg per press)
//  Motor 2 "Spinner": slow, closed-loop full turn (default 1 turn at 15 deg/s)
//                     with anti-stall: if it stops making progress it
//                     rocks back/forward to break free, then faults if jammed
//
//  App   : join Wi-Fi "MotorCtrl" (password rotate120), open http://192.168.4.1
//  SW38  : tap = Indexer step, hold 0.7 s = Spinner turn
//  Serial: 115200.  a<deg> / b<deg> absolute, + step, r spin, s stop, z zero
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "This sketch needs the ESP32 Arduino core 3.x. Update 'esp32 by Espressif' in Boards Manager."
#endif

#define ENCODER_TEST 0   // 1 = motors off, print both encoders for a hand-spin test

// ----------------------------- WI-FI -----------------------------
const char* AP_SSID   = "MotorCtrl";
const char* AP_PASS   = "rotate120";        // 8+ characters
const char* MDNS_NAME = "motors";           // also reachable at http://motors.local
const IPAddress AP_IP(192, 168, 4, 1);

// ----------------------------- PINS ------------------------------
struct MotorPins { uint8_t in1, in2, encA, encB; };
const MotorPins PINS[2] = {
  {26, 25, 27, 33},   // M1: A0->AIN1, A1->AIN2, encoder A->27, B->33  (unchanged)
  {19, 21, 22, 32},   // M2: MO->BIN1, MI->BIN2, encoder A->22 (SDA), B->32   <-- match YOUR wiring
};
const uint8_t  BTN_PIN  = 38;               // on-board SW38, LOW when pressed
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 8;
const uint32_t DUTY_FULL = 1U << PWM_RES;   // 256

// ------------------------ CONTROL CONSTANTS ----------------------
const uint32_t CTRL_PERIOD_MS   = 2;        // 500 Hz control loop (own RTOS task)
const float    I_LIMIT          = 80.0f;    // max PWM from the integral term
const float    FF_RAMP          = 8.0f;     // friction comp fades in over this much |u|
const float    TOL_DEG          = 0.5f;     // arrived tolerance
const float    SETTLE_VEL       = 8.0f;     // deg/s, must be slower than this
const uint32_t SETTLE_MS        = 150;      // ...for this long
const float    CLOSE_ENOUGH_DEG = 3.0f;     // after profile ends, accept this after...
const uint32_t CLOSE_ENOUGH_MS  = 4000;     // ...this long (stops endless hunting)
const float    REENGAGE_DEG     = 1.5f;     // pushed this far while holding -> correct
const float    MAX_LAG_DEG      = 20.0f;    // setpoint never runs further ahead than this
// anti-stall
const float    STALL_ERR_DEG    = 3.0f;     // behind by this much...
const uint32_t STALL_MS         = 300;      // ...without real progress this long = stuck
const float    PROGRESS_DEG     = 1.0f;     // "real progress" = this far toward the target
const uint16_t JITTER_COUNTS    = 150;      // hundreds of count flips with no progress = bad encoder
const uint8_t  MAX_KICKS        = 4;        // failed unstick attempts before faulting
const uint32_t KICK_REV_MS      = 40;       // rock back...
const uint32_t KICK_FWD_MS      = 120;      // ...then shove forward
const int      KICK_REV_PWM     = 150;      // short bursts may exceed Max PWM to break free,
const int      KICK_FWD_PWM     = 255;      // and end the moment the shaft moves KICK_FREE_DEG
const float    KICK_FREE_DEG    = 2.0f;     // stop the shove as soon as it moves this much
const uint32_t WRONG_WAY_MS     = 250;      // driving one way, moving the other this long = fault

// ----------------------------- TYPES -----------------------------
enum MState : uint8_t { M_HOLD, M_MOVING, M_CAL, M_FAULT };

struct Params {
  float cpr;        // encoder counts per OUTPUT shaft revolution
  float kp, ki, kd; // PWM per deg, per deg*s, per deg/s
  float vmax, acc;  // deg/s, deg/s^2
  float ff;         // PWM to overcome friction
  float maxPwm;     // hard cap on drive strength (also caps top speed)
  float step;       // degrees per button press / spin
  bool  flipMotor, flipEnc;
};

struct Motor {
  Params   p;
  MState   st = M_HOLD;
  float    angle = 0, lastAngle = 0, vel = 0;
  float    target = 0, sp = 0, spVel = 0, integ = 0, holdAngle = 0, moveStart = 0;
  int      pwm = 0;
  bool     counted = false;
  uint32_t moveT0 = 0, settledSince = 0, profDoneSince = 0, wrongWaySince = 0;
  int32_t  lastCount = 0;
  uint32_t lastCountMs = 0;
  float    progRef = 0;
  uint32_t progMs = 0;
  uint16_t jitter = 0;
  uint8_t  boost = 0;       // how hard it's allowed to push after struggling
  bool     everMoved = false;
  uint8_t  kicks = 0, kickPhase = 0;
  int8_t   kickDir = 1;
  uint32_t kickUntil = 0, kickEndMs = 0;
  float    kickRef = 0;
  uint32_t moves = 0, lastMoveMs = 0, totalKicks = 0;
  float    lastErr = 0;
  char     fault[110] = "";
};

struct Enc { volatile int32_t count; volatile uint8_t last; uint8_t a, b; };

// ----------------------------- GLOBALS ---------------------------
Motor M[2];
Enc   enc[2];
SemaphoreHandle_t mtx;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
WebServer   server(80);
DNSServer   dns;
Preferences prefs;

inline void lock()   { xSemaphoreTake(mtx, portMAX_DELAY); }
inline void unlock() { xSemaphoreGive(mtx); }

void setDefaults() {
  //            cpr          kp     ki     kd    vmax   acc     ff    maxPwm  step   flipM  flipE
  M[0].p = { 12.0f*75.81f, 8.0f,  25.0f, 0.15f, 60.0f, 120.0f, 40.0f, 160.0f, 120.0f, false, false };  // 120 deg in ~2.5 s
  M[1].p = { 12.0f*75.81f, 10.0f, 30.0f, 0.10f, 15.0f,  30.0f, 40.0f, 130.0f, 360.0f, false, false };  // 1 turn in ~24 s
}

// ----------------------------- ENCODERS --------------------------
DRAM_ATTR static const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

void IRAM_ATTR encISR(void* arg) {
  Enc* e = (Enc*)arg;
  uint8_t s = (digitalRead(e->a) << 1) | digitalRead(e->b);
  portENTER_CRITICAL_ISR(&encMux);
  e->count += QEM[(e->last << 2) | s];
  e->last = s;
  portEXIT_CRITICAL_ISR(&encMux);
}

int32_t readCount(int i) {
  portENTER_CRITICAL(&encMux);
  int32_t c = enc[i].count;
  portEXIT_CRITICAL(&encMux);
  return M[i].p.flipEnc ? -c : c;
}
void zeroCount(int i) {
  portENTER_CRITICAL(&encMux);
  enc[i].count = 0;
  portEXIT_CRITICAL(&encMux);
}
inline float cpd(int i) { return M[i].p.cpr / 360.0f; }
inline uint32_t stamp(uint32_t now) { return now ? now : 1; }        // 0 means "not set"
inline int32_t  since(uint32_t now, uint32_t t) { return (int32_t)(now - t); }

// ----------------------------- MOTOR OUTPUT ----------------------
// DRV8833 slow-decay drive (drive/brake chopping): much smoother at low speed
// than drive/coast, which is what keeps the slow Spinner from stuttering.
void driveRaw(int i, int pwm) {
  const MotorPins& P = PINS[i];
  if (M[i].p.flipMotor) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);
  // 256 = pin solid high (255 would only be 99.6% on this core)
  uint32_t chop = DUTY_FULL - (uint32_t)abs(pwm) * DUTY_FULL / 255;   // high time of the chopping pin
  if (pwm > 0)      { ledcWrite(P.in1, DUTY_FULL); ledcWrite(P.in2, chop); }
  else if (pwm < 0) { ledcWrite(P.in1, chop);      ledcWrite(P.in2, DUTY_FULL); }
  else              { ledcWrite(P.in1, DUTY_FULL); ledcWrite(P.in2, DUTY_FULL); }  // brake
}
void coast(int i) { ledcWrite(PINS[i].in1, 0); ledcWrite(PINS[i].in2, 0); }

// ----------------------------- MOTION API (call with lock held) --
void beginMove(int i, float tgt, bool counted) {
  Motor& m = M[i];
  if (m.st == M_CAL) return;
  uint32_t now = millis();
  if (m.st != M_MOVING) {
    m.sp = m.angle; m.spVel = 0; m.integ = 0;
    m.moveStart = m.angle; m.moveT0 = now;
    m.kicks = 0; m.kickPhase = 0;
    m.lastCount = readCount(i); m.lastCountMs = now;
    m.progRef = m.angle; m.progMs = now; m.jitter = 0;
    m.boost = 0; m.everMoved = false;
    m.settledSince = m.profDoneSince = m.wrongWaySince = 0;
    m.counted = false;
  }
  if (counted) m.counted = true;
  m.target = tgt;
  m.fault[0] = 0;
  m.st = M_MOVING;
}

void stopMotor(int i) {
  Motor& m = M[i];
  if (m.st == M_CAL) return;
  m.target = m.sp = m.holdAngle = m.angle;
  m.spVel = 0; m.integ = 0; m.kickPhase = 0; m.counted = false;
  m.fault[0] = 0;
  m.st = M_HOLD;
}

void relMove(int i, float d) {
  if (M[i].st == M_FAULT) stopMotor(i);   // start from where it really is
  beginMove(i, M[i].target + d, true);
}

void zeroMotor(int i) {
  Motor& m = M[i];
  zeroCount(i);
  m.angle = m.lastAngle = m.target = m.sp = m.holdAngle = m.moveStart = 0;
  m.spVel = 0; m.integ = 0; m.kickPhase = 0; m.counted = false;
  m.fault[0] = 0;
  if (m.st != M_CAL) m.st = M_HOLD;
}

void rebaseMotor(int i) {   // after cpr change: keep physical position, recompute degrees
  Motor& m = M[i];
  m.angle = m.lastAngle = readCount(i) / cpd(i);
  if (m.st != M_CAL) stopMotor(i);
}

void setFault(int i, const char* msg) {
  Motor& m = M[i];
  m.st = M_FAULT; m.pwm = 0; m.integ = 0; m.kickPhase = 0; m.counted = false;
  strncpy(m.fault, msg, sizeof(m.fault) - 1);
  m.fault[sizeof(m.fault) - 1] = 0;
  driveRaw(i, 0);
}

void finishMove(int i, uint32_t now, float finalErr) {
  Motor& m = M[i];
  m.st = M_HOLD; m.integ = 0; m.kicks = 0;
  m.settledSince = m.profDoneSince = 0;
  m.holdAngle = m.angle;
  if (m.counted) { m.moves++; m.lastMoveMs = now - m.moveT0; m.lastErr = finalErr; m.counted = false; }
  m.pwm = 0;
  driveRaw(i, 0);
}

// ----------------------------- CONTROL LOOP ----------------------
void controlStep(int i, float dt, uint32_t now) {
  Motor& m = M[i];
  int32_t c = readCount(i);
  m.angle = c / cpd(i);
  float vRaw = (m.angle - m.lastAngle) / dt;
  m.lastAngle = m.angle;
  m.vel += 0.1f * (vRaw - m.vel);

  if (m.st == M_CAL)   { coast(i); m.pwm = 0; return; }
  if (m.st == M_FAULT) { driveRaw(i, 0); m.pwm = 0; return; }
  if (m.st == M_HOLD) {
    driveRaw(i, 0); m.pwm = 0;
    if (fabsf(m.angle - m.holdAngle) > REENGAGE_DEG) beginMove(i, m.target, false);  // got bumped
    return;
  }

  // ---- unstick kick in progress: rock back, then shove forward ----
  if (m.kickPhase) {
    bool freed = m.kickPhase == 2 && fabsf(m.angle - m.kickRef) > KICK_FREE_DEG;  // moving again: stop shoving
    if (freed && m.kicks) m.kicks--;     // a kick that worked doesn't use up the budget
    if (!freed && since(now, m.kickUntil) < 0) {
      int k = (m.kickPhase == 1 ? -KICK_REV_PWM : KICK_FWD_PWM) * m.kickDir;
      driveRaw(i, k); m.pwm = k; return;
    }
    if (!freed && m.kickPhase == 1) {
      m.kickPhase = 2; m.kickUntil = now + KICK_FWD_MS;
      m.kickRef = m.angle;
      int k = KICK_FWD_PWM * m.kickDir;
      driveRaw(i, k); m.pwm = k; return;
    }
    m.kickPhase = 0; m.kickEndMs = stamp(now);
    m.lastCount = c; m.lastCountMs = now;
    m.progRef = m.angle; m.progMs = now; m.jitter = 0;
  }

  // ---- 1) trapezoidal motion profile for the setpoint ----
  float dist  = m.target - m.sp;
  float vDes  = copysignf(fminf(m.p.vmax, sqrtf(2.0f * m.p.acc * fabsf(dist))), dist);
  float maxDv = m.p.acc * dt;
  m.spVel += constrain(vDes - m.spVel, -maxDv, maxDv);
  m.sp    += m.spVel * dt;
  bool profDone = false;
  if (fabsf(m.target - m.sp) < 0.05f && fabsf(m.spVel) <= 2.0f * maxDv) {
    m.sp = m.target; m.spVel = 0; profDone = true;
  }
  // if the motor falls behind (sticky spot), the setpoint waits for it instead of
  // running away, so there's no lurch when it breaks free
  float lag = m.sp - m.angle;
  bool  aheadOfMotor = (lag > 0) == (m.target - m.angle > 0);
  if (aheadOfMotor && fabsf(lag) > MAX_LAG_DEG) m.sp = m.angle + copysignf(MAX_LAG_DEG, lag);

  // ---- 2) PID + friction compensation ----
  float e  = m.sp - m.angle;
  float ve = m.spVel - m.vel;
  if (m.p.ki > 0) {
    m.integ += e * dt;
    float lim = I_LIMIT / m.p.ki;
    m.integ = constrain(m.integ, -lim, lim);
  } else m.integ = 0;
  float u   = m.p.kp * e + m.p.ki * m.integ + m.p.kd * ve;
  float ffs = m.p.ff * fminf(1.0f, fabsf(u) / FF_RAMP);
  // normal cap is Max PWM; each unstick try lets it push a bit harder
  int lim   = (int)fminf(255.0f, m.p.maxPwm + m.boost * (255.0f - m.p.maxPwm) / MAX_KICKS);
  int pwm   = constrain((int)(u + copysignf(ffs, u)), -lim, lim);

  // ---- 3) arrived? (inside tolerance AND still, for SETTLE_MS) ----
  float fe = m.target - m.angle;
  if (profDone) {
    if (!m.profDoneSince) m.profDoneSince = stamp(now);
    if (fabsf(fe) < TOL_DEG && fabsf(m.vel) < SETTLE_VEL) {
      if (!m.settledSince) m.settledSince = stamp(now);
      if (since(now, m.settledSince) > (int32_t)SETTLE_MS) { finishMove(i, now, fe); return; }
    } else m.settledSince = 0;
    if (since(now, m.profDoneSince) > (int32_t)CLOSE_ENOUGH_MS && fabsf(fe) < CLOSE_ENOUGH_DEG) {
      finishMove(i, now, fe); return;
    }
  } else { m.profDoneSince = 0; m.settledSince = 0; }

  // ---- 4) anti-stall, based on real progress toward the target (not just "any count") ----
  if (c != m.lastCount) { m.lastCount = c; m.lastCountMs = now; if (m.jitter < 60000) m.jitter++; }
  float want = (e >= 0) ? 1.0f : -1.0f;
  if (fabsf(m.angle - m.moveStart) > 2.0f * PROGRESS_DEG) m.everMoved = true;
  if (fabsf(e) < STALL_ERR_DEG || (m.angle - m.progRef) * want > PROGRESS_DEG) {
    m.progRef = m.angle; m.progMs = now; m.jitter = 0;          // tracking fine / making progress
  } else if (since(now, m.progMs) > (int32_t)STALL_MS) {
    if (!m.everMoved && m.kicks >= 1) {   // never moved at all, even after a full-power burst
      setFault(i, "Motor was driven but the encoder saw no motion. Check the encoder plug, 3V and GND, or raise Max PWM if the motor can't start.");
      return;
    }
    if (m.everMoved && m.jitter > JITTER_COUNTS) {
      setFault(i, "Encoder counts flicker but the angle never changes: one encoder channel is dead or noisy. Check OUT A/B, 3V and GND.");
      return;
    }
    if (m.kicks < MAX_KICKS) {
      m.kicks++; m.totalKicks++;
      if (m.boost < MAX_KICKS) m.boost++;
      m.kickDir = (e > 0) ? 1 : -1;
      m.kickRef = m.angle;
      m.kickPhase = 1; m.kickUntil = now + KICK_REV_MS;
      int k = -KICK_REV_PWM * m.kickDir;
      driveRaw(i, k); m.pwm = k;
      return;
    }
    setFault(i, "Not moving: no progress after 4 unstick tries. Check that the encoder is plugged in and powered, or for a jam.");
    return;
  }
  if (fabsf(e) < 1.0f) { m.kicks = 0; m.boost = 0; }   // tracking again -> back to normal

  // ---- 4b) move taking far too long (catches anything else) ----
  {
    float span    = fabsf(m.target - m.moveStart);
    float expectS = span / m.p.vmax + 2.0f * m.p.vmax / m.p.acc + 2.0f;
    if (since(now, m.moveT0) > (int32_t)(1000.0f * (2.0f * expectS + 5.0f))) {
      setFault(i, "Move timed out: took far longer than the speed setting allows. Check encoder wiring and the Counts per rev setting.");
      return;
    }
  }

  // ---- 5) wrong direction (motor vs encoder reversed) ----
  bool recentKick = m.kickEndMs && since(now, m.kickEndMs) < 500;
  if (!recentKick && abs(pwm) > m.p.ff + 30 && fabsf(m.vel) > 60 && ((pwm > 0) != (m.vel > 0))) {
    if (!m.wrongWaySince) m.wrongWaySince = stamp(now);
    if (since(now, m.wrongWaySince) > (int32_t)WRONG_WAY_MS) {
      setFault(i, "Motor and encoder disagree on direction. Open Tuning and tick Flip motor direction.");
      return;
    }
  } else m.wrongWaySince = 0;

  m.pwm = pwm;
  driveRaw(i, pwm);
}

void controlTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastUs = micros();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    uint32_t nowUs = micros();
    float dt = constrain((nowUs - lastUs) * 1e-6f, 0.0005f, 0.02f);
    lastUs = nowUs;
    uint32_t now = millis();
    lock();
    controlStep(0, dt, now);
    controlStep(1, dt, now);
    unlock();
  }
}

// ----------------------------- SETTINGS (flash) ------------------
const char* NS[2] = {"lab2_m1", "lab2_m2"};   // new names = new slow defaults load
void loadParams(int i) {
  Params& p = M[i].p;
  prefs.begin(NS[i], true);
  p.cpr  = prefs.getFloat("cpr", p.cpr);   p.kp = prefs.getFloat("kp", p.kp);
  p.ki   = prefs.getFloat("ki", p.ki);     p.kd = prefs.getFloat("kd", p.kd);
  p.vmax = prefs.getFloat("vmax", p.vmax); p.acc = prefs.getFloat("acc", p.acc);
  p.ff   = prefs.getFloat("ff", p.ff);     p.step = prefs.getFloat("step", p.step);
  p.maxPwm = prefs.getFloat("mp", p.maxPwm);
  p.flipMotor = prefs.getBool("fm", p.flipMotor);
  p.flipEnc   = prefs.getBool("fe", p.flipEnc);
  prefs.end();
}
void saveParams(int i, const Params& p) {
  prefs.begin(NS[i], false);
  prefs.putFloat("cpr", p.cpr);   prefs.putFloat("kp", p.kp);   prefs.putFloat("ki", p.ki);
  prefs.putFloat("kd", p.kd);     prefs.putFloat("vmax", p.vmax); prefs.putFloat("acc", p.acc);
  prefs.putFloat("ff", p.ff);     prefs.putFloat("step", p.step);   prefs.putFloat("mp", p.maxPwm);
  prefs.putBool("fm", p.flipMotor); prefs.putBool("fe", p.flipEnc);
  prefs.end();
}

// ----------------------------- COMBO SEQUENCER -------------------
enum SeqSt : uint8_t { SQ_IDLE, SQ_M1, SQ_M2, SQ_BOTH, SQ_PAUSE };
struct {
  SeqSt st = SQ_IDLE; bool both = false;
  int total = 1, done = 0; float dwell = 1.0f; uint32_t t = 0;
  char msg[64] = "";
} seq;

void seqStop(const char* why) {      // lock held
  if (seq.st != SQ_IDLE && why) snprintf(seq.msg, sizeof(seq.msg), "%s", why);
  seq.st = SQ_IDLE;
}
void seqCycle() {                    // lock held
  relMove(0, M[0].p.step);
  if (seq.both) { relMove(1, M[1].p.step); seq.st = SQ_BOTH; }
  else seq.st = SQ_M1;
}
void seqTick() {                     // lock held
  if (seq.st == SQ_IDLE) return;
  if (M[0].st == M_FAULT || M[1].st == M_FAULT) { seqStop("Combo stopped: motor fault"); return; }
  if (M[0].st == M_CAL   || M[1].st == M_CAL)   { seqStop("Combo stopped: calibrating"); return; }
  bool cycleDone = false;
  switch (seq.st) {
    case SQ_M1:    if (M[0].st == M_HOLD) { relMove(1, M[1].p.step); seq.st = SQ_M2; } break;
    case SQ_M2:    if (M[1].st == M_HOLD) cycleDone = true; break;
    case SQ_BOTH:  if (M[0].st == M_HOLD && M[1].st == M_HOLD) cycleDone = true; break;
    case SQ_PAUSE: if (millis() - seq.t >= (uint32_t)(seq.dwell * 1000)) seqCycle(); break;
    default: break;
  }
  if (cycleDone) {
    seq.done++;
    if (seq.total > 0 && seq.done >= seq.total) {
      snprintf(seq.msg, sizeof(seq.msg), "Done: %d cycle%s", seq.done, seq.done == 1 ? "" : "s");
      seq.st = SQ_IDLE;
    } else { seq.st = SQ_PAUSE; seq.t = millis(); }
  }
}

// ----------------------------- WEB APP ---------------------------
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0e1116"><title>Motor Lab</title>
<style>
:root{--bg:#0e1116;--card:#161b22;--line:#262d36;--tx:#e6edf3;--mut:#8b949e;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--cal:#58a6ff}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.4 -apple-system,system-ui,Segoe UI,Roboto,sans-serif}
header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:10px;padding:10px 16px;background:rgba(14,17,22,.92);-webkit-backdrop-filter:blur(8px);backdrop-filter:blur(8px);border-bottom:1px solid var(--line)}
header h1{font-size:17px;margin:0;flex:1}
#conn{color:var(--mut);font-size:12px;white-space:nowrap}#dot{width:9px;height:9px;border-radius:50%;background:var(--mut);display:inline-block;margin-right:6px}
#estop{flex:0 0 auto;background:var(--bad);color:#fff;border:0;border-radius:10px;padding:10px 18px;font-weight:800;letter-spacing:.5px}
main{max-width:1000px;margin:0 auto;padding:12px 16px 40px;display:grid;gap:12px;grid-template-columns:minmax(0,1fr)}
@media(min-width:760px){main{grid-template-columns:minmax(0,1fr) minmax(0,1fr)}.wide{grid-column:1/-1}}
.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:14px}
.hd{display:flex;align-items:baseline;gap:8px;margin-bottom:6px}.hd b{font-size:16px}.hd small{color:var(--mut)}
.chip{margin-left:auto;font-size:11px;font-weight:700;letter-spacing:.6px;padding:3px 9px;border-radius:999px;background:#21262d;color:var(--mut)}
.chip.HOLD{color:var(--ok)}.chip.MOVING,.chip.RUNNING,.chip.PAUSE{color:var(--warn)}.chip.FAULT{color:#fff;background:var(--bad)}.chip.CAL{color:var(--cal)}
.top{display:flex;gap:14px;align-items:center}
svg.dial{width:150px;height:150px;flex:0 0 150px}
.big{font-size:38px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.05}
.row{color:var(--mut);font-size:13px;font-variant-numeric:tabular-nums;margin-top:4px}.row span{color:var(--tx)}
.bar{height:6px;background:#21262d;border-radius:4px;overflow:hidden;margin:12px 0 4px}.bar i{display:block;height:100%;width:0;transition:width .15s linear}
.flt{color:var(--bad);font-size:13px}
.btns{display:flex;gap:8px;margin-top:8px}
button{font:inherit;color:var(--tx);background:#21262d;border:1px solid var(--line);border-radius:12px;padding:13px 10px;flex:1;cursor:pointer;touch-action:manipulation}
button:active{transform:scale(.97)}button.pri{border:0;color:#0b0d10;font-weight:700}
label.f{display:block;font-size:12px;color:var(--mut);margin-top:10px;margin-bottom:3px}
input,select{font:inherit;width:100%;background:#0b0e13;color:var(--tx);border:1px solid var(--line);border-radius:10px;padding:9px 10px}
input[type=range]{padding:0;border:0;background:none;accent-color:#ff9f43;height:38px}input[type=checkbox]{width:auto;margin:0 6px 0 0;vertical-align:-2px}
.inl{display:flex;gap:8px;align-items:flex-end}.inl>*{flex:1;min-width:0}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:12px}
.stats div{background:#0f1319;border-radius:10px;padding:7px 9px}.stats small{display:block;color:var(--mut);font-size:11px}.stats span{font-variant-numeric:tabular-nums}
details{margin-top:12px}summary{color:var(--mut);cursor:pointer;font-size:13px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:0 8px}
.hint{color:var(--mut);font-size:12px;line-height:1.5}
canvas{width:100%;height:270px;display:block}
kbd{background:#21262d;border:1px solid var(--line);border-radius:5px;padding:0 5px;font-size:11px}
</style></head><body>
<header><h1>&#9881; Motor Lab</h1><span id="conn"><span id="dot"></span>connecting&hellip;</span><button id="estop" onclick="stopAll()">STOP</button></header>
<main>
<section class="card" id="card0"></section>
<section class="card" id="card1"></section>
<section class="card wide">
 <div class="hd"><b>Combo</b><small>chain both motors</small><span class="chip" id="sqc">IDLE</span></div>
 <div class="inl">
  <div><label class="f">Mode</label><select id="sqm"><option value="seq">Indexer, then Spinner</option><option value="both">Both together</option></select></div>
  <div><label class="f">Cycles (0 = forever)</label><input id="sqn" type="number" min="0" value="1"></div>
  <div><label class="f">Pause (s)</label><input id="sqd" type="number" min="0" step="0.1" value="1"></div>
 </div>
 <div class="btns"><button class="pri" style="background:#a371f7" onclick="seqRun()">&#9654; Run combo</button><button onclick="q('/seq?op=stop')">&#9632; Stop combo</button></div>
 <div class="hint" id="sqmsg" style="margin-top:8px"></div>
</section>
<section class="card wide">
 <div class="hd"><b>Live trace</b><small>last 20 s &middot; solid = actual, dashed = commanded</small></div>
 <canvas id="cv"></canvas>
</section>
<section class="card wide hint">Keys: <kbd>Space</kbd> stop &middot; <kbd>&larr;</kbd>/<kbd>&rarr;</kbd> indexer step &middot; <kbd>S</kbd> spin &middot; <kbd>C</kbd> combo. Board button SW38: tap = indexer step, hold = spin.</section>
</main>
<script>
const $=id=>document.getElementById(id);
const COL=['#4da3ff','#ff9f43'];
const MOT=[{n:'Indexer',s:'Motor 1 · step moves'},{n:'Spinner',s:'Motor 2 · slow full turn'}];
const KEYS=['kp','ki','kd','ff','mp','vmax','acc','cpr'];
const LBL={kp:'Kp (PWM/deg)',ki:'Ki (PWM/deg·s)',kd:'Kd (PWM per deg/s)',ff:'Friction PWM',mp:'Max PWM (30-255)',vmax:'Max speed (deg/s)',acc:'Accel (deg/s²)',cpr:'Counts per output rev'};
const WIN=20000,hist=[[],[]];
let cfg=null,last=null,busy=false,fails=0;
const mod=x=>((x%360)+360)%360;
const num=(v,d)=>{v=parseFloat(v);return isNaN(v)?d:v};
const buzz=()=>{try{navigator.vibrate&&navigator.vibrate(12)}catch(e){}};

function dial(i){
 let t='';
 for(let k=0;k<72;k++){const a=k*5,mj=a%30==0;t+=`<line x1="100" y1="${mj?20:15}" x2="100" y2="11" transform="rotate(${a} 100 100)" stroke="${mj?'#8b949e':'#30363d'}" stroke-width="${mj?2:1}"/>`}
 ['0','90','180','270'].forEach((s,k)=>{const a=k*Math.PI/2;t+=`<text x="${(100+64*Math.sin(a)).toFixed(1)}" y="${(104-64*Math.cos(a)).toFixed(1)}" fill="#8b949e" font-size="11" text-anchor="middle">${s}</text>`});
 return `<svg class="dial" viewBox="0 0 200 200"><circle cx="100" cy="100" r="93" fill="#0f1319" stroke="#262d36" stroke-width="2"/>${t}
 <path id="arc${i}" d="" fill="none" stroke="${COL[i]}" stroke-opacity=".35" stroke-width="7" stroke-linecap="round"/>
 <g id="tg${i}"><path d="M92 3 L108 3 L100 17 Z" fill="${COL[i]}"/></g>
 <g id="nd${i}"><line x1="100" y1="114" x2="100" y2="24" stroke="${COL[i]}" stroke-width="4" stroke-linecap="round"/></g>
 <circle cx="100" cy="100" r="7" fill="${COL[i]}"/><circle cx="100" cy="100" r="3" fill="#0e1116"/></svg>`;
}
function arc(a0,a1){const r=72,d=a1-a0;if(Math.abs(d)<0.5)return'';
 if(Math.abs(d)>=359.5)return`M100 ${100-r} A${r} ${r} 0 1 1 99.9 ${100-r}`;
 const p=a=>{a=a*Math.PI/180;return(100+r*Math.sin(a)).toFixed(2)+' '+(100-r*Math.cos(a)).toFixed(2)};
 return`M${p(a0)} A${r} ${r} 0 ${Math.abs(d)>180?1:0} ${d>0?1:0} ${p(a1)}`;}

function card(i){
 const s=i==1;
 return `<div class="hd"><b style="color:${COL[i]}">${MOT[i].n}</b><small>${MOT[i].s}</small><span class="chip" id="st${i}">--</span></div>
 <div class="top">${dial(i)}<div>
  <div class="big"><span id="a${i}">--</span>&deg;</div>
  <div class="row">target <span id="t${i}">--</span>&deg; &middot; err <span id="e${i}">--</span>&deg;</div>
  <div class="row">${s?'turns <span id="tn1">--</span> &middot; ':''}PWM <span id="p${i}">--</span></div>
  ${s?'<div class="row">ETA <span id="eta1">--</span></div>':''}
 </div></div>
 <div class="bar"><i id="b${i}" style="background:${COL[i]}"></i></div>
 <div class="flt" id="f${i}"></div>
 ${s?`<div class="inl"><div style="flex:0 0 34%"><label class="f">Turns per spin</label><input id="turns1" type="number" step="any" value="1"></div>
   <div><label class="f">Speed: <span id="spdl1">--</span></label><input id="spd1" type="range" min="2" max="90" step="1" value="15"></div></div>
   <div class="btns"><button onclick="mv(1,-1)">&#8634; Reverse</button><button class="pri" style="background:${COL[1]}" onclick="mv(1,1)">&#8635; Spin</button></div>`
 :`<label class="f">Step (degrees)</label><input id="step0" type="number" step="any" value="120">
   <label class="f">Speed: <span id="spdl0">--</span></label><input id="spd0" type="range" min="5" max="240" step="5" value="60" style="accent-color:#4da3ff">
   <div class="btns"><button onclick="mv(0,-1)">&minus; <span class="sv">120</span>&deg;</button><button class="pri" style="background:${COL[0]}" onclick="mv(0,1)">+ <span class="sv">120</span>&deg;</button></div>`}
 <div class="inl" style="margin-top:8px"><div><label class="f">Go to angle</label><input id="go${i}" type="number" step="any" placeholder="e.g. 0"></div>
  <button style="flex:0 0 64px" onclick="goAbs(${i})">Go</button><button style="flex:0 0 76px" onclick="q('/zero?i=${i}')">Zero</button></div>
 <div class="stats"><div><small>moves</small><span id="n${i}">0</span></div><div><small>last move</small><span id="lm${i}">--</span></div>
  <div><small>${s?'unstick kicks':'final error'}</small><span id="x${i}">--</span></div></div>
 <details><summary>Tuning &amp; calibration</summary>
  <div class="grid">${KEYS.map(k=>`<div><label class="f">${LBL[k]}</label><input id="${k}${i}" type="number" step="any"></div>`).join('')}</div>
  <label class="f"><input type="checkbox" id="fm${i}">Flip motor direction (fixes the direction fault)</label>
  <label class="f"><input type="checkbox" id="fe${i}">Flip encoder (re-zeros this motor)</label>
  <div class="btns"><button class="pri" style="background:${COL[i]}" onclick="save(${i})">Save</button></div>
  <p class="hint">Calibrate: mark the output shaft, tap Start (motor goes limp), turn the output shaft exactly one full turn by hand, tap Done.</p>
  <div class="btns"><button onclick="q('/cal?i=${i}&op=start')">Start</button><button onclick="q('/cal?i=${i}&op=done')">Done</button><button onclick="q('/cal?i=${i}&op=cancel')">Cancel</button></div>
  <p class="hint">Raw encoder counts: <span id="c${i}">--</span></p>
 </details>`;
}
$('card0').innerHTML=card(0);$('card1').innerHTML=card(1);

function upd(){const st=num($('step0').value,120);document.querySelectorAll('.sv').forEach(e=>e.textContent=st);
 const v0=+$('spd0').value,v1=+$('spd1').value,tn=num($('turns1').value,1);
 $('spdl0').textContent=v0+'°/s · ~'+(Math.abs(st)/v0).toFixed(1)+' s per step';
 $('spdl1').textContent=v1+'°/s · ~'+(Math.abs(tn)*360/v1).toFixed(0)+' s per spin';}
function q(u){return fetch(u,{cache:'no-store'}).then(r=>r.json()).then(d=>{if(d&&d.m&&d.m[0]&&('a' in d.m[0]))render(d)}).catch(()=>{})}
function loadCfg(){return fetch('/cfg',{cache:'no-store'}).then(r=>r.json()).then(c=>{cfg=c;
  c.m.forEach((m,i)=>{KEYS.forEach(k=>$(k+i).value=m[k]);$('fm'+i).checked=!!m.fm;$('fe'+i).checked=!!m.fe});
  $('step0').value=c.m[0].step;$('turns1').value=+(c.m[1].step/360).toFixed(3);$('spd0').value=c.m[0].vmax;$('spd1').value=c.m[1].vmax;upd();
 }).catch(()=>setTimeout(loadCfg,1000))}
function mv(i,s){buzz();const d=i==0?num($('step0').value,120):num($('turns1').value,1)*360;q(`/m?i=${i}&d=${s*d}`)}
function goAbs(i){const v=parseFloat($('go'+i).value);if(!isNaN(v)){buzz();q(`/m?i=${i}&a=${v}`)}}
function stopAll(){buzz();q('/stop')}
function seqRun(){buzz();q(`/seq?op=start&mode=${$('sqm').value}&n=${Math.max(0,parseInt($('sqn').value)||0)}&dwell=${Math.max(0,num($('sqd').value,1))}`)}
function save(i){const p=KEYS.map(k=>k+'='+encodeURIComponent($(k+i).value)).join('&');
 fetch(`/set?i=${i}&${p}&fm=${$('fm'+i).checked?1:0}&fe=${$('fe'+i).checked?1:0}`,{cache:'no-store'}).then(loadCfg).catch(()=>{})}
$('step0').addEventListener('input',upd);
$('step0').addEventListener('change',()=>fetch('/set?i=0&step='+num($('step0').value,120)).catch(()=>{}));
$('turns1').addEventListener('change',()=>fetch('/set?i=1&step='+num($('turns1').value,1)*360).catch(()=>{}));
$('turns1').addEventListener('input',upd);
[0,1].forEach(i=>{const s=$('spd'+i);s.addEventListener('input',upd);
 s.addEventListener('change',()=>{const v=+s.value;$('vmax'+i).value=v;if(cfg)cfg.m[i].vmax=v;fetch(`/set?i=${i}&vmax=${v}`).catch(()=>{})})});

function render(d){
 if(last&&d.up<last.up){hist[0].length=0;hist[1].length=0}
 last=d;fails=0;
 d.m.forEach((m,i)=>{
  $('a'+i).textContent=m.a.toFixed(1);$('t'+i).textContent=m.t.toFixed(1);$('e'+i).textContent=(m.t-m.a).toFixed(1);
  $('p'+i).textContent=m.pwm;const c=$('st'+i);c.textContent=m.st;c.className='chip '+m.st;
  $('nd'+i).setAttribute('transform',`rotate(${mod(m.a).toFixed(2)} 100 100)`);
  $('tg'+i).setAttribute('transform',`rotate(${mod(m.t).toFixed(2)} 100 100)`);
  $('arc'+i).setAttribute('d',m.st=='MOVING'?arc(m.a,m.t):'');
  $('b'+i).style.width=(m.p*100).toFixed(1)+'%';$('f'+i).textContent=m.f;
  $('n'+i).textContent=m.n;$('lm'+i).textContent=m.lm?(m.lm/1000).toFixed(2)+' s':'--';$('c'+i).textContent=m.c;
  hist[i].push([d.up,m.a,m.sp]);while(hist[i].length&&hist[i][0][0]<d.up-WIN)hist[i].shift();
 });
 $('x0').textContent=d.m[0].n?d.m[0].le.toFixed(2)+'°':'--';$('x1').textContent=d.m[1].tk;
 const s=d.m[1];$('tn1').textContent=(s.a/360).toFixed(2);
 $('eta1').textContent=(s.st=='MOVING'&&cfg)?(Math.abs(s.t-s.a)/cfg.m[1].vmax).toFixed(1)+' s':'--';
 $('sqc').textContent=d.seq.st;$('sqc').className='chip '+d.seq.st;
 $('sqmsg').textContent=d.seq.st=='IDLE'?d.seq.msg:('Cycle '+(d.seq.done+1)+(d.seq.tot?' of '+d.seq.tot:' (forever)'));
 draw();
}
const cv=$('cv'),cx=cv.getContext('2d');
function draw(){
 const r=window.devicePixelRatio||1,W=cv.clientWidth,H=cv.clientHeight;
 if(cv.width!=Math.round(W*r)||cv.height!=Math.round(H*r)){cv.width=Math.round(W*r);cv.height=Math.round(H*r)}
 cx.setTransform(r,0,0,r,0,0);cx.clearRect(0,0,W,H);if(!last)return;
 const t1=last.up,t0=t1-WIN,gap=12,ph=(H-gap)/2;
 for(let i=0;i<2;i++){
  const h=hist[i],y0=i*(ph+gap);let lo=Infinity,hi=-Infinity;
  h.forEach(p=>{lo=Math.min(lo,p[1],p[2]);hi=Math.max(hi,p[1],p[2])});
  if(!isFinite(lo)){lo=0;hi=1}if(hi-lo<10){const m=(hi+lo)/2;lo=m-5;hi=m+5}const pd=(hi-lo)*.08;lo-=pd;hi+=pd;
  const L=46,X=t=>L+(t-t0)/WIN*(W-L-6),Y=v=>y0+ph-(v-lo)/(hi-lo)*ph;
  cx.fillStyle='#0f1319';cx.fillRect(L,y0,W-L-6,ph);cx.strokeStyle='#262d36';cx.lineWidth=1;cx.setLineDash([]);
  cx.font='11px system-ui,sans-serif';cx.textAlign='right';cx.fillStyle='#8b949e';
  for(let k=0;k<=4;k++){const v=lo+(hi-lo)*k/4,y=Y(v);cx.beginPath();cx.moveTo(L,y);cx.lineTo(W-6,y);cx.stroke();cx.fillText(v.toFixed(0)+'°',L-5,y+4)}
  cx.textAlign='left';cx.fillStyle=COL[i];cx.fillText(MOT[i].n,L+6,y0+15);
  [[2,[5,4],.55,1.5],[1,[],1,2]].forEach(([k,dash,al,lw])=>{cx.setLineDash(dash);cx.globalAlpha=al;cx.strokeStyle=COL[i];cx.lineWidth=lw;
   cx.beginPath();h.forEach((p,j)=>{const x=X(p[0]),y=Y(p[k]);j?cx.lineTo(x,y):cx.moveTo(x,y)});cx.stroke()});
  cx.setLineDash([]);cx.globalAlpha=1;
 }
}
function poll(){if(busy)return;busy=true;fetch('/s',{cache:'no-store'}).then(r=>r.json()).then(render).catch(()=>{fails++})
 .finally(()=>{busy=false;const ok=fails<3;$('dot').style.background=ok?'#3fb950':'#f85149';$('conn').lastChild.textContent=ok?'live':'offline'})}
document.addEventListener('keydown',e=>{const t=e.target.tagName;if(t=='INPUT'||t=='SELECT'||t=='TEXTAREA')return;
 if(e.code=='Space'){e.preventDefault();stopAll()}else if(e.key=='ArrowRight')mv(0,1);else if(e.key=='ArrowLeft')mv(0,-1);
 else if(e.key=='s'||e.key=='S')mv(1,1);else if(e.key=='c'||e.key=='C')seqRun()});
window.addEventListener('resize',draw);
upd();loadCfg();poll();setInterval(poll,150);
</script></body></html>)HTML";

const char* stName(MState s) {
  switch (s) { case M_HOLD: return "HOLD"; case M_MOVING: return "MOVING"; case M_CAL: return "CAL"; default: return "FAULT"; }
}
const char* seqName() {
  switch (seq.st) { case SQ_IDLE: return "IDLE"; case SQ_PAUSE: return "PAUSE"; default: return "RUNNING"; }
}

void sendJson(const char* s) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

void sendStatus() {
  static char buf[1400];
  int n = 0;
  lock();
  n += snprintf(buf + n, sizeof(buf) - n,
    "{\"up\":%lu,\"seq\":{\"st\":\"%s\",\"done\":%d,\"tot\":%d,\"msg\":\"%s\"},\"m\":[",
    (unsigned long)millis(), seqName(), seq.done, seq.total, seq.msg);
  for (int i = 0; i < 2; i++) {
    Motor& m = M[i];
    float span = m.target - m.moveStart;
    float prog = 1.0f;
    if (m.st == M_MOVING && fabsf(span) > 0.01f) prog = constrain((m.angle - m.moveStart) / span, 0.0f, 1.0f);
    n += snprintf(buf + n, sizeof(buf) - n,
      "{\"a\":%.2f,\"t\":%.2f,\"sp\":%.2f,\"pwm\":%d,\"st\":\"%s\",\"f\":\"%s\",\"p\":%.3f,"
      "\"k\":%u,\"tk\":%lu,\"n\":%lu,\"lm\":%lu,\"le\":%.2f,\"c\":%ld}%s",
      m.angle, m.target, m.sp, m.pwm, stName(m.st), m.fault, prog,
      (unsigned)m.kicks, (unsigned long)m.totalKicks, (unsigned long)m.moves,
      (unsigned long)m.lastMoveMs, m.lastErr, (long)readCount(i), i == 0 ? "," : "");
  }
  unlock();
  snprintf(buf + n, sizeof(buf) - n, "]}");
  sendJson(buf);
}

void sendCfg() {
  Params p[2];
  lock(); p[0] = M[0].p; p[1] = M[1].p; unlock();
  char buf[700]; int n = 0;
  n += snprintf(buf + n, sizeof(buf) - n, "{\"m\":[");
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n,
      "{\"cpr\":%.2f,\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"vmax\":%.1f,\"acc\":%.1f,\"ff\":%.1f,\"mp\":%.0f,\"step\":%.2f,\"fm\":%d,\"fe\":%d}%s",
      p[i].cpr, p[i].kp, p[i].ki, p[i].kd, p[i].vmax, p[i].acc, p[i].ff, p[i].maxPwm, p[i].step,
      p[i].flipMotor ? 1 : 0, p[i].flipEnc ? 1 : 0, i == 0 ? "," : "");
  snprintf(buf + n, sizeof(buf) - n, "]}");
  sendJson(buf);
}

bool argSet(const char* name)  { return server.hasArg(name) && server.arg(name).length() > 0; }
float argF(const char* name, float def) { return argSet(name) ? server.arg(name).toFloat() : def; }
int motorArg() {
  if (!argSet("i")) return -1;
  int i = server.arg("i").toInt();
  return (i == 0 || i == 1) ? i : -1;
}
void badReq() { server.send(400, "text/plain", "bad request"); }

void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS, 6);
  WiFi.setSleep(false);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", AP_IP);                         // captive portal
  if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", 80);

  server.on("/", sendPage);
  server.on("/generate_204", sendPage);              // Android
  server.on("/gen_204", sendPage);
  server.on("/hotspot-detect.html", sendPage);       // iOS / macOS
  server.on("/library/test/success.html", sendPage);
  server.on("/connecttest.txt", sendPage);           // Windows
  server.on("/ncsi.txt", sendPage);
  server.on("/favicon.ico", []() { server.send(204); });

  server.on("/s", sendStatus);
  server.on("/cfg", sendCfg);

  server.on("/m", []() {                             // move: d=relative or a=absolute
    int i = motorArg(); if (i < 0) return badReq();
    lock();
    seqStop("Combo stopped: manual move");
    if (argSet("d"))      relMove(i, argF("d", 0));
    else if (argSet("a")) { if (M[i].st == M_FAULT) stopMotor(i); beginMove(i, argF("a", 0), true); }
    unlock();
    sendStatus();
  });
  server.on("/stop", []() {
    lock(); seqStop("Combo stopped"); stopMotor(0); stopMotor(1); unlock();
    sendStatus();
  });
  server.on("/zero", []() {
    int i = motorArg();
    lock();
    if (i < 0) { zeroMotor(0); zeroMotor(1); } else zeroMotor(i);
    unlock();
    sendStatus();
  });
  server.on("/cal", []() {
    int i = motorArg(); if (i < 0 || !argSet("op")) return badReq();
    String op = server.arg("op");
    bool doSave = false; Params copy;
    lock();
    if (op == "start") {
      seqStop("Combo stopped: calibrating");
      M[i].st = M_CAL; zeroMotor(i);
    } else if (M[i].st == M_CAL && op == "done") {
      int32_t cnt = labs(readCount(i));
      M[i].st = M_HOLD;
      if (cnt > 50) { M[i].p.cpr = (float)cnt; doSave = true; copy = M[i].p; zeroMotor(i); }
      else { zeroMotor(i); strncpy(M[i].fault, "Calibration ignored: almost no encoder counts. Is the encoder powered?", sizeof(M[i].fault) - 1); }
    } else if (M[i].st == M_CAL && op == "cancel") {
      M[i].st = M_HOLD; rebaseMotor(i);
    }
    unlock();
    if (doSave) saveParams(i, copy);
    sendStatus();
  });
  server.on("/set", []() {
    int i = motorArg(); if (i < 0) return badReq();
    lock();
    Params& p = M[i].p;
    float oldCpr = p.cpr; bool oldFe = p.flipEnc; bool oldFm = p.flipMotor;
    p.kp   = constrain(argF("kp", p.kp), 0.0f, 100.0f);
    p.ki   = constrain(argF("ki", p.ki), 0.0f, 500.0f);
    p.kd   = constrain(argF("kd", p.kd), 0.0f, 5.0f);
    p.ff   = constrain(argF("ff", p.ff), 0.0f, 200.0f);
    p.maxPwm = constrain(argF("mp", p.maxPwm), 30.0f, 255.0f);
    p.vmax = constrain(argF("vmax", p.vmax), 1.0f, 3000.0f);
    p.acc  = constrain(argF("acc", p.acc), 1.0f, 50000.0f);
    p.step = constrain(argF("step", p.step), -36000.0f, 36000.0f);
    float c = argF("cpr", p.cpr); if (c >= 10.0f) p.cpr = c;
    if (argSet("fm")) p.flipMotor = server.arg("fm").toInt() != 0;
    if (argSet("fe")) p.flipEnc   = server.arg("fe").toInt() != 0;
    if (p.flipEnc != oldFe) zeroMotor(i);
    else if (p.cpr != oldCpr) rebaseMotor(i);
    if (p.flipMotor != oldFm && M[i].st == M_FAULT) stopMotor(i);
    Params copy = p;
    unlock();
    saveParams(i, copy);
    sendCfg();
  });
  server.on("/seq", []() {
    String op = server.arg("op");
    lock();
    if (op == "start") {
      if (M[0].st == M_CAL || M[1].st == M_CAL) snprintf(seq.msg, sizeof(seq.msg), "Finish calibration first");
      else {
        seq.both  = server.arg("mode") == "both";
        seq.total = max(0, (int)argF("n", 1));
        seq.dwell = constrain(argF("dwell", 1.0f), 0.0f, 600.0f);
        seq.done  = 0; seq.msg[0] = 0;
        seqCycle();
      }
    } else seqStop("Combo stopped");
    unlock();
    sendStatus();
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();
}

// ----------------------------- STATUS LED ------------------------
void updateLed() {
#if defined(PIN_NEOPIXEL)
  static uint32_t t = 0, lastCol = 0xFFFFFFFF;
  if (millis() - t < 40) return;
  t = millis();
  lock();
  bool flt = M[0].st == M_FAULT || M[1].st == M_FAULT;
  bool cal = M[0].st == M_CAL   || M[1].st == M_CAL;
  bool mov = M[0].st == M_MOVING || M[1].st == M_MOVING;
  bool sq  = seq.st != SQ_IDLE;
  unlock();
  bool blink = (millis() / 250) & 1;
  uint8_t r = 0, g = 0, b = 0;
  if (flt)      { r = blink ? 50 : 0; }
  else if (cal) { b = 40; }
  else if (sq)  { r = 25; b = 40; }
  else if (mov) { r = 45; g = 25; }
  else          { g = 14; }
  uint32_t col = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (col != lastCol) { rgbLedWrite(PIN_NEOPIXEL, r, g, b); lastCol = col; }
#endif
}

// ----------------------------- BUTTON / SERIAL -------------------
void handleButton() {
  static bool lastRead = HIGH, stable = HIGH, down = false, longFired = false;
  static uint32_t changedAt = 0, pressedAt = 0;
  bool r = digitalRead(BTN_PIN);
  uint32_t now = millis();
  if (r != lastRead) { lastRead = r; changedAt = now; }
  if (now - changedAt > 25 && r != stable) {
    stable = r;
    if (stable == LOW) { down = true; longFired = false; pressedAt = now; }
    else {
      if (down && !longFired) { lock(); seqStop("Combo stopped: manual move"); relMove(0, M[0].p.step); unlock(); }
      down = false;
    }
  }
  if (down && !longFired && now - pressedAt >= 700) {
    longFired = true;
    lock(); seqStop("Combo stopped: manual move"); relMove(1, M[1].p.step); unlock();
  }
}

void handleSerial() {
  static char line[32]; static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { if (len < sizeof(line) - 1) line[len++] = ch; continue; }
    if (!len) continue;
    line[len] = 0; len = 0;
    lock();
    switch (line[0]) {
      case 'a': case 'A': if (M[0].st == M_FAULT) stopMotor(0); beginMove(0, atof(line + 1), true); break;
      case 'b': case 'B': if (M[1].st == M_FAULT) stopMotor(1); beginMove(1, atof(line + 1), true); break;
      case '+': relMove(0, M[0].p.step); break;
      case 'r': case 'R': relMove(1, M[1].p.step); break;
      case 's': case 'S': seqStop("Combo stopped"); stopMotor(0); stopMotor(1); break;
      case 'z': case 'Z': zeroMotor(0); zeroMotor(1); break;
      default: break;
    }
    unlock();
  }
}

// ----------------------------- SETUP / LOOP ----------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  mtx = xSemaphoreCreateMutex();

  setDefaults();
  for (int i = 0; i < 2; i++) loadParams(i);

  for (int i = 0; i < 2; i++) {
    if (!ledcAttach(PINS[i].in1, PWM_FREQ, PWM_RES) || !ledcAttach(PINS[i].in2, PWM_FREQ, PWM_RES))
      Serial.printf("PWM attach failed on motor %d\n", i + 1);
    coast(i);
    enc[i].a = PINS[i].encA; enc[i].b = PINS[i].encB; enc[i].count = 0;
    pinMode(enc[i].a, INPUT_PULLUP);
    pinMode(enc[i].b, INPUT_PULLUP);
    enc[i].last = (digitalRead(enc[i].a) << 1) | digitalRead(enc[i].b);
    attachInterruptArg(digitalPinToInterrupt(enc[i].a), encISR, &enc[i], CHANGE);
    attachInterruptArg(digitalPinToInterrupt(enc[i].b), encISR, &enc[i], CHANGE);
  }
  pinMode(BTN_PIN, INPUT);

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

  setupWeb();

#if !ENCODER_TEST
  for (int i = 0; i < 2; i++) driveRaw(i, 0);
  xTaskCreatePinnedToCore(controlTask, "motorCtl", 4096, nullptr, 3, nullptr, 1);
#endif

  Serial.printf("\nMotor Lab ready. Join Wi-Fi \"%s\" (pw %s), open http://%s or http://%s.local\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str(), MDNS_NAME);
  Serial.println("Serial: a<deg> b<deg> absolute | + step | r spin | s stop | z zero");
}

void loop() {
  dns.processNextRequest();
  server.handleClient();

#if ENCODER_TEST
  static uint32_t tp = 0;
  if (millis() - tp >= 50) {
    tp = millis();
    Serial.printf("M1_A:%d,M1_B:%d,M1_counts:%ld,M2_A:%d,M2_B:%d,M2_counts:%ld\n",
      digitalRead(PINS[0].encA), digitalRead(PINS[0].encB), (long)readCount(0),
      digitalRead(PINS[1].encA), digitalRead(PINS[1].encB), (long)readCount(1));
  }
  delay(1);
  return;
#endif

  handleButton();
  handleSerial();
  lock(); seqTick(); unlock();
  updateLed();

  static uint32_t lastPrint = 0;          // Serial Plotter: named traces
  if (millis() - lastPrint >= 40) {
    lastPrint = millis();
    lock();
    float a0 = M[0].angle, s0 = M[0].sp, a1 = M[1].angle, s1 = M[1].sp;
    unlock();
    Serial.printf("M1_angle:%.1f,M1_cmd:%.1f,M2_angle:%.1f,M2_cmd:%.1f\n", a0, s0, a1, s1);
  }
  delay(1);
}
