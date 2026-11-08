// Pololu #2215 (75:1 HP micro metal gearmotor) + magnetic encoder
// Board: Adafruit Feather ESP32 V2 (ESP32 Arduino core 3.x)
// Driver: DRV8833 (IN/IN)
//
// Closed-loop position control:
//   motion profile (smooth accel/decel) -> PID + friction compensation
//   -> settle check (must sit inside tolerance before it counts as "done")
//   -> active hold (re-corrects if it gets pushed off target)
//
// Control page: join Wi-Fi "MotorCtrl" (pw rotate120) -> http://192.168.4.1
// SW38 on the board = rotate by the current step (default 120 deg).
// Serial: type a number = go to that absolute angle, z = zero.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#define ENCODER_TEST 0   // 1 = motor off, print encoder A/B for hand-spin test

// ---------------- WI-FI ----------------
const char* AP_SSID = "MotorCtrl";
const char* AP_PASS = "rotate120";
const IPAddress AP_IP(192, 168, 4, 1);
WebServer server(80);
DNSServer dns;
Preferences prefs;

// ---------------- PINS ----------------
const int IN1 = 26, IN2 = 25;     // DRV8833 AIN1 / AIN2
const int ENC_A = 27, ENC_B = 33; // encoder OUT A / OUT B
const int BTN = 38;               // on-board SW38, active LOW
const int PWM_FREQ = 20000, PWM_RES = 8;

// ---------------- SETTINGS (editable from the page, saved to flash) ----------------
float cpr     = 12.0 * 75.81; // encoder counts per OUTPUT shaft rev (~910)
float stepDeg = 120.0;        // default rotate amount
float kp      = 8.0;          // PWM per deg of error
float ki      = 25.0;         // PWM per (deg*s) - removes steady-state error
float kd      = 0.15;         // PWM per (deg/s) of velocity error - damping
float vmax    = 360.0;        // max speed, deg/s
float acc     = 1500.0;       // accel/decel, deg/s^2
float ffPwm   = 45.0;         // PWM needed to just overcome friction

// ---------------- FIXED CONTROL CONSTANTS ----------------
const int   MAX_PWM      = 230;
const float I_LIMIT      = 80.0;   // max PWM contribution from integral
const float TOL_DEG      = 0.5;    // "arrived" tolerance
const float REENGAGE_DEG = 1.5;    // if pushed this far off while holding, correct
const float SETTLE_VEL   = 8.0;    // deg/s, must be slower than this to count as settled
const uint32_t SETTLE_MS = 150;    // must stay settled this long
const float DT           = 0.002;  // 500 Hz control loop

// ---------------- ENCODER ----------------
volatile long encCount = 0;
volatile uint8_t lastState = 0;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};

void IRAM_ATTR encISR() {
  uint8_t s = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  portENTER_CRITICAL_ISR(&encMux);
  encCount += QEM[(lastState << 2) | s];
  lastState = s;
  portEXIT_CRITICAL_ISR(&encMux);
}
long readCount() { portENTER_CRITICAL(&encMux); long c = encCount; portEXIT_CRITICAL(&encMux); return c; }
void zeroCount() { portENTER_CRITICAL(&encMux); encCount = 0;      portEXIT_CRITICAL(&encMux); }

// ---------------- MOTOR ----------------
void setMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { ledcWrite(IN1, pwm); ledcWrite(IN2, 0); }
  else if (pwm < 0) { ledcWrite(IN1, 0);   ledcWrite(IN2, -pwm); }
  else              { ledcWrite(IN1, 255); ledcWrite(IN2, 255); }  // brake
}
void coastMotor() { ledcWrite(IN1, 0); ledcWrite(IN2, 0); }

// ---------------- STATE ----------------
enum State { HOLD, MOVING, CALIBRATING, FAULT };
State state = HOLD;
const char* faultMsg = "";

float target = 0;      // where the user wants to be (deg)
float sp = 0;          // profiled setpoint the loop actually chases
float spVel = 0;       // profile velocity
float angle = 0;       // measured
float vel = 0;         // measured, filtered (deg/s)
float integ = 0;
int   pwmOut = 0;

float countsPerDeg() { return cpr / 360.0; }

void startMove(float newTarget) {
  if (state == CALIBRATING) return;
  if (state != MOVING) {            // start profile from where we actually are
    sp = angle; spVel = 0; integ = 0;
  }
  target = newTarget;
  faultMsg = "";
  state = MOVING;
}

void stopHere() {
  target = sp = angle; spVel = 0; integ = 0;
  if (state != CALIBRATING) state = HOLD;
  faultMsg = "";
}

void setFault(const char* msg) { state = FAULT; faultMsg = msg; pwmOut = 0; integ = 0; }

// ---------------- CONTROL LOOP ----------------
void controlStep(float dt) {
  static float lastAngle = 0;
  static uint32_t settledSince = 0;
  static long lastCount = 0;
  static uint32_t lastCountChange = 0;
  static uint32_t wrongWaySince = 0;

  long c = readCount();
  angle = c / countsPerDeg();
  float vRaw = (angle - lastAngle) / dt;
  lastAngle = angle;
  vel += 0.1 * (vRaw - vel);

  if (state == CALIBRATING) { coastMotor(); pwmOut = 0; return; }
  if (state == FAULT)       { setMotor(0);  pwmOut = 0; return; }

  float finalErr = target - angle;

  if (state == HOLD) {
    pwmOut = 0;
    setMotor(0);                                   // brake
    if (fabs(finalErr) > REENGAGE_DEG) startMove(target);   // got pushed, fix it
    lastCount = c; lastCountChange = millis();
    return;
  }

  // ---- MOVING: 1) advance the motion profile ----
  float dist = target - sp;
  float vDes = (dist >= 0 ? 1 : -1) * min(vmax, sqrtf(2.0f * acc * fabs(dist)));
  float maxDv = acc * dt;
  spVel += constrain(vDes - spVel, -maxDv, maxDv);
  sp += spVel * dt;
  bool profileDone = false;
  if (fabs(target - sp) < 0.05 && fabs(spVel) <= maxDv * 2) { sp = target; spVel = 0; profileDone = true; }

  // ---- 2) PID on (setpoint - measured) ----
  float e = sp - angle;
  float ve = spVel - vel;
  integ += e * dt;
  if (ki > 0) integ = constrain(integ, -I_LIMIT / ki, I_LIMIT / ki);
  float u = kp * e + ki * integ + kd * ve;

  // ---- 3) friction compensation ----
  int pwm = 0;
  if (fabs(u) > 1.0) pwm = (int)(u + (u > 0 ? ffPwm : -ffPwm));
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  // ---- 4) arrived? must be inside tolerance AND slow for SETTLE_MS ----
  if (profileDone && fabs(finalErr) < TOL_DEG && fabs(vel) < SETTLE_VEL) {
    if (settledSince == 0) settledSince = millis();
    if (millis() - settledSince > SETTLE_MS) {
      state = HOLD; integ = 0; settledSince = 0;
      pwmOut = 0; setMotor(0);
      return;
    }
  } else {
    settledSince = 0;
  }

  // ---- 5) safety ----
  // driving hard but encoder not counting
  if (c != lastCount || abs(pwm) < ffPwm + 15) { lastCount = c; lastCountChange = millis(); }
  else if (millis() - lastCountChange > 800) { setFault("Motor is driven but encoder isn't counting - check encoder power/wiring or a jam."); return; }
  // driving one way, moving the other way
  if (abs(pwm) > ffPwm + 30 && fabs(vel) > 30 && ((pwm > 0) != (vel > 0))) {
    if (wrongWaySince == 0) wrongWaySince = millis();
    if (millis() - wrongWaySince > 250) { setFault("Motor and encoder directions are reversed - swap ENC_A/ENC_B in code or swap motor leads."); return; }
  } else wrongWaySince = 0;

  pwmOut = pwm;
  setMotor(pwm);
}

// ---------------- SETTINGS STORAGE ----------------
void loadSettings() {
  prefs.begin("motor", true);
  cpr     = prefs.getFloat("cpr", cpr);
  stepDeg = prefs.getFloat("step", stepDeg);
  kp      = prefs.getFloat("kp", kp);
  ki      = prefs.getFloat("ki", ki);
  kd      = prefs.getFloat("kd", kd);
  vmax    = prefs.getFloat("vmax", vmax);
  acc     = prefs.getFloat("acc", acc);
  ffPwm   = prefs.getFloat("ff", ffPwm);
  prefs.end();
}
void saveSettings() {
  prefs.begin("motor", false);
  prefs.putFloat("cpr", cpr);   prefs.putFloat("step", stepDeg);
  prefs.putFloat("kp", kp);     prefs.putFloat("ki", ki);   prefs.putFloat("kd", kd);
  prefs.putFloat("vmax", vmax); prefs.putFloat("acc", acc); prefs.putFloat("ff", ffPwm);
  prefs.end();
}

// ---------------- WEB PAGE ----------------
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Motor Control</title>
<style>
 *{box-sizing:border-box}
 body{font-family:-apple-system,system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px;max-width:460px;margin:auto}
 h1{font-size:20px;margin:4px 0 12px}
 .card{background:#1c1c1e;border-radius:14px;padding:14px;margin-bottom:12px}
 .big{font-size:44px;font-weight:700;font-variant-numeric:tabular-nums}
 .row{display:flex;gap:8px;align-items:center;margin:6px 0}
 .lbl{color:#999;font-size:13px}
 .val{font-variant-numeric:tabular-nums}
 button{flex:1;font-size:18px;padding:16px;border:0;border-radius:12px;color:#fff;background:#3a3a3c}
 button.go{background:#2b7cff} button.warn{background:#c0392b}
 button:active{transform:scale(.97)}
 input{width:100%;font-size:18px;padding:10px;border-radius:10px;border:1px solid #444;background:#000;color:#eee}
 .state{font-weight:700} .HOLD{color:#4cd964} .MOVING{color:#ffcc00} .FAULT{color:#ff5a5a} .CALIBRATING{color:#5ac8fa}
 #fault{color:#ff5a5a;font-size:14px;margin-top:6px}
 details summary{cursor:pointer;color:#aaa}
 .grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
 .grid label{font-size:12px;color:#999}
 .small{font-size:13px;color:#999;line-height:1.4}
</style></head><body>
<h1>Gearmotor Control</h1>

<div class="card">
 <div class="lbl">Angle</div>
 <div class="big"><span id="ang">--</span>&deg;</div>
 <div class="row"><span class="lbl">Target</span><span class="val" id="tgt">--</span>&deg;
  <span class="lbl" style="margin-left:12px">Error</span><span class="val" id="err">--</span>&deg;</div>
 <div class="row"><span class="lbl">State</span><span id="st" class="state">--</span>
  <span class="lbl" style="margin-left:12px">PWM</span><span class="val" id="pwm">--</span></div>
 <div id="fault"></div>
</div>

<div class="card">
 <div class="lbl">Step (degrees)</div>
 <input id="step" type="number" step="any" value="120">
 <div class="row">
  <button onclick="rot(-1)">&minus;<span class="sv">120</span>&deg;</button>
  <button class="go" onclick="rot(1)">+<span class="sv">120</span>&deg;</button>
 </div>
 <div class="lbl" style="margin-top:10px">Go to absolute angle</div>
 <div class="row"><input id="abs" type="number" step="any" placeholder="e.g. 0, 90, 360"><button style="flex:0 0 80px" onclick="go()">Go</button></div>
 <div class="row">
  <button class="warn" onclick="q('/stop')">Stop</button>
  <button onclick="q('/zero')">Set 0&deg; here</button>
 </div>
</div>

<div class="card">
 <details><summary>Calibrate (if 120&deg; isn't really 120&deg;)</summary>
  <p class="small">1. Mark the output shaft. 2. Tap Start (motor goes limp).
  3. Turn the output shaft exactly one full turn by hand. 4. Tap Done.</p>
  <div class="row"><button onclick="q('/cal/start')">Start</button><button class="go" onclick="q('/cal/done')">Done</button></div>
  <div class="small">Counts per output rev: <span id="cprv">--</span> &nbsp; Raw counts: <span id="cnt">--</span></div>
 </details>
</div>

<div class="card">
 <details><summary>Tuning</summary>
  <div class="grid">
   <div><label>Kp (PWM/deg)</label><input id="kp" type="number" step="any"></div>
   <div><label>Ki (PWM/deg&middot;s)</label><input id="ki" type="number" step="any"></div>
   <div><label>Kd (PWM per deg/s)</label><input id="kd" type="number" step="any"></div>
   <div><label>Friction PWM</label><input id="ff" type="number" step="any"></div>
   <div><label>Max speed (deg/s)</label><input id="vmax" type="number" step="any"></div>
   <div><label>Accel (deg/s&sup2;)</label><input id="acc" type="number" step="any"></div>
   <div><label>Counts / output rev</label><input id="cpr" type="number" step="any"></div>
  </div>
  <div class="row"><button class="go" onclick="save()">Save</button></div>
 </details>
</div>

<script>
const $=id=>document.getElementById(id);
const KEYS=['kp','ki','kd','ff','vmax','acc','cpr'];
let loaded=false,busy=false;
function show(d){
 $('ang').textContent=d.angle.toFixed(1);
 $('tgt').textContent=d.target.toFixed(1);
 $('err').textContent=(d.target-d.angle).toFixed(1);
 $('pwm').textContent=d.pwm;
 $('st').textContent=d.state; $('st').className='state '+d.state;
 $('fault').textContent=d.fault;
 $('cprv').textContent=d.cpr.toFixed(1); $('cnt').textContent=d.counts;
 if(!loaded){ KEYS.forEach(k=>$(k).value=d[k]); $('step').value=d.step; upLbl(); loaded=true; }
}
function q(u){return fetch(u,{cache:'no-store'}).then(r=>r.json()).then(show).catch(()=>{});}
function poll(){ if(busy)return; busy=true; q('/status').finally(()=>busy=false); }
function stepVal(){ const v=parseFloat($('step').value); return isNaN(v)?120:v; }
function upLbl(){ document.querySelectorAll('.sv').forEach(e=>e.textContent=stepVal()); }
function rot(s){ q('/move?d='+(s*stepVal())); }
function go(){ const v=parseFloat($('abs').value); if(!isNaN(v)) q('/goto?a='+v); }
function save(){ q('/set?'+KEYS.map(k=>k+'='+encodeURIComponent($(k).value)).join('&')); }
$('step').addEventListener('input',upLbl);
$('step').addEventListener('change',()=>q('/set?step='+stepVal()));
setInterval(poll,400); poll();
</script></body></html>
)HTML";

const char* stateName() {
  switch (state) { case HOLD: return "HOLD"; case MOVING: return "MOVING";
                   case CALIBRATING: return "CALIBRATING"; default: return "FAULT"; }
}

void sendStatus() {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"angle\":%.2f,\"target\":%.2f,\"pwm\":%d,\"state\":\"%s\",\"fault\":\"%s\","
    "\"counts\":%ld,\"cpr\":%.2f,\"step\":%.2f,\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,"
    "\"vmax\":%.1f,\"acc\":%.1f,\"ff\":%.1f}",
    angle, target, pwmOut, stateName(), faultMsg, readCount(), cpr, stepDeg,
    kp, ki, kd, vmax, acc, ffPwm);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

float argF(const char* name, float def) {
  return server.hasArg(name) ? server.arg(name).toFloat() : def;
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS, 6);
  WiFi.setSleep(false);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", AP_IP);

  server.on("/", sendPage);
  server.on("/generate_204", sendPage);
  server.on("/gen_204", sendPage);
  server.on("/hotspot-detect.html", sendPage);
  server.on("/library/test/success.html", sendPage);
  server.on("/connecttest.txt", sendPage);
  server.on("/ncsi.txt", sendPage);
  server.on("/favicon.ico", []() { server.send(204); });

  server.on("/status", sendStatus);
  server.on("/move", []() { startMove(target + argF("d", stepDeg)); sendStatus(); });
  server.on("/goto", []() { if (server.hasArg("a")) startMove(argF("a", target)); sendStatus(); });
  server.on("/stop", []() { stopHere(); sendStatus(); });
  server.on("/zero", []() {
    zeroCount(); angle = 0; target = sp = 0; spVel = 0; integ = 0;
    if (state != CALIBRATING) state = HOLD;
    faultMsg = ""; sendStatus();
  });
  server.on("/cal/start", []() {
    state = CALIBRATING; zeroCount(); angle = 0; faultMsg = ""; sendStatus();
  });
  server.on("/cal/done", []() {
    if (state == CALIBRATING) {
      long n = labs(readCount());
      if (n > 50) { cpr = n; saveSettings(); }
      else faultMsg = "Calibration ignored: barely any counts. Was the encoder working?";
      zeroCount(); angle = 0; target = sp = 0; spVel = 0; integ = 0;
      state = HOLD;
    }
    sendStatus();
  });
  server.on("/set", []() {
    stepDeg = argF("step", stepDeg);
    kp = argF("kp", kp); ki = argF("ki", ki); kd = argF("kd", kd);
    vmax = max(10.0f, argF("vmax", vmax));
    acc  = max(10.0f, argF("acc", acc));
    ffPwm = constrain(argF("ff", ffPwm), 0.0f, 200.0f);
    float newCpr = argF("cpr", cpr);
    if (newCpr > 10) cpr = newCpr;
    saveSettings();
    sendStatus();
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();
  Serial.printf("\nWi-Fi AP \"%s\" up. Open http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ---------------- PHYSICAL BUTTON / SERIAL ----------------
void handleButton() {
  static bool lastStable = HIGH, lastRead = HIGH;
  static uint32_t changedAt = 0;
  bool r = digitalRead(BTN);
  if (r != lastRead) { lastRead = r; changedAt = millis(); }
  if (millis() - changedAt > 30 && r != lastStable) {
    lastStable = r;
    if (r == LOW) startMove(target + stepDeg);
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;
  if (s == "z" || s == "Z") { zeroCount(); angle = 0; target = sp = 0; spVel = 0; integ = 0; state = HOLD; }
  else startMove(s.toFloat());
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcAttach(IN2, PWM_FREQ, PWM_RES);
  setMotor(0);

  pinMode(BTN, INPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  lastState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);
  zeroCount();

  loadSettings();
  setupWeb();
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
  handleButton();
  handleSerial();

#if ENCODER_TEST
  setMotor(0);
  static uint32_t tp = 0;
  if (millis() - tp > 20) { tp = millis();
    Serial.printf("A:%d,B:%d,counts:%ld\n", digitalRead(ENC_A), digitalRead(ENC_B), readCount()); }
  return;
#endif

  // fixed-rate control (catches up if Wi-Fi handling took a few ms)
  static uint32_t lastUs = micros();
  uint32_t now = micros();
  float elapsed = (now - lastUs) * 1e-6;
  if (elapsed >= DT) {
    lastUs = now;
    controlStep(min(elapsed, 0.02f));
  }

  // Serial Plotter
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 20) {
    lastPrint = millis();
    Serial.printf("target:%.1f,setpoint:%.1f,angle:%.1f,pwm:%d\n", target, sp, angle, pwmOut);
  }
}
