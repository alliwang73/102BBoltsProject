// ============================================================
//  Bolt inspection / sorting state machine  (ESP32, Arduino core 2.x)
//  No status LED is used or required.
//
//  Axes:
//    feeder    = hobby servo
//    boltMotor = DC motor + encoder (60 deg steps)
//    sorter    = DC motor + encoder
//    flap      = DC motor + encoder
//
//  Serial commands (115200 baud):
//    k = bolt present (same as pressing the bolt button)
//    g = inspection GOOD     b = inspection BAD
//    f = release flap        r = reset after FAULT
//
//  ALL PIN NUMBERS / POSITIONS BELOW ARE PLACEHOLDERS: set them for your build.
// ============================================================
#include <ESP32Encoder.h>
#include <ESP32Servo.h>

// ---------------- Pins ----------------
#define BOLT_BTN    36      // bolt-sensed button (input-only pin: needs EXTERNAL pull-up)
#define LIMIT_PIN   39      // bolt-seated limit switch (input-only: EXTERNAL pull-up)
#define ACTIVE_LEVEL LOW    // level when button/switch is pressed

// bolt rotator (DC + encoder)
#define BOLT_ENC_A  33
#define BOLT_ENC_B  27
#define BOLT_PWM_F  26
#define BOLT_PWM_R  25
// sorter (DC + encoder)
#define SORT_ENC_A  34
#define SORT_ENC_B  35
#define SORT_PWM_F  14
#define SORT_PWM_R  32
// flap (DC + encoder)
#define FLAP_ENC_A  16
#define FLAP_ENC_B  17
#define FLAP_PWM_F  21
#define FLAP_PWM_R  22
// feeder servo
#define FEEDER_PIN  18

// ---------------- Tuning ----------------
#define COUNTS_PER_REV     1440     // bolt motor output-shaft counts per rev
#define SORT_GOOD_POS      300      // encoder counts
#define SORT_BAD_POS      -300
#define SORT_HOME_POS      0
#define FLAP_OPEN_POS      200      // encoder counts
#define FLAP_CLOSED_POS    0
#define FLAP_DWELL_MS      500      // hold flap open so the bolt can drop
#define FEEDER_START_ANGLE 0
#define FEEDER_END_ANGLE   180

#define SEAT_TIME_MS       5000     // bolt must stay seated this long
#define INSPECT_TIMEOUT_MS 10000    // wait for good/bad and flap button
#define MOVE_TIMEOUT_MS    3000     // any motor move
#define CONTROL_PERIOD_US  10000    // 10 ms control tick
#define DEBOUNCE_US        20000    // 20 ms button debounce

// ---------------- DC motor + encoder axis ----------------
struct DCAxis {
  ESP32Encoder enc;
  uint8_t chFwd = 0, chRev = 0;
  float kp = 20, ki = 0, kd = 5;     // velocity loop
  float kpPos = 0.2, kiPos = 0;      // position loop
  long pos = 0, posDes = 0;
  int speed = 0, speedLast = 0;
  long sumErrPos = 0, sumErrV = 0;
  bool enabled = false;

  void begin(int encA, int encB, int pinFwd, int pinRev, uint8_t chF, uint8_t chR) {
    chFwd = chF; chRev = chR;
    enc.attachHalfQuad(encA, encB);
    enc.clearCount();
    ledcSetup(chFwd, 20000, 8); ledcAttachPin(pinFwd, chFwd);
    ledcSetup(chRev, 20000, 8); ledcAttachPin(pinRev, chRev);
    stop();
  }
  void moveBy(long counts) { moveTo(posDes + counts); }
  void moveTo(long counts) { posDes = counts; sumErrPos = 0; sumErrV = 0; enabled = true; }
  bool done(int posTol = 10, int spdTol = 2) {
    return abs(posDes - pos) < posTol && abs(speed) <= spdTol;
  }
  void stop() {
    enabled = false;
    ledcWrite(chFwd, 0);
    ledcWrite(chRev, 0);
  }
  void update() {                    // call once per control tick
    speed = (int)enc.getCount();
    enc.clearCount();
    pos += speed;
    if (!enabled) return;

    long errPos = posDes - pos;
    sumErrPos += errPos;
    int omegaDes = (int)(kpPos * errPos + kiPos * sumErrPos);
    int errV = omegaDes - speed;
    sumErrV += errV;
    int dV = speed - speedLast;
    speedLast = speed;

    int drive = (int)(kp * errV + ki * sumErrV + kd * dV);
    if (abs(drive) > 255) {          // anti-windup
      sumErrV -= errV;
      drive = (drive > 0) ? 255 : -255;
    }
    if (drive > 0) { ledcWrite(chRev, 0);      ledcWrite(chFwd, drive); }
    else           { ledcWrite(chFwd, 0);      ledcWrite(chRev, -drive); }
  }
};

// ---------------- Hobby servo axis (non-blocking stepped movement) ----------------
// Moves in 2-degree steps every 15 ms, then holds at the target for 2 seconds.
struct ServoAxis {
  Servo s;
  int angle = 0;
  int targetAngle = 0;

  unsigned long nextStepMs = 0;
  unsigned long holdStartMs = 0;
  const unsigned long stepIntervalMs = 15;
  const unsigned long endpointHoldMs = 2000;
  bool holding = true;

  void begin(int pin, int startAngle) {
    s.setPeriodHertz(50);
    s.attach(pin, 500, 2400);

    angle = constrain(startAngle, 0, 180);
    targetAngle = angle;
    s.write(angle);

    // Start in the settled position. The first moveTo() starts a new movement.
    holding = true;
    holdStartMs = millis();
  }

  void moveTo(int deg) {
    targetAngle = constrain(deg, 0, 180);
    holding = false;
    nextStepMs = millis();
  }

  void update() {
    unsigned long now = millis();

    if (holding) return;

    if (angle != targetAngle) {
      // Signed subtraction keeps the time comparison safe across millis() rollover.
      if ((long)(now - nextStepMs) >= 0) {
        if (angle < targetAngle) {
          angle += 2;
          if (angle > targetAngle) angle = targetAngle;
        } else {
          angle -= 2;
          if (angle < targetAngle) angle = targetAngle;
        }

        s.write(angle);
        nextStepMs = now + stepIntervalMs;
      }
    }

    // Begin the endpoint dwell only after the servo reaches its target angle.
    if (angle == targetAngle) {
      holding = true;
      holdStartMs = now;
    }
  }

  bool done() {
    return holding && (millis() - holdStartMs >= endpointHoldMs);
  }
};

DCAxis    boltMotor;
DCAxis    sorter;
DCAxis    flap;
ServoAxis feeder;

// ---------------- States ----------------
enum State : byte {
  WAIT_FOR_BOLT,
  ROTATE_FEEDER_WHEEL,
  BOLT_CHAMBER,
  ROTATE_BOLT,
  BUTTON_PRESSED,      // waiting for good/bad inspection input
  ROTATE_SORTER,
  WAIT_FLAP_BUTTON,    // waiting for operator flap command
  RELEASE_FLAP,
  RESET_FLAP,
  RESET_FUNNEL,
  FAULT
};
State state = WAIT_FOR_BOLT;
State faultFrom = WAIT_FOR_BOLT;

unsigned long stateStartMs = 0;
unsigned long seatedSince = 0;
int  capture = 0;
int  sorterInitial = 0;              // 0 none, 1 good, 2 bad
bool dispenseJam = false;
bool feederAtEnd = false;
char cmd = 0;                        // last serial command this tick (0 = none)

// ---------------- Timers / interrupts ----------------
volatile bool BLUEflag = false;      // bolt button edge seen
volatile bool debounceT = false;     // debounce timer expired
volatile bool deltaT = false;        // control tick
hw_timer_t *timer0 = NULL;           // debounce
hw_timer_t *timer1 = NULL;           // control loop
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  debounceT = true;
  portEXIT_CRITICAL_ISR(&timerMux0);
  timerStop(timer0);
}
void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  deltaT = true;
  portEXIT_CRITICAL_ISR(&timerMux1);
}
void IRAM_ATTR BLUEisr() {
  BLUEflag = true;
  timerWrite(timer0, 0);
  timerStart(timer0);
}

// ---------------- Forward declarations ----------------
void setState(State next);
void fault(const char *why);
void stopAllMotors();

// ---------------- Event checkers (non-blocking) ----------------
bool timedOut(unsigned long ms) { return millis() - stateStartMs > ms; }

bool boltDetected() {
  bool hit = false;
  if (BLUEflag && debounceT) {
    noInterrupts();
    BLUEflag = false;
    debounceT = false;
    interrupts();
    hit = (digitalRead(BOLT_BTN) == ACTIVE_LEVEL);   // still pressed after debounce
  }
  if (cmd == 'k') hit = true;                          // keyboard substitute
  return hit;
}

bool rotateFeederWheel180() { return feeder.done(); }

bool boltSeated() {                  // limit switch held for SEAT_TIME_MS
  if (digitalRead(LIMIT_PIN) == ACTIVE_LEVEL) {
    if (seatedSince == 0) seatedSince = millis();
    if (millis() - seatedSince >= SEAT_TIME_MS) return true;
  } else {
    seatedSince = 0;
  }
  return false;
}

bool rotateBoltStepDone() { return boltMotor.done(); }
bool sorterRotated()      { return sorter.done(); }
bool flapButtonPress()    { return cmd == 'f'; }
bool flapReleased()       { return flap.done() && timedOut(FLAP_DWELL_MS); }
bool flapReset()          { return flap.done(); }
bool funnelReset()        { return sorter.done(); }

// ---------------- Event services ----------------
void stopAllMotors() {
  boltMotor.stop();
  sorter.stop();
  flap.stop();
  // feeder servo holds its last position
}

void fault(const char *why) {
  faultFrom = state;
  Serial.print("FAULT in state ");
  Serial.print((int)state);
  Serial.print(": ");
  Serial.println(why);
  stopAllMotors();
  setState(FAULT);
}

void setState(State next) {
  state = next;
  stateStartMs = millis();
  seatedSince = 0;

  // ---- entry actions (run once per entry) ----
  switch (next) {
    case WAIT_FOR_BOLT:
      Serial.println("[WAIT_FOR_BOLT] Press 'k' when a bolt is loaded");
      capture = 0;
      sorterInitial = 0;
      dispenseJam = false;
      noInterrupts(); BLUEflag = false; debounceT = false; interrupts();
      flap.moveTo(FLAP_CLOSED_POS);
      sorter.moveTo(SORT_HOME_POS);
      break;
    case ROTATE_FEEDER_WHEEL:
      Serial.println("[FEEDER] Rotating feeder wheel...");
      feederAtEnd = !feederAtEnd;                       // toggle 0 <-> 180
      feeder.moveTo(feederAtEnd ? FEEDER_END_ANGLE : FEEDER_START_ANGLE);
      break;
    case BOLT_CHAMBER:
      Serial.println("[CHAMBER] Waiting for bolt to seat (5 s)...");
      break;
    case ROTATE_BOLT:
      Serial.print("[ROTATE_BOLT] Step ");
      Serial.print(capture + 1);
      Serial.println(" of 6");
      boltMotor.moveBy(COUNTS_PER_REV / 6);             // 60 deg step
      break;
    case BUTTON_PRESSED:
      Serial.println("[INSPECT] Press 'g' (good) or 'b' (bad) within 10 s");
      break;
    case ROTATE_SORTER:
      Serial.println("[SORTER] Moving sorter...");
      sorter.moveTo(sorterInitial == 1 ? SORT_GOOD_POS : SORT_BAD_POS);
      break;
    case WAIT_FLAP_BUTTON:
      Serial.println("[FLAP] Press 'f' to release the flap within 10 s");
      break;
    case RELEASE_FLAP:
      Serial.println("[FLAP] Opening...");
      flap.moveTo(FLAP_OPEN_POS);
      break;
    case RESET_FLAP:
      Serial.println("[FLAP] Closing...");
      flap.moveTo(FLAP_CLOSED_POS);
      break;
    case RESET_FUNNEL:
      Serial.println("[FUNNEL] Resetting sorter...");
      sorter.moveTo(SORT_HOME_POS);
      break;
    case FAULT:
      Serial.println("[FAULT] Press 'r' to reset");
      break;
    default: break;
  }
}

// ---------------- State machine (runs every control tick) ----------------
void runStateMachine() {
  switch (state) {

    case WAIT_FOR_BOLT:
      if (boltDetected()) setState(ROTATE_FEEDER_WHEEL);
      break;

    case ROTATE_FEEDER_WHEEL:
      if (rotateFeederWheel180())            setState(BOLT_CHAMBER);
      break;                                  // servo completion includes its 2 s endpoint hold

    case BOLT_CHAMBER:
      if (boltSeated())                       setState(ROTATE_BOLT);
      else if (timedOut(SEAT_TIME_MS + 3000)) fault("bolt not seated");
      break;

    case ROTATE_BOLT:
      if (rotateBoltStepDone()) {
        capture++;
        if (capture >= 6) setState(BUTTON_PRESSED);
        else              setState(ROTATE_BOLT);    // re-enter => next 60 deg step
      } else if (timedOut(MOVE_TIMEOUT_MS)) {
        capture = 0;
        fault("bolt rotation failed");
      }
      break;

    case BUTTON_PRESSED:                      // waiting for good/bad
      if (cmd == 'g' || cmd == 'b') {
        sorterInitial = (cmd == 'g') ? 1 : 2;
        setState(ROTATE_SORTER);
      } else if (timedOut(INSPECT_TIMEOUT_MS)) {
        sorterInitial = 0;
        fault("no inspection input");
      }
      break;

    case ROTATE_SORTER:
      if (sorterRotated())                    setState(WAIT_FLAP_BUTTON);
      else if (timedOut(MOVE_TIMEOUT_MS))     fault("sorter did not rotate");
      break;

    case WAIT_FLAP_BUTTON:
      if (flapButtonPress())                  setState(RELEASE_FLAP);
      else if (timedOut(INSPECT_TIMEOUT_MS))  fault("flap button not pressed");
      break;

    case RELEASE_FLAP:
      if (flapReleased())                     setState(RESET_FLAP);
      else if (timedOut(MOVE_TIMEOUT_MS + FLAP_DWELL_MS)) fault("flap did not open");
      break;

    case RESET_FLAP:
      if (flapReset())                        setState(RESET_FUNNEL);
      else if (timedOut(MOVE_TIMEOUT_MS))     fault("flap reset failed");
      break;

    case RESET_FUNNEL:
      if (funnelReset())                      setState(WAIT_FOR_BOLT);
      else if (timedOut(MOVE_TIMEOUT_MS))     fault("funnel reset failed");
      break;

    case FAULT:
      stopAllMotors();                        // stay stopped until operator resets
      if (cmd == 'r') setState(WAIT_FOR_BOLT);
      break;

    default:
      fault("invalid state");
      break;
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(BOLT_BTN, INPUT);       // GPIO36/39 have no internal pull-ups
  pinMode(LIMIT_PIN, INPUT);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Attach the servo FIRST so it claims LEDC channel 0 (50 Hz timer).
  // Motors then use channels 2-7: LEDC channels share a timer in pairs
  // (0/1, 2/3, 4/5, 6/7), so the 50 Hz servo must not share a pair with 20 kHz PWM.
  ESP32PWM::allocateTimer(0);
  feeder.begin(FEEDER_PIN, FEEDER_START_ANGLE);

  boltMotor.begin(BOLT_ENC_A, BOLT_ENC_B, BOLT_PWM_F, BOLT_PWM_R, 2, 3);
  sorter.begin(SORT_ENC_A, SORT_ENC_B, SORT_PWM_F, SORT_PWM_R, 4, 5);
  flap.begin(FLAP_ENC_A, FLAP_ENC_B, FLAP_PWM_F, FLAP_PWM_R, 6, 7);

  // Debounce timer (one-shot, started from the button ISR)
  timer0 = timerBegin(0, 80, true);                      // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(timer0, &onTime0, true);
  timerAlarmWrite(timer0, DEBOUNCE_US, false);
  timerAlarmEnable(timer0);
  timerStop(timer0);

  // Control loop timer (periodic)
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &onTime1, true);
  timerAlarmWrite(timer1, CONTROL_PERIOD_US, true);
  timerAlarmEnable(timer1);

  attachInterrupt(digitalPinToInterrupt(BOLT_BTN), BLUEisr,
                  ACTIVE_LEVEL == LOW ? FALLING : RISING);

  setState(WAIT_FOR_BOLT);
  Serial.println("Ready. Commands: k=bolt g=good b=bad f=flap r=reset");
}

// ---------------- Main loop ----------------
void loop() {
  // Update the servo independently of the 10 ms motor-control tick.
  // This allows its 15 ms stepping interval to run without blocking the state machine.
  feeder.update();

  if (deltaT) {
    portENTER_CRITICAL(&timerMux1);
    deltaT = false;
    portEXIT_CRITICAL(&timerMux1);

    // one serial command per tick, visible only to the state that wants it
    cmd = 0;
    while (Serial.available()) {
      char c = Serial.read();
      if (c != '\n' && c != '\r') { cmd = c; break; }
    }

    boltMotor.update();
    sorter.update();
    flap.update();
    runStateMachine();
  }
}

