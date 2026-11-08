// Standalone HUZZAH32 bench test. ESP32 Arduino core 2.x (same as production).
// Requires ESP32Encoder; no servo library or servo is used.
// Fill in ALL five GPIO placeholders before running. Use 3.3 V encoder signals.
// Switch: C -> GND, NO -> TEST_SWITCH_PIN. Motor connects through its driver.
// Serial 115200: g=GOOD, b=BAD, r=recover FAULT, d=optional diagnostic.
#include <Arduino.h>
#include <ESP32Encoder.h>

// ---------------- Hardware placeholders: NOT final pin assignments ----------------
constexpr int TEST_SWITCH_PIN = -1; // Choose an input with internal pull-up support.
constexpr int MOTOR_ENC_A = -1;
constexpr int MOTOR_ENC_B = -1;
constexpr int MOTOR_PWM_F = -1;     // Driver forward PWM input
constexpr int MOTOR_PWM_R = -1;     // Driver reverse PWM input
constexpr uint8_t PWM_CH_F = 2, PWM_CH_R = 3;
constexpr int ENCODER_SIGN = 1;     // Change to -1 if forward gives negative counts.

// ---------------- TEST values: calibrate for this N20 and encoder ----------------
// Counts are as measured by attachHalfQuad(), including gearbox ratio.
constexpr long COUNTS_PER_REV = 1440;
constexpr long FEEDER_TEST_COUNTS = 720;
constexpr long SORTER_TEST_COUNTS = 300;
constexpr long FLAP_TEST_COUNTS = 200;
constexpr int GOOD_DIRECTION = 1;   // Change to -1 to reverse GOOD/BAD mapping.
constexpr long POSITION_TOLERANCE = 10;
constexpr int SPEED_TOLERANCE = 2;  // Counts per 10 ms control tick
constexpr unsigned long MOVE_TIMEOUT_MS = 3000;
constexpr unsigned long INSPECT_TIMEOUT_MS = 10000;
constexpr unsigned long SWITCH_TIMEOUT_MS = 10000;
constexpr unsigned long FLAP_DWELL_MS = 500;
constexpr unsigned long CONTROL_PERIOD_US = 10000;
constexpr unsigned long DEBOUNCE_MS = 20;

enum State : uint8_t {
  WAIT_FOR_BOLT, ROTATE_FEEDER_WHEEL, BOLT_CHAMBER, ROTATE_BOLT,
  BUTTON_PRESSED, ROTATE_SORTER, WAIT_FLAP_BUTTON, RELEASE_FLAP,
  RESET_FLAP, RESET_FUNNEL, FAULT
};

const char *stateName(State s) {
  switch (s) {
    case WAIT_FOR_BOLT: return "WAIT_FOR_BOLT";
    case ROTATE_FEEDER_WHEEL: return "ROTATE_FEEDER_WHEEL";
    case BOLT_CHAMBER: return "BOLT_CHAMBER";
    case ROTATE_BOLT: return "ROTATE_BOLT";
    case BUTTON_PRESSED: return "BUTTON_PRESSED";
    case ROTATE_SORTER: return "ROTATE_SORTER";
    case WAIT_FLAP_BUTTON: return "WAIT_FLAP_BUTTON";
    case RELEASE_FLAP: return "RELEASE_FLAP";
    case RESET_FLAP: return "RESET_FLAP";
    case RESET_FUNNEL: return "RESET_FUNNEL";
    case FAULT: return "FAULT";
    default: return "INVALID";
  }
}

// One debounced falling-edge event, rearmed only by a debounced release.
// Starting with the switch held does not count as a new press.
struct TestSwitch {
  int raw = HIGH, stable = HIGH;
  unsigned long changedMs = 0;
  bool armed = false;
  void begin() {
    pinMode(TEST_SWITCH_PIN, INPUT_PULLUP);
    raw = stable = digitalRead(TEST_SWITCH_PIN);
    changedMs = millis();
    armed = false;
  }
  bool update() {
    int reading = digitalRead(TEST_SWITCH_PIN);
    if (reading != raw) { raw = reading; changedMs = millis(); }
    if (millis() - changedMs < DEBOUNCE_MS) return false;
    if (raw == HIGH) { stable = HIGH; armed = true; return false; }
    if (stable != LOW) {
      stable = LOW;
      if (armed) { armed = false; return true; }
    }
    return false;
  }
};

// Same cascaded position/velocity PWM architecture as the production sketch.
// Read the cumulative hardware count without clearing it on every control tick.
struct TestMotor {
  ESP32Encoder enc;
  bool ready = false, enabled = false;
  int64_t origin = 0, previous = 0;
  long pos = 0, target = 0;
  int speed = 0, speedLast = 0, drive = 0;
  long sumErrV = 0;
  float kp = 20, ki = 0, kd = 5, kpPos = 0.2;
  unsigned long moveStartMs = 0;

  int64_t count() { return enc.getCount() * ENCODER_SIGN; }
  void begin() {
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    enc.attachHalfQuad(MOTOR_ENC_A, MOTOR_ENC_B);
    enc.clearCount();
    ledcSetup(PWM_CH_F, 20000, 8); ledcAttachPin(MOTOR_PWM_F, PWM_CH_F);
    ledcSetup(PWM_CH_R, 20000, 8); ledcAttachPin(MOTOR_PWM_R, PWM_CH_R);
    ready = true;
    stop();
  }
  void stop() {
    enabled = false; drive = 0;
    if (!ready) return;
    ledcWrite(PWM_CH_F, 0); ledcWrite(PWM_CH_R, 0);
  }
  void moveBy(long counts) {
    stop();
    origin = previous = count(); // Relative target rebased to actual shaft position.
    pos = 0; target = counts; speed = speedLast = 0; sumErrV = 0;
    moveStartMs = millis();
    enabled = true;
    Serial.printf("[MOTOR] Moving %s%ld counts; relative target=%ld, origin=%lld\n",
                  counts >= 0 ? "+" : "", counts, target, (long long)origin);
  }
  // Returns true ONCE per real, completed movement and stops before reporting.
  bool update() {
    int64_t current = count();
    speed = (int)(current - previous); previous = current;
    pos = (long)(current - origin);
    if (!enabled) return false;
    long errPos = target - pos;
    if (abs(errPos) < POSITION_TOLERANCE && abs(speed) <= SPEED_TOLERANCE) {
      stop();
      Serial.printf("[MOTOR] Target reached; relative=%ld, target=%ld, encoder=%lld\n",
                    pos, target, (long long)current);
      return true;
    }
    int omegaDes = (int)(kpPos * errPos);
    int errV = omegaDes - speed;
    sumErrV += errV;
    int dV = speed - speedLast; speedLast = speed;
    drive = (int)(kp * errV + ki * sumErrV + kd * dV);
    if (abs(drive) > 255) {
      sumErrV -= errV;
      drive = drive > 0 ? 255 : -255;
    }
    if (drive > 0) { ledcWrite(PWM_CH_R, 0); ledcWrite(PWM_CH_F, drive); }
    else { ledcWrite(PWM_CH_F, 0); ledcWrite(PWM_CH_R, -drive); }
    return false;
  }
};

TestSwitch testSwitch;
TestMotor motor;
State state = WAIT_FOR_BOLT;
unsigned long stateStartMs = 0, phaseStartMs = 0, lastControlUs = 0;
int boltStep = 0;
long sorterDelta = 0;
bool hardwareReady = false, sorterAtTarget = false, flapAtTarget = false;

void enterState(State next);
void fault(const char *reason);
bool validPins();

void fault(const char *reason) {
  motor.stop(); // Stop before serial output, including on timeout.
  Serial.printf("[FAULT] State=%s; reason=%s\n", stateName(state), reason);
  enterState(FAULT);
}

void enterState(State next) {
  motor.stop(); // All states share one motor; never overlap commands.
  Serial.printf("[TRANSITION] %s -> %s\n", stateName(state), stateName(next));
  state = next;
  stateStartMs = millis();
  Serial.printf("[STATE] %s\n", stateName(state));
  switch (state) {
    case WAIT_FOR_BOLT:
      boltStep = 0; sorterDelta = 0;
      Serial.println("[ACTION] Release switch, then press to detect a bolt.");
      break;
    case ROTATE_FEEDER_WHEEL:
      Serial.println("[ACTION] Encoder-controlled feeder substitute; advances automatically.");
      motor.moveBy(FEEDER_TEST_COUNTS);
      break;
    case BOLT_CHAMBER:
      Serial.println("[ACTION] Fresh switch press to simulate seated bolt (no 5 s hold).");
      break;
    case ROTATE_BOLT:
      Serial.printf("[ACTION] Bolt rotation step %d of 6; waiting for encoder.\n", boltStep + 1);
      motor.moveBy(COUNTS_PER_REV / 6);
      break;
    case BUTTON_PRESSED:
      Serial.println("[ACTION] Send g=GOOD or b=BAD within 10 seconds.");
      break;
    case ROTATE_SORTER:
      sorterAtTarget = false;
      Serial.println("[ACTION] Wait for encoder target, THEN press switch to confirm endpoint.");
      motor.moveBy(sorterDelta);
      break;
    case WAIT_FLAP_BUTTON:
      Serial.println("[ACTION] Release, then press switch again to release flap (10 s).");
      break;
    case RELEASE_FLAP:
      flapAtTarget = false;
      Serial.println("[ACTION] Encoder-controlled flap opening, then 500 ms dwell.");
      motor.moveBy(FLAP_TEST_COUNTS);
      break;
    case RESET_FLAP:
      Serial.println("[ACTION] Encoder-controlled flap reset.");
      motor.moveBy(-FLAP_TEST_COUNTS);
      break;
    case RESET_FUNNEL:
      Serial.println("[ACTION] Encoder-controlled funnel reset (bench test only).");
      motor.moveBy(-sorterDelta);
      break;
    case FAULT:
      Serial.println("[ACTION] Motor stopped. Send r to recover, d for diagnostic.");
      break;
  }
}

void diagnostic() {
  Serial.printf("[DIAGNOSTIC] state=%s ready=%d enabled=%d PWM=%d\n",
                stateName(state), hardwareReady, motor.enabled, motor.drive);
  if (hardwareReady) {
    Serial.printf("[DIAGNOSTIC] encoder=%lld relative=%ld target=%ld speed=%d switchRaw=%d switchStable=%d\n",
                  (long long)motor.count(), motor.pos, motor.target, motor.speed,
                  digitalRead(TEST_SWITCH_PIN), testSwitch.stable);
  } else Serial.println("[DIAGNOSTIC] GPIO configuration incomplete; hardware not initialized.");
}

// Events exist for this loop pass only. A press in any other state is discarded.
void runStateMachine(bool pressed, char cmd, bool moveDone) {
  switch (state) {
    case WAIT_FOR_BOLT:
      if (pressed) {
        Serial.println("[EVENT] Bolt detected."); enterState(ROTATE_FEEDER_WHEEL);
      }
      break;
    case ROTATE_FEEDER_WHEEL:
      if (moveDone) enterState(BOLT_CHAMBER);
      break;
    case BOLT_CHAMBER:
      if (pressed) {
        Serial.println("[EVENT] Bolt seated."); enterState(ROTATE_BOLT);
      } else if (millis() - stateStartMs >= SWITCH_TIMEOUT_MS) fault("no seated-bolt press");
      break;
    case ROTATE_BOLT:
      if (moveDone) {
        ++boltStep;
        if (boltStep == 6) enterState(BUTTON_PRESSED);
        else enterState(ROTATE_BOLT);
      }
      break;
    case BUTTON_PRESSED:
      if (cmd == 'g' || cmd == 'b') {
        Serial.println(cmd == 'g' ? "[INSPECT] GOOD selected." : "[INSPECT] BAD selected.");
        sorterDelta = (cmd == 'g' ? GOOD_DIRECTION : -GOOD_DIRECTION) * SORTER_TEST_COUNTS;
        enterState(ROTATE_SORTER);
      } else if (millis() - stateStartMs >= INSPECT_TIMEOUT_MS) fault("no inspection input");
      break;
    case ROTATE_SORTER:
      if (moveDone) {
        sorterAtTarget = true; phaseStartMs = millis();
        Serial.println("[ACTION] Sorter stopped. Release switch, then press to confirm endpoint (10 s).");
        // A press coincident with completion is not used as endpoint confirmation.
      } else if (sorterAtTarget) {
        if (pressed) {
          Serial.println("[EVENT] Sorter endpoint confirmed."); enterState(WAIT_FLAP_BUTTON);
        } else if (millis() - phaseStartMs >= SWITCH_TIMEOUT_MS) fault("no sorter endpoint confirmation");
      }
      break;
    case WAIT_FLAP_BUTTON:
      if (pressed) {
        Serial.println("[EVENT] Flap release requested."); enterState(RELEASE_FLAP);
      } else if (millis() - stateStartMs >= SWITCH_TIMEOUT_MS) fault("no flap release press");
      break;
    case RELEASE_FLAP:
      if (moveDone) { flapAtTarget = true; phaseStartMs = millis(); }
      if (flapAtTarget && millis() - phaseStartMs >= FLAP_DWELL_MS) enterState(RESET_FLAP);
      break;
    case RESET_FLAP:
      if (moveDone) enterState(RESET_FUNNEL);
      break;
    case RESET_FUNNEL:
      if (moveDone) {
        Serial.println("[CYCLE] Full sorting cycle complete."); enterState(WAIT_FOR_BOLT);
      }
      break;
    case FAULT:
      if (cmd == 'r') {
        if (hardwareReady) enterState(WAIT_FOR_BOLT);
        else Serial.println("[FAULT] Assign valid GPIOs, rebuild and upload before recovery.");
      }
      break;
    default: fault("invalid state"); break;
  }
}

// Classic ESP32 / HUZZAH32 GPIO checks: exclude flash pins and duplicate pins.
bool validPins() {
  const int pins[] = {TEST_SWITCH_PIN, MOTOR_ENC_A, MOTOR_ENC_B, MOTOR_PWM_F, MOTOR_PWM_R};
  for (int i = 0; i < 5; ++i) {
    int p = pins[i];
    if (p < 0 || p > 39 || (p >= 6 && p <= 11) || p == 20 || p == 24 ||
        (p >= 28 && p <= 31)) return false;
    for (int j = 0; j < i; ++j) if (p == pins[j]) return false;
  }
  // GPIO34-39 are input-only and have no internal pull-ups.
  return TEST_SWITCH_PIN < 34 && MOTOR_PWM_F < 34 && MOTOR_PWM_R < 34;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n[TEST] One N20 + encoder + switch; ESP32 Arduino core 2.x.");
  if (!validPins()) {
    fault("assign five distinct, board-accessible GPIOs at top of sketch");
    return;
  }
  motor.begin();
  testSwitch.begin();
  hardwareReady = true;
  lastControlUs = micros();
  enterState(WAIT_FOR_BOLT);
}

void loop() {
  bool pressed = hardwareReady && testSwitch.update();
  if (pressed) Serial.printf("[EVENT] Valid switch press in %s\n", stateName(state));

  // Bound serial processing to one byte per pass so motor checks remain responsive.
  char cmd = Serial.available() ? (char)Serial.read() : 0;
  if (cmd == 'd') diagnostic(); // Optional diagnostic; never advances a state.

  // Check deadline before another drive update; no artificial completion on failure.
  if (motor.enabled && millis() - motor.moveStartMs >= MOVE_TIMEOUT_MS) {
    fault("motor did not reach encoder target within 3000 ms");
    return;
  }
  bool moveDone = false;
  unsigned long nowUs = micros();
  if (hardwareReady && nowUs - lastControlUs >= CONTROL_PERIOD_US) {
    lastControlUs = nowUs;
    moveDone = motor.update();
  }
  runStateMachine(pressed, cmd, moveDone);
}
