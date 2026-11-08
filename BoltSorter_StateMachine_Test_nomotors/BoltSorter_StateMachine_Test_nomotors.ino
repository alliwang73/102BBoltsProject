// Manual-count version of BoltSorter_StateMachine_Test.ino.
// HUZZAH32: switch C -> GND, NO -> TEST_SWITCH_PIN. No motor hardware required.
// Arduino core 2.x compatible; only Arduino.h is needed. No PWM/encoder setup.
// Serial Monitor: 115200 baud, Newline or Both NL & CR.
// Commands (one per line): g, b, r, d, x, or signed relative encoder position.
#include <Arduino.h>
#include <errno.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>

// ---------------- Only hardware placeholder: assign before uploading ----------------
constexpr int TEST_SWITCH_PIN = -1; // Select an exposed GPIO supporting INPUT_PULLUP.

// ---------------- Original test values, retained for comparison ----------------
constexpr long COUNTS_PER_REV = 1440;
constexpr long FEEDER_TEST_COUNTS = 720;
constexpr long SORTER_TEST_COUNTS = 300;
constexpr long FLAP_TEST_COUNTS = 200;
constexpr int GOOD_DIRECTION = 1; // GOOD positive, BAD negative; reverse with -1.
constexpr long POSITION_TOLERANCE = 10; // Original strict abs(error) < 10 comparison.
constexpr int SPEED_TOLERANCE = 2; // Reference only: no simulated velocity needed.
constexpr unsigned long MOVE_TIMEOUT_MS = 3000; // Reference only: NEVER enforced here.
constexpr unsigned long INSPECT_TIMEOUT_MS = 10000;
constexpr unsigned long SWITCH_TIMEOUT_MS = 10000;
constexpr unsigned long FLAP_DWELL_MS = 500;
constexpr unsigned long CONTROL_PERIOD_US = 10000; // Reference only: no control tick.
constexpr unsigned long DEBOUNCE_MS = 20;
// Enable only if desired. Counts always wait indefinitely for manual entry.
constexpr bool ENABLE_INSPECTION_TIMEOUT = false;
constexpr bool ENABLE_SWITCH_TIMEOUT = false;
constexpr size_t SERIAL_LINE_CAPACITY = 48; // Includes terminating NUL.

enum State : uint8_t {
  WAIT_FOR_BOLT, ROTATE_FEEDER_WHEEL, BOLT_CHAMBER, ROTATE_BOLT,
  BUTTON_PRESSED, ROTATE_SORTER, WAIT_FLAP_BUTTON, RELEASE_FLAP,
  RESET_FLAP, RESET_FUNNEL, FAULT
};

// Explicit declarations keep Arduino's prototype generation unambiguous.
const char *stateName(State s);
void enterState(State next);
void fault(const char *reason);
void diagnostic();
void processLine(char *line, char &cmd, bool &moveDone);
void readSerial(char &cmd, bool &moveDone);
void runStateMachine(bool pressed, char cmd, bool moveDone);
bool validSwitchPin();

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

// Same debounced falling-edge behavior as the motor-enabled test.
// A switch held at startup does not count until released and pressed again.
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

// Manual replacement for TestMotor: NO encoder, PWM, or automatic progress.
struct ManualMotor {
  bool enabled = false, hasEntry = false;
  long pos = 0, target = 0;
  void stop() { enabled = false; }
  void prompt() { Serial.println("[INPUT] Enter simulated relative encoder count (signed integer):"); }
  void moveBy(long counts) {
    stop();
    pos = 0; target = counts; hasEntry = false; enabled = true;
    Serial.printf("[MOTOR SIM] Target: %+ld counts; direction=%s; relative position reset to 0\n",
                  target, target >= 0 ? "forward (+)" : "reverse (-)");
    prompt();
  }
  bool submit(long entered) {
    if (!enabled) return false;
    pos = entered; hasEntry = true;
    // Widen first: extreme valid 32-bit input cannot overflow error arithmetic.
    int64_t error = (int64_t)target - (int64_t)pos;
    Serial.printf("[ENCODER SIM] Entered: %ld; target: %ld\n", pos, target);
    if (error > -POSITION_TOLERANCE && error < POSITION_TOLERANCE) {
      stop(); // Accepted position is assumed stopped; no velocity check.
      Serial.printf("[ENCODER SIM] Target reached: %ld / %ld; simulated motor stopped\n", pos, target);
      return true;
    }
    Serial.printf("[ENCODER SIM] Target not reached; remaining signed error=%lld counts\n", (long long)error);
    prompt();
    return false;
  }
};

TestSwitch testSwitch;
ManualMotor motor;
State state = WAIT_FOR_BOLT;
unsigned long stateStartMs = 0, phaseStartMs = 0;
int boltStep = 0;
long sorterDelta = 0;
char inspection = 0;
bool hardwareReady = false, sorterAtTarget = false, flapAtTarget = false;
char serialLine[SERIAL_LINE_CAPACITY];
size_t serialLength = 0;
bool serialOverflow = false;

void fault(const char *reason) {
  motor.stop();
  Serial.printf("[FAULT] State=%s; reason=%s\n", stateName(state), reason);
  enterState(FAULT);
}

void enterState(State next) {
  motor.stop();
  Serial.printf("[TRANSITION] %s -> %s\n", stateName(state), stateName(next));
  state = next; stateStartMs = millis();
  Serial.printf("[STATE] %s\n", stateName(state));
  switch (state) {
    case WAIT_FOR_BOLT:
      boltStep = 0; sorterDelta = 0; inspection = 0;
      sorterAtTarget = flapAtTarget = false;
      motor.pos = motor.target = 0; motor.hasEntry = false;
      Serial.println("[ACTION] Release switch, then press to detect a bolt.");
      break;
    case ROTATE_FEEDER_WHEEL:
      Serial.println("[ACTION] Enter feeder substitute's simulated encoder position.");
      motor.moveBy(FEEDER_TEST_COUNTS);
      break;
    case BOLT_CHAMBER:
      Serial.println("[ACTION] Fresh switch press to simulate seated bolt (no 5 s hold).");
      break;
    case ROTATE_BOLT:
      Serial.printf("[ACTION] Bolt rotation step %d of 6; enter relative encoder position.\n", boltStep + 1);
      motor.moveBy(COUNTS_PER_REV / 6);
      break;
    case BUTTON_PRESSED:
      Serial.println("[ACTION] Send g=GOOD or b=BAD on its own line.");
      break;
    case ROTATE_SORTER:
      sorterAtTarget = false;
      Serial.println("[ACTION] Enter sorter position, THEN press switch to confirm selected endpoint.");
      motor.moveBy(sorterDelta);
      break;
    case WAIT_FLAP_BUTTON:
      Serial.println("[ACTION] Release, then press switch again to release flap.");
      break;
    case RELEASE_FLAP:
      flapAtTarget = false;
      Serial.println("[ACTION] Enter flap opening position; 500 ms dwell follows acceptance.");
      motor.moveBy(FLAP_TEST_COUNTS);
      break;
    case RESET_FLAP:
      Serial.println("[ACTION] Enter reverse flap reset position.");
      motor.moveBy(-FLAP_TEST_COUNTS);
      break;
    case RESET_FUNNEL:
      Serial.println("[ACTION] Enter reverse funnel reset position (test only).");
      motor.moveBy(-sorterDelta);
      break;
    case FAULT:
      Serial.println("[ACTION] Simulation stopped. Send r to recover, d for diagnostics.");
      break;
  }
}

void diagnostic() {
  bool switchExpected = state == WAIT_FOR_BOLT || state == BOLT_CHAMBER ||
                        state == WAIT_FLAP_BUTTON || (state == ROTATE_SORTER && sorterAtTarget);
  const char *decision = inspection == 'g' ? "GOOD" : inspection == 'b' ? "BAD" : "none";
  Serial.printf("[DIAGNOSTIC] state=%s switchReady=%d switchExpected=%d encoderExpected=%d\n",
                stateName(state), hardwareReady, switchExpected, motor.enabled);
  Serial.printf("[DIAGNOSTIC] target=%ld latest=%ld hasEntry=%d inspection=%s boltStep=%d\n",
                motor.target, motor.pos, motor.hasEntry, decision, boltStep + (state == ROTATE_BOLT ? 1 : 0));
  Serial.printf("[DIAGNOSTIC] inspectionTimeout=%d switchTimeout=%d movementTimeout=disabled\n",
                ENABLE_INSPECTION_TIMEOUT, ENABLE_SWITCH_TIMEOUT);
  if (hardwareReady) Serial.printf("[DIAGNOSTIC] switchRaw=%d switchStable=%d\n",
                                   digitalRead(TEST_SWITCH_PIN), testSwitch.stable);
  if (!motor.enabled && state == BUTTON_PRESSED) Serial.println("[DIAGNOSTIC] Expected: g or b.");
  if (state == FAULT) Serial.println("[DIAGNOSTIC] Expected: r (after valid switch GPIO configuration).");
  if (state == RELEASE_FLAP && flapAtTarget) Serial.println("[DIAGNOSTIC] Expected: automatic dwell completion.");
}

// Each line yields at most one command OR one manual movement-completion event.
void processLine(char *line, char &cmd, bool &moveDone) {
  while (isspace((unsigned char)*line)) ++line;
  size_t length = strlen(line);
  while (length && isspace((unsigned char)line[length - 1])) line[--length] = '\0';
  if (!length) return; // Includes the LF half of CRLF and blank lines.
  if (length == 1 && (line[0] == 'g' || line[0] == 'b' || line[0] == 'r' || line[0] == 'd' || line[0] == 'x')) {
    char c = line[0];
    if (c == 'd') diagnostic();
    else if (c == 'x') fault("manual x command");
    else if ((c == 'g' || c == 'b') && state == BUTTON_PRESSED) cmd = c;
    else if (c == 'r' && state == FAULT) cmd = c;
    else Serial.printf("[INPUT] Command %c does not apply in %s. Send d for guidance.\n", c, stateName(state));
    return;
  }
  // Require optional sign followed ONLY by decimal digits; reject 720abc, 2.4, etc.
  char *digits = line;
  if (*digits == '+' || *digits == '-') ++digits;
  bool valid = *digits != '\0';
  for (char *p = digits; *p; ++p) if (!isdigit((unsigned char)*p)) valid = false;
  errno = 0;
  char *end = nullptr;
  long long value = valid ? strtoll(line, &end, 10) : 0;
  if (!valid || errno == ERANGE || !end || *end || value < INT32_MIN || value > INT32_MAX) {
    Serial.println("[INPUT] Invalid line. Use g/b/r/d/x or a signed 32-bit integer, e.g. 720 or -300.");
    return;
  }
  if (!motor.enabled) {
    Serial.println("[INPUT] No encoder position is expected now. Send d for guidance.");
    return;
  }
  moveDone = motor.submit((long)value);
}

// Bounded work, fixed buffer, no parseInt/readStringUntil waits or dynamic Strings.
// Stop at each line ending so one line cannot complete two successive movements.
void readSerial(char &cmd, bool &moveDone) {
  for (int budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialOverflow) Serial.println("[INPUT] Line too long; discarded. Send one command or signed integer per line.");
      else { serialLine[serialLength] = '\0'; processLine(serialLine, cmd, moveDone); }
      serialLength = 0; serialOverflow = false;
      return;
    }
    // Reject embedded control characters rather than silently changing a number.
    if ((unsigned char)c < 32 && c != '\t') { serialOverflow = true; continue; }
    if (serialLength + 1 < SERIAL_LINE_CAPACITY && !serialOverflow) serialLine[serialLength++] = c;
    else serialOverflow = true;
  }
}

void runStateMachine(bool pressed, char cmd, bool moveDone) {
  switch (state) {
    case WAIT_FOR_BOLT:
      if (pressed) { Serial.println("[EVENT] Bolt detected."); enterState(ROTATE_FEEDER_WHEEL); }
      else if (ENABLE_SWITCH_TIMEOUT && millis() - stateStartMs >= SWITCH_TIMEOUT_MS) fault("no bolt detection press");
      break;
    case ROTATE_FEEDER_WHEEL:
      if (moveDone) enterState(BOLT_CHAMBER);
      break;
    case BOLT_CHAMBER:
      if (pressed) { Serial.println("[EVENT] Bolt seated."); enterState(ROTATE_BOLT); }
      else if (ENABLE_SWITCH_TIMEOUT && millis() - stateStartMs >= SWITCH_TIMEOUT_MS) fault("no seated-bolt press");
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
        inspection = cmd;
        Serial.println(cmd == 'g' ? "[INSPECT] GOOD selected." : "[INSPECT] BAD selected.");
        sorterDelta = (cmd == 'g' ? GOOD_DIRECTION : -GOOD_DIRECTION) * SORTER_TEST_COUNTS;
        enterState(ROTATE_SORTER);
      } else if (ENABLE_INSPECTION_TIMEOUT && millis() - stateStartMs >= INSPECT_TIMEOUT_MS) fault("no inspection input");
      break;
    case ROTATE_SORTER:
      if (moveDone) {
        sorterAtTarget = true; phaseStartMs = millis();
        Serial.println("[ACTION] Sorter stopped. Release switch, then press to confirm selected endpoint.");
      } else if (sorterAtTarget) {
        if (pressed) { Serial.println("[EVENT] Sorter endpoint confirmed."); enterState(WAIT_FLAP_BUTTON); }
        else if (ENABLE_SWITCH_TIMEOUT && millis() - phaseStartMs >= SWITCH_TIMEOUT_MS) fault("no sorter endpoint confirmation");
      }
      break;
    case WAIT_FLAP_BUTTON:
      if (pressed) { Serial.println("[EVENT] Flap release requested."); enterState(RELEASE_FLAP); }
      else if (ENABLE_SWITCH_TIMEOUT && millis() - stateStartMs >= SWITCH_TIMEOUT_MS) fault("no flap release press");
      break;
    case RELEASE_FLAP:
      if (moveDone) { flapAtTarget = true; phaseStartMs = millis(); }
      if (flapAtTarget && millis() - phaseStartMs >= FLAP_DWELL_MS) enterState(RESET_FLAP);
      break;
    case RESET_FLAP:
      if (moveDone) enterState(RESET_FUNNEL);
      break;
    case RESET_FUNNEL:
      if (moveDone) { Serial.println("[CYCLE] Full sorting cycle complete."); enterState(WAIT_FOR_BOLT); }
      break;
    case FAULT:
      if (cmd == 'r') {
        if (hardwareReady) { Serial.println("[RECOVERY] Restarting sequence."); enterState(WAIT_FOR_BOLT); }
        else Serial.println("[FAULT] Assign a valid TEST_SWITCH_PIN, rebuild and upload before recovery.");
      }
      break;
    default: fault("invalid state"); break;
  }
}

bool validSwitchPin() {
  int p = TEST_SWITCH_PIN;
  // Classic ESP32: reject flash pins, nonexistent pins, and inputs without pull-ups.
  return p >= 0 && p <= 33 && !(p >= 6 && p <= 11) && p != 20 && p != 24 && !(p >= 28 && p <= 31);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n[TEST] No motors: one switch plus manually entered relative counts.");
  Serial.println("[INPUT] 115200 baud; Newline or Both NL & CR. g/b/r/d/x or signed count per line.");
  if (!validSwitchPin()) { fault("assign TEST_SWITCH_PIN to an exposed GPIO with internal pull-up"); return; }
  testSwitch.begin(); hardwareReady = true;
  enterState(WAIT_FOR_BOLT);
}

void loop() {
  bool pressed = hardwareReady && testSwitch.update();
  if (pressed) Serial.printf("[EVENT] Valid switch press in %s\n", stateName(state));
  char cmd = 0;
  bool moveDone = false;
  readSerial(cmd, moveDone);
  // A press coincident with accepted counts belongs to the old movement only.
  // The sorter completion branch ignores it; no event is queued for later states.
  runStateMachine(pressed, cmd, moveDone);
}
