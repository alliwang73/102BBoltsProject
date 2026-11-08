# Single-motor state-machine bench test

Open `BoltSorter_StateMachine_Test.ino` in Arduino IDE. This is a separate sketch;
the production `bolttransitioncode.ino` is unchanged.

Use ESP32 Arduino **core 2.x**, as in the production sketch, and install the
ESP32Encoder library. Select the Adafruit ESP32 Feather board for the HUZZAH32.
This sketch uses the core 2.x `ledcSetup` / `ledcAttachPin` APIs; core 3.x requires
a PWM API port. No hobby servo is connected or required.

## Configuration and wiring

Fill in these five GPIO placeholders, currently `-1`:

| Constant | Connection |
| --- | --- |
| `TEST_SWITCH_PIN` | Switch NO; switch C connects to GND |
| `MOTOR_ENC_A` | N20 quadrature encoder A |
| `MOTOR_ENC_B` | N20 quadrature encoder B |
| `MOTOR_PWM_F` | Motor driver's forward PWM input |
| `MOTOR_PWM_R` | Motor driver's reverse PWM input |

Use five distinct GPIOs exposed by the board. The switch input must support an
internal pull-up; GPIO34–39 do not. Motor PWM pins must support output. Avoid
pins reserved by board peripherals. Use 3.3 V encoder signal levels and a common
ground with the driver and encoder. Power the motor through its driver and a
suitable supply, never an ESP32 GPIO. The motor driver uses the same two-input,
20 kHz, 8-bit PWM interface as production. Configure any driver enable input
according to the driver's requirements.

The sketch refuses to initialize motor hardware with unset or invalid pins.
Calibrate `COUNTS_PER_REV` for `attachHalfQuad()` counts at the gearbox output;
the supplied 1440 is only a test placeholder. All other movement counts, gains,
tolerances, and `GOOD_DIRECTION` are also bench-test settings. Positive motor
drive must produce positive logical counts; adjust `ENCODER_SIGN` if necessary.
Use a freely rotating bench motor: the one motor represents several mechanisms.
This encoder-based sorter test is not the production endpoint-stop controller.

## State progression

| State | Required completion event |
| --- | --- |
| `WAIT_FOR_BOLT` | Fresh switch press |
| `ROTATE_FEEDER_WHEEL` | Encoder target reached |
| `BOLT_CHAMBER` | Fresh switch press; no seating hold |
| `ROTATE_BOLT` | Six automatic 60-degree encoder movements |
| `BUTTON_PRESSED` | Serial `g` or `b` |
| `ROTATE_SORTER` | Encoder target reached, motor stopped, then fresh switch press |
| `WAIT_FLAP_BUTTON` | Another fresh switch press |
| `RELEASE_FLAP` | Encoder target reached, then 500 ms stopped dwell |
| `RESET_FLAP` | Reverse flap movement reaches encoder target |
| `RESET_FUNNEL` | Reverse selected sorter movement reaches encoder target |
| `FAULT` | Serial `r` returns to waiting, once hardware is configured |

`RESET_FUNNEL` intentionally remains in this requested bench sequence even
though the updated production sketch no longer centers the sorter. Test targets
are relative to the actual encoder position at the start of each movement.
Reset movements reverse the configured test displacement; they are not homing.

## One complete cycle

1. Assign GPIOs, compile/upload, and open Serial Monitor at **115200 baud**.
2. Release the switch for at least 20 ms, then press and release to detect a bolt.
3. Wait for the motor's feeder movement to finish and `BOLT_CHAMBER` to appear.
   Press and release again to indicate that the bolt is seated.
4. Wait for all six bolt rotation steps. Send `g` or `b` when prompted.
5. Wait for the sorter encoder target and the endpoint-confirmation prompt.
   Press and release to confirm the selected endpoint.
6. At `WAIT_FLAP_BUTTON`, press and release again. Opening, dwell, flap reset,
   and funnel reset happen automatically. Look for `[CYCLE]` and `WAIT_FOR_BOLT`.
7. Repeat with the other inspection result to check the opposite sorter direction.

A held switch cannot satisfy multiple transitions. Presses during movement or
inspection are logged but discarded, including a press coincident with sorter
movement completion. Release and press again after the appropriate prompt.
Switch and inspection prompts time out after 10 seconds (except initial bolt
waiting). Every movement has its own 3-second deadline; sorter confirmation and
flap dwell have separate timers after motor completion.

Send **`d`** for an optional encoder/control diagnostic. It does not change state.
If a movement fails, the motor stops and the sketch enters `FAULT`; send **`r`**
to recover. Recovery starts a new logical cycle without homing the motor.
There is no serial `k` or `f` substitute in this test.

## Verification status

Source structure and required state coverage were checked. Arduino compilation
and physical motor/switch testing have not been performed in this environment.
