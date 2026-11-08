# No-motors state-machine test

Open `BoltSorter_StateMachine_Test_nomotors.ino` from this folder in Arduino IDE.
Select the HUZZAH32 / Adafruit ESP32 Feather board and use Serial Monitor at
**115200 baud**, with **Newline** or **Both NL & CR**. CR alone is also accepted.
The sketch uses only the standard Arduino core, with no encoder or servo library,
and no PWM initialization. It is compatible with the production sketch's ESP32
Arduino core 2.x APIs; it does not need the old motor PWM APIs.

The production sketch and motor-enabled test sketch are left unchanged.

## Hardware and configuration

Only **`TEST_SWITCH_PIN = -1`** needs a GPIO assignment. Choose a GPIO exposed on
the HUZZAH32 supporting an internal pull-up; do not use GPIO34–39. Wire switch
**C to GND** and **NO to that GPIO**. Avoid pins reserved by the board. No motor,
motor driver, encoder, or servo should be connected for this test.

Leaving the GPIO unset enters `FAULT` without configuring a pin. Set it, rebuild,
and upload before testing. `r` cannot bypass invalid GPIO configuration.

## Manual movement behavior

Every movement resets its relative position to zero and prints a signed target.
Enter a signed decimal integer on its own line. This represents the resulting
position for that movement, not an increment and not a cycle-wide total.
For example, entering `100` then `720` means two position readings, not 820 counts.

The original strict tolerance is retained: **absolute position error must be less
than 10 counts**. Thus, a 720-count target accepts 711–729; 710 and 730 do not
complete it. Wrong counts print the signed error and request another value.
Correct counts assume the motor has stopped and produce one completion event.
There is no automatic count generation, velocity check, or movement timeout.

The parser accepts optional signs and surrounding whitespace; malformed input,
32-bit integer overflow, and oversized lines are rejected. `g`, `b`, `r`, `d`,
and `x` are commands, not count entries. Counts are rejected when no movement is
pending. Commands in the wrong state produce guidance. Use one response per
prompt, rather than pasting future movement answers ahead of time.

All eleven states and the original motor-enabled test sequence remain:

| State | What advances it |
| --- | --- |
| `WAIT_FOR_BOLT` | Fresh switch press |
| `ROTATE_FEEDER_WHEEL` | Accepted manual count |
| `BOLT_CHAMBER` | Fresh switch press |
| `ROTATE_BOLT` | Six separate accepted manual counts |
| `BUTTON_PRESSED` | Serial `g` or `b` |
| `ROTATE_SORTER` | Accepted manual count, then fresh switch press |
| `WAIT_FLAP_BUTTON` | Another fresh switch press |
| `RELEASE_FLAP` | Accepted manual count, then non-blocking 500 ms dwell |
| `RESET_FLAP` | Accepted manual reverse count |
| `RESET_FUNNEL` | Accepted manual reverse count |
| `FAULT` | Serial `r`, once GPIO configuration is valid |

Release between presses for at least 20 ms; keep presses stable for at least
20 ms too. A switch held at startup must first be released. Presses during count
entry or inspection are logged and discarded. A press coincident with sorter
count acceptance does not also confirm the endpoint. The same held press cannot
confirm the sorter and release the flap.

Send **`d`** for state, expected input, target, latest submitted count, whether a
count has been submitted for this movement, inspection decision, switch status,
and timeout settings. Send **`x`** from any state to test `FAULT`; send **`r`**
from `FAULT` to restart at `WAIT_FOR_BOLT`. Recovery cancels any pending movement.

## Centralized values and assumptions

| Setting | Value / purpose |
| --- | --- |
| `TEST_SWITCH_PIN` | `-1`, only remaining GPIO placeholder |
| `COUNTS_PER_REV` | 1440 test counts/revolution |
| Each bolt step | +240 counts, six independent entries |
| `FEEDER_TEST_COUNTS` | +720 counts |
| `SORTER_TEST_COUNTS` | Magnitude 300 counts |
| `FLAP_TEST_COUNTS` | Opening +200, reset -200 counts |
| `GOOD_DIRECTION` | +1: GOOD +300, BAD -300; change to -1 to reverse |
| Funnel reset | Opposite selected sorter target: GOOD -300, BAD +300 |
| `POSITION_TOLERANCE` | Strict absolute error <10 counts |
| `DEBOUNCE_MS` | 20 ms for press and release |
| `FLAP_DWELL_MS` | 500 ms after opening count is accepted |
| `MOVE_TIMEOUT_MS` | Original 3000 ms, reference only; disabled unconditionally |
| `INSPECT_TIMEOUT_MS` | 10000 ms, used only if inspection timeout enabled |
| `SWITCH_TIMEOUT_MS` | 10000 ms, used only if switch timeout enabled |
| `ENABLE_INSPECTION_TIMEOUT` | false by default |
| `ENABLE_SWITCH_TIMEOUT` | false by default; applies to every switch-wait phase |
| `CONTROL_PERIOD_US` | Original 10000 us, reference only; no motor control tick |
| `SPEED_TOLERANCE` | Original 2 counts/tick, reference only; no velocity check |
| `SERIAL_LINE_CAPACITY` | 48 bytes including NUL; oversized lines discarded |
| Serial baud | 115200 |

Positive is labeled forward and negative reverse; these are logical test
directions, not finalized wiring directions. There is no encoder-polarity setting
or motor GPIO assignment because this sketch has no motor hardware. Counts retain
the original test placeholders. `RESET_FUNNEL` is retained specifically for this
test, even though production now leaves the funnel at its selected endpoint.

## Example: complete GOOD cycle

This is an illustrative, condensed Serial Monitor transcript, not a recorded
hardware run. `>` marks a typed line, and `[physical ...]` describes switch use.
Release the switch after every press. The sketch additionally prints targets,
accepted counts, action prompts, and every state entry.

```text
[STATE] WAIT_FOR_BOLT
[physical press/release]
[EVENT] Bolt detected.
[TRANSITION] WAIT_FOR_BOLT -> ROTATE_FEEDER_WHEEL
[MOTOR SIM] Target: +720 counts; direction=forward (+); relative position reset to 0
> 100
[ENCODER SIM] Target not reached; remaining signed error=620 counts
> 720
[ENCODER SIM] Target reached: 720 / 720; simulated motor stopped
[TRANSITION] ROTATE_FEEDER_WHEEL -> BOLT_CHAMBER
[physical press/release]
[EVENT] Bolt seated.
[TRANSITION] BOLT_CHAMBER -> ROTATE_BOLT
[ACTION] Bolt rotation step 1 of 6; enter relative encoder position.
> 240
[ENCODER SIM] Target reached: 240 / 240; simulated motor stopped
[TRANSITION] ROTATE_BOLT -> ROTATE_BOLT
[ACTION] Bolt rotation step 2 of 6; enter relative encoder position.
> 240
[ENCODER SIM] Target reached: 240 / 240; simulated motor stopped
[TRANSITION] ROTATE_BOLT -> ROTATE_BOLT
[ACTION] Bolt rotation step 3 of 6; enter relative encoder position.
> 240
[ENCODER SIM] Target reached: 240 / 240; simulated motor stopped
[TRANSITION] ROTATE_BOLT -> ROTATE_BOLT
[ACTION] Bolt rotation step 4 of 6; enter relative encoder position.
> 240
[ENCODER SIM] Target reached: 240 / 240; simulated motor stopped
[TRANSITION] ROTATE_BOLT -> ROTATE_BOLT
[ACTION] Bolt rotation step 5 of 6; enter relative encoder position.
> 240
[ENCODER SIM] Target reached: 240 / 240; simulated motor stopped
[TRANSITION] ROTATE_BOLT -> ROTATE_BOLT
[ACTION] Bolt rotation step 6 of 6; enter relative encoder position.
> 240
[ENCODER SIM] Target reached: 240 / 240; simulated motor stopped
[TRANSITION] ROTATE_BOLT -> BUTTON_PRESSED
> g
[INSPECT] GOOD selected.
[TRANSITION] BUTTON_PRESSED -> ROTATE_SORTER
[MOTOR SIM] Target: +300 counts; direction=forward (+); relative position reset to 0
> 300
[ENCODER SIM] Target reached: 300 / 300; simulated motor stopped
[ACTION] Sorter stopped. Release switch, then press to confirm selected endpoint.
[physical press/release]
[EVENT] Sorter endpoint confirmed.
[TRANSITION] ROTATE_SORTER -> WAIT_FLAP_BUTTON
[physical press/release]
[EVENT] Flap release requested.
[TRANSITION] WAIT_FLAP_BUTTON -> RELEASE_FLAP
[MOTOR SIM] Target: +200 counts; direction=forward (+); relative position reset to 0
> 200
[ENCODER SIM] Target reached: 200 / 200; simulated motor stopped
[500 ms non-blocking dwell]
[TRANSITION] RELEASE_FLAP -> RESET_FLAP
[MOTOR SIM] Target: -200 counts; direction=reverse (-); relative position reset to 0
> -200
[ENCODER SIM] Target reached: -200 / -200; simulated motor stopped
[TRANSITION] RESET_FLAP -> RESET_FUNNEL
[MOTOR SIM] Target: -300 counts; direction=reverse (-); relative position reset to 0
> -300
[ENCODER SIM] Target reached: -300 / -300; simulated motor stopped
[CYCLE] Full sorting cycle complete.
[TRANSITION] RESET_FUNNEL -> WAIT_FOR_BOLT
[STATE] WAIT_FOR_BOLT
```

For a BAD cycle, enter `b`, use `-300` for sorter movement, and `300` for funnel
reset. Other movement targets stay the same. Test `x`, `d`, and `r` separately,
and verify that wrong counts and held switches do not skip steps.

## Verification status

Source checks verify all eleven states, expected defaults, absence of hardware
motor/library calls and blocking delays, and unchanged hashes of both existing
sketches. Arduino compilation and physical switch testing remain unverified.
