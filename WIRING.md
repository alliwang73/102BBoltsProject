# BoltSorter wiring: Adafruit Feather ESP32 V2

These pin numbers match the constants at the top of `BoltSorter/BoltSorter.ino`.
If you move a wire, change the constant there too.

Labels in `[brackets]` are what's printed next to the pin on the Feather.

## Parts

| Qty | Part | Job |
| --- | --- | --- |
| 1 | Adafruit Feather ESP32 V2 | Controller, Wi-Fi + Bluetooth |
| 3 | Pololu #2215 75:1 HP 6 V micro metal gearmotor + magnetic encoder | Tube, flap, hatch |
| 2 | DRV8833 dual motor driver (Adafruit #3297 or Pololu #2130) | Driver #1 = tube + flap, driver #2 = hatch |
| 1 | Hobby servo (e.g. SG90 / MG90S) | Feeder flip (180°) |
| 1 | 6 V supply (bench supply, or 5 × AA NiMH), ≥ 2 A | Motor power (VM) |
| 1 | 5 V supply / BEC, ≥ 1 A | Servo power |
| 1 | 100 µF electrolytic capacitor (≥ 16 V) | Across VM/GND, next to the drivers |
| 0–2 | Limit switches (optional) | Hatch GOOD / BAD end stops |
| 0–1 | Microswitch or IR break-beam (optional) | Bolt-present sensor at the slide |

The DRV8833 handles these motors well (about 1.5 A per channel). Avoid the L298N:
it drops about 2 V, which leaves the 6 V motors weak. If you must use an L298N,
leave its ENA/ENB jumpers on and wire IN1–IN4 exactly as the AIN/BIN pins below.

## Pin map

### Motors → DRV8833 inputs

| Signal | GPIO | Feather pin | Connects to |
| --- | --- | --- | --- |
| Tube motor IN1 | 26 | `[A0]` | DRV8833 **#1 AIN1** |
| Tube motor IN2 | 25 | `[A1]` | DRV8833 **#1 AIN2** |
| Flap motor IN1 | 4 | `[A5]` | DRV8833 **#1 BIN1** |
| Flap motor IN2 | 13 | `[13]` | DRV8833 **#1 BIN2** (also the red LED, which flickers when the flap drives) |
| Hatch motor IN1 | 19 | `[MO]` | DRV8833 **#2 AIN1** |
| Hatch motor IN2 | 21 | `[MI]` | DRV8833 **#2 AIN2** |

Driver outputs → motors: **#1 AOUT1/AOUT2 → tube motor M1/M2**,
**#1 BOUT1/BOUT2 → flap motor**, **#2 AOUT1/AOUT2 → hatch motor**.
DRV8833 #2's B channel is unused.

### Encoders → Feather inputs

| Signal | GPIO | Feather pin |
| --- | --- | --- |
| Tube encoder A | 27 | `[27]` |
| Tube encoder B | 33 | `[33]` |
| Flap encoder A | 15 | `[15]` |
| Flap encoder B | 20 | `[SCL]` |
| Hatch encoder A | 7 | `[RX]` |
| Hatch encoder B | 8 | `[TX]` |

The tube motor, its driver channel and its encoder use the same pins as
`gearmotor_position_control.ino` (26/25 and 27/33). If that test rig is
already wired, the tube needs no changes.

### Servo, switches, sensor

| Signal | GPIO | Feather pin | Wiring |
| --- | --- | --- | --- |
| Feeder servo signal | 32 | `[32]` | Servo signal wire (orange/yellow/white) |
| Hatch GOOD limit (optional) | 14 | `[14]` | Switch COM → GND, NO → pin |
| Hatch BAD limit (optional) | 22 | `[SDA]` | Switch COM → GND, NO → pin |
| Bolt sensor (optional) | 5 | `[SCK]` | Switch COM → GND, NO → pin; or IR receiver signal → pin |
| Start / Stop | 38 | on-board **SW38** button | Already wired on the board |

All inputs use the ESP32's internal pull-ups and read *active LOW*, so no
resistors are needed. Turn the optional inputs on from the control page:
"Use limit switches" (Hatch tab) and "Wait for bolt sensor" (Cycle settings).

Spare pins if you need more later: 12, 34, 36, 37, 39. 34–39 are input-only
and have no pull-ups. 12 must stay LOW at boot.

## Power

| From | To |
| --- | --- |
| 6 V supply **+** | DRV8833 #1 **VM**, DRV8833 #2 **VM** |
| 5 V supply **+** | Servo **red** wire |
| Feather **3V** | Encoder **VCC** ×3, DRV8833 **SLP/nSLEEP** ×2, IR break-beam **+** (if used) |
| **GND** | 6 V −, 5 V −, Feather GND, both DRV8833 GND, encoder GND ×3, servo brown/black, switch COMs |
| 100 µF cap | + leg to VM, − leg to GND, right at the drivers |
| USB-C | Feather (programming + logic power) |

Two rules matter most:

1. **Power the encoders from 3V, not 6 V.** Their A/B outputs swing to VCC,
   and the ESP32 pins are 3.3 V only. A 5 V or 6 V signal can damage the Feather.
2. **Tie every ground together.** Without a common ground, encoder counts and
   PWM go haywire.

Don't power the servo from the Feather's 3V pin. Its stall current can brown
out the ESP32 and cause resets. A small SG90 can run from the Feather's **USB**
pin (5 V from the USB cable) for bench testing only.

`SLP`/`nSLEEP` enables each DRV8833. Some breakouts already pull it high, and
tying it to 3V is harmless either way. Leave `FLT`/`nFAULT` unconnected.

## Encoder cable (Pololu micro metal gearmotor encoder)

The encoder board's six pads are labelled **M1, M2, GND, VCC, OUT A, OUT B**.
Match by the silkscreen, not by wire colour, because cable colours vary.

```
M1, M2  -> driver AOUT1/AOUT2 (or BOUT1/BOUT2)
GND     -> GND rail
VCC     -> Feather 3V          (3.3 V!)
OUT A   -> encoder A pin above
OUT B   -> encoder B pin above
```

If an axis counts backwards, or faults with "motor and encoder directions
disagree", don't rewire it. Tick **Reverse encoder** or **Reverse motor** for
that axis on the control page, then save.

## Block diagram

```
                         ┌────────────────────────────┐
     USB-C ─────────────►│   Adafruit Feather ESP32 V2│
                         │                            │
   ┌─ 3V ────────────────┤3V                       GND├───────────────┐ common
   │                     │                            │               │ ground
   │   ┌─────────────────┤A0 (26)   [tube IN1]        │               │ rail
   │   │ ┌───────────────┤A1 (25)   [tube IN2]        │               │
   │   │ │ ┌─────────────┤A5 (4)    [flap IN1]        │               │
   │   │ │ │ ┌───────────┤13        [flap IN2]        │               │
   │   │ │ │ │           │                            │               │
   │   │ │ │ │ ┌─────────┤MO (19)   [hatch IN1]       │               │
   │   │ │ │ │ │ ┌───────┤MI (21)   [hatch IN2]       │               │
   │   │ │ │ │ │ │       │                            │               │
   │   │ │ │ │ │ │       │27, 33    ◄── tube enc A,B  │               │
   │   │ │ │ │ │ │       │15, SCL   ◄── flap enc A,B  │               │
   │   │ │ │ │ │ │       │RX, TX    ◄── hatch enc A,B │               │
   │   │ │ │ │ │ │       │32        ──► servo signal  │               │
   │   │ │ │ │ │ │       │14, SDA   ◄── GOOD/BAD limit│               │
   │   │ │ │ │ │ │       │SCK       ◄── bolt sensor   │               │
   │   │ │ │ │ │ │       └────────────────────────────┘               │
   │   ▼ ▼ ▼ ▼ │ │                                                    │
   │ ┌──────────────┐ AOUT1/2 ──► TUBE motor  (enc VCC=3V)            │
   ├►│ DRV8833 #1   │ BOUT1/2 ──► FLAP motor  (enc VCC=3V)            │
   │ │ SLP  VM  GND │                                                 │
   │ └───────▲───┬──┘                                                 │
   │         │   └────────────────────────────────────────────────────┤
   │         │ ▼ ▼                                                    │
   │ ┌──────────────┐ AOUT1/2 ──► HATCH motor (enc VCC=3V)            │
   └►│ DRV8833 #2   │                                                 │
     │ SLP  VM  GND │                                                 │
     └───────▲───┬──┘                                                 │
             │   └────────────────────────────────────────────────────┤
   6 V + ────┴── (both VM, + 100 µF cap to GND)     6 V − ────────────┤
   5 V + ─────────────────────► servo red           5 V − ────────────┘
```

A visual version of this diagram is in [docs/wiring.html](docs/wiring.html).

## First power-up checklist

1. Wire the logic side only: no 6 V and no servo power yet. Upload the sketch and open
   Serial Monitor at 115200. You should see `BoltSorter ready`.
2. Turn each motor shaft by hand and watch **Counts** change on the control page.
   This checks the encoder wiring.
3. Connect 6 V. On each axis tab, open **Wiring check** and tap **Run +PWM 1 s**.
   Counts must go **up**. Fix the direction with the two Reverse checkboxes.
4. Use **Calibrate counts per revolution** on each axis, or leave the default
   (~910 counts per output rev for a 75:1 gearbox with a 12 CPR encoder).
5. Run a **step test** on each axis and tune it. The defaults come from your
   gearmotor test sketch (Kp 8, Ki 25, Kd 0.15, friction 45).
6. Set the flap open/closed angles, the hatch GOOD/BAD angles and the servo's load/drop
   angles. Then **Save to flash**.
7. Power down with the flap closed and the hatch centered. Positions are
   zeroed at boot, so that's where the machine will think 0° is.
