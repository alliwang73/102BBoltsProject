# 102BBoltsProject: 1/4-20 bolt thread inspector and sorter

A bolt slides down a ramp. The **feeder servo** flips it 180° so it drops head-first
into the **rotary tube**. A Pololu 2215 gearmotor turns the tube slowly while a
camera watches the threads. When the inspection is done, the **hatch** motor
swings to the GOOD or BAD bin. Then the **flap** motor opens and the bolt rolls
out of the angled tube into that bin.

## The three pieces

| File | What it is | Runs on |
| --- | --- | --- |
| [`BoltSorter/BoltSorter.ino`](BoltSorter/BoltSorter.ino) | All four actuators, the sorting state machine, Wi-Fi + Bluetooth control | Adafruit Feather ESP32 V2 |
| [`bolt-control/index.html`](bolt-control/index.html) | Tuning and manual-control page: jog, step-response tests, PID gains, angles, save to flash | Any phone or laptop |
| [`bolt-inspector/index.html`](bolt-inspector/index.html) | Live camera inspection. Analyses every frame while the tube turns, sends GOOD/BAD/EMPTY to the machine, keeps a tally and history | The phone or laptop pointed at the tube |
| [`WIRING.md`](WIRING.md) · [`docs/wiring.html`](docs/wiring.html) | Pinout, power, parts list, first power-up checklist | n/a |

All three use one text protocol over two transports, **Wi-Fi (HTTP)** and
**Bluetooth LE**. So one device can tune while another runs the camera, both
connected to the machine at the same time.

## 1. Flash the ESP32

1. Arduino IDE → Boards Manager → **esp32 by Espressif**, version 3.x (tested with 3.3.12).
2. Tools → Board → **Adafruit Feather ESP32 V2**.
3. Tools → Partition Scheme → **Default (3MB APP/1.5MB SPIFFS)**.
   This is required. The sketch is about 1.8 MB with Wi-Fi and Bluetooth, and the default 1.2 MB partition can't hold it.
4. Open `BoltSorter/BoltSorter.ino` and upload. No extra libraries are needed.
5. Serial Monitor at 115200 should print `BoltSorter ready`.

To also join an existing network (a phone hotspot, say) so every device on it can reach
`http://boltsorter.local`, fill in `STA_SSID` / `STA_PASS` at the top of the
sketch. The machine's own network stays up either way.

## 2. Connect your devices

The ESP32 always runs its own Wi-Fi network, **`BoltSorter`** (password **`bolts1420`**),
and always advertises over Bluetooth as **BoltSorter**.

**Tuning device (any phone or laptop):** join the `BoltSorter` Wi-Fi and open
**http://192.168.4.1**. The control page is built into the firmware, so it works
offline and in every browser, iPhone Safari included.

**Camera device:** browsers only allow a live camera on **https** or **localhost**
pages, and an https page isn't allowed to call the ESP32's plain-http address.
So pick one of these:

| Camera device | Open the inspector from | Connect with |
| --- | --- | --- |
| Android phone (Chrome) | The https copy on GitHub Pages (below) | **Connect Bluetooth** |
| Laptop + webcam / USB camera (Chrome or Edge) | `bolt-inspector/index.html` opened as a file, or `python3 -m http.server` then `http://localhost:8000/bolt-inspector/` | **Connect Wi-Fi** (join `BoltSorter` first) or **Connect Bluetooth** |
| iPhone | GitHub Pages copy, inside the **Bluefy** app (Safari has no Web Bluetooth) | **Connect Bluetooth** |

Recent Chrome versions may also allow **Connect Wi-Fi** from the https page after
a "local network access" prompt. Bluetooth is the dependable route.

**Hosting on GitHub Pages:** repo Settings → Pages → deploy from `main` / root.
The inspector will then be at
`https://<user>.github.io/102BBoltsProject/bolt-inspector/`, and the control page at
`.../bolt-control/` (the Bluetooth version of the tuning page).

## 3. Bring the machine up

Follow the checklist at the end of [WIRING.md](WIRING.md). In short:

1. **Encoders**: turn each shaft by hand, and the Counts readout on the control page should move.
2. **Directions**: on each axis tab, open Wiring check and tap Run +PWM. Counts
   must rise. If not, tick *Reverse motor* or *Reverse encoder*.
3. **Counts per rev**: use the Calibrate panel, or keep about 910 (75:1 × 12 CPR).
4. **Step tests**: tap *Run step test* and read the chart. Overshoot means lower Kp or
   raise Kd. Stopping short or slow means raise Ki or the friction PWM. The defaults are
   your `gearmotor_position_control.ino` tuning.
5. **Positions**: set the flap's open/closed angles, the hatch's GOOD/BAD angles, the servo's load/drop
   angles, and the inspection speed (default 40°/s) and turn (default 360°).
6. **Save to flash.** Positions are zeroed at boot, so power up with the flap closed
   and the hatch centered.

## 4. Run it

1. On the camera device, open the inspector, connect, tap **Live camera**, then
   **Set inspection area** and drag a box around the bolt's spot in the tube.
2. Feed a known-good bolt (**One bolt**) and check it reads PASS. Then
   *Save as good reference*.
3. Press **Run** on either page, or the board's **SW38** button.

Each cycle goes `WAIT_BOLT → FEED → SETTLE → INSPECT → AWAIT_VERDICT → SORT → RELEASE → CLOSE`.
During INSPECT, the inspector analyses a frame every ~0.2 s and marks it on the
0–360° sweep. When the tube stops, it decides:

- **BAD**: two or more *consecutive* failing frames (a real defect stays visible
  across neighbouring angles), or ≥ 30% of frames failing, or a bolt is visible but the threads can't be read.
- **EMPTY**: no bolt in most frames. The machine still opens the flap over the BAD bin, to be safe.
- **GOOD**: otherwise.

If no verdict arrives within the timeout (5 s by default), the firmware falls back to
the BAD bin. You can change that under Cycle settings.

## Serial / text commands

The same commands work in Serial Monitor (type `help`), over Wi-Fi
(`http://192.168.4.1/api/cmd?c=...`) and over Bluetooth:

```
run | finish | stop | estop | reset | single | trigger(k) | v g|b|e [reason]   (or g / b / e)
jog <ax> <deg> | goto <ax> <deg> | step <ax> <deg> | spin <ax> <deg/s> | pwm <ax> <±255> [ms]
halt <ax> | zero <ax> | cal <ax> start|done        ax = t (tube), f (flap), h (hatch)
servo <deg> | feed | flap open|close | hatch good|bad | inspect
set key=val ... | get | save | load | defaults | tally reset | plot t|f|h|off
```

`plot t` streams target/setpoint/angle/PWM for the Arduino Serial Plotter.

## Editing the control page

The firmware serves a copy of `bolt-control/index.html`. After editing the HTML, run:

```
python3 tools/embed_control_page.py
```

then re-upload the sketch.

## Older test sketches

`bolttransitioncode.ino` (original state machine), `gearmotor_position_control.ino`
(single-motor tuning), `funnelmotortesting.ino`, `boltfeedservo.ino` and the
`BoltSorter_StateMachine_Test*` folders are kept for reference. Everything they do
is now in `BoltSorter/BoltSorter.ino`.
