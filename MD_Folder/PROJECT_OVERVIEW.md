# GooseV3-ESP — Project Overview for Agents

This document is a complete reference of the GooseV3-ESP firmware. It is written for an AI agent that will edit, extend, or debug the project. Every file, every type, every state, and every cross-file interaction is described here.

---

## 1. What this project is

**GooseV3-ESP** is an ESP32 firmware that drives **three stepper motors** (X, Y, Z) plus a Z-axis **endstop switch** over the Arduino framework. It is built with **PlatformIO** targeting the `esp32doit-devkit-v1` board.

Two high-level behaviors are implemented:

1. **Homing** the Z axis against a mechanical endstop (a 4-phase finite state machine).
2. After Z is homed, the firmware enters **one of two modes** depending on a compile-time flag:
   - **Tracking mode** (`Config::afterHomingManualTrackingTest == true`, currently active): a manual-jog test that lets the user drive Z, X, or Y over Serial and keeps a running "total movement" counter for Z.
   - **Searching mode** (when the flag is false): an autonomous oscillating sweep of Z between `searchMinSteps` and `searchMaxSteps`, with Serial pause/resume.

The firmware is **cooperatively scheduled in `loop()`** — there are no FreeRTOS tasks, no interrupts beyond Arduino defaults, and no `delay()` calls in the main loop. Stepper pulses are emitted in software by polling `micros()`.

---

## 2. Top-level directory layout

```
GooseV3-ESP/
├── platformio.ini              ; PlatformIO env: ESP32 DevKit V1, 115200 baud
├── .clangd                     ; clangd config
├── .vscode/                    ; editor config (gitignored partially)
├── .pio/                       ; PlatformIO build cache (gitignored)
├── .cache/                     ; clangd compilation DB cache
├── clangd/                     ; clangd state
├── compile_commands.json       ; clangd index for IntelliSense
├── include/                    ; public headers (#include <Name.h>)
│   ├── AppState.h
│   ├── Config.h
│   ├── Endstop.h
│   ├── Homing.h
│   ├── Motor.h
│   ├── Search.h
│   ├── Tracking.h
│   ├── UART.h                  ; empty placeholder file
│   ├── README                  ; PlatformIO boilerplate
│   └── GooseV3-ESP.code-workspace
├── src/                        ; main firmware sources
│   ├── main.cpp                ; setup() / loop() and Serial UI
│   ├── Motor.cpp
│   ├── homing.cpp
│   ├── Search.cpp
│   └── Tracking.cpp            ; stub for future closed-loop tracking
├── lib/
│   ├── Motor/                  ; EMPTY placeholder dir (Motor.cpp/.h are 0 bytes)
│   └── README                  ; PlatformIO boilerplate
├── test/
│   ├── test_xy_motors/         ; standalone test firmware: Serial jog for X/Y
│   │   ├── main.cpp
│   │   ├── Motor.cpp           ; thin shim that #includes ../../src/Motor.cpp
│   │   └── README
│   └── README
└── MD_Folder/                  ; This documentation lives here
```

Notes for editors:

- `lib/Motor/` contains **zero-byte** `Motor.cpp` and `Motor.h`. Do **not** assume any code lives there; the real `Motor` lives in `include/Motor.h` + `src/Motor.cpp`.
- `include/UART.h` is **empty** (0 bytes) — likely a planned module that was never written. Adding UART abstraction here is appropriate if needed.
- `compile_commands.json` is large (≈1.3 MB) and auto-generated; don't hand-edit it.

---

## 3. Build / target environment

**`platformio.ini`** — the only environment is `esp32doit-devkit-v1`:

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
monitor_port = COM3
upload_port = COM3
```

- **Board**: ESP32 DevKit V1.
- **Framework**: Arduino (uses `Serial`, `pinMode`, `digitalWrite`, `micros`, `millis`).
- **Baud**: 115200.
- **Ports**: hard-coded to Windows `COM3` (`monitor_port`/`upload_port`). On macOS/Linux you must override on the command line, e.g. `pio run -t upload --upload-port /dev/cu.usbserial-...`. The current developer is on macOS (`darwin`), so the ini is a leftover from a Windows host.

To build / upload / monitor:

```bash
pio run                            # build
pio run -t upload                  # upload main firmware
pio device monitor -b 115200       # open Serial monitor
pio test -e esp32doit-devkit-v1 -f test_xy_motors --upload   # X/Y jog test
```

---

## 4. Hardware model and pin map

All pin assignments and mechanical constants are centralized in [`include/Config.h`](../include/Config.h) under the `Config` namespace.

### 4.1 Pin map

| Axis | STEP | DIR | EN  | ENDSTOP |
|------|------|-----|-----|---------|
| X    | 18   | 19  | 27  | 34      |
| Y    | 25   | 26  | 27  | 35      |
| Z    | 32   | 33  | 27  | 13      |

**Important details:**

- `Config::X::enablePin`, `Config::Y::enablePin`, and `Config::Z::enablePin` are **all GPIO 27**. The drivers share a single active-low enable. Only Z's enable is driven explicitly (in `setup()` → `digitalWrite(Config::Z::enablePin, LOW)`); since all three constants resolve to the same pin, that one write enables every driver.
- Endstop pins for X (GPIO 34) and Y (GPIO 35) are declared in `Config.h` but **never read** by the main firmware — `main.cpp` only constructs an `Endstop` for Z. They are reserved for future use.
- GPIO 34 and 35 are **input-only** ESP32 pins; that matches their endstop role.
- Z's endstop is on GPIO 13. The `Endstop` class always uses `INPUT_PULLUP`.

### 4.2 Endstop wiring assumptions

- `Config::endstopDebounceMs = 10` — 10 ms software debounce.
- `Config::endstopPressedIsPinLow = true` — wiring is **normally-open switch to GND + INPUT_PULLUP**, so idle reads HIGH and pressing pulls the pin LOW. If you switch to NC wiring or active-high, flip this constant.

### 4.3 Mechanics / unit conversion

```cpp
fullStepsPerRev = 720
microstepping   = 1
maxSteps        = 720
maxAngleDeg     = 1296.0
degreesPerStep  = 1296.0 / 720 = 1.8°/step
```

The `maxAngleDeg = 1296` value (> 360°) implies a gearbox or that 720 "steps" refer to motor-shaft steps while the application angle is on the gearbox output. The firmware only uses `degreesPerStep` as an informational print — no kinematics depend on it.

### 4.4 Motion constants

| Constant | Value | Used by |
|---|---|---|
| `Z::homeDir` | `Motor::Direction::Reverse` | Homing seek/reapproach direction |
| `Z::seekSpeedStepsPerSec` | `70.0` | Homing seek phase |
| `Z::backoffSpeedStepsPerSec` | `16.0` | Homing backoff phase |
| `Z::reapproachSpeedStepsPerSec` | `12.0` | Homing slow re-approach |
| `Z::travelSpeedStepsPerSec` | `70.0` | Homing final-move-to-position |
| `Z::finalPositionSteps` | `-350` | Where Z parks after homing (in step units, signed) |
| `Z::finalMoveTimeoutMs` | `60000` | Timeout for final-move phase |
| `manualSpeedStepsPerSec` | `70.0` | All manual jog moves (X/Y/Z) |
| `searchMinSteps` | `210` | Lower bound of sweep |
| `searchMaxSteps` | `500` | Upper bound of sweep |
| `searchSpeedStepsPerSec` | `70.0` | Sweep speed |
| `afterHomingManualTrackingTest` | `true` | After homing: `true` → Tracking, `false` → Searching |

### 4.5 The direction sign convention (critical!)

This convention appears in **every** module and is the easiest place to introduce a sign-flip bug:

> **`Motor::Direction::Reverse` causes `positionSteps_` to INCREMENT.**
> **`Motor::Direction::Forward` causes `positionSteps_` to DECREMENT.**

That is, "Reverse" maps to "positive direction" in the firmware's coordinate frame. This is set in [`src/Motor.cpp:69`](../src/Motor.cpp). All other modules (`Homing`, `Search`, `main.cpp`'s jog logic) reason about position increments using this rule. When a user types `200` on the Serial console, the firmware converts that to `Direction::Reverse` because `deltaSteps > 0`.

The DIR pin level is `HIGH` for `Forward`, `LOW` for `Reverse` ([`src/Motor.cpp:10`](../src/Motor.cpp) and [`src/Motor.cpp:26`](../src/Motor.cpp)).

---

## 5. Module-by-module reference

### 5.1 `Motor` ([`include/Motor.h`](../include/Motor.h), [`src/Motor.cpp`](../src/Motor.cpp))

A software-timed step-pulse generator for a single STEP/DIR stepper driver (e.g., A4988, DRV8825, TMC2208).

**Public API:**

```cpp
Motor(uint8_t stepPin, uint8_t dirPin);
void begin();
void enable(bool enabled);
bool isEnabled() const;
void setDirection(Direction dir);              // Direction::Forward | ::Reverse
Direction direction() const;
void setSpeedStepsPerSec(float stepsPerSec);   // 0 = stop
float speedStepsPerSec() const;
void update();                                 // call every loop()
long positionSteps() const;
void setPositionSteps(long pos);               // used by Homing to set zero
```

**How it works:**

- `begin()` configures STEP and DIR as `OUTPUT`, drives STEP LOW, sets DIR according to the current logical direction, sets `enabled_ = true`, and seeds the timing baseline `nextEdgeDueUs_ = micros()`.
- `enable(false)` clears `stepHigh_` and writes STEP LOW so the driver sees a safe idle. There is **no actual EN pin write** here — the EN pin is driven once globally in `setup()` ([`src/main.cpp:304-305`](../src/main.cpp)). `enable(true)` simply re-enables stepping in software.
- `setSpeedStepsPerSec(s)`: stores |s|, computes `stepIntervalUs_ = 1e6 / s` (clamped to a minimum of 1 µs). Speed `0` stops stepping by setting `stepIntervalUs_ = 0`, after which `update()` becomes a no-op.
- `setDirection(dir)`: writes the DIR pin and applies a `dirSetupUs_ = 5 µs` guard time before the next step edge — protects driver setup time after a DIR change.
- `update()` is the heart of pulse generation. It uses a two-phase state machine on the STEP line:
  1. If `now >= nextEdgeDueUs_` and `stepHigh_` is false, write STEP HIGH, set `stepHigh_=true`, schedule the falling edge `pulseHighUs_ = 4 µs` later.
  2. Next time the threshold passes, write STEP LOW, decrement/increment `positionSteps_` based on direction, schedule the next rising edge `stepIntervalUs_ - pulseHighUs_` later.
- Wraparound-safe time comparisons are done via `(int32_t)(now - target) < 0` (the standard Arduino pattern).

**Performance ceiling:** because pulses are emitted from `loop()`, the max safe step rate is bounded by how often `loop()` runs. With Serial activity and a few module updates, expect a comfortable ceiling around a few kHz — well above the 70 steps/sec speeds used in this firmware.

### 5.2 `Endstop` ([`include/Endstop.h`](../include/Endstop.h)) — header-only

A debounced digital input wrapper.

**Constructor:** `Endstop(pin, debounceMs = 0, pressedMeansPinLow = true)`.

**API:** `begin()`, `update()`, `isPressed()`.

**Behavior:**

- `begin()` configures `INPUT_PULLUP`, samples the raw state once, and treats that as both raw and stable initial state.
- `update()`:
  - If `debounceMs_ == 0`, samples the raw state every call and reports immediately.
  - Otherwise, when the raw level differs from the previously-seen raw level it timestamps the change; once the new raw level has held longer than `debounceMs_`, the stable state is updated. This is a classic Horowitz-style debounce.
- `isPressed()` returns the *stable* (debounced) state.
- The polarity is mapped via `pressedMeansPinLow_` — if `true`, "pin LOW" means "pressed".

### 5.3 `Homing` ([`include/Homing.h`](../include/Homing.h), [`src/homing.cpp`](../src/homing.cpp))

A finite-state machine that homes a single axis against an endstop, then parks it at a configured position.

**States** (`enum class State`):

```
Idle → SeekToEndstop → Backoff → Reapproach → SetZero → MoveToFinal → Done
                                                                      ↘ Error (from any active state)
```

**Configurable parameters** (`Homing::Config`):

| Field | Default | What it does |
|---|---|---|
| `homeDir` | `Reverse` | Direction toward the switch |
| `seekSpeedStepsPerSec` | 800 | Fast approach speed |
| `backoffSpeedStepsPerSec` | 300 | Move away from switch |
| `reapproachSpeedStepsPerSec` | 200 | Slow re-engage |
| `travelSpeedStepsPerSec` | 1000 | Speed for the final park move |
| `seekStepsMax` | 40000 | Travel safety bound for seek |
| `seekTimeoutMs` | 30000 | Time safety bound for seek |
| `backoffStepsMax` | 400 | Travel safety bound for backoff |
| `backoffTimeoutMs` | 15000 | Time safety bound for backoff |
| `reapproachStepsMax` | 6000 | Travel safety bound for reapproach |
| `reapproachTimeoutMs` | 15000 | Time safety bound for reapproach |
| `finalPositionSteps` | 500 | Park position after zeroing |
| `finalMoveTimeoutMs` | 60000 | Time safety bound for park move |

The Z-axis defaults in `main.cpp` come from `Config::Z::*` and **override several of these** — notably the speeds (70/16/12/70) and `finalPositionSteps = -350`. The travel/seek **step limits and timeouts are NOT overridden** in `main.cpp`, so the defaults above are what's active.

**FSM behavior (in [`src/homing.cpp:31`](../src/homing.cpp)):**

- Every `update()` call always pumps `motor_.update()` first so step pulses keep flowing regardless of any state checks.
- `Idle`, `Done`, `Error` are terminal — `update()` early-returns.
- In every active state the code computes `travel = |positionSteps_ - phaseStartPos_|` and `nowMs - phaseStartMs_`, then checks the timeout and the travel max. Exceeding either calls `fail_(reason)` which sets speed to 0 and goes to `Error`.
- `SeekToEndstop`: drive in `homeDir` at seek speed until `endstop.isPressed()` → transition to `Backoff`.
- `Backoff`: reverse direction (`awayDir_()`), slow speed, until endstop releases → transition to `Reapproach`.
- `Reapproach`: drive back at `reapproachSpeed`, finer detection, until endstop presses again → `SetZero`.
- `SetZero`: snapshot `homeZeroPos_ = positionSteps()` for diagnostics, then `motor_.setPositionSteps(0)` — the current physical location is now origin. Transition to `MoveToFinal`.
- `MoveToFinal`: pick direction based on whether `finalPositionSteps` is above or below current position (using the inverted sign rule: `pos < tgt` → `Reverse`, else `Forward`), run at `travelSpeed`. When position reaches the target side, stop and become `Done`.

**Helpers / observers:** `isHomed()`, `isDone()`, `hasError()`, `state()`, `stateName()`, `errorReason()`.

**Recovery:** `abort(reason)` forces `Error`. Calling `start()` again resets `errorReason_` to `nullptr` and re-enters `SeekToEndstop`.

### 5.4 `Search` ([`include/Search.h`](../include/Search.h), [`src/Search.cpp`](../src/Search.cpp))

Continuous oscillating sweep between two step positions. Used only when `Config::afterHomingManualTrackingTest == false`.

**Config:** `minSteps`, `maxSteps`, `speedStepsPerSec`.

**Lifecycle:**

- `start(motor)`: enables motor, sets sweep speed, chooses an initial direction based on where the motor currently is relative to `[minSteps, maxSteps]`. Inside the band the default is `Reverse` (i.e. increase position toward `maxSteps`).
- `update(motor)`: if not paused, check whether the motor has reached or passed an end of the band; if so, flip direction. No deceleration — direction reversal is instantaneous at the current speed.
- `pause(motor)`: sets `paused_ = true` and `setSpeedStepsPerSec(0)`.
- `resume(motor)`: just calls `start(motor)` to recompute direction from current position.
- `isPaused()` is consulted by `main.cpp` to decide whether to allow manual jogs.

### 5.5 `Tracking` ([`include/Tracking.h`](../include/Tracking.h), [`src/Tracking.cpp`](../src/Tracking.cpp))

A free-function namespace, **currently a stub**: `Tracking::update(Motor&)` does nothing. Reserved for future closed-loop tracking. The "manual tracking test" that runs when `Config::afterHomingManualTrackingTest == true` is entirely implemented in `main.cpp`, not in this module.

### 5.6 `AppState` ([`include/AppState.h`](../include/AppState.h))

```cpp
enum class AppState : uint8_t { Homing, Searching, Tracking };
```

A flat top-level mode indicator. Used by `main.cpp` to decide which subsystem's `update()` to call after `Homing` finishes.

### 5.7 `Config` ([`include/Config.h`](../include/Config.h))

Already covered in §4. All values are `static constexpr` inside nested namespaces, so they evaluate at compile time and incur no RAM cost.

---

## 6. The full Serial UI (in `main.cpp`)

The Serial protocol is line-buffered, except `d`/`D` and `r`/`R` which are recognized as single-character commands that can interrupt motion mid-buffer.

### 6.1 Boot sequence

In [`setup()`](../src/main.cpp) ([`src/main.cpp:300-315`](../src/main.cpp)):

1. `Serial.begin(115200)` and a 1 s delay so the host terminal can attach.
2. Set `Config::Z::enablePin` (= GPIO 27, shared with all axes) to `OUTPUT` and drive it `LOW` (active-low → drivers enabled).
3. `motorZ.begin(); motorX.begin(); motorY.begin(); endstop.begin();`
4. `Serial.println("Starting homing...")`.
5. `homing.start()` → enters `SeekToEndstop`.
6. `printStateIfChanged()` prints the initial state.

### 6.2 `loop()` cycle ([`src/main.cpp:317-355`](../src/main.cpp))

Every iteration, in order:

1. `handleSerial()` — read any pending Serial bytes (see §6.3).
2. `endstop.update()` — refresh debounced switch state.
3. `homing.update()` — advance the FSM (this also pumps `motorZ.update()` internally).
4. `printStateIfChanged()` — print to Serial when the homing state changes.
5. **One-shot transition into the post-homing mode:** if state is still `Homing` but `homing.isHomed()` is now true and we haven't transitioned yet, set `transitionedAfterHoming = true`, print position, and either:
   - if `Config::afterHomingManualTrackingTest` → `appState = Tracking`, reset `totalManualMovementSteps = 0`, print the tracking-test banner.
   - else → `appState = Searching`, `search.start(motorZ)`, print sweep banner.
6. If `appState == Searching` → `search.update(motorZ)`. If `Tracking` → `Tracking::update(motorZ)` (a no-op stub).
7. **First-time manual prompt:** when manual moves become allowed for the first time after homing, print the manual-mode prompt.
8. Pump every motor and the manual move tracker: `motorZ.update(); motorX.update(); motorY.update(); updateManualMove();`.

Note: `motorZ.update()` is called twice per loop in some states (once by `homing.update()`, once at the bottom). This is harmless — `update()` is idempotent until the next scheduled edge.

### 6.3 Command grammar (`handleSerial`, [`src/main.cpp:200-285`](../src/main.cpp))

Carriage returns (`\r`) are ignored. Other characters are buffered into `inputBuffer` until newline (`\n`), with two single-character shortcuts:

- **`d` / `D`** — STOP. Calls `stopMotion("STOP (d) received")`, which sets all motor speeds to 0 and resets `manualAxis = None`. If not yet homed, also aborts homing. If searching, calls `search.pause(motorZ)`. Clears the input buffer. Also: if a Z manual move was in progress in Tracking mode, the partial signed travel is added to `totalManualMovementSteps` before printing the running total.
- **`r` / `R`** — RESUME (only meaningful in `Searching` while `search.isPaused() && manualAxis == None`). Calls `search.resume(motorZ)` and prints `[Search] Resumed.`.

On newline, the buffered line is trimmed and dispatched:

- If `!homing.isHomed()` → print "Not homed yet" and ignore.
- If `appState == Searching && !search.isPaused()` → any input prints a hint to type `d` first.
- If `manualAxis != None` → print "Busy moving. Type 'd' to stop.".
- Empty line and manual moves are allowed → re-print the manual prompt.
- Otherwise try to parse as an axis move (X/Y), then as a signed long for Z:
  - `X<int>` or `x<int>` → `startManualMove(ManualAxis::X, steps)`.
  - `Y<int>` or `y<int>` → `startManualMove(ManualAxis::Y, steps)`.
  - Bare signed integer like `200` or `-350` → `startManualMove(ManualAxis::Z, steps)`.
  - Anything else → print usage.

### 6.4 `startManualMove` ([`src/main.cpp:135`](../src/main.cpp))

- Rejects `0` steps with a message.
- Picks the right `Motor&` via `motorForAxis(axis)`.
- Records `manualLegStartPos = startPos` and `manualTargetPos = startPos + deltaSteps`.
- Enables motor, sets direction based on sign of `deltaSteps` (positive → `Reverse` because Reverse increments position), sets speed to `Config::manualSpeedStepsPerSec`.
- Prints `[Move <axis>] From <start> to <target> (delta <delta>)`.

### 6.5 `updateManualMove` ([`src/main.cpp:162`](../src/main.cpp))

- Called every loop. If `manualAxis == None`, returns.
- Reads current pos and direction. The "done" predicate is:
  - Forward (DIR pin HIGH, position decreasing) → done when `pos <= manualTargetPos`.
  - Reverse (DIR pin LOW, position increasing) → done when `pos >= manualTargetPos`.
- On done: if Tracking + Z, accumulate the leg into `totalManualMovementSteps` (this matches the partial-move accumulation done in `stopMotion`). Stop the motor, print `[Stop <axis>] move complete`, print position, and if Tracking + Z print the running total. Finally re-print the manual prompt if allowed.

### 6.6 Total-movement counter (Tracking mode only)

`totalManualMovementSteps` is a signed long accumulator. It is updated **only for Z, only when `afterHomingManualTrackingTest && appState == Tracking`**. It tracks the *signed* sum of Z travel since homing — positive Z steps add, negative subtract. It is updated both on natural completion (`updateManualMove`) and on user-aborted partial moves (`stopMotion` via `d`).

---

## 7. State transitions at a glance

```
┌──────────────┐  homing.isHomed()    ┌────────────────────────────────┐
│ AppState =   │ ───────────────────► │ afterHomingManualTrackingTest? │
│ Homing       │                      └─────────────┬──────────────────┘
└──────┬───────┘                                    │
       │ Homing FSM:                       true ───► AppState = Tracking
       │  Idle→Seek→Backoff→Reapproach              (manual jog test;
       │  →SetZero→MoveToFinal→Done                  Tracking::update is no-op)
       │                                  false ──► AppState = Searching
       │                                            search.start(motorZ)
       ▼
   (Error state on any homing fault — final / no further transitions)
```

Within `Searching`, `d` pauses (`search.pause`) and `r` resumes (`search.resume`). While paused, manual jogs are allowed via the same X/Y/Z command grammar. There is currently no command to leave Tracking mode.

---

## 8. The standalone X/Y test firmware

`test/test_xy_motors/` builds as a PlatformIO unit-test environment and overrides `src/main.cpp` so the device boots into a minimal X/Y jog tool.

- `test/test_xy_motors/main.cpp` defines its own `setup()` / `loop()`.
- `test/test_xy_motors/Motor.cpp` is a 1-line shim: `#include "../../src/Motor.cpp"` so the test uses the same `Motor` implementation as the main firmware.
- Pins are still pulled from `Config::X::*` and `Config::Y::*`.
- The shared enable pin is driven LOW once (`setupSharedEnable`).
- A line-based command parser accepts `x12`, `x-12`, `y50`, `y-5`, etc.
- An `AxisJog` struct keeps per-axis target and active flag, exactly mirroring the manual-move logic in the main firmware.

Run with:

```bash
pio test -e esp32doit-devkit-v1 -f test_xy_motors --upload
```

---

## 9. Things that look like dead/placeholder code

When editing or extending, be aware that **these are not load-bearing**:

| Path | Status | Notes |
|---|---|---|
| `include/UART.h` | 0 bytes | Planned UART module, never written. Safe to flesh out. |
| `lib/Motor/Motor.cpp`, `lib/Motor/Motor.h` | 0 bytes | Misleading: real `Motor` lives in `include/Motor.h` + `src/Motor.cpp`. Don't put new code under `lib/Motor/` expecting to be picked up — confirm PlatformIO's LDF behavior first. |
| `src/Tracking.cpp` | stub | `Tracking::update(Motor&)` does nothing. The "tracking test" mode in `main.cpp` does not use it. |
| `Config::X::endstopPin = 34`, `Config::Y::endstopPin = 35` | declared, unused | No `Endstop` is constructed for X or Y. |

---

## 10. Cross-cutting conventions for editors

1. **Direction sign rule** (already in §4.5): positive Δsteps ⇒ `Direction::Reverse`. Re-derive every time you write motion logic — it's the #1 source of bugs.
2. **No `delay()` in the main loop.** All timing is `micros()`-based with wraparound-safe comparisons (`(int32_t)(now - then) < 0`). Preserve this when adding code.
3. **`Motor::update()` must be called frequently.** Each loop iteration. Long-running operations in `loop()` will distort step timing.
4. **Speeds are in steps/sec, always positive.** `setSpeedStepsPerSec` takes |x|. Direction is set separately via `setDirection`.
5. **`Endstop::update()` must be called every loop** for debouncing to work; `isPressed()` is a cheap accessor.
6. **`Homing::update()` already calls `motor_.update()` internally** — calling it again at the bottom of `loop()` is harmless but worth knowing when reasoning about timing.
7. **Compile-time mode switch:** to swap between Tracking and Searching, flip `Config::afterHomingManualTrackingTest`. Do not try to switch modes at runtime — the boot transition is one-shot (`transitionedAfterHoming`).
8. **All enable pins share GPIO 27.** Don't try to disable one axis without affecting the others unless you re-wire the hardware and update `Config::X/Y/Z::enablePin`.
9. **Position is in raw step edges, not gear-output degrees.** `degreesPerStep` is printed but not used in any control math.
10. **The Serial port number in `platformio.ini` is `COM3` (Windows).** On macOS/Linux you must override `--upload-port` / `--monitor-port` on the command line, or edit the ini.

---

## 11. Quick reference: where to look when…

| Task | File(s) |
|---|---|
| Change pins | [`include/Config.h`](../include/Config.h) |
| Tune homing speeds / final park position | [`include/Config.h`](../include/Config.h) `Config::Z::*` |
| Tune sweep range | [`include/Config.h`](../include/Config.h) `searchMinSteps`, `searchMaxSteps`, `searchSpeedStepsPerSec` |
| Change post-homing behavior | [`include/Config.h`](../include/Config.h) `afterHomingManualTrackingTest` |
| Modify Serial commands | [`src/main.cpp:200`](../src/main.cpp) `handleSerial()` |
| Adjust step pulse timing | [`include/Motor.h`](../include/Motor.h) `pulseHighUs_`, `dirSetupUs_` |
| Change homing FSM logic | [`src/homing.cpp`](../src/homing.cpp) and [`include/Homing.h`](../include/Homing.h) |
| Implement closed-loop tracking | [`src/Tracking.cpp`](../src/Tracking.cpp) |
| Add a new axis | `Config.h` + create `Motor` in `main.cpp` + add parsing in `handleSerial`/`parseAxisMove` |
| Test X/Y on bench | [`test/test_xy_motors/`](../test/test_xy_motors/) |

---

## 12. Known caveats / TODOs an agent might want to fix

- **`platformio.ini` has Windows-only `COM3` ports** — should be removed or made conditional for cross-platform development.
- **`UART.h` is empty** — either remove or implement.
- **`lib/Motor/` empty files** — confusing duplicate of `src/Motor.cpp`. Delete the empty stubs.
- **`Tracking` is a stub** — closed-loop tracking is named but not built; the "tracking test" is actually a manual-jog test embedded in `main.cpp`.
- **Shared enable pin** is declared three times (`Config::X::enablePin`, `Config::Y::enablePin`, `Config::Z::enablePin` all = 27). A single shared symbol would be clearer.
- **`Search` direction reversal is instantaneous** with no acceleration ramp. At higher speeds this may stall the motor — currently fine at 70 steps/sec.
- **The `lastState` global in `main.cpp` and `transitionedAfterHoming` flag** are mutable globals at file scope; consider encapsulating if you refactor.
- **Manual moves can saturate `totalManualMovementSteps` (long)** in theory, but never in practice at human-typed step counts.
- **There is no command to re-home** without resetting the board. Adding an `h` command to call `homing.start()` again would be a small, useful change.

---

*End of document. Last updated for the repository state at commit `a4fe4a2` ("Initial GooseV3 ESP32 project commit") on branch `main`.*
