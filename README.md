# elevator-474

Two-board ESP32 smart elevator. CSE/EE 474 Embedded Systems final project.

**Hongyi Mei, Kevin Bi — Summer 2026**

---

## Status

**Delivered. Demonstrated on hardware 08/21/2026.**

The car closes a feedback loop and travels between both floor setpoints in
both directions, and both safety paths work: a detected fall stops the car,
and cutting power to the fall detector also stops it, under a distinct fault
code. Video: <https://youtu.be/DBCa3tUcbC8>

What is *not* finished is evidence, not function. Several checks in
[`docs/vv_table.md`](docs/vv_table.md) remain `NOT TESTED`, and two are
honest failures rather than omissions — see [Verification and
validation](#verification-and-validation). We have not promoted a row on the
strength of a run that happened to succeed; a single clean run is not the
acceptance criterion any of them states.

Each module was proven as a standalone sketch first and only then folded
into the layered structure here — see [Approval
boundaries](#approval-boundaries) and the workflow in `CLAUDE.md`. What went
wrong along the way is recorded in [What we got
wrong](#what-we-got-wrong), including the failures that did not announce
themselves.

| Component | State |
|---|---|
| Repository structure | Complete |
| Task/core/priority plan | Complete, running on hardware |
| Link protocol | Both copies verified byte-identical |
| Pin assignments | `board_config.h` is authoritative |
| Driver bodies | Complete. Zero `TODO` remaining in any `.cpp` |
| Control policy | Complete: floor requests, target selection, closed loop, emergency stop |
| Feedback sensor | **HC-SR04 rangefinder.** The AS5600 encoder was removed late — see [note](#the-feedback-sensor-changed-late) |
| Fall detection | Complete. Thresholds from 246 s of labelled bench data. **8 detections in 10** |
| Access control | 20-second authorisation window, gating the car buttons |
| PID gains | `CTRL_ULTRA_KP = 2.5`, Ki and Kd zero. Proportional-only was sufficient |
| Stack sizes, queue depths | Workable values with margin. **Not measured** — no high-water probe was run |
| Mechanical build | Complete: two floors at 30 mm and 153 mm |

### The feedback sensor changed late

The design in this README was written around the AS5600 magnetic encoder as
the loop sensor, chosen because it is quiet and absolute. It was removed
during the final week and the ultrasonic rangefinder became the primary
feedback.

The reason is that the encoder reads the *motor shaft*, so it cannot observe
a slipping line — it reports the position the motor was commanded to reach,
which is the one thing a position sensor must not do. The rangefinder
measures the car itself. It is much noisier, and the loop pays for that with
a 5 mm deadband where the encoder would have allowed a fraction of a
millimetre, but it measures the quantity that actually matters.

The rangefinder produces a false near echo at roughly 29 mm against a real
target near 39 mm. Because that failure mode is always *short*, a rolling
upper-envelope filter — the maximum of a short window — rejects it.

Sections below that describe the encoder as the loop sensor reflect the
original design and are left in place deliberately: the reasoning was sound
when written, and replacing it would erase why the substitution was needed.

---

## What it does

An elevator car travels between floor setpoints under closed-loop position
control, driven by a 100 Hz loop closed around an ultrasonic rangefinder.
The shipped build has two floors, at 30 mm and 153 mm; the third setpoint is
present in the configuration and disabled.

A rider presents an RFID card, which opens a twenty-second authorisation
window. Inside that window the three car-panel buttons are served; outside
it a press is refused and logged rather than queued — a queued press would
move the car later, when the window next opens, with nobody having asked. A
character LCD shows the floor, direction, access verdict, and any latched
fault.

A second, physically separate board watches for a fall using an inertial
sensor. It has no wires to the elevator — it talks over ESP-NOW. If it
detects a fall, or if it goes silent, the car stops.

The safety posture is the interesting part, and it is inverted from the
obvious design. See [Fail-safe posture](#fail-safe-posture).

---

## Hardware

### Boards

Two × **ESP32-S3-DevKitC-1**, module **ESP32-S3-WROOM-1 N16R8**
(16 MB flash, 8 MB octal PSRAM, dual-core Xtensa LX7 @ 240 MHz).

### Board A — elevator controller

| Peripheral | Interface | Role |
|---|---|---|
| NEMA-17 stepper + TMC2209 driver | STEP/DIR/EN + UART | Car motion |
| HC-SR04 ultrasonic rangefinder | Trigger + echo pulse | Car position feedback |
| AS5600 magnetic rotary encoder | I2C | Motor shaft angle, closed-loop position |
| 1602 character LCD | I2C (PCF8574 backpack) | Passenger display |
| RC522 contactless card reader | SPI | Card authentication |
| 3 × pushbutton | GPIO, `INPUT_PULLUP` | Car-panel floor selection |

### Board B — fall detector

| Peripheral | Interface | Role |
|---|---|---|
| MPU6050 six-axis IMU | I2C | Motion sensing |

### Not present

There is **no IR receiver** and there are **no hall-call buttons**. All
floor requests originate inside the car. This is a deliberate scope
reduction: the car cannot be summoned from a floor, and RFID stands in for
external authorisation.

### Wiring warnings

- **The HC-SR04 echo pin outputs 5 V.** The ESP32-S3 is 3.3 V and is not
  5 V tolerant. A divider or level shifter is mandatory; connecting it
  directly will damage the GPIO.
- **GPIO33–37 are consumed by the octal PSRAM** on the N16R8 module, and
  GPIO26–32 by the SPI flash. Neither range is available. Assigning a
  signal there produces a board that boots erratically or not at all,
  with no obvious error.
- Reserved pins are tabulated in [`docs/pinmap.md`](docs/pinmap.md).

---

## Repository layout

```
elevator-474/
├── CLAUDE.md              Working agreement. Read before changing anything.
├── README.md              This file.
├── .gitignore
├── docs/
│   ├── pinmap.md          Authoritative record of the physical wiring.
│   └── vv_table.md        Verification & validation table.
├── ElevatorA/             Sketch: elevator controller.
│   ├── ElevatorA.ino          setup() assembly, task entry points.
│   ├── board_config.h         Pins, rates, priorities, cores, stacks.
│   ├── app_types.h            Queue messages, fault codes, system_state_t.
│   ├── link_protocol.h        Wire format. Identical to Board B's copy.
│   ├── shared_state.h/.cpp    The one system_state_t + its spinlock.
│   ├── control.h/.cpp         Elevator policy. The only file that decides.
│   ├── tasks_ui.cpp           Display composition and serial logging.
│   ├── link.h/.cpp            ESP-NOW receive path.
│   ├── stepper.h/.cpp         Motion driver.
│   ├── hcsr04.h/.cpp          Rangefinder driver.
│   ├── rc522.h/.cpp           Card reader driver.
│   └── lcd_ui.h/.cpp          Display driver.
├── ElevatorB/             Sketch: fall detector.
│   ├── ElevatorB.ino          setup() assembly, task entry points, TX path.
│   ├── board_config.h
│   ├── link_protocol.h        Byte-identical copy of Board A's.
│   ├── mpu6050.h/.cpp         IMU driver.
│   └── fall_detect.h/.cpp     Detection policy.
└── logic/                 Host-testable. Zero platform dependencies.
    ├── pid.h/.c               Discrete PID controller.
    ├── filter.h/.c            Moving average, residual tracking.
    └── test/test_main.c       Host harness.
```

Two bring-up sketches live **outside** this tree, in the parent folder:
`mac_print/` prints a board's MAC address, and `stack_probe/` measures
stack high-water marks and queue occupancy. They are instruments, not
project code.

---

## Architecture

### Layering

```
App     control.cpp, tasks_ui.cpp, fall_detect.cpp, *.ino
  ↓
Driver  stepper, hcsr04, rc522, lcd_ui, mpu6050, link
  ↓
HAL     Arduino core, ESP-IDF
```

Enforced in both directions:

- **A driver must not know what a floor is.** No driver file may contain
  the word "elevator", or any floor, door, or request concept. `stepper.cpp`
  speaks in steps and directions; deciding that 400 steps means "second
  floor" is the application's job.
- **An app file must not touch GPIO, I2C, or SPI.** It reaches hardware
  only through a driver header.
- **`logic/` sits below all of it** with zero platform dependencies, so it
  compiles and runs on a development machine.

The payoff is that the position filter and the PID loop can be tested on a
laptop in a second, instead of by uploading to a board and watching a car
move.

### Task, core, and priority assignment

**The core split is load-bearing, not stylistic.** The Arduino-ESP32 WiFi
task is pinned to Core 0 at priority 23 — above every task listed below.
The 100 Hz control loop therefore runs on Core 1, where the radio stack
cannot preempt it. Moving a task between cores without re-measuring jitter
invalidates VV-15.

#### Board A

| Core | Task | Prio | Rate | Responsibility |
|---|---|---|---|---|
| 1 | `controlTask` | 6 | 100 Hz | Control loop. Sole writer of shared state. |
| 1 | `ultrasonicTask` | 5 | ~16 Hz | Ranging. Blocks on echo timeout. |
| 1 | `loadTask` | 1 | free | Background compute load for the deadline demo. |
| 0 | `linkTask` | 5 | 50 Hz | Frame validation, heartbeat deadline. |
| 0 | `inputTask` | 4 | 200 Hz | Button sampling and debounce. |
| 0 | `rfidTask` | 3 | ~10 Hz | Card polling. |
| 0 | `lcdTask` | 2 | 5 Hz | Display repaint. |
| 0 | `logTask` | 1 | 10 Hz | Serial output. Only task that may print. |

#### Board B

| Core | Task | Prio | Rate |
|---|---|---|---|
| 1 | `sampleTask` | 6 | TBD from sensor output rate |
| 1 | `detectTask` | 5 | TBD from detection window |
| 0 | `txTask` | 4 | TBD, chosen jointly with A's timeout |

### Inter-task communication

Three queues on Board A, with three deliberately different semantics. They
are not interchangeable and must not be used with the same send/receive
pattern.

| Queue | Depth | Semantics | Pattern |
|---|---|---|---|
| `g_q_input` | 16 | Event stream — every event matters | `xQueueSend` with 0 timeout; consumer **drains fully** each iteration |
| `g_q_ui` | 1 | Latest value — old frames are worthless | `xQueueOverwrite`; a stale floor on the display is worse than a dropped frame |
| `g_q_log` | 32 | Lossy — bursts absorbed, drops counted | `xQueueSend` with 0 timeout; **never block a producer on logging** |

`g_q_ui` has depth 1 on purpose. `controlTask` produces at 100 Hz and
`lcdTask` consumes at 5 Hz — a 20:1 ratio that would overflow any queue.
The fix is overwrite semantics, not a bigger buffer.

Shared state is a single `system_state_t` behind a `portMUX` spinlock, in
`shared_state.cpp`. One writer (`controlTask`), several readers, whole-struct
snapshot copies so a reader never observes a half-updated record.

---

## The link

Board B transmits, Board A receives. One direction, no acknowledgement.

ESP-NOW is connectionless: no router, no IP, no association. Each board
brings its radio up in station mode without joining an access point, and
frames are addressed by MAC. Board B registers Board A as a peer; Board A
registers nobody, because it never transmits.

### Frame format

Defined in `link_protocol.h`, 14 bytes, packed:

| Offset | Field | Purpose |
|---|---|---|
| 0 | `uint8_t ver` | Must equal `LINK_PROTO_VER`; mismatched frames are dropped |
| 1 | `uint8_t type` | `HEARTBEAT` or `STOP` |
| 2 | `uint32_t seq` | Monotonic, for loss and reorder detection |
| 6 | `uint8_t payload[8]` | `STOP` puts its reason code in `payload[0]` |

`ver` earns its byte the first time you reflash one board and forget the
other: the receiver rejects the frame instead of parsing an old layout into
plausible-looking garbage.

No application checksum — ESP-NOW already CRCs at the MAC layer.

### The two copies must stay identical

Arduino IDE compiles each sketch folder separately, so the header cannot be
shared and is duplicated. After touching either copy:

```bash
diff ElevatorA/link_protocol.h ElevatorB/link_protocol.h
```

An empty diff is the only acceptable result. A divergence produces wrong
field values with no error anywhere — the worst class of bug on this
project. Bump `LINK_PROTO_VER` on any change and reflash **both** boards.

### The receive callback

The ESP-NOW receive callback runs in the **WiFi task context: Core 0,
priority 23** — above every application task.

It must copy the bytes into a queue and return. No printing, no parsing, no
blocking calls, no lock acquisition. A millisecond spent there is a
millisecond stolen from a task that cannot preempt it.

`linkTask` exists to be the buffer between that callback and the
application. That is its entire reason to exist.

---

## Fail-safe posture

**The link is safe-on-silence, not safe-on-message.**

The naive design stops the car when a `STOP` frame arrives. That design
fails open: if Board B loses power, breaks its antenna, or hangs, it can
never send `STOP`, and the car runs forever with a broken safety monitor.

So the heartbeat is the safety mechanism, and `STOP` is the optimisation.
Board B continuously transmits proof of life; Board A stops the car when
that proof stops arriving. A dead Board B and a fallen passenger produce
the same outcome — the car stops — which is the correct behaviour in both
cases.

Consequences that show up in the code:

- `FAULT_LINK_TIMEOUT` and `FAULT_FALL_DETECTED` are **separate bits**.
  Both stop the car; the log must distinguish them.
- `STOP` frames are sent **repeatedly** (`STOP_FRAME_REPEATS`). ESP-NOW does
  not retry, so a one-shot stop can be lost to a single collision.
  Repetition trades a few frames for a much lower miss probability.
- The heartbeat timeout is a **measured** value, not a guessed one: it comes
  from Board B's transmit period and the observed packet loss rate. Too
  short and the car stops spuriously; too long and a real failure goes
  unnoticed. VV-13 is the test that pins it down.

---

## Build and upload

### Arduino IDE settings

Both boards, identical:

| Setting | Value | Why it matters |
|---|---|---|
| Board | ESP32S3 Dev Module | |
| **PSRAM** | **OPI PSRAM** | Required for N16R8. Wrong value → PSRAM unusable or boot loop |
| Flash Size | 16MB (128Mb) | |
| **USB CDC On Boot** | **Enabled** | Otherwise `Serial` output never appears |
| Partition Scheme | Default 4MB with spiffs | |
| Upload Speed | 921600 | Drop to 115200 if uploads fail |

`ElevatorA/` and `ElevatorB/` are separate sketches. Open each folder
directly; Arduino IDE requires the folder name to match the `.ino` name.

### Host-side logic

```bash
cd logic/test
gcc -Wall -Wextra -I.. -o test_logic test_main.c ../pid.c ../filter.c -lm
./test_logic
```

Right now this reports **7 run, 7 failed** and exits 1. That is correct and
intentional: every case is an empty stub asserting `0`. A harness that
prints PASS while asserting nothing is worse than no harness, because it
manufactures false confidence. The cases turn green as the bodies are
written.

Syntax check without linking:

```bash
gcc -Wall -Wextra -fsyntax-only logic/*.c
```

---

## Bring-up order

Each step must produce evidence before the next begins. Skipping ahead is
how a wiring fault gets misdiagnosed as a software bug at 2 a.m.

1. **MAC addresses.** Flash `mac_print/` on each board. Record both
   addresses and the WiFi channel in `docs/pinmap.md`. Put Board A's MAC
   into Board B's `PEER_MAC_BYTES`.
2. **Wiring.** Wire one peripheral at a time. Record each connection in
   `docs/pinmap.md` **as you make it**, then make `board_config.h` match.
   The table is the source of truth; the header is its mirror.
3. **Drivers, one at a time.** Fill in one driver, confirm it reports
   healthy on ten cold boots, move on. Do not wire everything and then
   debug everything at once.
4. **Link.** Bring up ESP-NOW with a broadcast peer
   (`FF:FF:FF:FF:FF:FF`) first — that removes MAC transcription errors from
   the picture. Once bytes flow, switch to unicast. Unicast matters at demo
   time, when other teams are also transmitting.
5. **Stack and queue sizing.** Paste each real task body into
   `stack_probe/`, exercise the system, read the `SUGGEST` column, fill in
   `STACK_*` and `QDEPTH_*`. Do **not** ship the 4096-word placeholder —
   eight tasks × 4096 words is 128 KB of the 512 KB budget, and it masks
   genuine overflow risk.
6. **Control loop.** Only after position feedback is trustworthy. Tune
   against the step response of the assembled car, not against intuition.
7. **Safety path last.** VV-13 (cut Board B's power mid-travel) and VV-14
   (trigger a fall mid-travel) are the tests that matter most and are
   easiest to fake. Record video of both.

---

## What we got wrong

Kept deliberately, and kept honest. Every entry below cost real time, and
several would have survived to the demo if they had not been caught. This
section feeds the "what you fixed" chapter of the AI report directly.

### 1. Two sources of truth for pin assignments

**Issue.** The bench prototypes and `board_config.h` drifted apart. At one
point the stepper block's comment said `SDA=8, SCL=9` while the `#define`
right below it said `14` and `13`. The RFID prototype used GPIO 4, 5 and 6
— which are the stepper's EN, STEP and DIR. Wiring both at once would have
shorted two subsystems together.

**Cause.** Prototypes were written against whatever pins were free at the
time, and nobody declared which file won.

**Fix.** `board_config.h` is authoritative, stated in the file header and in
`CLAUDE.md`. Prototype pins were moved to match it, never the reverse.
`docs/pinmap.md` records the physical wiring; the header mirrors it.

**Evidence.** Pin table in `docs/pinmap.md`; the "This header is
authoritative" note in `board_config.h`.

### 2. An IMU axis died silently for 51 seconds

**Issue.** In the 297-second drop-data session, `az` sat at exactly 32767 —
full scale, 16 g — for the first 51 seconds and then recovered. A stationary
sensor cannot read 16 g. Seventeen percent of the session was fabricated
data that looked entirely plausible in a serial monitor.

**Cause.** Almost certainly an intermittent connection. A partly-seated
jumper on a breadboard row that had earlier had a broken pin pushed into it.

**Fix.** `analyze_falls.py` now detects stuck-axis runs and cuts them before
deriving anything. The remaining 246 seconds were used.

**Why it matters more than it looks.** This is the failure mode that does
*not* announce itself. A disconnected sensor stops responding and every
check catches it; a stuck axis keeps streaming numbers. Any threshold fitted
across that block would have been quietly wrong.

**Evidence.** `imu_data.csv`; the STUCK-AXIS block in the analysis output.

### 3. The encoder magnet ended up inside the motor

**Issue.** The motor vibrated in place instead of turning, and the AS5600
reported `TOO WEAK` at the same time. Two subsystems failing at once
suggested two problems.

**Cause.** One problem. A neodymium magnet near a stepper is pulled into it;
once inside it fought the rotor's field, and the encoder was left reading
stray flux.

**Fix.** The magnet mounts on the shaft *end*, outside the motor body, on a
non-ferrous spacer. A steel spacer would conduct the motor's flux straight
to the sensor.

**Lesson.** When two subsystems fail simultaneously, look for one cause
before assuming two.

### 4. A stall counter that only counted up

**Issue.** The prototype's stall detector incremented a counter while the
stall condition held, and never cleared it when the condition lapsed.
Unrelated near-stalls minutes apart would accumulate and eventually latch a
fault that never happened.

**Cause.** The counter was `static` inside the `if` body, so the reset path
had nowhere to live.

**Fix.** The counter is cleared on every iteration the condition does not
hold. See `control.cpp`.

### 5. A blocking serial read inside a 1 kHz control loop

**Issue.** `Serial.readStringUntil('\n')` has a one-second default timeout.
A single stray byte with no newline would freeze the control loop for a
second — with the motor running.

**Cause.** Command parsing shared a loop with control, which is normal in a
prototype and unacceptable in the real system.

**Fix.** Command handling moved out of the control path entirely at
integration.

### 6. A buffer overread in the RFID comparison

**Issue.** The prototype compared `rfid.uid.size` bytes against a 4-byte
authorised-UID array. Plenty of MIFARE cards have 7-byte UIDs, and those
would have read three bytes past the end of the array.

**Cause.** UID length was assumed rather than checked.

**Fix.** `rc522.cpp` carries the length explicitly and compares lengths
before contents.

### 7. Authorisation logic living in a driver

**Issue.** The prototype decided access inside the card-reading code.

**Cause.** Convenience — it is where the UID already is.

**Fix.** The driver reports a UID and nothing else. Deciding what a card is
allowed to do is application policy. A driver that knows about
authorisation has to change every time the policy does.

### 8. Fixed-window segmentation found nothing in good data

**Issue.** The first pass at the drop analysis cut a window around each
`FALL` label and found no free-fall in any of them — every feature
overlapped with normal handling. The data looked useless.

**Cause.** The operator types the label and *then* picks up the rig and
drops it, one to three seconds later. The label marks intent, not the event.

**Fix.** Detect episodes from the signal — the physics is unambiguous — and
use the labels only to attribute them afterwards. This is also closer to
what the on-target detector does.

**Result.** Ten real drops, and three features that separate perfectly.

### 9. Assuming ±16 g was wide enough

**Issue.** The accelerometer range was set to its widest on the argument
that impacts exceed 4 g. Composite peaks still reached 21–25 g, meaning
individual axes clipped at 16 g during landings.

**Cause.** The estimate was for a single axis; the vector magnitude of three
axes can exceed any one of them.

**Fix.** Accepted rather than corrected. Free-fall detection reads the
*low*-g phase, where nothing clips, and the impact test only asks whether
the peak is large. A clipped large value is still large. The clipping is
recorded so nobody later mistakes a bounded peak for a measured one.

### 10. PID gains assumed to be portable across loop rates

**Issue.** The prototype's gains were tuned with the loop at 1 kHz. The
architecture runs `controlTask` at 100 Hz.

**Cause.** Gains feel like properties of the plant. Two of the three are
properties of the plant *and the sample interval*.

**Fix.** Gains ship as `0.0f` with a retuning procedure beside them, so the
system compiles and refuses to move rather than moving wrongly. Caught
before hardware, not after.

### 11. Git lock files left behind by a tool that cannot delete

**Issue.** `git branch -M main` failed with "another git process seems to be
running", and `.git/HEAD.lock`, `.git/index.lock` and
`.git/objects/maintenance.lock` were unremovable from the environment that
created them.

**Cause.** The repository was initialised from a sandbox whose mount is
write-but-not-delete. Git creates lock files and removes them on exit; the
removal silently failed.

**Fix.** Deleted the locks from Windows, and all subsequent git work is done
natively. Forty-two orphaned `tmp_obj_*` files remain in `.git/objects` and
are harmless; `git gc --prune=now` clears them.

### 12. A placeholder pasted into a real command

**Issue.** `git remote add origin https://github.com/<username>/...` was run
with the placeholder text still in it, and the second, correct `remote add`
was rejected because origin already existed.

**Fix.** `git remote set-url`. Commands handed over now carry real values,
never placeholders.

### 13. Spinning a stepper straight to full rate

**Issue.** The blind-spin test jumped from standstill to 3.3 kHz with no
ramp, and the motor stalled immediately.

**Cause.** A test written to be short rather than correct.

**Fix.** Acceleration limiting exists in `control.cpp` for exactly this
reason; the test is the only place it was missing.

### 14. A 33-nanosecond step pulse

The motor would not turn. Every software observable said it was being
driven: the commanded position advanced, the reported rate was non-zero, and
a multimeter on STEP and DIR read normal.

The pulse was eight `nop` instructions — roughly 33 ns at 240 MHz, against
the TMC2209's 100 ns minimum. The signal was present and simply too narrow
for the driver to sample, which is precisely the failure a voltmeter cannot
see.

What found it was not more measurement but a comparison: diffing the driver
against a throwaway sketch known to turn the motor. **When every instrument
agrees the system is working and it is not, the instrument is measuring the
wrong property, and a known-good reference is worth more than another
reading.**

### 15. A direction pin written only on change

The direction output was written only when the requested direction differed
from a cached copy of it. Once the cache and the hardware disagreed — after
any missed write — the axis could never recover. The step train was also
left running across the reversal, losing a step on every change of
direction, and the pin was written from inside a critical section using a
non-IRAM function.

The fix is four steps in a necessary order: zero the increment, write the
pin unconditionally outside the critical section, honour a setup delay,
restore the increment. **Caching a hardware state you cannot read back is a
bet that nothing else will ever touch it.**

### 16. One cause wearing three costumes

For part of an afternoon we chased three faults: the motor would not move,
the status command did not answer, and the buttons did nothing.

They were one fault. The USB cable was in the UART socket rather than the
native USB socket, and with `ARDUINO_USB_CDC_ON_BOOT=1` the application's
serial output goes to the native port. The firmware was running and
completely mute. The ROM bootloader banner still appeared — it comes out of
the UART port — which is exactly why the board looked alive.

**Confirm the program is running before reasoning about its logic.** The
value of the `Board A running.` line had been underrated all week.

### 17. A hypothesis that could not be cheaply falsified

Board A reported a link timeout with all four diagnostic counters at zero.
That is exactly the signature of a peer MAC pointing at the wrong board, and
the address had been recorded days earlier with no way to re-check it. We
treated it as the prime suspect for hours.

It was correct all along. What finally settled it was a probe that both
boards run unmodified and that broadcasts to `FF:FF:FF:FF:FF:FF`, removing
the peer address from the experiment entirely: 1106 frames over 264 s with
zero loss, using the same address the firmware uses.

The real cause was on the other board. `ElevatorB`'s `setup()` initialises
the IMU *before* Wi-Fi, ESP-NOW or the transmit task, and halts on failure,
so a detector whose IMU does not answer never sends a single heartbeat. The
halt is correct — a fall detector that cannot sense should not pretend to
work — but it makes an IMU fault present as a link fault, and Board A cannot
tell that from an unplugged board. That indistinguishability *is* the
safe-on-silence design.

**A plausible hypothesis that cannot be cheaply falsified will absorb
unlimited time. Build the falsifier first.** We had also read Board A's
serial output all week and never once opened Board B's.

### 18. A log that destroyed its own evidence

Log lines arrived spliced together mid-number. The ranging task logged every
sample at 16 Hz and the load task every 1024 primes, overrunning the USB CDC
transmit buffer — which drops bytes *silently*, so the failure looks like
corrupted data rather than lost output.

Ranging now logs on change, the load counter reports every 65536, and
emission is capped at 40 lines per second inside `logTask` with the number
of suppressed lines printed. The cap is the durable part: a flood cannot be
prevented at the source without knowing in advance which source will flood,
so it is bounded at the one point every line passes through. The count
matters as much as the cap — **"twelve thousand lines were dropped" and
"nothing happened" must not look the same on the wire.**

### 19. A driver failure that printed nothing

`setup()` latched a fault bit on a failed driver init and said nothing.
Because the panel renders only the highest-priority fault, a link timeout
masked a failed display entirely, and the log carried no init record at all.
"The LCD is blank" was a guess with no reading behind it.

Now every driver prints `OK`/`FAIL` at boot, and an I2C bus scan runs
*before* any driver touches the bus, at both 100 kHz and the configured
speed. A driver reporting `FAIL` cannot distinguish a wrong address from bad
wiring from a bus clocked faster than the part can follow — and those three
need opposite fixes.

### 20. Telemetry slower than the process it measured

The tuning stream ran at 10 Hz against a 100 Hz control loop. A step
response settling in 200 ms would leave two samples, from which neither
overshoot nor settling time can be read. **Sampling ten times slower than
the process does not give a coarse picture of it; it gives none.**

Raising the stream alone was not enough — `cmdTask` itself only woke at
20 Hz, so the faster stream would have been silently capped by its own task
rate.


---

## Verification and validation

[`docs/vv_table.md`](docs/vv_table.md) holds 16 rows across four tiers:
unit (host), integration (instrumented on-target), hardware (per
peripheral), and system (end-to-end behaviour). The full tables, with
per-subsystem build, normal, boundary and fault checks, are in the AI report
submitted alongside this repository.

**Outcome.**

| Check | Status | Note |
|---|---|---|
| Host logic suite | PASS | 7 cases, 0 failures, exit 0, clean under `-Wall -Wextra` |
| Protocol copies identical | PASS | `diff` of the two `link_protocol.h` copies is empty |
| Both builds | PASS | Both PlatformIO environments build |
| Closed-loop positioning | PASS | Reaches and holds a commanded height; decelerates as the error shrinks |
| Repeated bidirectional travel | PASS | Runs up and down between both setpoints |
| Link loss stops the car | PASS | `FAULT_LINK_TIMEOUT`; the bit clears on the first heartbeat back |
| Fall stops the car | PASS | Distinct fault code from a link loss |
| Car buttons, panel, ranging | PASS | Demonstrated in normal operation |
| Authorisation window | PASS | Served inside the window, refused after expiry |
| **Fall detection rate** | **FAIL** | **8 in 10 across 20 drops. The check accepts only a full rate** |
| **Motion driver init** | **FAIL** | **TMC2209 UART returns `0x00` — PDN_UART is unwired by decision** |
| Link idle | PARTIAL | 2 minutes clean; the check asks for 5 |
| Stack and queue sizing | NOT MEASURED | Nothing overflowed, but absence of failure is not a measured margin |
| Jitter, fall negatives, button repeatability, fault injection | NOT TESTED | Require sustained collection we ran out of time for |

**On the 8 in 10.** The thresholds are unlikely to be the cause. In the
clean logged data the two populations do not overlap anywhere: normal
handling never went below 0.66 g and never spent a millisecond under 0.5 g,
while every real drop reached 0.08–0.22 g and stayed there for 65–255 ms. A
miss is far more likely a drop too short to accumulate the 40 ms the
free-fall stage requires than a threshold set wrong.

Requiring free fall *before* impact is what buys zero false positives. A
rule firing on impact alone would have caught both misses — and would also
fire when the rig is set down briskly on a table, which is exactly the false
positive an elevator must not have. We chose a rule whose failures are
misses rather than false alarms, then measured a miss rate we did not have
time to reduce. Shortening the free-fall window and re-running the negative
set is the obvious next experiment. We did not run it, so we do not claim it
would work.

### Evidence policy

**A clean compile never makes a hardware test PASS.**

Neither does an upload that finished without an error, nor a serial line
printing the value someone hoped to see. Those demonstrate that the
toolchain works — not that the system does what it claims.

A row moves off `NOT TESTED` only when the artifact named in its evidence
column exists and has been reviewed. Rows are never marked PASS in advance,
never in bulk, and only by Hongyi.

If a test was not run, its status is `NOT TESTED`. Saying so is always
correct; guessing is not.

---

## Approval boundaries

Recorded in full in [`CLAUDE.md`](CLAUDE.md). The short version:

1. Every function body stays a stub until it is explicitly asked for — one
   `// TODO:` line and whatever return is needed to compile.
2. No unrequested algorithms: no PID math, no sensor fusion, no stepper
   sequencing tables, no debounce logic, no filter internals, no thresholds,
   no tuning constants. Each stays a `// TODO:` naming what must be decided
   and **which measurement it depends on**.
3. Pin numbers and constants are placeholders until filled from the bench.
   No plausible-looking invented values.
4. Headers declare, `.cpp` files define. `extern` in the header, defined
   once in the `.cpp`.
5. Anything used by one `.cpp` is `static` and stays out of every header.
6. Every `.cpp` includes `<Arduino.h>` first — except `logic/`, which must
   stay host-compilable.
7. `#pragma once` in every header.
8. A documentation block on every public function: purpose, parameters,
   return value, units, and which task on which core calls it.
9. If something looks missing, say so rather than adding it.

Hongyi does the wiring, the verify, the upload, and grants PASS.

---

## Team

| | |
|---|---|
| Hongyi Mei (2564361) | Repository architecture and layering; task, core and priority assignment; link protocol and fail-safe posture; drivers and bring-up diagnostics; the step-pulse and direction fixes; RFID authorisation; V&V design and sign-off. |
| Kevin Bi (2462768) | Board B: IMU, fall detection and transmit path. Ultrasonic closed-loop control and floor calibration; the sensor substitution; mechanical assembly and demonstration support. |

The link protocol is the contract between the two halves. It was frozen
before either half was written, so the boards could be developed in
parallel — the scarcest resource in a two-week project.
