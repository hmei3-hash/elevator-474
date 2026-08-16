# elevator-474

Two-board ESP32 smart elevator. CSE/EE 474 Embedded Systems final project.

**Hongyi Mei, Kevin Bi — Summer 2026**

---

## Status

**Skeleton. Every function body is a `// TODO:` stub.**

Nothing in this repository has been run on hardware. Every row in
[`docs/vv_table.md`](docs/vv_table.md) is `NOT TESTED`. No pin has been
verified against a physical wire, no tuning constant has been chosen, and
no algorithm has been written.

This is deliberate. The structure, the layering, and the interfaces between
the two boards are settled first; the bodies get filled in one at a time,
each against a measurement. See [Approval boundaries](#approval-boundaries).

| Component | State |
|---|---|
| Repository structure | Complete |
| Task/core/priority plan | Complete, not yet validated on hardware |
| Link protocol | Defined, both copies verified identical |
| Pin assignments | Assigned from the datasheet, **not verified against wiring** |
| Driver bodies | Stubs |
| Control policy | Stubs |
| Fall detection | Stubs |
| Host-testable logic | Stubs, harness builds and runs |
| Stack sizes, queue depths | Placeholders pending measurement |

---

## What it does

An elevator car travels between three floors under closed-loop position
control. Passengers select a floor from a panel inside the car, or present
an RFID card. A character LCD shows the current mode, direction, and floor.

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

## Verification and validation

[`docs/vv_table.md`](docs/vv_table.md) holds 16 rows across four tiers:
unit (host), integration (instrumented on-target), hardware (per
peripheral), and system (end-to-end behaviour).

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
| Hongyi Mei | Board A: control, motion, sensing, UI. Integration and sign-off. |
| Kevin Bi | Board B: IMU, fall detection, transmit path. |

The link protocol is the contract between the two halves. It was frozen
before either half was written, so the boards could be developed in
parallel — the scarcest resource in a two-week project.
