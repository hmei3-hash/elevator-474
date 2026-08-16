# CLAUDE.md -- working agreement for this repository

Read this before changing anything here.

## What this project is

A two-board ESP32 smart elevator, built for CSE/EE 474 as a two week final
project by a team of two: Hongyi Mei and Kevin Bi.

Board A is the elevator controller. Board B is an independent fall
detector. They are linked over ESP-NOW.

## Board and toolchain

| Item | Value |
|---|---|
| Board | 2x ESP32-S3-DevKitC-1, module ESP32-S3-WROOM-1 N16R8 |
| Framework | Arduino |
| RTOS | FreeRTOS, as shipped in the ESP32 Arduino core |
| IDE | Arduino IDE, used for both verify and upload |
| Board setting: PSRAM | OPI PSRAM, required for the N16R8 module |
| Board setting: Flash | 16MB (128Mb) |
| Board setting: USB CDC On Boot | Enabled, or serial output never appears |
| Host toolchain for `logic/` | plain gcc, no cross compiler |

## Hardware split

Board A owns the stepper driver, the ultrasonic rangefinder, the character
LCD on I2C, the card reader on SPI, and three car-panel buttons.

Board B owns the inertial sensor on I2C and does fall detection only.

There is no IR receiver and there are no hall-call buttons. All floor
requests originate inside the car.

## Pin map

Authoritative copy lives in `docs/pinmap.md`. `board_config.h` is the
code's mirror of it. If the two disagree, the bench wiring is right and
the header is wrong.

| Signal | Board | GPIO | Notes |
|---|---|---|---|
| | | | |

## Layering rule

    App  ->  Driver  ->  HAL

Enforced strictly, in both directions:

- A driver file must not contain the word "elevator", nor any floor, door,
  or request concept. A driver that knows what a floor is has already
  broken the rule.
- An app file must not touch GPIO, I2C, or SPI directly. It reaches
  hardware only through a driver header.
- `logic/` sits below all of this and must have zero Arduino and zero
  ESP32 dependencies, so it compiles and runs on a host machine.

## Task, priority, and core assignment

The core split is deliberate and load-bearing. The Arduino-ESP32 WiFi task
is pinned to Core 0 at priority 23. The 100 Hz control loop therefore runs
on Core 1, where the radio stack cannot preempt it. Do not move a task
between cores without re-measuring jitter and recording the result.

### Board A

| Core | Task | Priority | Rate |
|---|---|---|---|
| 1 | controlTask | 6 | 100 Hz |
| 1 | ultrasonicTask | 5 | ~16 Hz |
| 1 | loadTask | 1 | free-running |
| 0 | linkTask | 5 | 50 Hz |
| 0 | inputTask | 4 | 200 Hz |
| 0 | rfidTask | 3 | ~10 Hz |
| 0 | lcdTask | 2 | 5 Hz |
| 0 | logTask | 1 | 10 Hz |

### Board B

| Core | Task | Priority | Rate |
|---|---|---|---|
| 1 | sampleTask | 6 | TBD from sensor output rate |
| 1 | detectTask | 5 | TBD from detection window |
| 0 | txTask | 4 | TBD, set with Board A's timeout together |

## The ESP-NOW receive callback

The receive callback runs in the WiFi task context: Core 0, priority 23.
It must copy the incoming bytes into a queue and return. No printing, no
parsing, no blocking calls, no lock acquisition. Work done there runs above
every application task on the system and will break control timing.

`linkTask` exists to be the buffer between that callback and the
application. That is its whole purpose.

## Fail-safe posture

The link is not safe-on-message, it is safe-on-silence. A stop is not
conditional on receiving a STOP frame, because a dead Board B can never
send one. Board A treats the absence of a heartbeat as a fault and stops.

`FAULT_LINK_TIMEOUT` and `FAULT_FALL_DETECTED` are separate bits on
purpose: both stop the car, but the logs must distinguish them.

## The protocol file

`ElevatorA/link_protocol.h` and `ElevatorB/link_protocol.h` must be
byte-identical. Arduino IDE compiles each sketch folder separately, so the
file cannot be shared and is duplicated instead. After touching either
copy, run `diff` on the two and confirm it is empty. A silent divergence
produces garbage field values with no error message anywhere.

Bump `LINK_PROTO_VER` on any change, and reflash both boards.

## Development workflow

Modules are proven standalone first, then integrated. Not the other way
round.

1. **Prototype.** A module is developed as its own throwaway Arduino sketch,
   outside this tree, exercising one peripheral only. Whatever shape gets it
   working is fine at this stage: globals, magic numbers, `delay()`,
   everything in `loop()`. Prototypes are not held to the rules below.
2. **Prove it.** The prototype runs on the bench until it does the thing
   reliably. The evidence gets recorded against its row in
   `docs/vv_table.md`.
3. **Hand over.** The working prototype is handed to Claude for integration
   into the layered structure here: split across the driver header and its
   `.cpp`, app concepts lifted out into the app layer, measured constants
   moved into `board_config.h`, documentation blocks added.
4. **Re-verify.** Integration can break a working module. The bench test is
   repeated after integration; passing before does not count as passing
   after.

When handing a prototype over, include: which pins it actually used, which
library and version if any, and which numbers in it were measured versus
guessed. The last one matters most -- a measured constant moves into
`board_config.h` with its provenance, a guessed one stays a `// TODO:`.

Claude does not write module logic ahead of this. A stub waits for a proven
prototype; it is not a placeholder for Claude to fill in on its own.

## Approval boundaries

These are not suggestions.

1. Every function body is a stub until Hongyi asks for it to be filled.
   One `// TODO:` line stating what the function must do, plus whatever
   return is needed to compile. Nothing else.
2. Do not write, unasked: PID update math, sensor fusion blending, stepper
   sequencing tables, debounce logic, protocol decoding, moving-average
   internals, fault thresholds, or any numeric tuning constant. Leave a
   `// TODO:` naming what has to be decided and which measurement it
   depends on.
3. Pin numbers and tuning constants are placeholders until filled from the
   bench. Do not invent plausible-looking values.
4. Headers declare, `.cpp` files define. No variable definitions in
   headers: `extern` in the header, defined once in the `.cpp`.
5. Anything used by only one `.cpp` is `static` and does not appear in any
   header.
6. Every `.cpp` includes `<Arduino.h>` first, except everything under
   `logic/`, which must stay host-compilable.
7. `#pragma once` in every header.
8. A documentation block on every public function: purpose, parameters,
   return, units, and which task on which core calls it.
9. If something looks missing, say so. Do not add it.

Hongyi does the wiring, the verify, the upload, and grants PASS.

## Evidence policy

A clean compile never makes a hardware test PASS.

Neither does an upload that completed without an error, nor a serial line
that printed the value someone hoped to see. Those show the toolchain
worked, not that the system does what it claims.

Every row in `docs/vv_table.md` starts at NOT TESTED and moves only when
the artifact named in its evidence column exists and has been reviewed.
Rows are never marked PASS in advance of the test, never in bulk, and never
by anyone other than Hongyi.

If a test was not run, its status is NOT TESTED. Saying so is always
correct. Guessing is not.
