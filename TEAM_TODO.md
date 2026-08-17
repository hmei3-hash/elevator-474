# Kevin — where things stand and what to pick up

Written 08/16. Deadline is Friday 08/21.

Read this first, then take from the queue below. Ten minutes of orientation
saves an evening of duplicated work.

---

## Where the project actually is

**Done and proven on the bench**

- Fall detection *data* — 246 usable seconds, 10 real drops, labelled.
  Thresholds are already derived and separate perfectly: normal handling
  never went below 0.66 g and never spent a single millisecond under 0.5 g;
  real drops bottomed out at 0.08–0.22 g for 65–255 ms and landed at
  18–26 g. See `fall_analysis.png` and `analyze_falls.py`.
- MPU6050, RC522, LCD, HC-SR04, AS5600 all answer on the bench. Their
  drivers are written and integrated into the layered structure.
- Closed-loop stepper control worked at 1 kHz on a prototype and has been
  split into `stepper.cpp` / `as5600.cpp` / `logic/pid.c` / `control.cpp`.
- ESP-NOW protocol frozen; both copies of `link_protocol.h` are identical.
  Board A's receive path is written.

**Not done**

- **The elevator itself. Nothing is built.** This is the critical path and
  everything downstream waits on it.
- The AS5600 magnet needs remounting — it was pulled inside the motor,
  which explains both the motor vibrating and the encoder reading weak.
- The TMC2209's UART channel does not answer. It is optional (microsteps
  via MS1/MS2, current via the VREF pot), so it is not worth chasing.
- No V&V row has any evidence. All 16 are `NOT TESTED`.

**Read these two files before touching anything**

- `CLAUDE.md` — the working agreement: layering rule, core/priority split,
  approval boundaries, evidence policy.
- `README.md`, section "What we got wrong" — 13 entries, several of which
  will bite again if you don't know about them.

---

## Do NOT write code in these files

Claude is writing them right now, and two people editing the same file this
week is a merge conflict nobody has time for:

```
ElevatorA/ElevatorA.ino      ElevatorB/ElevatorB.ino
ElevatorB/fall_detect.cpp    logic/filter.c
logic/pid.c                  logic/test/test_main.c
ElevatorA/app_types.h
```

Everything else in the repo is settled or waiting on measurements.

If you want a file written, ask rather than writing it — the layering rules
in `CLAUDE.md` are strict and easier to follow if one hand applies them.

---

## Your queue, in order

### 1. The mechanical build — with Hongyi, highest priority

Nothing about the elevator exists yet: shaft, car, guide, counterweight,
motor mount, encoder mount, ultrasonic mount.

Cardboard, hot glue, string. It has to be **repeatable**, not pretty — the
same command must move the car the same distance every time, because the
whole control loop is built on that. A counterweight roughly matching the
car cuts the torque requirement by about an order of magnitude and stops
the car dropping when power is cut.

**Fishing line arrives Monday.** Dental floss or sewing thread works in the
meantime; swapping the line later is ten minutes and one recalibration.
Do not use anything elastic — stretch decouples the encoder from the car
and the control loop is built on the assumption that it doesn't.

**Done when:** the car moves up and down repeatably under commanded steps,
and both the ultrasonic and encoder readings change as it moves.

### 2. Bench verification and the numbers table

Everything below is measured, not calculated, and every one of them blocks
something downstream. Flash `bringup_a/bringup_a.ino` — it runs the checks
and has interactive commands for the rest.

| Measurement | How | Blocks |
|---|---|---|
| Steps : encoder counts | `bringup_a`, press `r` | PID retune |
| Encoder counts : car mm | Move car a measured distance | Floor mapping |
| Three floor positions | Tape measure, once the shaft exists | `control.cpp` |
| Max step rate before missed steps | Raise speed until it stalls | `STEP_MAX_SPS` |
| Ultrasonic error and jitter | 3 tape-measured distances; 100 reads at one | Filter window |
| Button bounce duration | Scope or a fast sampling loop | Debounce interval |
| ESP-NOW packet loss over 5 min | Both boards on USB, no wiring needed | Heartbeat timeout |
| MPU6050 `WHO_AM_I` value | It printed at `imu_logger` startup | `mpu6050_is_alive()` |
| Task stack high-water marks | `stack_probe/`, after real task bodies exist | `STACK_*` |
| Queue peak occupancy | Same sketch | `QDEPTH_*` |

Write each into `docs/pinmap.md` as you get it. Those values go straight
into `board_config.h`, which is currently 27 placeholders waiting on this
table.

### 3. V&V evidence — start now, do not save it for Thursday

`docs/vv_table.md` has 16 rows. Each names the artifact that lets it move
off `NOT TESTED`: a serial log, a photo, a video, a table. **A clean
compile is never evidence, and neither is an upload that completed.**

Several rows can be closed today, before the elevator exists:

- **VV-01/02/03** (host unit tests) — once `filter.c` and the test cases
  land, run `gcc -Wall -Wextra -I.. -o test_logic test_main.c ../pid.c
  ../filter.c -lm && ./test_logic` and capture the transcript.
- **VV-06** (protocol copies identical) — `diff
  ElevatorA/link_protocol.h ElevatorB/link_protocol.h`, capture it.
- **VV-09** (card reader) — `bringup_a`, press `c`, present an enrolled and
  an unenrolled card, capture the log.
- **VV-11** (buttons) — press each 20 times, capture.
- **VV-12** (heartbeat liveness) — needs both boards on USB only.

Only Hongyi grants PASS. Your job is to produce the artifact.

### 4. The AI report — you own the draft

Four required sections: AI collaboration, V&V design tables, a testing log,
and what was fixed (issue, cause, fix, evidence). Submitted as PDF.

Two of the four are already most of the way written:

- **What was fixed** → `README.md`, "What we got wrong", is already in
  issue/cause/fix/evidence form. Thirteen entries.
- **V&V design** → `docs/vv_table.md` is the table.
- **AI collaboration** → `CLAUDE.md` documents the approval boundaries,
  the prototype-then-integrate workflow, and the evidence policy.

Start the skeleton now and fill results as they arrive. Writing this on
Friday morning is how teams lose marks on work they already did.

### 5. Board B integration, once the code lands

`fall_detect.cpp` will arrive with the thresholds already filled from the
data. Your job is to verify it: replay the recorded drops, confirm all ten
fire and none of the 16 normal-handling segments do.

Then the part that is **not** in the dataset and is the most likely source
of a false trigger at the demo: **the elevator's own acceleration**. Once
the car moves, ride the sensor on it for twenty trips and confirm zero
false positives. If it triggers, that is a real finding, not a nuisance.

---

## The demo, so you can aim at it

Two separate demonstrations, and the second matters more than it looks:

1. Car travelling → drop the "passenger" rig → car stops, LCD shows
   `FALL - STOPPED`, log shows `FAULT_FALL_DETECTED`.
2. Car travelling → **pull Board B's power** → car stops, log shows
   `FAULT_LINK_TIMEOUT`, a *different* code.

The second proves the safety logic is safe-on-silence rather than
safe-on-message: a dead detector stops the car, it does not disable the
safety. Expect to be asked "what if the fall detector fails?" — answering
by pulling the plug on the spot beats any explanation.

Also needed: 20 normal handling motions with zero false triggers, and 20
car trips with zero false triggers. Those negatives are a whole grading
criterion, and they are cheap.

---

## Four rubric gaps, so nobody assumes they're covered

1. **Three ESP32 timers including a hardware ISR** — half done. The step
   pulse generator is a real hardware timer ISR. Still missing: the 100 Hz
   control timer and an `esp_timer` for the ultrasonic timeout.
2. **A semaphore or mutex with a stated reason** — not done. Two are
   planned: a binary semaphore released by the control timer, and an I2C
   mutex because the LCD and encoder share a bus across two cores.
3. **"Serial communication" between the boards** — the rubric says serial;
   ESP-NOW is not. **Ask a TA Monday.** Fallback is UART over two wires,
   with the frame format unchanged. Half a day if planned, a disaster if
   discovered at the demo.
4. **Three physical parameters** — exceeded: distance, shaft angle,
   acceleration, angular rate.

---

## Cut lines, agreed in advance

- Car not moving by Monday morning → drop the moving car. Motor becomes a
  rotating floor indicator, ultrasonic measures a hand-moved platform.
  Closed-loop control still holds; every criterion survives.
- PID not stable by Tuesday night → ship proportional-only with a slow
  approach and a tolerance you can actually meet, and say so in the report.
- Anything unfinished Wednesday night → RFID goes first, then the third
  floor.
- **Never cut:** the fault path, the V&V evidence, the AI report. Whole
  criteria, cheap to do, and exactly what the course is assessing.
