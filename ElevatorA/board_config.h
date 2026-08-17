/*
 * ============================================================================
 * FILE: board_config.h
 *
 * PURPOSE:
 *    Board A compile-time configuration: GPIO assignments, task rates, task
 *    priorities, core affinities, and stack sizes. This file contains no
 *    behavior and no tuning constants. Every pin number is a placeholder to
 *    be filled from the bench wiring.
 *
 * AUTHOR:
 *    Hongyi Mei / Kevin Bi
 *
 * DATE CREATED:
 *    08/14/2026
 *
 * LAST MODIFIED:
 *    08/14/2026
 *
 * DEPENDENCIES:
 *    None. Macro definitions only.
 *
 * NOTES:
 *    - Core split is deliberate. The Arduino-ESP32 WiFi task is pinned to
 *      Core 0 at priority 23. The 100 Hz control loop therefore runs on
 *      Core 1 so it is not preempted by the radio stack.
 *    - Rates and priorities below are design decisions, not tuning values.
 *      Control gains, thresholds, and filter lengths do NOT belong here.
 *
 * ============================================================================
 */

#pragma once

/* ========================================================================== */
/*                     SECTION: GPIO ASSIGNMENTS (BOARD A)                    */
/*   Assigned for ESP32-S3-DevKitC-1 (N16R8). Reserved / unavailable:          */
/*     GPIO26-32  SPI flash          GPIO33-37  octal PSRAM (R8 part)         */
/*     GPIO0,3,45,46  strapping      GPIO19,20  native USB                    */
/*     GPIO43,44  UART0 console      GPIO38     on-board RGB LED              */
/*   Verify every line against the physical wiring before first power-on.      */
/* ========================================================================== */

/* --- Nema17 stepper via TMC2209  --- */
/* --- This header is authoritative. The bench prototype used a different --- */
/* --- set; the wiring was moved to match these values, not the reverse.  --- */
#define PIN_STEPPER_EN         4   // TMC EN, output only, active LOW
#define PIN_STEPPER_STEP       5   // TMC STEP, output only
#define PIN_STEPPER_DIR        6   // TMC DIR, output only
#define PIN_STEPPER_TX         17   // U1TXD -> driver PDN_UART, via 1k
#define PIN_STEPPER_RX         18   // U1RXD <- driver PDN_UART

/* --- TMC2209 driver settings, bench-verified 08/16/2026 --------------- */
#define TMC_R_SENSE            0.11f   // set by the driver board, not a choice
#define TMC_UART_ADDR          0b00    // MS1=GND, MS2=GND
#define TMC_UART_BAUD          115200
#define TMC_MICROSTEPS         16
#define TMC_RMS_CURRENT_MA     1000    // ~70-85% of the motor rating
#define TMC_VERSION_EXPECTED   0x21    // read back to prove UART is alive

#define MOTOR_FULL_STEPS_REV   200
#define STEPS_PER_REV          ((float)MOTOR_FULL_STEPS_REV * TMC_MICROSTEPS)
#define GEAR_RATIO             1.0f    // TODO: change if a reduction is added

/* --- Step pulse generator ---------------------------------------------- */
/* A hardware timer ISR runs a phase accumulator and emits one STEP pulse
 * per overflow. The ISR rate sets the resolution of the achievable step
 * frequency, not the step frequency itself. Bench-verified. */
#define STEP_TIMER_HZ          40000
#define STEP_MAX_SPS           8000.0f   // fastest rate before missed steps
#define STEP_MAX_ACCEL_SPS2    40000.0f  // rate-of-change ceiling

/* --- HC-SR04 ultrasonic --- */
#define PIN_HCSR04_TRIG        15   // output only
#define PIN_HCSR04_ECHO        16   // 5V echo: MUST be divided to 3V3

/* Trigger pulse width and the quiet time before it, microseconds. Both are
 * datasheet minima for the part, not tuning values. */
#define HCSR04_TRIG_US         10
#define HCSR04_SETTLE_US       2

/* How long to wait for an echo before giving up, microseconds. This bounds
 * how long ultrasonicTask blocks, so it is a scheduling decision as much as
 * a sensing one: it must stay well inside PERIOD_MS_ULTRASONIC. */
#define HCSR04_TIMEOUT_US      30000

/* Echo time to distance:  distance_mm = (echo_us - intercept) / slope
 *
 * DERIVED, NOT FITTED. Sound travels 343 m/s at 20 C, the echo covers the
 * distance twice, so one millimetre of range costs
 *     2 / 343000 mm/us  =  5.83 us
 * The intercept is left at zero: the module's fixed internal delay is a few
 * microseconds, well under one millimetre of error.
 *
 * This is accepted UNFITTED on purpose. The rangefinder is not in the
 * control loop -- the encoder is. This sensor exists to cross-check what
 * the encoder claims, and a few percent of scale error cannot change the
 * verdict of that comparison. Fit it properly only if the ultrasonic ever
 * becomes the primary feedback, which is what cut line 1b would do. */
#define HCSR04_US_PER_MM       5.83f
#define HCSR04_US_INTERCEPT    0.0f

/* Physically possible range for this part, millimetres. Below the blind
 * zone the module hears its own transmit burst; above the range limit it
 * reports noise. Datasheet figures, not tuning. */
#define HCSR04_MIN_MM          20
#define HCSR04_MAX_MM          4000

/* Consecutive missed echoes before the application calls it a fault. One
 * miss is ordinary -- a bad angle, a soft target. A run of them means the
 * sensor is not seeing the car at all. */
#define HCSR04_FAULT_AFTER_MISSES  5

/* --- Shared I2C bus. Two devices hang off it: the LCD and the encoder. --- */
/* --- Because two different tasks reach this bus, access must be        --- */
/* --- serialised with a mutex. See I2C_MUTEX_TIMEOUT_MS below.          --- */
#define PIN_I2C_SDA            8   // Wire default SDA on S3
#define PIN_I2C_SCL            9   // Wire default SCL on S3
#define I2C_BUS_HZ             400000UL

/* --- 1602 LCD over I2C --- */
#define LCD_I2C_ADDR           0x27
#define LCD_COLS               16
#define LCD_ROWS               2

/* --- AS5600 magnetic rotary encoder over I2C --- */
/* --- Measures motor shaft angle. Address is fixed at 0x36 by the part; --- */
/* --- it cannot be changed, so only one may sit on a bus.               --- */
#define AS5600_I2C_ADDR        0x36
#define AS5600_COUNTS_PER_REV  4096      // 12-bit absolute output

/* How long a task will wait for the I2C mutex before giving up and
 * reporting a bus fault. Must be shorter than the period of the fastest
 * task that uses the bus, or a missed acquisition turns into a missed
 * deadline instead of a reported fault. */
#define I2C_MUTEX_TIMEOUT_MS   0   // TODO: fill after measuring worst-case
                                   //       transaction time on the bench

/* --- RC522 RFID over SPI --- */
#define PIN_SPI_SCK            12   // FSPI CLK
#define PIN_SPI_MOSI           11   // FSPI MOSI
#define PIN_SPI_MISO           13   // FSPI MISO
#define PIN_RC522_CS           10   // FSPI CS0
#define PIN_RC522_RST          14

/* --- Car-panel floor selection buttons (inside the car).            --- */
/* --- No hall-call buttons and no IR receiver in this architecture.   --- */
/* --- Wire each to GND and use INPUT_PULLUP; no button LEDs, the LCD  --- */
/* --- is the only request feedback.                                  --- */
#define PIN_BTN_CAR_0          21   // INPUT_PULLUP
#define PIN_BTN_CAR_1          47   // INPUT_PULLUP
#define PIN_BTN_CAR_2          48   // INPUT_PULLUP

/* Consecutive 200 Hz samples a button must read pressed before the press is
 * accepted. At PERIOD_MS_INPUT this is the debounce interval in samples.
 * TODO: set from the measured bounce duration of the actual switches. */
#define BTN_DEBOUNCE_SAMPLES   4

/* ========================================================================== */
/*                        SECTION: CORE ASSIGNMENTS                           */
/* ========================================================================== */

#define CORE_CONTROL           1   // controlTask
#define CORE_ULTRASONIC        1   // ultrasonicTask
#define CORE_LOAD              1   // loadTask
#define CORE_LINK              0   // linkTask
#define CORE_INPUT             0   // inputTask
#define CORE_RFID              0   // rfidTask
#define CORE_LCD               0   // lcdTask
#define CORE_LOG               0   // logTask

/* ========================================================================== */
/*                        SECTION: TASK PRIORITIES                            */
/*   Higher number = higher FreeRTOS priority.                                */
/* ========================================================================== */

#define PRIO_CONTROL           6
#define PRIO_ULTRASONIC        5
#define PRIO_LINK              5
#define PRIO_INPUT             4
#define PRIO_RFID              3
#define PRIO_LCD               2
#define PRIO_LOG               1
#define PRIO_LOAD              1

/* ========================================================================== */
/*                          SECTION: TASK RATES                               */
/*   Period in milliseconds, derived from the design rate in the comment.     */
/* ========================================================================== */

#define PERIOD_MS_CONTROL      10   // 100 Hz
#define PERIOD_MS_ULTRASONIC   60   // ~16 Hz
#define PERIOD_MS_LINK         20   // 50 Hz
#define PERIOD_MS_INPUT        5    // 200 Hz
#define PERIOD_MS_RFID         100  // ~10 Hz
#define PERIOD_MS_LCD          200  // 5 Hz
#define PERIOD_MS_LOG          100  // 10 Hz
/* loadTask is free-running background work; it has no period. */

/* ========================================================================== */
/*                        SECTION: TASK STACK SIZES                           */
/*   Words, not bytes. TODO: size from uxTaskGetStackHighWaterMark() on the    */
/*   bench; do not guess.                                                     */
/* ========================================================================== */

#define STACK_CONTROL          0   // TODO: fill from high-water-mark measurement
#define STACK_ULTRASONIC       0   // TODO: fill from high-water-mark measurement
#define STACK_LOAD             0   // TODO: fill from high-water-mark measurement
#define STACK_LINK             0   // TODO: fill from high-water-mark measurement
#define STACK_INPUT            0   // TODO: fill from high-water-mark measurement
#define STACK_RFID             0   // TODO: fill from high-water-mark measurement
#define STACK_LCD              0   // TODO: fill from high-water-mark measurement
#define STACK_LOG              0   // TODO: fill from high-water-mark measurement

/* ========================================================================== */
/*                          SECTION: QUEUE DEPTHS                             */
/* ========================================================================== */

#define QDEPTH_INPUT_EVENT     16   // TODO: shrink to measured peak x2
#define QDEPTH_UI_MSG          1    // mailbox, use xQueueOverwrite
#define QDEPTH_LOG_MSG         32   // TODO: shrink to measured peak x2

/* Frames buffered between the ESP-NOW receive callback and linkTask.
 * Board B sends one heartbeat per period plus a short burst on STOP, and
 * linkTask drains at 50 Hz, so this only needs to cover a burst. */
#define QDEPTH_LINK_RX         8    // TODO: shrink to measured peak x2

/* Both boards must agree. Unassociated stations default to 1; setting it
 * explicitly removes "sends report success but nothing arrives" from the
 * list of things that can go wrong. Verified with mac_print. */
#define LINK_WIFI_CHANNEL      1

/* How long Board A tolerates silence before stopping the car. This must
 * span several missed heartbeats so ordinary packet loss does not stop the
 * elevator, while staying far shorter than the time a stopped car matters.
 * TODO: derive from Board B's heartbeat period and the measured loss rate;
 *       VV-13 is the test that pins it down. */
#define LINK_TIMEOUT_MS        0    // TODO: fill from bench

/* ========================================================================== */
/*                        SECTION: CONTROL LOOP                               */
/* ========================================================================== */

/* Position error below which the loop stops commanding motion. Prevents the
 * car hunting around the setpoint against stepper detent torque.
 * Bench-verified at 1 kHz; re-check after the move to 100 Hz. */
#define CTRL_DEADBAND_DEG      0.15f

/* Error that, while the loop is already commanding near-maximum rate, means
 * the motor is not following: missed steps or a jam. Bench-verified. */
#define CTRL_STALL_DEG         15.0f

/* How long that condition must persist before the fault latches, in
 * controlTask iterations. TODO: re-derive for 100 Hz. The prototype used
 * 500 iterations at 1 kHz, i.e. 0.5 s; the same wall-clock time is 50
 * iterations here, but confirm 0.5 s is still the right window once the
 * car has mass on it. */
#define CTRL_STALL_TICKS       0   // TODO: fill from bench at 100 Hz

/* Integrator clamp, in degree-seconds. Bench-verified. */
#define CTRL_INTEGRAL_LIMIT    200.0f

/* PID gains.
 * The prototype's gains (12.0 / 3.0 / 0.35) were tuned with the loop
 * running at 1 kHz. controlTask runs at 100 Hz, so dt is ten times larger
 * and those values do NOT carry over: the same Kp overshoots more, and the
 * derivative term has a completely different character.
 *
 * Retune in this order, one at a time, using the live 'p' command:
 *   1. Ki = 0, Kd = 0. Reduce Kp from 12 until it reaches the target
 *      without oscillating. Steady-state error is fine at this stage.
 *   2. Raise Kd from a small value until overshoot stops shrinking.
 *   3. Raise Ki only enough to remove the steady-state error. */
#define CTRL_KP                0.0f   // TODO: retune at 100 Hz
#define CTRL_KI                0.0f   // TODO: retune at 100 Hz
#define CTRL_KD                0.0f   // TODO: retune at 100 Hz

/* ========================================================================== */
/*                        SECTION: PLATFORM CONSTANTS                         */
/* ========================================================================== */

#define SERIAL_BAUD            115200
#define NUM_FLOORS             3
