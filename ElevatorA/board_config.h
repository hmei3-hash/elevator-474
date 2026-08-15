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
#define PIN_STEPPER_EN         4   // TMC EN, output only
#define PIN_STEPPER_STEP       5   // TMC STEP, output only
#define PIN_STEPPER_DIR        6   // TMC DIR, output only
#define PIN_STEPPER_TX         17   // U1TXD -> driver RX
#define PIN_STEPPER_RX         18   // U1RXD <- driver TX

/* --- HC-SR04 ultrasonic --- */
#define PIN_HCSR04_TRIG        15   // output only
#define PIN_HCSR04_ECHO        16   // 5V echo: divide to 3V3

/* --- 1602 LCD over I2C --- */
#define PIN_I2C_SDA            8   // Wire default SDA on S3
#define PIN_I2C_SCL            9   // Wire default SCL on S3
#define LCD_I2C_ADDR           0x27
#define LCD_COLS               16
#define LCD_ROWS               2

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

/* ========================================================================== */
/*                        SECTION: PLATFORM CONSTANTS                         */
/* ========================================================================== */

#define SERIAL_BAUD            115200
#define NUM_FLOORS             3
