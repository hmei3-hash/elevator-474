/*
 * ============================================================================
 * FILE: board_config.h
 *
 * PURPOSE:
 *    Board B compile-time configuration: GPIO assignments, task rates,
 *    priorities, and core affinities. No behavior, no tuning constants.
 *
 * AUTHOR:
 *    Hongyi Mei / Kevin Bi
 *
 * DATE CREATED:
 *    08/15/2026
 *
 * LAST MODIFIED:
 *    08/15/2026
 *
 * DEPENDENCIES:
 *    None. Macro definitions only.
 *
 * NOTES:
 *    Board B owns the motion sensor and nothing else. It has no display, no
 *    motor, and no buttons.
 *    Pins are placeholders until the sensor is wired.
 * ============================================================================
 */

#pragma once

/* ========================================================================== */
/*                     SECTION: GPIO ASSIGNMENTS (BOARD B)                    */
/*   Reserved / unavailable on ESP32-S3-DevKitC-1 (N16R8):                     */
/*     GPIO26-32  SPI flash          GPIO33-37  octal PSRAM                    */
/*     GPIO0,3,45,46  strapping      GPIO19,20  native USB                     */
/*     GPIO43,44  UART0 console      GPIO38     on-board RGB LED               */
/* ========================================================================== */

/* --- Motion sensor over I2C --- */
#define PIN_I2C_SDA            0   // TODO: fill from bench
#define PIN_I2C_SCL            0   // TODO: fill from bench
#define MPU_I2C_ADDR           0   // TODO: fill from bench (scan the bus)
#define PIN_MPU_INT            0   // TODO: fill from bench, or leave unused

/* ========================================================================== */
/*                        SECTION: CORE ASSIGNMENTS                           */
/*   The WiFi task is pinned to Core 0 at priority 23. Sampling stays on      */
/*   Core 1 so the radio cannot stretch the sample interval.                  */
/* ========================================================================== */

#define CORE_SAMPLE            1   // sampleTask
#define CORE_DETECT            1   // detectTask
#define CORE_TX                0   // txTask

/* ========================================================================== */
/*                        SECTION: TASK PRIORITIES                            */
/* ========================================================================== */

#define PRIO_SAMPLE            6
#define PRIO_DETECT            5
#define PRIO_TX                4

/* ========================================================================== */
/*                          SECTION: TASK RATES                               */
/* ========================================================================== */

#define PERIOD_MS_SAMPLE       0   // TODO: set from the sensor output rate
#define PERIOD_MS_DETECT       0   // TODO: set from the detection window
#define PERIOD_MS_HEARTBEAT    0   // TODO: set with Board A's timeout together

/* ========================================================================== */
/*                        SECTION: TASK STACK SIZES                           */
/*   Words, not bytes. Measure with uxTaskGetStackHighWaterMark().            */
/* ========================================================================== */

#define STACK_SAMPLE           0   // TODO: fill from high-water-mark measurement
#define STACK_DETECT           0   // TODO: fill from high-water-mark measurement
#define STACK_TX               0   // TODO: fill from high-water-mark measurement

/* ========================================================================== */
/*                          SECTION: QUEUE DEPTHS                             */
/* ========================================================================== */

#define QDEPTH_SAMPLE          0   // TODO: fill after measuring burst rate

/* ========================================================================== */
/*                        SECTION: PLATFORM CONSTANTS                         */
/* ========================================================================== */

#define SERIAL_BAUD            115200

/** Hardware address of Board A. Fill from the mac_print bring-up sketch. */
#define PEER_MAC_BYTES         {0, 0, 0, 0, 0, 0}   // TODO: fill from bench

/** Number of times a STOP frame is repeated. ESP-NOW does not retry, so a
 *  one-shot stop can be lost. Repetition trades a few frames for a much
 *  lower miss probability. */
#define STOP_FRAME_REPEATS     0   // TODO: fill after measuring packet loss
