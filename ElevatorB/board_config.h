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

/* --- MPU6050 motion sensor over I2C --- */
/* --- Verified on the bench with the imu_logger prototype, 08/16/2026. --- */
#define PIN_I2C_SDA            8       // bench-verified
#define PIN_I2C_SCL            9       // bench-verified
#define I2C_BUS_HZ             400000UL
/* The burst read is 14 bytes. At 100 kHz that single read costs over a
 * millisecond and starts to distort the sample interval, so 400 kHz is a
 * requirement here rather than an optimisation. */

/* AD0 low gives 0x68, AD0 high gives 0x69. mpu6050_init() probes both and
 * uses whichever answers, so a rewired module does not need a recompile. */
#define MPU_I2C_ADDR_LOW       0x68
#define MPU_I2C_ADDR_HIGH      0x69

#define PIN_MPU_INT            0   // TODO: fill from bench, or leave unused
                                   //       (polled at PERIOD_MS_SAMPLE today,
                                   //        so the INT line is not required)

/* --- Sensor range and scale ---------------------------------------------- */
/* Accelerometer at the widest range on purpose: impact peaks from even a
 * soft landing run well past 4 g, and a clipped peak is unrecoverable.
 * Free-fall detection reads values near zero, where the coarser resolution
 * costs nothing. Verified against recorded drop traces.
 *    0 = +/-2g   1 = +/-4g   2 = +/-8g   3 = +/-16g                        */
#define MPU_ACCEL_FS_SEL       3
#define MPU_ACCEL_LSB_PER_G    2048.0f   // must match MPU_ACCEL_FS_SEL

/*    0 = +/-250  1 = +/-500  2 = +/-1000  3 = +/-2000  deg/s              */
#define MPU_GYRO_FS_SEL        3
#define MPU_GYRO_LSB_PER_DPS   16.4f     // must match MPU_GYRO_FS_SEL

/* Digital low-pass filter, 0 = widest bandwidth and least smoothing.
 * Deliberately wide: filtering in the sensor would round off the impact
 * edge before the detector ever sees it. Any smoothing belongs in
 * fall_detect.cpp, where it can be reasoned about and undone. */
#define MPU_DLPF_CFG           0

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

#define PERIOD_MS_SAMPLE       5   // 200 Hz, bench-verified.
                                   // A ~30 cm drop has a free-fall phase of
                                   // roughly 250 ms; at 200 Hz that is ~50
                                   // samples, enough to resolve the onset,
                                   // the plateau, and the impact edge.
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
#define PEER_MAC_BYTES         {0x80, 0xB5, 0x4E, 0xE3, 0x22, 0x50}  // Board A, 80:B5:4E:E3:22:50

/** Number of times a STOP frame is repeated. ESP-NOW does not retry, so a
 *  one-shot stop can be lost. Repetition trades a few frames for a much
 *  lower miss probability. */
#define STOP_FRAME_REPEATS     0   // TODO: fill after measuring packet loss
