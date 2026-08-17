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
/* --- Bench-verified with the imu_logger prototype, 08/15-08/17/2026. --- */
#define PIN_I2C_SDA            8       // bench-verified
#define PIN_I2C_SCL            9       // bench-verified
#define I2C_BUS_HZ             400000UL
/* The burst read is 14 bytes. At 100 kHz that single read costs over a
 * millisecond and starts to distort the 5 ms sample interval, so 400 kHz is
 * a requirement here rather than an optimisation. */

/* AD0 low gives 0x68, AD0 high gives 0x69. The bench part answered on
 * 0x68. mpu6050_init() probes both, so a rewired module needs no recompile. */
#define MPU_I2C_ADDR_LOW       0x68
#define MPU_I2C_ADDR_HIGH      0x69

/* Identity register value THIS module actually returns, read on the bench
 * 08/17/2026. A genuine InvenSense MPU-6050 reports 0x68; ours reports
 * 0x70, the MPU-6500 family value, so the part is a clone or a 6500 die
 * sold under the 6050 name.
 *
 * Harmless for what this driver does: PWR_MGMT_1, ACCEL_CONFIG,
 * GYRO_CONFIG and the burst read from ACCEL_XOUT_H are identical across the
 * two parts, and the 246 s of drop data the thresholds came from was
 * captured with this exact module. What matters is that the value is READ
 * OFF the bench part -- the datasheet's 0x68 would have made
 * mpu6050_is_alive() reject a perfectly good sensor. */
#define MPU_WHO_AM_I_EXPECTED  0x70

#define PIN_MPU_INT            0   // TODO: fill from bench, or leave unused
                                   //       (polled at PERIOD_MS_SAMPLE today,
                                   //        so the INT line is not required)

/* --- Sensor range and scale ---------------------------------------------- */
/* Accelerometer at its widest range on purpose: impact peaks from even a
 * soft landing run well past 4 g, and a clipped peak is unrecoverable.
 * Free-fall detection reads values near zero, where the coarser resolution
 * costs nothing. Confirmed against the recorded drop traces, where
 * composite peaks reached 21-26 g.
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
                                   // the plateau and the impact edge.
#define PERIOD_MS_DETECT       5   // the detector consumes every sample, so
                                   // it runs at the sample rate
#define PERIOD_MS_HEARTBEAT    20  // 50 Hz. Chosen with Board A's timeout:
                                   // the timeout must span several missed
                                   // heartbeats so ordinary packet loss does
                                   // not stop the car, while still being far
                                   // shorter than a person can fall and be
                                   // hurt. TODO: confirm against the measured
                                   // packet-loss rate.

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

#define QDEPTH_SAMPLE          16  // TODO: shrink to measured peak x2

/* ========================================================================== */
/*                        SECTION: FALL DETECTION                             */
/*                                                                            */
/*   Every value below was READ OFF recorded bench data, not chosen. From     */
/*   246 s of labelled capture containing 10 real drops and 16 normal         */
/*   handling motions:                                                        */
/*                                                                            */
/*     feature              normal handling      real drops                   */
/*     lowest |a|           0.66 - 0.89 g        0.08 - 0.22 g                */
/*     time below 0.5 g     0 ms, every time     65 - 255 ms                  */
/*     impact peak          1.0 - 1.34 g         18.4 - 25.8 g                */
/*                                                                            */
/*   The three ranges do not overlap anywhere. Each threshold below sits on   */
/*   the safe side of the worst real drop, not at the midpoint, because a     */
/*   missed fall is worse than a late one.                                    */
/*                                                                            */
/*   NOT YET COVERED BY THE DATA: the elevator's own acceleration. That must  */
/*   be measured once the car moves, and these values re-checked against it.  */
/* ========================================================================== */

/* Free fall: magnitude collapses toward zero. The worst real drop reached
 * 0.22 g, so this leaves 0.28 g of margin; the nearest normal motion was
 * 0.66 g away on the other side. */
#define FALL_FREEFALL_G        0.50f

/* ...and stays there. The shortest real drop held for 65 ms; no normal
 * motion spent any time at all below the threshold. */
#define FALL_FREEFALL_MS       40

/* Impact. The weakest real landing was 18.4 g and the strongest handling
 * transient 1.34 g, so this is an order of magnitude clear of both. */
#define FALL_IMPACT_G          8.00f

/* How long after free fall ends an impact must arrive to count as part of
 * the same event. Longer than any observed flight-to-impact gap; short
 * enough that an unrelated later bump is not attributed to this fall. */
#define FALL_IMPACT_WINDOW_MS  300

/* Once confirmed, the detector stays confirmed for this long before it can
 * arm again, so one drop that bounces reports one fall. */
#define FALL_REARM_MS          2000

/* ========================================================================== */
/*                        SECTION: PLATFORM CONSTANTS                         */
/* ========================================================================== */

#define SERIAL_BAUD            115200

/** Hardware address of Board A. Fill from the mac_print bring-up sketch. */
#define PEER_MAC_BYTES         {0x80, 0xB5, 0x4E, 0xE3, 0x19, 0x58}
/* Board A, the controller: 80:B5:4E:E3:19:58, read with mac_print
 * 08/17/2026. This is the address Board B TRANSMITS TO, so it is the
 * controller's MAC and never this board's.
 *
 * 80:B5:4E:E3:22:50 sat here for a while and is in fact THIS board's own
 * address. Board B would have heartbeat to itself, Board A would have
 * stopped the car on a link timeout, and nothing on the radio side would
 * ever have explained it. The two addresses differ only in the last two
 * bytes, which is exactly why both boards carry a physical label. */

/** Number of times a STOP frame is repeated. ESP-NOW does not retry, so a
 *  one-shot stop can be lost. Repetition trades a few frames for a much
 *  lower miss probability. */
#define STOP_FRAME_REPEATS     5   // TODO: reduce once packet loss is measured
