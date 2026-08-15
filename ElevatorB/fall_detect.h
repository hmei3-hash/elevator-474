/*
 * ============================================================================
 * FILE: fall_detect.h
 *
 * PURPOSE:
 *    Fall detection policy. Consumes inertial samples and decides whether a
 *    fall has occurred. This is the only place on Board B where sensor data
 *    is interpreted.
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
 *    - Arduino.h
 *    - mpu6050.h: imu_sample_t
 *
 * NOTES:
 *    App layer. Must not touch I2C directly.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>
#include "mpu6050.h"

/** Detector verdict for one sample. */
typedef enum {
    FALL_STATE_NORMAL = 0,
    FALL_STATE_SUSPECT,      // candidate event, not yet confirmed
    FALL_STATE_CONFIRMED     // fall confirmed, STOP must be sent
} fall_state_t;

/**
 * Reset the detector to its normal state.
 *
 * @return void
 * Called from: setup(), before tasks start.
 */
void fall_detect_init(void);

/**
 * Feed one sample to the detector and read back the resulting state.
 *
 * @param s  sample to process, must not be NULL
 * @return the detector state after processing this sample
 * Called from: detectTask (Core 1).
 */
fall_state_t fall_detect_update(const imu_sample_t *s);

/**
 * Clear a confirmed detection so the detector can arm again.
 *
 * @return void
 * Called from: detectTask (Core 1), after the STOP frames have been sent.
 */
void fall_detect_clear(void);
