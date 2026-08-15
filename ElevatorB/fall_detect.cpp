/*
 * ============================================================================
 * FILE: fall_detect.cpp
 *
 * PURPOSE:
 *    Fall detection policy implementation.
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
 *    - fall_detect.h
 *    - mpu6050.h
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "fall_detect.h"

/* Current detector state. Only this file may change it. */
static fall_state_t s_state;

void fall_detect_init(void) {
    // TODO: set s_state to FALL_STATE_NORMAL and clear any history buffers
    return;
}

fall_state_t fall_detect_update(const imu_sample_t *s) {
    // TODO: the detection rule is undecided. Whatever it becomes, both its
    //       thresholds and its confirmation window must come from recorded
    //       bench data: drop the sensor rig repeatedly, log raw samples, and
    //       pick values that separate real drops from handling noise.
    // TODO: decide whether the free-fall phase, the impact peak, or the
    //       post-impact stillness is the trigger. Do not guess; look at the
    //       recorded traces first.
    (void)s;
    return FALL_STATE_NORMAL;
}

void fall_detect_clear(void) {
    // TODO: return s_state to FALL_STATE_NORMAL
    return;
}
