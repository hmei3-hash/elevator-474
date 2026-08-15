/*
 * ============================================================================
 * FILE: filter.c
 *
 * PURPOSE:
 *    Moving average and residual tracker implementation.
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
 *    - filter.h
 *
 * NOTES:
 *    Host-testable. This file must have ZERO platform and ZERO framework
 *    dependencies so it compiles and unit-tests with plain gcc.
 *    Do not add any platform or framework header here under any
 *    circumstances.
 *
 * ============================================================================
 */

#include "filter.h"

int moving_avg_init(moving_avg_t *f, uint8_t window) {
    /* TODO: validate window against FILTER_MAX_WINDOW, store it, and clear
     * the buffer, indices, and running sum.
     * The window LENGTH is a caller decision: it trades noise rejection
     * against added lag, and cannot be chosen until the ultrasonic noise
     * floor and the car's travel speed are measured. */
    (void)f;
    (void)window;
    return 0;
}

float moving_avg_push(moving_avg_t *f, float sample) {
    /* TODO: subtract the sample being evicted, add the new one, advance the
     * head, saturate count at window, and return sum divided by count. */
    (void)f;
    (void)sample;
    return 0.0f;
}

void moving_avg_reset(moving_avg_t *f) {
    /* TODO: zero the buffer, indices, and running sum; keep the window. */
    (void)f;
    return;
}

void residual_init(residual_t *r) {
    /* TODO: zero both fields. */
    (void)r;
    return;
}

float residual_push(residual_t *r, float raw, float filtered) {
    /* TODO: compute raw minus filtered, store it, and update the running
     * peak magnitude.
     * No threshold is applied here. Deciding what residual counts as a
     * fault belongs to the application layer and depends on bench data. */
    (void)r;
    (void)raw;
    (void)filtered;
    return 0.0f;
}
