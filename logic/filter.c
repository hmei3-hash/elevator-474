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
    if (f == 0) return -1;
    if (window == 0 || window > FILTER_MAX_WINDOW) return -1;

    /* The window LENGTH stays a caller decision: it trades noise rejection
     * against added lag, and the right value depends on the measured
     * ultrasonic noise floor and the car's travel speed. */
    f->window = window;
    moving_avg_reset(f);
    return 0;
}

float moving_avg_push(moving_avg_t *f, float sample) {
    if (f == 0 || f->window == 0) return 0.0f;

    /* Once the window is full, every push evicts exactly one sample. Before
     * that, nothing is evicted and count grows. Keeping a running sum makes
     * this O(1) per sample instead of O(window). */
    if (f->count == f->window) {
        f->running_sum -= f->buf[f->head];
    } else {
        f->count++;
    }

    f->buf[f->head] = sample;
    f->running_sum += sample;

    f->head++;
    if (f->head >= f->window) f->head = 0;

    /* Divide by count, not by window. Dividing by window before the buffer
     * fills would drag the first readings toward zero, which on a position
     * signal reads as the car being somewhere it is not. */
    return f->running_sum / (float)f->count;
}

void moving_avg_reset(moving_avg_t *f) {
    if (f == 0) return;
    for (uint8_t i = 0; i < FILTER_MAX_WINDOW; i++) f->buf[i] = 0.0f;
    f->head        = 0;
    f->count       = 0;
    f->running_sum = 0.0f;
    /* window is deliberately preserved: a reset discards data, not setup. */
}

void residual_init(residual_t *r) {
    if (r == 0) return;
    r->last_residual = 0.0f;
    r->peak_residual = 0.0f;
}

float residual_push(residual_t *r, float raw, float filtered) {
    if (r == 0) return 0.0f;

    float d = raw - filtered;
    r->last_residual = d;

    /* Track magnitude, not signed value: a large negative excursion is just
     * as much a sign of trouble as a positive one, and keeping the signed
     * maximum would hide it. */
    float mag = (d < 0.0f) ? -d : d;
    if (mag > r->peak_residual) r->peak_residual = mag;

    /* No threshold is applied here. Deciding what residual counts as a
     * fault is application policy and depends on bench data. */
    return d;
}
