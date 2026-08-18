/*
 * ============================================================================
 * FILE: filter.h
 *
 * PURPOSE:
 *    Signal conditioning primitives: a fixed-window moving average and a
 *    residual tracker used to judge how far a measurement has drifted from
 *    its recent trend.
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
 *    - stdint.h: fixed-width integer types
 *
 *    LINKAGE
 *    The declarations below are wrapped in extern "C". This file is C, but
 *    every translation unit on the target that includes it is C++ (Arduino
 *    compiles .cpp and .ino as C++). Without the wrapper the caller emits a
 *    mangled C++ symbol while the definition is a plain C symbol, and the
 *    build fails at link time with "undefined reference" to a function that
 *    plainly exists.
 *
 * NOTES:
 *    Host-testable. This file must have ZERO platform and ZERO framework
 *    dependencies so it compiles and unit-tests with plain gcc.
 *    Do not add any platform or framework header here under any
 *    circumstances.
 *
 * ============================================================================
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/** Largest window this implementation supports, in samples. */
#define FILTER_MAX_WINDOW  32

/**
 * Fixed-window moving average over a circular buffer owned by the caller.
 */
typedef struct {
    float    buf[FILTER_MAX_WINDOW];
    uint8_t  window;      /* active window length, 1..FILTER_MAX_WINDOW */
    uint8_t  head;        /* next write index */
    uint8_t  count;       /* samples seen, saturating at window */
    float    running_sum; /* sum of the active samples */
} moving_avg_t;

/**
 * Tracks the difference between the newest sample and the filtered value,
 * so a caller can notice a measurement that no longer follows its trend.
 */
typedef struct {
    float last_residual;   /* newest sample minus filtered value */
    float peak_residual;   /* largest magnitude residual seen since reset */
} residual_t;

/**
 * Set the window length and clear the buffer.
 *
 * @param f       filter to initialise, must not be NULL
 * @param window  window length in samples, 1..FILTER_MAX_WINDOW
 * @return 0 on success, negative if window is out of range
 * Called from: control_init() on Board A, and from the host test harness.
 */
int moving_avg_init(moving_avg_t *f, uint8_t window);

/**
 * Push one sample and read back the new average.
 *
 * @param f       filter state, must not be NULL
 * @param sample  new sample
 * @return the average over the samples currently held
 * Called from: controlTask (Core 1, 100 Hz), and from the host test harness.
 */
float moving_avg_push(moving_avg_t *f, float sample);

/**
 * Discard all held samples without changing the window length.
 *
 * @param f  filter state, must not be NULL
 * @return void
 * Called from: control_emergency_stop(), and from the host test harness.
 */
void moving_avg_reset(moving_avg_t *f);

/**
 * Clear the residual tracker.
 *
 * @param r  tracker to initialise, must not be NULL
 * @return void
 * Called from: control_init(), and from the host test harness.
 */
void residual_init(residual_t *r);

/**
 * Record the gap between a raw sample and its filtered value.
 *
 * @param r         tracker state, must not be NULL
 * @param raw       unfiltered sample
 * @param filtered  filtered value for the same instant
 * @return the signed residual for this sample
 * Called from: controlTask (Core 1, 100 Hz), and from the host test harness.
 */
float residual_push(residual_t *r, float raw, float filtered);

#ifdef __cplusplus
}   /* extern "C" */
#endif
