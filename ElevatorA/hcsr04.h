/*
 * ============================================================================
 * FILE: hcsr04.h
 *
 * PURPOSE:
 *    Driver for an ultrasonic time-of-flight rangefinder with a trigger pin
 *    and a pulse-width echo pin. Reports distance only.
 *
 *    Interrupt driven. Nothing here blocks: a ping is started, the echo
 *    edges are timed in an ISR, and the result is collected on a later
 *    call. A blocking pulseIn() implementation would hold Core 1 for up to
 *    the full timeout on every missing echo, which is exactly when the
 *    control loop most needs the core.
 *
 * AUTHOR:
 *    Hongyi Mei / Kevin Bi
 *
 * DATE CREATED:
 *    08/15/2026
 *
 * LAST MODIFIED:
 *    08/16/2026
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *
 * NOTES:
 *    Driver layer. Must not reference application concepts.
 *
 *    USAGE, once per period:
 *        hcsr04_reading_t r;
 *        if (hcsr04_collect(&r)) { ...use r... }
 *        hcsr04_trigger();
 *
 *    Collect first, then trigger. The result being collected belongs to the
 *    ping started one period ago, so the reading is one period old by
 *    construction -- account for that when it feeds anything time critical.
 *
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/** Result of one ranging attempt. */
typedef struct {
    bool     valid;         // false if no echo arrived within the window
    uint32_t distance_mm;   // measured distance, millimetres; 0 if invalid
    uint32_t echo_us;       // raw echo pulse width, microseconds
} hcsr04_reading_t;

/**
 * Configure the pins, attach the echo interrupt, and create the timeout
 * timer.
 *
 * @return true on success
 * Called from: setup(), before tasks start.
 */
bool hcsr04_init(void);

/**
 * Start one ranging cycle and return immediately. Emits the trigger pulse
 * and arms the timeout. Calling this while a ping is still outstanding
 * abandons the outstanding one.
 *
 * @return true if a ping was started
 * Called from: ultrasonicTask (Core 1, ~16 Hz).
 */
bool hcsr04_trigger(void);

/**
 * Collect the result of the most recently completed ping, if one has
 * completed since the last call. Never blocks.
 *
 * @param out  destination for the reading, must not be NULL
 * @return true if a fresh result was written to out
 * Called from: ultrasonicTask (Core 1, ~16 Hz).
 */
bool hcsr04_collect(hcsr04_reading_t *out);

/**
 * Count of pings that timed out without an echo since init. A rising count
 * means the sensor is not seeing its target; the fault decision belongs to
 * the application layer.
 *
 * @return cumulative timeout count
 * Called from: logTask (Core 0, 10 Hz).
 */
uint32_t hcsr04_get_timeout_count(void);
