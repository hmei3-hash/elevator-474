/*
 * ============================================================================
 * FILE: hcsr04.h
 *
 * PURPOSE:
 *    Driver for an ultrasonic time-of-flight rangefinder with a trigger pin
 *    and a pulse-width echo pin. Reports distance only.
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
 *
 * NOTES:
 *    Driver layer. Must not reference application concepts.
 *    The echo pin carries a 5 V signal; a level shifter or divider is
 *    required in hardware. This driver assumes that is already in place.
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
 * Configure the trigger and echo pins.
 *
 * @return true on success
 * Called from: setup(), before tasks start.
 */
bool hcsr04_init(void);

/**
 * Perform one blocking ranging cycle: emit a trigger pulse, time the echo,
 * and convert to distance.
 *
 * @param out  destination for the reading, must not be NULL
 * @return true if a valid echo was captured
 * Called from: ultrasonicTask (Core 1, ~16 Hz). Blocks for up to the echo
 *              timeout, so it must not be called from a faster task.
 */
bool hcsr04_read(hcsr04_reading_t *out);
