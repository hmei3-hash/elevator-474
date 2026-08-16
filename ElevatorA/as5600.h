/*
 * ============================================================================
 * FILE: as5600.h
 *
 * PURPOSE:
 *    Driver for a 12-bit absolute magnetic rotary encoder on I2C. Reports
 *    shaft angle and accumulated revolutions, and reports whether the
 *    magnet is positioned correctly for a trustworthy reading. It has no
 *    knowledge of what the shaft is attached to.
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
 *    Driver layer. Must not reference any application concept or policy.
 *    Converting an angle into a position along a track, or into a stop
 *    target, is the application's job.
 *
 *    The part reports a single-turn absolute angle. Multi-turn tracking is
 *    done here by detecting wrap between consecutive reads, which means
 *    as5600_update() must be called often enough that the shaft cannot
 *    move more than half a revolution between calls. Violating that is
 *    silent: the count simply goes the wrong way.
 *
 *    This device shares its bus with the display. The caller is responsible
 *    for holding the I2C mutex across any call below; this driver does not
 *    take it, because a driver that acquires a lock it does not own cannot
 *    be composed with other bus users.
 *
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/** Magnet placement as reported by the device's status register. */
typedef enum {
    AS5600_MAGNET_OK = 0,     // field strength within range
    AS5600_MAGNET_WEAK,       // magnet too far away, reading untrustworthy
    AS5600_MAGNET_STRONG,     // magnet too close, reading untrustworthy
    AS5600_MAGNET_ABSENT      // no field detected at all
} as5600_magnet_t;

/** One angular reading. */
typedef struct {
    uint16_t raw_counts;      // single-turn absolute, 0..AS5600_COUNTS_PER_REV-1
    int32_t  total_counts;    // accumulated across turns, signed
    int32_t  revolutions;     // completed turns, signed
    bool     valid;           // false if the bus read failed
} as5600_reading_t;

/**
 * Confirm the device answers on the bus and latch a starting reference.
 * Does not configure the device; its defaults are used.
 *
 * @return true if the device acknowledged and returned a plausible angle
 * Called from: setup(), before tasks start. Caller holds the I2C mutex.
 */
bool as5600_init(void);

/**
 * Read the current angle, update the multi-turn accumulator, and fill the
 * caller's buffer.
 *
 * @param out  destination for the reading, must not be NULL
 * @return true if the bus read succeeded
 * Called from: controlTask (Core 1, 100 Hz). Caller holds the I2C mutex.
 */
bool as5600_update(as5600_reading_t *out);

/**
 * Set the current shaft angle as the zero reference. Clears the multi-turn
 * accumulator.
 *
 * @return true if a reference angle was captured
 * Called from: control_init() during homing (Core 1). Caller holds the
 *              I2C mutex.
 */
bool as5600_zero(void);

/**
 * Read the magnet placement status.
 *
 * @param out  destination for the status, must not be NULL
 * @return true if the status register was read successfully
 * Called from: controlTask (Core 1, 100 Hz), periodically rather than every
 *              iteration. Caller holds the I2C mutex.
 */
bool as5600_get_magnet_status(as5600_magnet_t *out);
