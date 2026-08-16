/*
 * ============================================================================
 * FILE: as5600.cpp
 *
 * PURPOSE:
 *    Magnetic rotary encoder driver implementation.
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
 *    - Wire.h
 *    - board_config.h: bus pins, device address, counts per revolution
 *    - as5600.h
 *
 * NOTES:
 *    This driver does not take the I2C mutex. The caller holds it across
 *    every entry point here.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"
#include "as5600.h"

/* ========================================================================== */
/*                          SECTION: DEVICE REGISTERS                         */
/* ========================================================================== */

#define AS5600_REG_STATUS        0x0B
#define AS5600_REG_RAW_ANGLE_H   0x0C
#define AS5600_REG_ANGLE_H       0x0E
#define AS5600_REG_AGC           0x1A
#define AS5600_REG_MAGNITUDE_H   0x1B

/* Status register bits. */
#define AS5600_STATUS_MH         (1u << 3)   // magnet too strong
#define AS5600_STATUS_ML         (1u << 4)   // magnet too weak
#define AS5600_STATUS_MD         (1u << 5)   // magnet detected

/* ========================================================================== */
/*                          SECTION: FILE-LOCAL STATE                         */
/* ========================================================================== */

/* Angle from the previous update, used to detect wrap. */
static uint16_t s_prev_counts;

/* Completed turns, signed. */
static int32_t s_revolutions;

/* Zero reference captured by as5600_zero(). */
static uint16_t s_zero_counts;

/* Set once init has succeeded; guards against reading before setup. */
static bool s_ready;

/*
 * ============================================================================
 * FUNCTION: as5600_read_u16
 *
 * PURPOSE:
 *    Read a big-endian 12-bit value from a register pair.
 *
 * PARAMETERS:
 *    reg_high (uint8_t) - address of the high byte
 *    out (uint16_t*) - destination, masked to 12 bits
 *
 * RETURN VALUE:
 *    bool - true if both bytes were read
 *
 * CALLED FROM:
 *    Every public function in this file. Caller holds the I2C mutex.
 * ============================================================================
 */
static bool as5600_read_u16(uint8_t reg_high, uint16_t *out) {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(reg_high);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AS5600_I2C_ADDR, 2) != 2) return false;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    *out = (uint16_t)((((uint16_t)hi << 8) | lo) & 0x0FFF);
    return true;
}

/*
 * ============================================================================
 * FUNCTION: as5600_read_u8
 *
 * PURPOSE:
 *    Read one byte from a register.
 *
 * PARAMETERS:
 *    reg (uint8_t) - register address
 *    out (uint8_t*) - destination
 *
 * RETURN VALUE:
 *    bool - true if a byte was returned
 *
 * CALLED FROM:
 *    as5600_get_magnet_status(). Caller holds the I2C mutex.
 * ============================================================================
 */
static bool as5600_read_u8(uint8_t reg, uint8_t *out) {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)AS5600_I2C_ADDR, 1) != 1) return false;
    *out = Wire.read();
    return true;
}

/*
 * ============================================================================
 * FUNCTION: as5600_accumulate
 *
 * PURPOSE:
 *    Fold a new single-turn angle into the multi-turn revolution count by
 *    detecting wrap across the zero crossing.
 *
 * PARAMETERS:
 *    counts (uint16_t) - newest single-turn reading
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    as5600_update(), controlTask, Core 1.
 * ============================================================================
 */
static void as5600_accumulate(uint16_t counts) {
    int32_t delta = (int32_t)counts - (int32_t)s_prev_counts;

    /* Half a revolution is the only defensible wrap threshold: any jump
     * larger than that is more cheaply explained by a wrap than by real
     * motion. The inference fails, silently and in the wrong direction, if
     * the shaft truly moves more than half a turn between calls -- which is
     * why the caller's rate matters. See the note in as5600.h. */
    if (delta >  (AS5600_COUNTS_PER_REV / 2)) s_revolutions--;
    if (delta < -(AS5600_COUNTS_PER_REV / 2)) s_revolutions++;

    s_prev_counts = counts;
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool as5600_init(void) {
    s_ready       = false;
    s_revolutions = 0;
    s_zero_counts = 0;

    Wire.beginTransmission(AS5600_I2C_ADDR);
    if (Wire.endTransmission() != 0) return false;

    /* A magnet that is absent, too far or too close makes every angle this
     * part reports meaningless. Refuse to come up rather than hand the
     * control loop numbers that look plausible and are not. */
    as5600_magnet_t m;
    if (!as5600_get_magnet_status(&m)) return false;
    if (m != AS5600_MAGNET_OK) return false;

    if (!as5600_read_u16(AS5600_REG_RAW_ANGLE_H, &s_prev_counts)) return false;

    s_ready = true;
    return true;
}

bool as5600_update(as5600_reading_t *out) {
    if (out == NULL) return false;

    out->valid = false;
    if (!s_ready) return false;

    uint16_t counts;
    if (!as5600_read_u16(AS5600_REG_RAW_ANGLE_H, &counts)) return false;

    as5600_accumulate(counts);

    out->raw_counts   = counts;
    out->revolutions  = s_revolutions;
    out->total_counts = s_revolutions * (int32_t)AS5600_COUNTS_PER_REV
                      + (int32_t)counts
                      - (int32_t)s_zero_counts;
    out->valid        = true;
    return true;
}

bool as5600_zero(void) {
    if (!s_ready) return false;

    uint16_t counts;
    if (!as5600_read_u16(AS5600_REG_RAW_ANGLE_H, &counts)) return false;

    s_zero_counts = counts;
    s_prev_counts = counts;
    s_revolutions = 0;
    return true;
}

bool as5600_get_magnet_status(as5600_magnet_t *out) {
    if (out == NULL) return false;

    uint8_t st = 0;
    if (!as5600_read_u8(AS5600_REG_STATUS, &st)) return false;

    if (!(st & AS5600_STATUS_MD))      *out = AS5600_MAGNET_ABSENT;
    else if (st & AS5600_STATUS_ML)    *out = AS5600_MAGNET_WEAK;
    else if (st & AS5600_STATUS_MH)    *out = AS5600_MAGNET_STRONG;
    else                               *out = AS5600_MAGNET_OK;
    return true;
}
