/*
 * ============================================================================
 * FILE: mpu6050.cpp
 *
 * PURPOSE:
 *    Inertial measurement unit driver implementation.
 *
 *    Integrated from the imu_logger bench prototype, 08/16/2026. The
 *    register sequence, the full-scale settings, the wide DLPF setting and
 *    the single burst read are all carried over unchanged, because that is
 *    the configuration the recorded drop traces were captured under.
 *    Changing any of them invalidates the thresholds derived from that data.
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
 *    - Wire.h
 *    - board_config.h: pins, address candidates, range and scale
 *    - mpu6050.h
 *    No MPU6050 library, so there is no library version to disagree about
 *    later and the configuration in force stays visible in this file.
 *
 * DIFFERENCES FROM THE PROTOTYPE
 *    - Nothing here prints. The prototype wrote CSV to Serial; on Board B
 *      only the logging path may touch Serial, so failures are reported
 *      through return values instead.
 *    - The startup bus scan is reduced to probing the two legal addresses.
 *      A full 1..126 sweep is a bring-up diagnostic, not something a driver
 *      should do on every boot.
 *    - Sample pacing moved out. The prototype paced itself in loop(); the
 *      task now owns the period, which is what makes the rate schedulable.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"
#include "mpu6050.h"

/* ========================================================================== */
/*                          SECTION: DEVICE REGISTERS                         */
/* ========================================================================== */

#define REG_SMPLRT_DIV        0x19
#define REG_CONFIG            0x1A
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_ACCEL_XOUT_H      0x3B
#define REG_PWR_MGMT_1        0x6B
#define REG_WHO_AM_I          0x75

/* Bytes returned by one burst read: accel 6, temperature 2, gyro 6. */
#define BURST_BYTES           14

/* ========================================================================== */
/*                          SECTION: FILE-LOCAL STATE                         */
/* ========================================================================== */

/* Address the device actually answered on. Zero until init succeeds. */
static uint8_t s_addr;

/* Consecutive failed reads. Cleared by any success. The fault decision
 * belongs to the application layer, so this is only a local counter. */
static uint32_t s_consecutive_errors;

/* ========================================================================== */
/*                          SECTION: I2C HELPERS                              */
/* ========================================================================== */

/*
 * ============================================================================
 * FUNCTION: reg_write
 *
 * PURPOSE:
 *    Write one byte to a device register.
 *
 * PARAMETERS:
 *    reg (uint8_t) - register address
 *    val (uint8_t) - value to write
 *
 * RETURN VALUE:
 *    bool - true if the device acknowledged
 *
 * CALLED FROM:
 *    mpu6050_init(), before any task exists.
 * ============================================================================
 */
static bool reg_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(s_addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

/*
 * ============================================================================
 * FUNCTION: reg_read
 *
 * PURPOSE:
 *    Read one byte from a device register.
 *
 * PARAMETERS:
 *    reg (uint8_t) - register address
 *    out (uint8_t*) - destination, must not be NULL
 *
 * RETURN VALUE:
 *    bool - true if a byte was returned
 *
 * CALLED FROM:
 *    mpu6050_init() and mpu6050_is_alive().
 * ============================================================================
 */
static bool reg_read(uint8_t reg, uint8_t *out) {
    Wire.beginTransmission(s_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)s_addr, 1) != 1) return false;
    *out = Wire.read();
    return true;
}

/*
 * ============================================================================
 * FUNCTION: probe_address
 *
 * PURPOSE:
 *    Test whether a device acknowledges at the given address.
 *
 * PARAMETERS:
 *    addr (uint8_t) - 7-bit I2C address to probe
 *
 * RETURN VALUE:
 *    bool - true if something acknowledged
 *
 * CALLED FROM:
 *    mpu6050_init(), before any task exists.
 * ============================================================================
 */
static bool probe_address(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool mpu6050_init(void) {
    s_addr = 0;
    s_consecutive_errors = 0;

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_HZ);

    /* AD0 decides between the two legal addresses. Probing both means a
     * rewired module does not need a recompile. */
    if (probe_address(MPU_I2C_ADDR_LOW)) {
        s_addr = MPU_I2C_ADDR_LOW;
    } else if (probe_address(MPU_I2C_ADDR_HIGH)) {
        s_addr = MPU_I2C_ADDR_HIGH;
    } else {
        return false;
    }

    /* The device powers up asleep. Skipping this yields all-zero samples,
     * which read exactly like a wiring fault and cost an hour to diagnose.
     * Value 0x01 also selects the X gyro PLL as clock source, which is more
     * stable than the internal oscillator. */
    if (!reg_write(REG_PWR_MGMT_1, 0x01)) {
        s_addr = 0;
        return false;
    }
    delay(100);                        /* PLL settling; setup context only */

    /* Confirm identity before trusting any sample. A device that
     * acknowledges its address but reports the wrong identity is a
     * different part on the same bus, not a working sensor. */
    uint8_t who = 0;
    if (!reg_read(REG_WHO_AM_I, &who) || who != MPU_WHO_AM_I_EXPECTED) {
        /* Something acknowledged the address but is not the part the fall
         * thresholds were characterised against. Refusing here beats
         * silently deriving a safety decision from another sensor's scale
         * factors. */
        s_addr = 0;
        return false;
    }

    bool ok = true;
    ok &= reg_write(REG_CONFIG,       MPU_DLPF_CFG);
    ok &= reg_write(REG_GYRO_CONFIG,  (uint8_t)(MPU_GYRO_FS_SEL  << 3));
    ok &= reg_write(REG_ACCEL_CONFIG, (uint8_t)(MPU_ACCEL_FS_SEL << 3));

    /* The internal output rate is far above the polling rate; sampleTask
     * paces the reads, so no divider is needed. */
    ok &= reg_write(REG_SMPLRT_DIV, 0);

    if (!ok) {
        s_addr = 0;
        return false;
    }
    return true;
}

bool mpu6050_read(imu_sample_t *out) {
    if (out == NULL || s_addr == 0) return false;

    /* One burst transaction for all six axes. Reading them separately would
     * let the axes come from different instants, which corrupts the vector
     * magnitude during exactly the fast transients the detector looks for. */
    Wire.beginTransmission(s_addr);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        s_consecutive_errors++;
        return false;
    }
    if (Wire.requestFrom((int)s_addr, BURST_BYTES) != BURST_BYTES) {
        s_consecutive_errors++;
        return false;
    }

    out->accel_x = (int16_t)((Wire.read() << 8) | Wire.read());
    out->accel_y = (int16_t)((Wire.read() << 8) | Wire.read());
    out->accel_z = (int16_t)((Wire.read() << 8) | Wire.read());

    (void)((Wire.read() << 8) | Wire.read());   /* temperature, discarded */

    out->gyro_x  = (int16_t)((Wire.read() << 8) | Wire.read());
    out->gyro_y  = (int16_t)((Wire.read() << 8) | Wire.read());
    out->gyro_z  = (int16_t)((Wire.read() << 8) | Wire.read());

    out->timestamp_ms = millis();

    s_consecutive_errors = 0;
    return true;
}

bool mpu6050_is_alive(void) {
    if (s_addr == 0) return false;

    uint8_t who = 0;
    if (!reg_read(REG_WHO_AM_I, &who)) return false;

    return (who == MPU_WHO_AM_I_EXPECTED);
}
