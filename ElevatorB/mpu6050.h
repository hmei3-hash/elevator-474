/*
 * ============================================================================
 * FILE: mpu6050.h
 *
 * PURPOSE:
 *    Driver for a six-axis inertial measurement unit on I2C. Reports raw
 *    acceleration and angular rate. It makes no judgement about what the
 *    motion means.
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
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/** One synchronised sample from the sensor. */
typedef struct {
    int16_t  accel_x;     // raw counts
    int16_t  accel_y;     // raw counts
    int16_t  accel_z;     // raw counts
    int16_t  gyro_x;      // raw counts
    int16_t  gyro_y;      // raw counts
    int16_t  gyro_z;      // raw counts
    uint32_t timestamp_ms;
} imu_sample_t;

/**
 * Bring up I2C, wake the sensor, and confirm its identity register.
 *
 * @return true if the sensor answered with the expected identity
 * Called from: setup(), before tasks start.
 */
bool mpu6050_init(void);

/**
 * Read one synchronised sample.
 *
 * @param out  destination, must not be NULL
 * @return true if the read completed without a bus error
 * Called from: sampleTask (Core 1).
 */
bool mpu6050_read(imu_sample_t *out);

/**
 * Check that the sensor still acknowledges on the bus.
 *
 * @return true if the sensor is healthy
 * Called from: sampleTask (Core 1), periodically.
 */
bool mpu6050_is_alive(void);
