/*
 * ============================================================================
 * FILE: stepper.h
 *
 * PURPOSE:
 *    Driver for a step/direction stepper driver module with a UART control
 *    channel. Speaks only in steps, direction, and enable state. It has no
 *    knowledge of what the motor is attached to.
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
 *    - Arduino.h: GPIO and HardwareSerial types
 *
 * NOTES:
 *    Driver layer. Must not reference application concepts.
 *    All public functions are called from a single task; this driver is not
 *    internally synchronised.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/** Direction of rotation as seen by the driver hardware. */
typedef enum {
    STEPPER_DIR_FORWARD = 0,
    STEPPER_DIR_REVERSE = 1
} stepper_dir_t;

/**
 * Configure the driver pins and open the UART control channel.
 *
 * @return true on success, false if the driver did not answer on UART
 * Called from: setup(), before tasks start.
 */
bool stepper_init(void);

/**
 * Energise or de-energise the motor coils.
 *
 * @param on  true to enable holding torque, false to release
 * @return void
 * Called from: controlTask (Core 1, 100 Hz).
 */
void stepper_enable(bool on);

/**
 * Set the direction applied on the next step pulse.
 *
 * @param dir  requested direction
 * @return void
 * Called from: controlTask (Core 1, 100 Hz).
 */
void stepper_set_direction(stepper_dir_t dir);

/**
 * Request a constant step rate. The driver emits pulses at this rate until
 * asked to change or stop.
 *
 * @param steps_per_sec  pulse rate in steps per second; 0 stops pulsing
 * @return void
 * Called from: controlTask (Core 1, 100 Hz).
 */
void stepper_set_rate(uint32_t steps_per_sec);

/**
 * Read the signed count of step pulses emitted since init.
 *
 * @return step count; positive is STEPPER_DIR_FORWARD
 * Called from: controlTask (Core 1, 100 Hz).
 */
int32_t stepper_get_position(void);

/**
 * Stop pulsing immediately without waiting for the current motion to finish.
 *
 * @return void
 * Called from: controlTask (Core 1, 100 Hz), on the emergency path.
 */
void stepper_emergency_stop(void);
