/*
 * ============================================================================
 * FILE: stepper.h
 *
 * PURPOSE:
 *    Driver for a step/direction stepper driver module with a UART control
 *    channel. Speaks only in steps, direction, and enable state. It has no
 *    knowledge of what the motor is attached to.
 *
 *    Step pulses are produced by a hardware timer ISR running a phase
 *    accumulator, so the caller sets a RATE and the pulse train continues
 *    without further attention. Nothing here blocks.
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
 *    - Arduino.h: GPIO and HardwareSerial types
 *
 * NOTES:
 *    Driver layer. Must not reference application concepts.
 *    All public functions are called from a single task; this driver is not
 *    internally synchronised beyond the spinlock guarding the ISR handoff.
 *
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/**
 * Configure the driver pins, open the UART control channel, apply the
 * current and microstep settings, and start the step pulse timer.
 *
 * @return true if the driver answered on UART with the expected version
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
 * Command a signed step rate. Sign selects direction; magnitude is clamped
 * to STEP_MAX_SPS. Zero stops the pulse train without releasing the coils.
 *
 * @param steps_per_sec  signed rate; positive is the direction that makes
 *                       stepper_get_position() increase
 * @return void
 * Called from: controlTask (Core 1, 100 Hz).
 */
void stepper_set_rate(float steps_per_sec);

/**
 * Read the signed count of step pulses emitted since init. This is the
 * COMMANDED position, not a measurement: it does not know about missed
 * steps. Compare it against an encoder to detect those.
 *
 * @return step count; positive is the direction of a positive rate
 * Called from: controlTask (Core 1, 100 Hz).
 */
int32_t stepper_get_position(void);

/**
 * Stop pulsing immediately and release the coils. Safe to call repeatedly
 * and safe to call before init.
 *
 * @return void
 * Called from: controlTask (Core 1, 100 Hz), on the emergency path.
 */
void stepper_emergency_stop(void);
