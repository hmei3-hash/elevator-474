/*
 * ============================================================================
 * FILE: control.h
 *
 * PURPOSE:
 *    Elevator control policy. Owns the operating mode, the pending request
 *    set, target selection, and the emergency stop path. This is the only
 *    place where floors and elevator behaviour are decided.
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
 *    - app_types.h: elevator vocabulary and message types
 *
 * NOTES:
 *    App layer. Must not touch GPIO, I2C, or SPI directly; it reaches the
 *    hardware only through the driver headers.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>
#include "app_types.h"

/**
 * Initialise the control policy and bring the car to a known floor.
 *
 * @return true if homing succeeded
 * Called from: setup(), before tasks start.
 */
bool control_init(void);

/**
 * Record a floor request from any source.
 *
 * @param floor   requested floor index, 0 based
 * @param source  where the request came from
 * @return true if the request was accepted
 * Called from: controlTask (Core 1, 100 Hz).
 */
bool control_request_floor(uint8_t floor, req_source_t source);

/**
 * Choose which pending request to serve next.
 *
 * @return floor index to travel to, or the current floor if none pending
 * Called from: controlTask (Core 1, 100 Hz).
 */
uint8_t control_select_target(void);

/**
 * Run one iteration of the control loop: read position, update the mode,
 * and command the motion driver.
 *
 * @param dt_ms  elapsed milliseconds since the previous call
 * @return void
 * Called from: controlTask (Core 1, 100 Hz).
 */
void control_step(uint32_t dt_ms);

/**
 * Assert emergency stop. Halts motion and latches the fault. Safe to call
 * repeatedly.
 *
 * @param cause  fault_code_t bit describing why
 * @return void
 * Called from: controlTask (Core 1, 100 Hz), on the link-fault path.
 */
void control_emergency_stop(uint32_t cause);

/* ========================================================================== */
/*                    SECTION: BENCH TUNING INTERFACE                         */
/*                                                                            */
/*   These exist so gains can be changed while the loop runs. Retuning by     */
/*   recompiling costs a full build-and-flash cycle per attempt, and the      */
/*   move from 1 kHz to 100 Hz needs dozens of attempts. They are a bench     */
/*   facility, not part of the elevator's behaviour: nothing in the           */
/*   application calls them.                                                  */
/* ========================================================================== */

/**
 * Replace the controller gains and clear the accumulated state.
 *
 * @param kp  proportional gain
 * @param ki  integral gain, per second
 * @param kd  derivative gain, seconds
 * @return void
 * Called from: cmdTask (Core 0, priority 1). Safe while the loop runs --
 *              the integrator is reset, so the change does not arrive as a
 *              step disturbance from stale accumulated error.
 */
void control_set_gains(float kp, float ki, float kd);

/**
 * Read the gains currently in force.
 *
 * @param kp  destination, may be NULL
 * @param ki  destination, may be NULL
 * @param kd  destination, may be NULL
 * @return void
 * Called from: cmdTask (Core 0, priority 1).
 */
void control_get_gains(float *kp, float *ki, float *kd);

/**
 * Command an absolute shaft angle directly, bypassing floor selection.
 * Used to exercise the servo loop before the floor map is measured.
 *
 * @param deg  target angle in degrees at the output shaft
 * @return void
 * Called from: cmdTask (Core 0, priority 1).
 */
void control_set_target_deg(float deg);

/**
 * Read back the loop's live numbers for a status print.
 *
 * @param target_deg  destination, may be NULL
 * @param pos_deg     destination, may be NULL
 * @param rate_sps    destination, may be NULL
 * @return true if the position reading was valid
 * Called from: cmdTask (Core 0, priority 1).
 */
bool control_get_debug(float *target_deg, float *pos_deg, float *rate_sps);

/**
 * Take the current shaft angle as zero and clear the target.
 *
 * @return true if the encoder answered
 * Called from: cmdTask (Core 0, priority 1).
 */
bool control_zero_here(void);

/**
 * Clear a latched stall so the loop will drive again.
 *
 * @return void
 * Called from: cmdTask (Core 0, priority 1).
 */
void control_clear_stall(void);
