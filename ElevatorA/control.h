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
