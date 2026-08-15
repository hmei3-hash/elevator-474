/*
 * ============================================================================
 * FILE: shared_state.h
 *
 * PURPOSE:
 *    Owns the single system_state_t instance and the spinlock that guards it.
 *    Every task reads and writes elevator state through these accessors; no
 *    task may reference the instance directly. Accessors copy the whole
 *    struct so a reader never observes a half-updated record.
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
 *    - Arduino.h: FreeRTOS portMUX types
 *    - app_types.h: system_state_t
 *
 * NOTES:
 *    App layer. Drivers must not include this file.
 *    The critical sections must stay short: they run with interrupts masked
 *    on the calling core.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>
#include "app_types.h"

/**
 * Initialise the shared state to a known power-on value and prepare the
 * spinlock.
 *
 * @return void
 * Called from: setup(), before any task is created. Not task-safe.
 */
void shared_state_init(void);

/**
 * Copy the entire current state into the caller's buffer.
 *
 * @param out  destination buffer, must not be NULL
 * @return void
 * Called from: lcdTask (Core 0, 5 Hz), logTask (Core 0, 10 Hz),
 *              linkTask (Core 0, 50 Hz).
 */
void shared_state_get(system_state_t *out);

/**
 * Overwrite the entire state from the caller's buffer.
 *
 * @param in  source buffer, must not be NULL
 * @return void
 * Called from: controlTask (Core 1, 100 Hz) only. Single writer by design.
 */
void shared_state_set(const system_state_t *in);

/**
 * Set the given fault bits without disturbing any other field.
 *
 * @param mask  bitwise OR of fault_code_t values to latch
 * @return void
 * Called from: any task, any core.
 */
void shared_state_raise_fault(uint32_t mask);

/**
 * Record that a valid frame arrived from Board B.
 *
 * @param seq         sequence number from the accepted frame
 * @param now_ms      millis() at the time of acceptance
 * @return void
 * Called from: linkTask (Core 0, 50 Hz).
 */
void shared_state_note_heartbeat(uint32_t seq, uint32_t now_ms);
