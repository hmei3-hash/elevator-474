/*
 * shared_state.h
 * Board A shared-state accessors.
 */
#pragma once

#include <Arduino.h>
#include "app_types.h"

/* Initialise shared state before application tasks are created. */
void shared_state_init(void);

/* Copy the current state into *out. */
void shared_state_get(system_state_t *out);

/* Publish a state snapshot. Existing fault bits remain latched. */
void shared_state_set(const system_state_t *in);

/* Set one or more fault bits. */
void shared_state_raise_fault(uint32_t mask);

/*
 * Clear one or more RECOVERABLE fault bits after the condition has been
 * positively proven healthy again.
 *
 * Do not use this to auto-clear FAULT_FALL_DETECTED.
 */
void shared_state_clear_fault(uint32_t mask);

/* Record a valid Board-B heartbeat and clear LINK_TIMEOUT. */
void shared_state_note_heartbeat(uint32_t seq, uint32_t now_ms);
