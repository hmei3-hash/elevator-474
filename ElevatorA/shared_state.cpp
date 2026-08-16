/*
 * ============================================================================
 * FILE: shared_state.cpp
 *
 * PURPOSE:
 *    Definitions for the shared elevator state and its spinlock.
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
 *    - shared_state.h
 *
 * NOTES:
 *    WHY A SPINLOCK AND NOT A MUTEX
 *    Both cores touch this record and the critical sections are a struct
 *    copy long. A FreeRTOS mutex would let the holder be preempted while
 *    holding it, and a reader on the other core would then block on a task
 *    that is not running. portMUX masks interrupts on the calling core for
 *    the few microseconds the copy takes, which is the right trade at this
 *    size -- and the reason every section below must stay short.
 *
 *    Never call a driver, print, or wait inside these functions.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "shared_state.h"

/* The one and only instance. Never exposed; reach it via the accessors. */
static system_state_t g_state;

/* Spinlock guarding g_state. */
static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

void shared_state_init(void) {
    /* Runs before any task exists, so no lock is needed or wanted here. */
    memset(&g_state, 0, sizeof(g_state));
    g_state.mode        = ELEV_MODE_INIT;
    g_state.direction   = ELEV_DIR_NONE;
    g_state.fault_flags = FAULT_NONE;
    g_state.estop_active = false;
}

void shared_state_get(system_state_t *out) {
    if (out == NULL) return;
    portENTER_CRITICAL(&g_state_mux);
    *out = g_state;
    portEXIT_CRITICAL(&g_state_mux);
}

void shared_state_set(const system_state_t *in) {
    if (in == NULL) return;
    portENTER_CRITICAL(&g_state_mux);
    /* Faults are latched, not published. controlTask holds a working copy
     * that may be stale with respect to a fault another task raised in the
     * meantime; OR-ing rather than assigning means a fault can never be
     * lost to that race. */
    uint32_t latched = g_state.fault_flags;
    g_state = *in;
    g_state.fault_flags |= latched;
    portEXIT_CRITICAL(&g_state_mux);
}

void shared_state_raise_fault(uint32_t mask) {
    portENTER_CRITICAL(&g_state_mux);
    g_state.fault_flags |= mask;
    portEXIT_CRITICAL(&g_state_mux);
}

void shared_state_note_heartbeat(uint32_t seq, uint32_t now_ms) {
    portENTER_CRITICAL(&g_state_mux);
    g_state.link_seq          = seq;
    g_state.last_heartbeat_ms = now_ms;
    /* Clearing the timeout bit here is deliberate: the link is proven alive
     * by this very frame. FAULT_FALL_DETECTED is NOT cleared -- a fall is
     * latched until an operator clears it. */
    g_state.fault_flags &= ~((uint32_t)FAULT_LINK_TIMEOUT);
    portEXIT_CRITICAL(&g_state_mux);
}
