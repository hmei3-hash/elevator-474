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
 *    08/15/2026
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *    - shared_state.h
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "shared_state.h"

/* The one and only instance. Never exposed; reach it via the accessors. */
static system_state_t g_state;

/* Spinlock guarding g_state. portMUX is the correct primitive here because
 * both cores touch the state and the critical sections are very short. */
static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

void shared_state_init(void) {
    // TODO: zero g_state, set mode to ELEV_MODE_INIT, clear all fault bits
    return;
}

void shared_state_get(system_state_t *out) {
    // TODO: enter the spinlock, copy g_state into *out, exit the spinlock
    (void)out;
    return;
}

void shared_state_set(const system_state_t *in) {
    // TODO: enter the spinlock, copy *in into g_state, exit the spinlock
    (void)in;
    return;
}

void shared_state_raise_fault(uint32_t mask) {
    // TODO: enter the spinlock, OR mask into g_state.fault_flags, exit
    (void)mask;
    return;
}

void shared_state_note_heartbeat(uint32_t seq, uint32_t now_ms) {
    // TODO: enter the spinlock, store seq and now_ms, clear FAULT_LINK_TIMEOUT
    (void)seq;
    (void)now_ms;
    return;
}
