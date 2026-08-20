/*
 * shared_state.cpp
 * Board A shared-state implementation.
 */
#include <Arduino.h>
#include <string.h>

#include "shared_state.h"

static system_state_t g_state;
static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

void shared_state_init(void) {
    memset(&g_state, 0, sizeof(g_state));
    g_state.mode         = ELEV_MODE_INIT;
    g_state.direction    = ELEV_DIR_NONE;
    g_state.fault_flags  = FAULT_NONE;
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

    /*
     * Preserve fault bits raised asynchronously by another task after the
     * caller took its snapshot.
     */
    const uint32_t latched = g_state.fault_flags;

    g_state = *in;
    g_state.fault_flags |= latched;

    portEXIT_CRITICAL(&g_state_mux);
}

void shared_state_raise_fault(uint32_t mask) {
    portENTER_CRITICAL(&g_state_mux);
    g_state.fault_flags |= mask;
    portEXIT_CRITICAL(&g_state_mux);
}

void shared_state_clear_fault(uint32_t mask) {
    /*
     * Never silently clear a fall emergency through the generic recovery
     * helper. A fall remains operator-latched.
     */
    mask &= ~((uint32_t)FAULT_FALL_DETECTED);

    portENTER_CRITICAL(&g_state_mux);
    g_state.fault_flags &= ~mask;
    portEXIT_CRITICAL(&g_state_mux);
}

void shared_state_note_heartbeat(uint32_t seq, uint32_t now_ms) {
    portENTER_CRITICAL(&g_state_mux);

    g_state.link_seq          = seq;
    g_state.last_heartbeat_ms = now_ms;

    /* A valid frame positively proves the link has recovered. */
    g_state.fault_flags &= ~((uint32_t)FAULT_LINK_TIMEOUT);

    portEXIT_CRITICAL(&g_state_mux);
}
