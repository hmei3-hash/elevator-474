/*
 * ============================================================================
 * FILE: control.cpp
 *
 * PURPOSE:
 *    Elevator control policy implementation.
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
 *    - app_types.h
 *    - shared_state.h
 *    - control.h
 *    - stepper.h, hcsr04.h: hardware reached only through drivers
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "stepper.h"
#include "hcsr04.h"

/* Local working copy of the state, published through shared_state_set(). */
static system_state_t s_local;

/*
 * ============================================================================
 * FUNCTION: control_floor_to_position
 *
 * PURPOSE:
 *    Map a floor index to its target position along the shaft.
 *
 * PARAMETERS:
 *    floor (uint8_t) - floor index, 0 based
 *
 * RETURN VALUE:
 *    int32_t - target position in millimetres
 *
 * CALLED FROM:
 *    control_step(), controlTask, Core 1.
 * ============================================================================
 */
static int32_t control_floor_to_position(uint8_t floor) {
    // TODO: the floor-to-millimetre mapping cannot be written until the
    //       shaft is built and each floor position is measured on the bench
    (void)floor;
    return 0;
}

bool control_init(void) {
    // TODO: clear the request set, enter ELEV_MODE_INIT, and home the car
    return false;
}

bool control_request_floor(uint8_t floor, req_source_t source) {
    // TODO: bounds-check floor against NUM_FLOORS and set its pending bit
    (void)floor;
    (void)source;
    return false;
}

uint8_t control_select_target(void) {
    // TODO: the request-scheduling policy is undecided; choose between
    //       nearest-first and directional sweep after the mechanics are
    //       built and travel time between floors is measured
    return 0;
}

void control_step(uint32_t dt_ms) {
    // TODO: read the filtered position, run the position loop, and command
    //       the motion driver. The loop gains live in logic/pid.c and are
    //       not known until the car mass and travel are measured.
    (void)dt_ms;
    return;
}

void control_emergency_stop(uint32_t cause) {
    // TODO: stop the motion driver immediately, enter ELEV_MODE_ESTOP, and
    //       latch cause into the fault flags
    (void)cause;
    return;
}
