/*
 * ============================================================================
 * FILE: pid.c
 *
 * PURPOSE:
 *    Discrete PID controller implementation.
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
 *    - pid.h
 *
 * NOTES:
 *    Host-testable. This file must have ZERO platform and ZERO framework
 *    dependencies so it compiles and unit-tests with plain gcc.
 *    Do not add any platform or framework header here under any
 *    circumstances.
 *
 * ============================================================================
 */

#include "pid.h"

void pid_init(pid_t *c, float kp, float ki, float kd, float out_min, float out_max) {
    /* TODO: store the gains and clamps, then clear integrator and history.
     * The gain VALUES are not decided here; they are supplied by the caller
     * and must come from step-response measurements on the built car. */
    (void)c;
    (void)kp;
    (void)ki;
    (void)kd;
    (void)out_min;
    (void)out_max;
    return;
}

float pid_update(pid_t *c, float setpoint, float measured, float dt_s) {
    /* TODO: compute the error, accumulate the integral term, form the
     * derivative from the previous error, sum the three terms, clamp to
     * the configured range, and apply integrator anti-windup. */
    (void)c;
    (void)setpoint;
    (void)measured;
    (void)dt_s;
    return 0.0f;
}

void pid_reset(pid_t *c) {
    /* TODO: zero the integrator and the previous-error history. */
    (void)c;
    return;
}
