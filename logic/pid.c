/*
 * ============================================================================
 * FILE: pid.c
 *
 * PURPOSE:
 *    Discrete PID controller implementation.
 *
 *    Integrated from the closed-loop bench prototype, 08/16/2026. The
 *    prototype's loop ran inline; the arithmetic is unchanged, only lifted
 *    into a reusable unit so it can be exercised on a host.
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
 *    - pid.h
 *
 * NOTES:
 *    Host-testable. This file must have ZERO platform and ZERO framework
 *    dependencies so it compiles and unit-tests with plain gcc.
 *    Do not add any platform or framework header here under any
 *    circumstances.
 *
 *    NO GAIN VALUES LIVE HERE. The caller supplies them. The prototype's
 *    values were tuned at 1 kHz and do not transfer to the 100 Hz loop.
 *
 * ============================================================================
 */

#include "pid.h"

void pid_init(pid_t *c, float kp, float ki, float kd, float out_min, float out_max) {
    if (c == 0) return;

    c->kp = kp;
    c->ki = ki;
    c->kd = kd;

    c->out_min = out_min;
    c->out_max = out_max;

    /* The integrator is clamped to the same range as the output. The
     * prototype clamped it separately, in error-seconds, on the argument
     * that the useful integrator range is not the output range. That is
     * true in general; here the caller passes the integrator limit as
     * out_min/out_max and scales the result, which keeps one clamp instead
     * of two that can disagree. */

    pid_reset(c);
}

float pid_update(pid_t *c, float setpoint, float measured, float dt_s) {
    if (c == 0 || dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measured;

    /* Multiplying by dt is what makes the gain independent of the loop
     * rate: change the rate and the accumulated integral is unchanged. */
    c->integrator += error * dt_s;

    /* Anti-windup. Without it, a saturated output lets the integrator grow
     * without bound while the plant cannot respond, and the controller then
     * overshoots badly on the way back. */
    if (c->integrator > c->out_max) c->integrator = c->out_max;
    if (c->integrator < c->out_min) c->integrator = c->out_min;

    float derivative = (error - c->prev_error) / dt_s;
    c->prev_error = error;

    float out = c->kp * error
              + c->ki * c->integrator
              + c->kd * derivative;

    if (out > c->out_max) out = c->out_max;
    if (out < c->out_min) out = c->out_min;
    return out;
}

void pid_reset(pid_t *c) {
    if (c == 0) return;
    c->integrator = 0.0f;
    c->prev_error = 0.0f;
}
