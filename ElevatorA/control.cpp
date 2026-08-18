/*
 * ============================================================================
 * FILE: control.cpp
 *
 * PURPOSE:
 *    Elevator control policy implementation.
 *
 *    The servo loop is integrated from the closed-loop bench prototype,
 *    08/16/2026: deadband, integrator bleed, acceleration limiting and
 *    stall detection are carried over. The prototype ran the loop at 1 kHz
 *    inline in loop(); here it runs at 100 Hz inside controlTask, so the
 *    gains in board_config.h must be retuned before this does anything
 *    useful. See the note beside CTRL_KP.
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
 *    - board_config.h, app_types.h
 *    - shared_state.h, control.h
 *    - stepper.h, as5600.h, hcsr04.h: hardware reached only through drivers
 *    - ../logic/pid.h: platform-independent controller
 *
 * NOTES:
 *    App layer. Reaches hardware only through driver headers.
 *
 *    FEEDBACK ARCHITECTURE
 *    The encoder is the loop's feedback: 12-bit, quiet, no dead zone. The
 *    ultrasonic sensor is NOT in the loop; it measures the car itself and
 *    serves as an independent check on what the encoder claims. The encoder
 *    reads the motor shaft, so it cannot see a slipping line; the ultrasonic
 *    reads ground truth but is too noisy to close a loop around. Sustained
 *    disagreement between them is slip or a jam.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "stepper.h"
#include "as5600.h"
#include "hcsr04.h"
#include "../logic/pid.h"

/* The position controller. Gains come from board_config.h. */
static pid_ctl_t s_pid;

/* Commanded shaft angle, degrees, in encoder terms. */
static float s_target_deg;

/* Last commanded step rate, kept so acceleration can be limited. */
static float s_rate_sps;

/* Consecutive iterations the stall condition has held. */
static uint32_t s_stall_ticks;

/* Latched: cleared only by a new request. */
static bool s_stalled;

/*
 * ============================================================================
 * FUNCTION: control_read_angle_deg
 *
 * PURPOSE:
 *    Read the encoder and convert its multi-turn count to degrees at the
 *    output shaft.
 *
 * PARAMETERS:
 *    out (float*) - destination, degrees
 *
 * RETURN VALUE:
 *    bool - false if the encoder read failed; *out is untouched
 *
 * CALLED FROM:
 *    control_step(), controlTask, Core 1, 100 Hz.
 * ============================================================================
 */
static bool control_read_angle_deg(float *out) {
    as5600_reading_t r;
    if (!as5600_update(&r) || !r.valid) return false;

    *out = (r.total_counts * 360.0f / (float)AS5600_COUNTS_PER_REV)
         / GEAR_RATIO;
    return true;
}

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
 *    control_step(), controlTask, Core 1. Not called yet -- it is wired in
 *    once the three floor positions have been measured on the built shaft,
 *    so it is marked unused to keep the build warning-clean until then.
 * ============================================================================
 */
__attribute__((unused))
static int32_t control_floor_to_position(uint8_t floor) {
    // TODO: the floor-to-millimetre mapping cannot be written until the
    //       shaft is built and each floor position is measured on the bench
    (void)floor;
    return 0;
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool control_init(void) {
    s_target_deg  = 0.0f;
    s_rate_sps    = 0.0f;
    s_stall_ticks = 0;
    s_stalled     = false;

    pid_init(&s_pid, CTRL_KP, CTRL_KI, CTRL_KD,
             -CTRL_INTEGRAL_LIMIT, CTRL_INTEGRAL_LIMIT);

    if (!as5600_zero()) return false;

    // TODO: home the car against a known reference and enter ELEV_MODE_IDLE.
    //       Homing needs a physical datum -- a limit switch, or driving
    //       gently until the ultrasonic reading stops changing. Which one
    //       depends on how the shaft is finally built.
    return true;
}

bool control_request_floor(uint8_t floor, req_source_t source) {
    // TODO: bounds-check floor against NUM_FLOORS, set its pending bit, and
    //       record source for the log
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
    if (dt_ms == 0) return;
    const float dt_s = dt_ms / 1000.0f;

    float pos_deg;
    if (!control_read_angle_deg(&pos_deg)) {
        /* No trustworthy position means no business driving the motor. */
        stepper_set_rate(0.0f);
        s_rate_sps = 0.0f;
        shared_state_raise_fault(FAULT_POSITION_LIMIT);
        return;
    }

    const float error = s_target_deg - pos_deg;

    /* --- stall detection ------------------------------------------------
     * A large error WHILE already commanding near-maximum rate means the
     * motor is not following: missed steps or a jam. Either condition alone
     * is normal -- a big error at the start of a move, or full rate mid-
     * travel -- so both must hold, and hold for a while. */
    if (fabsf(error) > CTRL_STALL_DEG &&
        fabsf(s_rate_sps) > STEP_MAX_SPS * 0.9f) {
        if (++s_stall_ticks > CTRL_STALL_TICKS) {
            s_stalled = true;
            s_stall_ticks = 0;
            stepper_emergency_stop();
            s_rate_sps = 0.0f;
            shared_state_raise_fault(FAULT_STEPPER_STALL);
        }
    } else {
        /* Reset on any iteration the condition does not hold. Leaving this
         * out lets unrelated near-stalls accumulate over minutes and
         * eventually latch a fault that never happened. */
        s_stall_ticks = 0;
    }
    if (s_stalled) return;

    /* --- deadband -------------------------------------------------------
     * Inside the deadband the loop stops commanding motion, so the car does
     * not hunt against detent torque. The integrator is bled rather than
     * cleared: clearing it makes the loop forget a real standing load and
     * sag when it re-engages. */
    if (fabsf(error) < CTRL_DEADBAND_DEG) {
        s_rate_sps *= 0.5f;
        stepper_set_rate(s_rate_sps);
        return;
    }

    /* --- controller ------------------------------------------------------ */
    float sps = pid_update(&s_pid, s_target_deg, pos_deg, dt_s)
              * (STEPS_PER_REV / 360.0f);

    if (sps >  STEP_MAX_SPS) sps =  STEP_MAX_SPS;
    if (sps < -STEP_MAX_SPS) sps = -STEP_MAX_SPS;

    /* --- acceleration limit ---------------------------------------------
     * A stepper that is commanded from standstill straight to full rate
     * simply skips steps. Bounding the rate of change is what makes the
     * commanded position trustworthy at all. */
    const float max_delta = STEP_MAX_ACCEL_SPS2 * dt_s;
    if (sps - s_rate_sps >  max_delta) sps = s_rate_sps + max_delta;
    if (sps - s_rate_sps < -max_delta) sps = s_rate_sps - max_delta;

    s_rate_sps = sps;
    stepper_set_rate(s_rate_sps);

    // TODO: cross-check the encoder against the ultrasonic reading and
    //       raise FAULT_STEPPER_STALL on sustained disagreement. The
    //       tolerance depends on the encoder-counts-to-millimetres ratio,
    //       which is not known until the car is built and measured.

    // TODO: publish the updated state through shared_state_set()
}

void control_emergency_stop(uint32_t cause) {
    stepper_emergency_stop();
    s_rate_sps = 0.0f;
    pid_reset(&s_pid);
    shared_state_raise_fault(cause);

    // TODO: enter ELEV_MODE_ESTOP in the published state once the mode
    //       transitions are written
}

/* ========================================================================== */
/*                    SECTION: BENCH TUNING INTERFACE                         */
/* ========================================================================== */

void control_set_gains(float kp, float ki, float kd) {
    /* Re-initialising rather than poking the gains in place also clears the
     * integrator. Without that, error accumulated under the old gains is
     * multiplied by the new ki the moment it changes, and the loop kicks --
     * which reads as "the new gains are unstable" when they are not. */
    pid_init(&s_pid, kp, ki, kd, -CTRL_INTEGRAL_LIMIT, CTRL_INTEGRAL_LIMIT);
    s_rate_sps = 0.0f;
}

void control_get_gains(float *kp, float *ki, float *kd) {
    if (kp) *kp = s_pid.kp;
    if (ki) *ki = s_pid.ki;
    if (kd) *kd = s_pid.kd;
}

void control_set_target_deg(float deg) {
    s_target_deg = deg;
    s_stalled    = false;
    s_stall_ticks = 0;
    pid_reset(&s_pid);
}

bool control_get_debug(float *target_deg, float *pos_deg, float *rate_sps) {
    if (target_deg) *target_deg = s_target_deg;
    if (rate_sps)   *rate_sps   = s_rate_sps;

    float p;
    bool ok = control_read_angle_deg(&p);
    if (pos_deg) *pos_deg = ok ? p : 0.0f;
    return ok;
}

bool control_zero_here(void) {
    if (!as5600_zero()) return false;
    s_target_deg  = 0.0f;
    s_rate_sps    = 0.0f;
    s_stalled     = false;
    s_stall_ticks = 0;
    pid_reset(&s_pid);
    return true;
}

void control_clear_stall(void) {
    s_stalled     = false;
    s_stall_ticks = 0;
    pid_reset(&s_pid);
}
