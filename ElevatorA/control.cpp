#include <Arduino.h>
#include <math.h>

#include "board_config.h"
#include "ultrasonic_control_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "stepper.h"
#include "../logic/pid.h"

#if NUM_FLOORS != 3
#error "This ultrasonic floor map currently expects NUM_FLOORS == 3"
#endif

static pid_ctl_t s_pid;

/* Controller state. */
static float    s_target_mm;
static bool     s_target_valid;
static float    s_rate_sps;
static bool     s_stalled;

/* Runtime height envelope. Serial may change these while running. */
static float    s_limit_min_mm;
static float    s_limit_max_mm;

/* Latest ultrasonic sample. Written by ultrasonicTask, copied by controlTask. */
static portMUX_TYPE s_ultra_mux = portMUX_INITIALIZER_UNLOCKED;
static bool     s_have_sample;
static uint32_t s_raw_mm;
static float    s_filtered_mm;
static uint32_t s_sample_ms;
static uint32_t s_sample_seq;

/* Last sample consumed by the PID. */
static uint32_t s_processed_seq;
static uint32_t s_prev_pid_sample_ms;

/* Stall window. */
static uint32_t s_stall_samples;
static float    s_stall_start_mm;

static const float s_floor_mm[NUM_FLOORS] = {
    CTRL_FLOOR0_MM,
    CTRL_FLOOR1_MM,
    CTRL_FLOOR2_MM
};

static bool floor_configured(uint8_t floor) {
    if (floor >= NUM_FLOORS) return false;

    const float mm = s_floor_mm[floor];

    return mm >= (float)HCSR04_MIN_MM &&
           mm <= (float)HCSR04_MAX_MM &&
           mm >= s_limit_min_mm &&
           mm <= s_limit_max_mm;
}

bool control_init(void) {
    s_target_mm          = 0.0f;
    s_target_valid       = false;
    s_rate_sps           = 0.0f;
    s_stalled            = false;

    s_limit_min_mm       = CTRL_HEIGHT_MIN_MM;
    s_limit_max_mm       = CTRL_HEIGHT_MAX_MM;

    s_have_sample        = false;
    s_raw_mm             = 0;
    s_filtered_mm        = 0.0f;
    s_sample_ms          = 0;
    s_sample_seq         = 0;
    s_processed_seq      = 0;
    s_prev_pid_sample_ms = 0;

    s_stall_samples      = 0;
    s_stall_start_mm     = 0.0f;

    /* PID output is directly steps/s. */
    pid_init(&s_pid,
             CTRL_ULTRA_KP,
             CTRL_ULTRA_KI,
             CTRL_ULTRA_KD,
             -CTRL_ULTRA_MAX_SPS,
              CTRL_ULTRA_MAX_SPS);

    /* No encoder homing is required. The first valid ultrasonic reading
     * becomes the initial hold target so power-up never commands a move.
     */
    return true;
}

void control_update_ultrasonic(uint32_t raw_mm, uint32_t timestamp_ms) {
    portENTER_CRITICAL(&s_ultra_mux);

    if (!s_have_sample) {
        s_filtered_mm = (float)raw_mm;
        s_have_sample = true;
    } else {
        s_filtered_mm += CTRL_ULTRA_FILTER_ALPHA *
                         ((float)raw_mm - s_filtered_mm);
    }

    s_raw_mm    = raw_mm;
    s_sample_ms = timestamp_ms;
    s_sample_seq++;

    portEXIT_CRITICAL(&s_ultra_mux);
}

bool control_request_floor(uint8_t floor, req_source_t source) {
    (void)source;

    if (!floor_configured(floor)) return false;

    s_target_mm    = s_floor_mm[floor];
    s_target_valid = true;
    s_stalled      = false;
    s_stall_samples = 0;
    pid_reset(&s_pid);

    system_state_t st;
    shared_state_get(&st);
    st.target_floor = floor;
    st.request_pending[floor] = true;

    if (floor > st.current_floor)      st.direction = ELEV_DIR_UP;
    else if (floor < st.current_floor) st.direction = ELEV_DIR_DOWN;
    else                               st.direction = ELEV_DIR_NONE;

    st.mode = (floor == st.current_floor) ? ELEV_MODE_IDLE : ELEV_MODE_MOVING;
    shared_state_set(&st);
    return true;
}

uint8_t control_select_target(void) {
    system_state_t st;
    shared_state_get(&st);
    return st.target_floor;
}

static void publish_position(uint32_t raw_mm, float filtered_mm) {
    system_state_t st;
    shared_state_get(&st);

    st.position_raw_mm = (int32_t)raw_mm;
    st.position_mm     = (int32_t)lroundf(filtered_mm);
    st.stepper_steps   = stepper_get_position();

    /* If we are at a configured floor, update the logical floor. */
    for (uint8_t i = 0; i < NUM_FLOORS; ++i) {
        if (!floor_configured(i)) continue;
        if (fabsf(filtered_mm - s_floor_mm[i]) <= CTRL_ULTRA_DEADBAND_MM) {
            st.current_floor = i;

            if (s_target_valid &&
                fabsf(s_target_mm - s_floor_mm[i]) <= CTRL_ULTRA_DEADBAND_MM) {
                st.target_floor = i;
                st.request_pending[i] = false;
                st.direction = ELEV_DIR_NONE;
                st.mode = ELEV_MODE_IDLE;
            }
            break;
        }
    }

    shared_state_set(&st);
}

void control_step(uint32_t dt_ms) {
    (void)dt_ms;

    bool     have;
    uint32_t raw;
    float    pos;
    uint32_t sample_ms;
    uint32_t seq;

    portENTER_CRITICAL(&s_ultra_mux);
    have      = s_have_sample;
    raw       = s_raw_mm;
    pos       = s_filtered_mm;
    sample_ms = s_sample_ms;
    seq       = s_sample_seq;
    portEXIT_CRITICAL(&s_ultra_mux);

    if (!have) {
        stepper_set_rate(0.0f);
        s_rate_sps = 0.0f;
        return;
    }

    const uint32_t now = millis();

    /* Primary position sensor is stale -> motion is unsafe. */
    if ((now - sample_ms) > CTRL_ULTRA_STALE_MS) {
        stepper_set_rate(0.0f);
        s_rate_sps = 0.0f;
        shared_state_raise_fault(FAULT_ULTRASONIC_LOST);
        return;
    }

    publish_position(raw, pos);

    /* The PID should update only when a NEW range sample arrives.
     * Running it at 100 Hz on a 16 Hz measurement would create repeated
     * identical samples followed by a jump, especially bad for D.
     */
    if (seq == s_processed_seq) return;
    s_processed_seq = seq;

    if (!s_target_valid) {
        /* Safe power-up behavior: hold where the first good measurement says
         * the car actually is.
         */
        s_target_mm    = pos;
        s_target_valid = true;
        s_prev_pid_sample_ms = sample_ms;
        s_stall_start_mm = pos;
        pid_reset(&s_pid);
        stepper_set_rate(0.0f);
        s_rate_sps = 0.0f;
        return;
    }

    float sample_dt_s;
    if (s_prev_pid_sample_ms == 0 || sample_ms <= s_prev_pid_sample_ms) {
        sample_dt_s = PERIOD_MS_ULTRASONIC / 1000.0f;
    } else {
        sample_dt_s = (sample_ms - s_prev_pid_sample_ms) / 1000.0f;
    }
    s_prev_pid_sample_ms = sample_ms;

    const float error_mm = s_target_mm - pos;

    if (s_stalled) {
        stepper_set_rate(0.0f);
        s_rate_sps = 0.0f;
        return;
    }

    /* Do not hunt around the noisy ultrasonic setpoint. */
    if (fabsf(error_mm) <= CTRL_ULTRA_DEADBAND_MM) {
        stepper_set_rate(0.0f);
        s_rate_sps = 0.0f;
        s_stall_samples = 0;
        pid_reset(&s_pid);
        return;
    }

    /* PID output is directly interpreted as steps/s. */
    float desired_sps =
        CTRL_ULTRA_DIRECTION_SIGN *
        pid_update(&s_pid, s_target_mm, pos, sample_dt_s);

    if (desired_sps >  CTRL_ULTRA_MAX_SPS) desired_sps =  CTRL_ULTRA_MAX_SPS;
    if (desired_sps < -CTRL_ULTRA_MAX_SPS) desired_sps = -CTRL_ULTRA_MAX_SPS;

    /* Acceleration limit based on the measurement interval. */
    const float max_delta = CTRL_ULTRA_MAX_ACCEL_SPS2 * sample_dt_s;
    if (desired_sps - s_rate_sps >  max_delta)
        desired_sps = s_rate_sps + max_delta;
    if (desired_sps - s_rate_sps < -max_delta)
        desired_sps = s_rate_sps - max_delta;

    /*
     * Hard travel-envelope protection.
     *
     * CTRL_ULTRA_DIRECTION_SIGN maps step-rate sign to the direction of
     * increasing ultrasonic distance.  If the car is already at/beyond a
     * boundary, block only motion that would push farther outside.  Motion
     * back toward the legal region is still allowed.
     */
    const float sensor_direction = desired_sps * CTRL_ULTRA_DIRECTION_SIGN;

    if (pos <= s_limit_min_mm && sensor_direction < 0.0f) {
        desired_sps = 0.0f;
        pid_reset(&s_pid);
    }

    if (pos >= s_limit_max_mm && sensor_direction > 0.0f) {
        desired_sps = 0.0f;
        pid_reset(&s_pid);
    }

    s_rate_sps = desired_sps;
    stepper_set_rate(s_rate_sps);

    /*
     * DEMO BUILD:
     *
     * The ultrasonic-based software stall detector is intentionally disabled.
     *
     * Reason:
     *   The elevator's motor can move correctly with direct g<mm> commands, but
     *   the HC-SR04 position update can pause, quantize, or filter slowly around
     *   some heights.  The old detector interpreted "not enough ultrasonic
     *   position change over a few samples" as MOTOR STALLED, even when the
     *   motor subsystem itself was not stalled.
     *
     * Keep the hard travel-envelope protection above.  That still prevents the
     * controller from commanding farther outside 20..198 mm.
     */
    s_stall_samples  = 0;
    s_stall_start_mm = pos;
}

void control_emergency_stop(uint32_t cause) {
    stepper_emergency_stop();
    s_rate_sps = 0.0f;
    pid_reset(&s_pid);
    shared_state_raise_fault(cause);
}

void control_set_gains(float kp, float ki, float kd) {
    pid_init(&s_pid, kp, ki, kd,
             -CTRL_ULTRA_MAX_SPS,
              CTRL_ULTRA_MAX_SPS);
    s_rate_sps = 0.0f;
    s_stall_samples = 0;
}

void control_get_gains(float *kp, float *ki, float *kd) {
    if (kp) *kp = s_pid.kp;
    if (ki) *ki = s_pid.ki;
    if (kd) *kd = s_pid.kd;
}

bool control_set_limits(float min_mm, float max_mm) {
    if (!isfinite(min_mm) || !isfinite(max_mm)) return false;
    if (min_mm >= max_mm) return false;

    /* Never allow a software limit outside the sensor's valid envelope. */
    if (min_mm < (float)HCSR04_MIN_MM) return false;
    if (max_mm > (float)HCSR04_MAX_MM) return false;

    s_limit_min_mm = min_mm;
    s_limit_max_mm = max_mm;

    /* If a target already exists, immediately pull it back into the
     * newly-selected legal travel range.
     */
    if (s_target_valid) {
        if (s_target_mm < s_limit_min_mm) s_target_mm = s_limit_min_mm;
        if (s_target_mm > s_limit_max_mm) s_target_mm = s_limit_max_mm;
        pid_reset(&s_pid);
        s_rate_sps = 0.0f;
        s_stall_samples = 0;
    }

    return true;
}

void control_get_limits(float *min_mm, float *max_mm) {
    if (min_mm) *min_mm = s_limit_min_mm;
    if (max_mm) *max_mm = s_limit_max_mm;
}

void control_set_target_mm(float mm) {
    /* Clamp every Serial/floor-derived target to the active travel limits. */
    if (mm < s_limit_min_mm) mm = s_limit_min_mm;
    if (mm > s_limit_max_mm) mm = s_limit_max_mm;

    s_target_mm     = mm;
    s_target_valid  = true;
    s_stalled       = false;
    s_stall_samples = 0;
    pid_reset(&s_pid);
}

bool control_get_debug(float *target_mm, float *pos_mm, float *rate_sps) {
    bool  have;
    float pos;

    portENTER_CRITICAL(&s_ultra_mux);
    have = s_have_sample;
    pos  = s_filtered_mm;
    portEXIT_CRITICAL(&s_ultra_mux);

    if (target_mm) *target_mm = s_target_mm;
    if (pos_mm)    *pos_mm    = have ? pos : 0.0f;
    if (rate_sps)  *rate_sps  = s_rate_sps;

    return have;
}

bool control_hold_here(void) {
    float target, pos, rate;
    if (!control_get_debug(&target, &pos, &rate)) return false;
    (void)target;
    (void)rate;

    s_target_mm     = pos;
    s_target_valid  = true;
    s_stalled       = false;
    s_stall_samples = 0;
    s_rate_sps      = 0.0f;
    pid_reset(&s_pid);
    stepper_set_rate(0.0f);
    return true;
}

void control_clear_stall(void) {
    s_stalled       = false;
    s_stall_samples = 0;
    pid_reset(&s_pid);
}