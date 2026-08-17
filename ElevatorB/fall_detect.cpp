/*
 * ============================================================================
 * FILE: fall_detect.cpp
 *
 * PURPOSE:
 *    Fall detection policy implementation.
 *
 *    A three-stage rule: free fall, then impact, then re-arm. Every
 *    threshold comes from recorded bench data; see the FALL DETECTION
 *    section of board_config.h for the measured distributions the values
 *    were read off.
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
 *    - board_config.h: thresholds and scale factors
 *    - fall_detect.h, mpu6050.h
 *
 * NOTES:
 *    WHY FREE FALL FIRST AND NOT THE IMPACT
 *    An impact peak alone is not evidence of a fall: setting the rig down
 *    firmly produces one. What no ordinary handling produced, in 16
 *    recorded attempts, is a sustained collapse of the acceleration
 *    magnitude toward zero. Free fall is the discriminating feature; the
 *    impact is the confirmation that the fall ended on the floor rather
 *    than in someone's hand.
 *
 *    ORDER MATTERS. Requiring the stages in sequence is what rejects a
 *    shake, which produces low and high magnitudes but not in that order.
 *
 *    The detector is fed one sample at a time and keeps no history buffer,
 *    so its cost is constant and its state is inspectable.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "fall_detect.h"

/* Where the detector is in the sequence. */
typedef enum {
    PHASE_ARMED = 0,      // waiting for magnitude to collapse
    PHASE_FREEFALL,       // below threshold, timing how long
    PHASE_AWAIT_IMPACT,   // free fall ended, waiting for the landing
    PHASE_CONFIRMED       // fall declared, holding until re-armed
} phase_t;

static phase_t  s_phase;
static uint32_t s_freefall_start_ms;   // when the collapse began
static uint32_t s_freefall_end_ms;     // when it ended
static uint32_t s_confirmed_ms;        // when the fall was declared

/* Published state, mirrored so the caller sees a stable value. */
static fall_state_t s_state;

/*
 * ============================================================================
 * FUNCTION: sample_magnitude_g
 *
 * PURPOSE:
 *    Vector magnitude of the acceleration, in g.
 *
 * PARAMETERS:
 *    s (const imu_sample_t*) - raw sample
 *
 * RETURN VALUE:
 *    float - magnitude in g. At rest this sits near 1.0; in free fall it
 *            collapses toward 0; on impact it spikes.
 *
 * CALLED FROM:
 *    fall_detect_update(), detectTask, Core 1.
 *
 * NOTE:
 *    Individual axes clip at the configured full scale, so a landing peak
 *    is a LOWER BOUND on the true magnitude. That is acceptable here: the
 *    impact test only asks whether the peak is large, and a clipped large
 *    value is still large.
 * ============================================================================
 */
static float sample_magnitude_g(const imu_sample_t *s) {
    float x = (float)s->accel_x / MPU_ACCEL_LSB_PER_G;
    float y = (float)s->accel_y / MPU_ACCEL_LSB_PER_G;
    float z = (float)s->accel_z / MPU_ACCEL_LSB_PER_G;
    return sqrtf(x * x + y * y + z * z);
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

void fall_detect_init(void) {
    s_phase             = PHASE_ARMED;
    s_state             = FALL_STATE_NORMAL;
    s_freefall_start_ms = 0;
    s_freefall_end_ms   = 0;
    s_confirmed_ms      = 0;
}

fall_state_t fall_detect_update(const imu_sample_t *s) {
    if (s == NULL) return s_state;

    const float    mag = sample_magnitude_g(s);
    const uint32_t now = s->timestamp_ms;

    switch (s_phase) {

    case PHASE_ARMED:
        if (mag < FALL_FREEFALL_G) {
            s_phase             = PHASE_FREEFALL;
            s_freefall_start_ms = now;
            s_state             = FALL_STATE_SUSPECT;
        }
        break;

    case PHASE_FREEFALL:
        if (mag < FALL_FREEFALL_G) {
            /* Still falling. Nothing to decide yet: the duration test is
             * applied when the collapse ends, not while it continues. */
            break;
        }
        /* The collapse ended. Was it long enough to be a fall rather than a
         * momentary dip from a jolt? */
        if ((now - s_freefall_start_ms) >= FALL_FREEFALL_MS) {
            s_phase           = PHASE_AWAIT_IMPACT;
            s_freefall_end_ms = now;
        } else {
            s_phase = PHASE_ARMED;
            s_state = FALL_STATE_NORMAL;
        }
        break;

    case PHASE_AWAIT_IMPACT:
        if (mag >= FALL_IMPACT_G) {
            s_phase        = PHASE_CONFIRMED;
            s_state        = FALL_STATE_CONFIRMED;
            s_confirmed_ms = now;
            break;
        }
        /* No landing within the window. Something was briefly weightless
         * and then was not -- a lift, a toss that was caught. Not a fall. */
        if ((now - s_freefall_end_ms) > FALL_IMPACT_WINDOW_MS) {
            s_phase = PHASE_ARMED;
            s_state = FALL_STATE_NORMAL;
        }
        break;

    case PHASE_CONFIRMED:
        /* Hold. A drop that bounces produces a second free-fall episode
         * milliseconds later, and reporting it as a second fall would
         * double-count one event. Only fall_detect_clear(), or the re-arm
         * timeout, leaves this state. */
        if ((now - s_confirmed_ms) > FALL_REARM_MS) {
            s_phase = PHASE_ARMED;
            s_state = FALL_STATE_NORMAL;
        }
        break;
    }

    return s_state;
}

void fall_detect_clear(void) {
    s_phase = PHASE_ARMED;
    s_state = FALL_STATE_NORMAL;
}
