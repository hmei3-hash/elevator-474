/*
 * ============================================================================
 * FILE: hcsr04.cpp
 *
 * PURPOSE:
 *    Interrupt-driven ultrasonic rangefinder driver implementation.
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
 *    - esp_timer.h: one-shot timeout timer
 *    - board_config.h: pin assignments and timing
 *    - hcsr04.h
 *
 * NOTES:
 *    HOW IT WORKS
 *      1. hcsr04_trigger() emits the 10 us trigger pulse and arms a
 *         one-shot esp_timer for HCSR04_TIMEOUT_US.
 *      2. The echo pin interrupt fires on both edges. The rising edge
 *         stamps a start time; the falling edge computes the width and
 *         publishes a completed reading.
 *      3. If the timeout fires first, the ping is abandoned and counted.
 *
 *    WHY esp_timer_get_time() AND NOT micros()
 *    Both return microseconds, but esp_timer_get_time() is a 64-bit counter
 *    that is safe to call from an ISR and does not wrap in any timescale
 *    this project cares about. micros() wraps every ~71 minutes, which is
 *    inside a demo session.
 *
 *    THE ARMED FLAG IS NOT OPTIONAL
 *    An echo from an abandoned ping can arrive after the timeout, and a
 *    reflection off a far surface can arrive after the next trigger. Both
 *    would otherwise be timed against the wrong start and produce a
 *    plausible, wrong distance. Every edge is ignored unless a ping is
 *    outstanding.
 *
 *    ISR CONSTRAINTS
 *    Both handlers below are IRAM_ATTR and touch only volatiles. No
 *    printing, no allocation, no locks, no floating point.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <esp_timer.h>
#include "board_config.h"
#include "hcsr04.h"

/* Timeout timer for the outstanding ping. */
static esp_timer_handle_t s_timeout_timer;

/* Guards the handoff between the two ISRs and the task. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* True between trigger and completion. Edges arriving outside this window
 * belong to an abandoned ping and are discarded. */
static volatile bool s_armed;

/* Timestamp of the rising edge, microseconds. Zero until it arrives. */
static volatile int64_t s_echo_start_us;

/* Completed measurement waiting to be collected. */
static volatile uint32_t s_result_us;
static volatile bool     s_result_ready;

/* Pings that expired without a complete echo. */
static volatile uint32_t s_timeouts;

static bool s_inited;

/*
 * ============================================================================
 * FUNCTION: hcsr04_echo_isr
 *
 * PURPOSE:
 *    Time the echo pulse. Stamps the rising edge, and on the falling edge
 *    publishes the width.
 *
 * PARAMETERS:
 *    None
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    GPIO interrupt on both edges of PIN_HCSR04_ECHO. Not a task.
 * ============================================================================
 */
static void IRAM_ATTR hcsr04_echo_isr(void) {
    if (!s_armed) return;                       /* stale edge, ignore */

    int64_t now = esp_timer_get_time();
    bool level  = (GPIO.in >> PIN_HCSR04_ECHO) & 0x1;

    if (level) {
        /* Rising: the burst has left. Only the first rising edge of a ping
         * counts; a second one means noise, and keeping the first keeps the
         * measurement conservative. */
        if (s_echo_start_us == 0) s_echo_start_us = now;
        return;
    }

    /* Falling: the echo returned. */
    if (s_echo_start_us == 0) return;           /* fall without a rise */

    s_result_us    = (uint32_t)(now - s_echo_start_us);
    s_result_ready = true;
    s_armed        = false;
}

/* The ISR reads the echo level from the low GPIO input register, which
 * only covers GPIO0-31. Catch a future pin move at compile time. */
static_assert(PIN_HCSR04_ECHO < 32,
              "PIN_HCSR04_ECHO must be below 32 for the GPIO.in read");

/*
 * ============================================================================
 * FUNCTION: hcsr04_timeout_cb
 *
 * PURPOSE:
 *    Abandon a ping whose echo never arrived.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    esp_timer dispatch, a high-priority system task. Not an application
 *    task, and not a hardware ISR: it may not block, but it is not subject
 *    to the strict IRAM rules the echo handler is.
 * ============================================================================
 */
static void hcsr04_timeout_cb(void *arg) {
    (void)arg;
    portENTER_CRITICAL(&s_mux);
    if (s_armed) {
        s_armed = false;
        s_timeouts++;
    }
    portEXIT_CRITICAL(&s_mux);
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool hcsr04_init(void) {
    s_inited       = false;
    s_armed        = false;
    s_result_ready = false;
    s_timeouts     = 0;

    pinMode(PIN_HCSR04_TRIG, OUTPUT);
    digitalWrite(PIN_HCSR04_TRIG, LOW);
    pinMode(PIN_HCSR04_ECHO, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_HCSR04_ECHO),
                    hcsr04_echo_isr, CHANGE);

    const esp_timer_create_args_t args = {
        .callback = &hcsr04_timeout_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "hcsr04_to",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&args, &s_timeout_timer) != ESP_OK) return false;

    s_inited = true;
    return true;
}

bool hcsr04_trigger(void) {
    if (!s_inited) return false;

    /* Abandon anything outstanding before re-arming, so a late echo from
     * the previous ping cannot be timed against this one's start. */
    esp_timer_stop(s_timeout_timer);

    portENTER_CRITICAL(&s_mux);
    s_armed         = false;      /* disarm across the trigger pulse itself */
    s_echo_start_us = 0;
    portEXIT_CRITICAL(&s_mux);

    /* Trigger pulse. Short enough to sit inside a critical section is
     * tempting, but 10 us of masked interrupts on the core running the
     * control loop is not worth saving; the disarm above already makes the
     * window safe. */
    digitalWrite(PIN_HCSR04_TRIG, LOW);
    delayMicroseconds(HCSR04_SETTLE_US);
    digitalWrite(PIN_HCSR04_TRIG, HIGH);
    delayMicroseconds(HCSR04_TRIG_US);
    digitalWrite(PIN_HCSR04_TRIG, LOW);

    portENTER_CRITICAL(&s_mux);
    s_armed = true;
    portEXIT_CRITICAL(&s_mux);

    esp_timer_start_once(s_timeout_timer, HCSR04_TIMEOUT_US);
    return true;
}

bool hcsr04_collect(hcsr04_reading_t *out) {
    if (out == NULL || !s_inited) return false;

    uint32_t us;
    portENTER_CRITICAL(&s_mux);
    if (!s_result_ready) {
        portEXIT_CRITICAL(&s_mux);
        return false;
    }
    us = s_result_us;
    s_result_ready = false;
    portEXIT_CRITICAL(&s_mux);

    out->echo_us = us;

    /* Derived from the speed of sound; see the note in board_config.h for
     * why this is not fitted. */
    float mm = ((float)us - HCSR04_US_INTERCEPT) / HCSR04_US_PER_MM;
    if (mm < 0.0f) mm = 0.0f;
    out->distance_mm = (uint32_t)mm;

    /* Reject readings the part cannot physically produce. Below the blind
     * zone the module reports the ring-down of its own transmit burst, not
     * a target; above the range limit it reports whatever noise arrived
     * before the timeout. Both look like ordinary numbers. */
    if (out->distance_mm < HCSR04_MIN_MM || out->distance_mm > HCSR04_MAX_MM) {
        out->valid = false;
        return false;
    }

    out->valid = true;
    return true;
}

uint32_t hcsr04_get_timeout_count(void) {
    return s_timeouts;
}
