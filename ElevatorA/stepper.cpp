/*
 * TMC2209 standalone STEP/DIR driver.
 * UART is intentionally not used.
 *
 * Microstep resolution is selected physically with MS1/MS2.
 * Motor current must be adjusted with the driver's VREF/current potentiometer.
 */

#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "esp_rom_sys.h"      /* esp_rom_delay_us() */
#include "board_config.h"
#include "stepper.h"

/* DIR must be stable before the next STEP edge. The TMC2209 datasheet asks
 * for 20 ns; 5 us is far more than needed and costs nothing, since this runs
 * only on a rate change and never inside the ISR. */
#ifndef STEP_DIR_SETUP_US
#define STEP_DIR_SETUP_US   5
#endif

/* STEP high time. The TMC2209 requires at least 100 ns; 1 us matches the
 * bench-proven tmc2209_motor_test and leaves an order of magnitude of
 * margin. Do not shrink this to save ISR time without an oscilloscope. */
#ifndef STEP_PULSE_US
#define STEP_PULSE_US       1
#endif

/* Timer producing the step pulse train. */
static hw_timer_t *s_timer = nullptr;

/* Protects rate + direction handoff to ISR. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* Phase accumulator increment. */
static volatile uint32_t s_increment = 0;

/* Current direction. */
static volatile bool s_dir_forward = true;

/* Net commanded microsteps since init. */
static volatile int32_t s_position = 0;

/* True after GPIO + timer setup succeeds. */
static bool s_ready = false;


/* -------------------------------------------------------------------------- */
/* STEP TIMER ISR                                                             */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR onStepTimer(void) {
    static uint32_t phase = 0;

    uint32_t inc = s_increment;
    if (inc == 0) return;

    uint32_t prev = phase;
    phase += inc;

    if (phase < prev) {
        /* One STEP pulse.
         *
         * The TMC2209 samples DIR on the rising STEP edge and needs the line
         * held high for at least 100 ns. The previous version used eight
         * nops, which at 240 MHz is about 33 ns -- roughly a third of the
         * requirement. The pulses were emitted and the position counter
         * advanced, so every software-visible signal said the motor was
         * being driven, but the driver never sampled a single edge and the
         * shaft did not move. A pulse too short to be seen is worse than no
         * pulse at all: it fails while looking exactly like success.
         *
         * One microsecond matches tmc2209_motor_test, which is bench-proven
         * to turn this motor. It costs 1 us of a 25 us ISR period, or 4% of
         * one core at the full 40 kHz tick -- affordable, and the price of
         * an edge the hardware actually registers. */
        GPIO.out_w1ts = (1U << PIN_STEPPER_STEP);
        esp_rom_delay_us(STEP_PULSE_US);
        GPIO.out_w1tc = (1U << PIN_STEPPER_STEP);

        s_position += s_dir_forward ? 1 : -1;
    }
}

static_assert(PIN_STEPPER_STEP < 32,
              "PIN_STEPPER_STEP must be below GPIO32 for direct GPIO access");


/* -------------------------------------------------------------------------- */
/* PUBLIC API                                                                 */
/* -------------------------------------------------------------------------- */

bool stepper_init(void) {
    s_increment   = 0;
    s_position    = 0;
    s_dir_forward = true;
    s_ready       = false;

    /* TMC2209 STEP/DIR pins. */
    pinMode(PIN_STEPPER_EN, OUTPUT);
    pinMode(PIN_STEPPER_STEP, OUTPUT);
    pinMode(PIN_STEPPER_DIR, OUTPUT);

    /* EN is active-low. Keep motor disabled during setup. */
    digitalWrite(PIN_STEPPER_EN, HIGH);
    digitalWrite(PIN_STEPPER_STEP, LOW);
    digitalWrite(PIN_STEPPER_DIR, HIGH);

    Serial.println(F("TMC2209: standalone STEP/DIR mode"));
    Serial.printf("TMC2209: configured microsteps = %d (must match MS1/MS2)\n",
                  TMC_MICROSTEPS);

#if ESP_ARDUINO_VERSION_MAJOR >= 3

    s_timer = timerBegin(STEP_TIMER_HZ);

    if (s_timer == nullptr) {
        Serial.println(F("TMC2209: step timer FAILED"));
        return false;
    }

    timerAttachInterrupt(s_timer, &onStepTimer);
    timerAlarm(s_timer, 1, true, 0);

#else

    /* Arduino-ESP32 2.x */
    s_timer = timerBegin(
        1,
        (uint16_t)(80000000UL / STEP_TIMER_HZ),
        true
    );

    if (s_timer == nullptr) {
        Serial.println(F("TMC2209: step timer FAILED"));
        return false;
    }

    timerAttachInterrupt(s_timer, &onStepTimer, true);
    timerAlarmWrite(s_timer, 1, true);
    timerAlarmEnable(s_timer);

#endif

    /* Enable driver. No UART acknowledgement is required. */
    digitalWrite(PIN_STEPPER_EN, LOW);

    s_ready = true;

    Serial.println(F("TMC2209: STEP/DIR INIT OK"));

    return true;
}


void stepper_enable(bool on) {
    digitalWrite(PIN_STEPPER_EN, on ? LOW : HIGH);
}


void stepper_set_rate(float steps_per_sec) {
    if (!s_ready) return;

    bool forward = (steps_per_sec >= 0.0f);
    float mag = fabsf(steps_per_sec);

    if (mag > STEP_MAX_SPS)
        mag = STEP_MAX_SPS;

    uint32_t inc =
        (mag < 1.0f)
            ? 0u
            : (uint32_t)(
                  (mag / (float)STEP_TIMER_HZ) *
                  4294967296.0f
              );

    /* Order matters, and all three steps are load-bearing.
     *
     * 1. Stop the pulse train first. Moving DIR while pulses are still going
     *    out violates the TMC2209's DIR setup time and costs one step at
     *    every reversal -- invisible in open loop, which is exactly the kind
     *    of error that is worth spending three lines to avoid.
     *
     * 2. Write DIR unconditionally, and outside the critical section. The
     *    previous version wrote the pin only when the requested direction
     *    differed from the cached s_dir_forward. That caches hardware state
     *    in a variable and then assumes the two can never disagree; once
     *    they do -- a reset leaving the pin somewhere unexpected, a pinMode
     *    elsewhere, any path that updates the variable without the pin --
     *    the condition is false forever and the direction can never be
     *    corrected again. The saving was one GPIO write per reversal. The
     *    cost was a state with no way back, which is what made one direction
     *    work and the other silently do nothing.
     *
     *    digitalWrite() is also not IRAM-resident, so calling it with
     *    interrupts masked was a second hazard independent of the first.
     *
     * 3. Only then hand the new rate to the ISR.
     */
    portENTER_CRITICAL(&s_mux);
    s_increment = 0;
    portEXIT_CRITICAL(&s_mux);

    digitalWrite(PIN_STEPPER_DIR, forward ? HIGH : LOW);
    esp_rom_delay_us(STEP_DIR_SETUP_US);

    portENTER_CRITICAL(&s_mux);
    s_dir_forward = forward;
    s_increment   = inc;
    portEXIT_CRITICAL(&s_mux);
}


int32_t stepper_get_position(void) {
    int32_t p;

    portENTER_CRITICAL(&s_mux);
    p = s_position;
    portEXIT_CRITICAL(&s_mux);

    return p;
}


void stepper_emergency_stop(void) {
    portENTER_CRITICAL(&s_mux);
    s_increment = 0;
    portEXIT_CRITICAL(&s_mux);

    /* Release motor coils. */
    digitalWrite(PIN_STEPPER_EN, HIGH);
}