/*
 * TMC2209 standalone STEP/DIR driver.
 * UART is intentionally not used.
 *
 * Microstep resolution is selected physically with MS1/MS2.
 * Motor current must be adjusted with the driver's VREF/current potentiometer.
 */

#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "board_config.h"
#include "stepper.h"

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
        /* One STEP pulse. */
        GPIO.out_w1ts = (1U << PIN_STEPPER_STEP);

        /* TMC2209 minimum STEP high time is tiny; a few CPU nops are enough. */
        __asm__ __volatile__(
            "nop; nop; nop; nop; nop; nop; nop; nop;"
        );

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

    portENTER_CRITICAL(&s_mux);

    if (forward != s_dir_forward) {
        s_dir_forward = forward;

        digitalWrite(
            PIN_STEPPER_DIR,
            forward ? HIGH : LOW
        );
    }

    s_increment = inc;

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