/*
 * stepper.cpp
 * Board A TMC2209 STEP/DIR driver, adapted from the bench-verified
 * tmc2209_motor_test.ino that successfully runs the motor with f / b.
 *
 * Keeps the elevator project's public API unchanged:
 *   stepper_init()
 *   stepper_enable()
 *   stepper_set_rate()
 *   stepper_get_position()
 *   stepper_emergency_stop()
 *
 * Key differences from the current Kevin_bi repo stepper.cpp:
 *   - Configures TMC2209 over UART1 using TMCStepper.
 *   - Uses the same DDS pulse generator style as the working f/b test.
 *   - Uses esp_rom_delay_us(STEP_PULSE_US) inside the ISR, not CPU NOPs.
 *   - Stops the pulse train before changing DIR.
 *   - Writes DIR unconditionally on every rate update, outside the critical section.
 */

#include <Arduino.h>
#include <math.h>

#include <TMCStepper.h>

#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"

#include "board_config.h"
#include "stepper.h"

/* -------------------------------------------------------------------------- */
/* Local timing constants                                                      */
/* -------------------------------------------------------------------------- */

#ifndef STEP_PULSE_US
#define STEP_PULSE_US  1u   /* TMC2209 needs >= 100 ns; 1 us is safe. */
#endif

#ifndef DIR_SETUP_US
#define DIR_SETUP_US   5u   /* DIR stable before next STEP edge. */
#endif

/* -------------------------------------------------------------------------- */
/* Direct STEP pin register selection                                          */
/* -------------------------------------------------------------------------- */

#if PIN_STEPPER_STEP < 32
  #define STEP_SET_REG   GPIO_OUT_W1TS_REG
  #define STEP_CLR_REG   GPIO_OUT_W1TC_REG
  #define STEP_BIT       (1U << (PIN_STEPPER_STEP))
#else
  #define STEP_SET_REG   GPIO_OUT1_W1TS_REG
  #define STEP_CLR_REG   GPIO_OUT1_W1TC_REG
  #define STEP_BIT       (1U << ((PIN_STEPPER_STEP) - 32))
#endif

static_assert(PIN_STEPPER_STEP >= 0 && PIN_STEPPER_STEP <= 48,
              "PIN_STEPPER_STEP is outside the ESP32-S3 GPIO range");

/* -------------------------------------------------------------------------- */
/* TMC2209 UART driver                                                         */
/* -------------------------------------------------------------------------- */

static TMC2209Stepper s_driver(&Serial1, TMC_R_SENSE, TMC_UART_ADDR);

/* -------------------------------------------------------------------------- */
/* Pulse generator state                                                       */
/* -------------------------------------------------------------------------- */

static hw_timer_t *s_timer = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* DDS phase increment.  Frequency = STEP_TIMER_HZ * increment / 2^32. */
static volatile uint32_t s_increment = 0;

/* Direction used by the ISR when counting commanded steps. */
static volatile bool s_dir_forward = true;

/* -1 means continuous; 0 means no bounded move active; >0 counts down. */
static volatile int32_t s_steps_remaining = -1;

/* Net commanded microsteps since init.  This is NOT encoder feedback. */
static volatile int32_t s_position = 0;

static bool s_ready = false;
static bool s_driver_enabled = false;
static uint8_t s_tmc_version = 0x00;

/* -------------------------------------------------------------------------- */
/* STEP TIMER ISR                                                              */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR onStepTimer(void) {
    static uint32_t phase = 0;

    const uint32_t inc = s_increment;
    if (inc == 0u) {
        return;
    }

    const uint32_t prev = phase;
    phase += inc;

    if (phase >= prev) {
        return;  /* no overflow, no STEP pulse this tick */
    }

    /*
     * This follows the known-good f/b motor test:
     * raise STEP, hold it with esp_rom_delay_us(), then clear it.
     */
    REG_WRITE(STEP_SET_REG, STEP_BIT);
    esp_rom_delay_us(STEP_PULSE_US);
    REG_WRITE(STEP_CLR_REG, STEP_BIT);

    s_position += s_dir_forward ? 1 : -1;

    if (s_steps_remaining > 0) {
        if (--s_steps_remaining == 0) {
            s_increment = 0;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                            */
/* -------------------------------------------------------------------------- */

static uint32_t rate_to_increment(float steps_per_sec) {
    float mag = fabsf(steps_per_sec);

    if (mag > STEP_MAX_SPS) {
        mag = STEP_MAX_SPS;
    }

    if (mag < 1.0f) {
        return 0u;
    }

    return (uint32_t)((mag / (float)STEP_TIMER_HZ) * 4294967296.0f);
}

static void stop_pulse_train_only(void) {
    portENTER_CRITICAL(&s_mux);
    s_increment = 0u;
    portEXIT_CRITICAL(&s_mux);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

bool stepper_init(void) {
    s_increment       = 0u;
    s_position        = 0;
    s_dir_forward     = true;
    s_steps_remaining = -1;
    s_ready           = false;
    s_driver_enabled  = false;
    s_tmc_version     = 0x00;

    pinMode(PIN_STEPPER_EN, OUTPUT);
    pinMode(PIN_STEPPER_STEP, OUTPUT);
    pinMode(PIN_STEPPER_DIR, OUTPUT);

    /* TMC2209 EN is active-low.  Disable while configuring. */
    digitalWrite(PIN_STEPPER_EN, HIGH);
    digitalWrite(PIN_STEPPER_STEP, LOW);
    digitalWrite(PIN_STEPPER_DIR, HIGH);

    Serial.println(F("TMC2209: UART + STEP/DIR mode"));
    Serial.printf("TMC2209: STEP=%d DIR=%d EN=%d TX=%d RX=%d\n",
                  PIN_STEPPER_STEP,
                  PIN_STEPPER_DIR,
                  PIN_STEPPER_EN,
                  PIN_STEPPER_TX,
                  PIN_STEPPER_RX);

    /*
     * UART1 setup matches the working standalone motor test:
     * ESP32 TX -> PDN_UART through 1k, ESP32 RX <- PDN_UART.
     */
    Serial1.begin(TMC_UART_BAUD,
                  SERIAL_8N1,
                  PIN_STEPPER_RX,
                  PIN_STEPPER_TX);
    delay(50);

    s_driver.begin();
    s_driver.toff(5);
    s_driver.rms_current(TMC_RMS_CURRENT_MA);
    s_driver.microsteps(TMC_MICROSTEPS);
    s_driver.en_spreadCycle(false);   /* StealthChop */
    s_driver.pwm_autoscale(true);
    s_driver.blank_time(24);

    s_tmc_version = s_driver.version();

    Serial.printf("TMC2209: version 0x%02X %s\n",
                  s_tmc_version,
                  (s_tmc_version == TMC_VERSION_EXPECTED)
                      ? "(OK)"
                      : "(UART not confirmed)");

    if (s_tmc_version != TMC_VERSION_EXPECTED) {
        Serial.println(F("TMC2209: UART readback failed or unexpected."));
        Serial.println(F("TMC2209: STEP/DIR may still work if VREF/MS pins are set."));
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3

    s_timer = timerBegin(STEP_TIMER_HZ);

    if (s_timer == nullptr) {
        Serial.println(F("TMC2209: step timer FAILED"));
        return false;
    }

    timerAttachInterrupt(s_timer, &onStepTimer);
    timerAlarm(s_timer, 1, true, 0);

#else

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

    s_ready = true;

    stepper_enable(true);

    Serial.println(F("TMC2209: STEP/DIR INIT OK"));
    Serial.printf("TMC2209: timer %lu Hz, STEP high %u us, DIR setup %u us\n",
                  (unsigned long)STEP_TIMER_HZ,
                  (unsigned)STEP_PULSE_US,
                  (unsigned)DIR_SETUP_US);

    /*
     * Return true if the pulse generator is usable.
     * Do not fail the whole elevator solely because UART readback failed:
     * the bench test itself notes STEP/DIR can still work without UART
     * if hardware current/microstep settings are valid.
     */
    return true;
}

void stepper_enable(bool on) {
    s_driver_enabled = on;
    digitalWrite(PIN_STEPPER_EN, on ? LOW : HIGH);

    if (!on) {
        stop_pulse_train_only();
    }
}

void stepper_set_rate(float steps_per_sec) {
    if (!s_ready) {
        return;
    }

    const bool forward = (steps_per_sec >= 0.0f);
    const uint32_t inc = rate_to_increment(steps_per_sec);

    /*
     * Follow the working f/b test's order:
     *   1. stop pulses
     *   2. write DIR unconditionally, outside the critical section
     *   3. wait for DIR setup
     *   4. resume pulse train
     *
     * This also fixes the old cached-DIR failure mode where software believed
     * the direction was already correct but the physical DIR pin was not.
     */
    portENTER_CRITICAL(&s_mux);
    s_increment = 0u;
    portEXIT_CRITICAL(&s_mux);

    digitalWrite(PIN_STEPPER_DIR, forward ? LOW : HIGH);
    esp_rom_delay_us(DIR_SETUP_US);

    portENTER_CRITICAL(&s_mux);
    s_dir_forward = forward;
    s_steps_remaining = -1;   /* elevator control always commands continuous rate */
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
    s_increment = 0u;
    s_steps_remaining = 0;
    portEXIT_CRITICAL(&s_mux);

    /* Release motor coils. */
    digitalWrite(PIN_STEPPER_EN, HIGH);
    s_driver_enabled = false;
}