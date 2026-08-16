/*
 * ============================================================================
 * FILE: stepper.cpp
 *
 * PURPOSE:
 *    Step/direction stepper driver implementation.
 *
 *    Integrated from the closed-loop bench prototype, 08/16/2026. The phase
 *    accumulator, the ISR pulse shape and the driver configuration sequence
 *    are carried over unchanged; only the pins moved, to match
 *    board_config.h, which is authoritative.
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
 *    - TMCStepper.h: "TMCStepper by teemuatlut", library manager
 *    - soc/gpio_struct.h: direct register access for the ISR
 *    - board_config.h: pin assignments and driver settings
 *    - stepper.h
 *
 * NOTES:
 *    Requires arduino-esp32 core 3.x for the timerBegin/timerAlarm API.
 *
 *    WHY A PHASE ACCUMULATOR
 *    A fixed-rate ISR adds a 32-bit increment to a phase register and emits
 *    one pulse per overflow. Output frequency is increment * ISR_RATE / 2^32,
 *    which is settable to a fraction of a hertz without ever reprogramming
 *    the timer. Reprogramming a timer period on every rate change instead
 *    would glitch the pulse train exactly during acceleration.
 *
 *    ISR CONSTRAINTS
 *    onStepTimer runs at STEP_TIMER_HZ and must stay a handful of
 *    instructions. It touches GPIO registers directly rather than calling
 *    digitalWrite(), and it must never log, allocate, or take a mutex.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include "soc/gpio_struct.h"
#include "board_config.h"
#include "stepper.h"

/* Driver handle. UART1 is dedicated to it. */
static TMC2209Stepper s_driver(&Serial1, TMC_R_SENSE, TMC_UART_ADDR);

/* Timer producing the step pulse train. */
static hw_timer_t *s_timer = nullptr;

/* Guards the handoff of rate and direction into the ISR. The two fields
 * must change together: a direction flipped without its matching rate would
 * count pulses the wrong way for one ISR tick. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* Phase increment. Output frequency = s_increment * STEP_TIMER_HZ / 2^32. */
static volatile uint32_t s_increment;

/* Direction currently applied to the DIR pin. */
static volatile bool s_dir_forward = true;

/* Net commanded steps since init. */
static volatile int32_t s_position;

/* Set once init has succeeded; the ISR and the public API check it. */
static bool s_ready;

/*
 * ============================================================================
 * FUNCTION: onStepTimer
 *
 * PURPOSE:
 *    Advance the phase accumulator and emit one STEP pulse per overflow.
 *
 * PARAMETERS:
 *    None
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    Hardware timer interrupt at STEP_TIMER_HZ. Not a task. Must not block,
 *    log, allocate, or take a mutex.
 * ============================================================================
 */
static void IRAM_ATTR onStepTimer(void) {
    static uint32_t phase = 0;

    uint32_t inc = s_increment;
    if (inc == 0) return;

    uint32_t prev = phase;
    phase += inc;
    if (phase < prev) {                     /* wrapped: one pulse due */
        GPIO.out_w1ts = (1U << PIN_STEPPER_STEP);
        /* The driver needs a minimum high time. A few nops are cheaper and
         * far more predictable here than delayMicroseconds(). */
        __asm__ __volatile__("nop; nop; nop; nop; nop; nop; nop; nop;");
        GPIO.out_w1tc = (1U << PIN_STEPPER_STEP);
        s_position += s_dir_forward ? 1 : -1;
    }
}

/* The ISR writes STEP through the low GPIO register, which only covers
 * GPIO0-31. Catch a future pin move at compile time rather than with a
 * motor that silently never turns. */
static_assert(PIN_STEPPER_STEP < 32,
              "PIN_STEPPER_STEP must be below 32 for GPIO.out_w1ts");

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool stepper_init(void) {
    s_increment   = 0;
    s_position    = 0;
    s_dir_forward = true;
    s_ready       = false;

    pinMode(PIN_STEPPER_EN, OUTPUT);
    digitalWrite(PIN_STEPPER_EN, HIGH);          /* EN is active low: off */
    pinMode(PIN_STEPPER_STEP, OUTPUT);
    digitalWrite(PIN_STEPPER_STEP, LOW);
    pinMode(PIN_STEPPER_DIR, OUTPUT);
    digitalWrite(PIN_STEPPER_DIR, HIGH);

    Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_STEPPER_RX, PIN_STEPPER_TX);

    s_driver.begin();
    s_driver.toff(5);
    s_driver.rms_current(TMC_RMS_CURRENT_MA);
    s_driver.microsteps(TMC_MICROSTEPS);
    s_driver.en_spreadCycle(false);              /* StealthChop, quiet */
    s_driver.pwm_autoscale(true);
    s_driver.blank_time(24);

    /* Reading the version back is the only proof the UART link works. Every
     * write above is fire-and-forget; without this check a disconnected
     * PDN_UART wire looks exactly like a successful configuration. */
    if (s_driver.version() != TMC_VERSION_EXPECTED) {
        return false;
    }

    s_timer = timerBegin(STEP_TIMER_HZ);
    if (s_timer == nullptr) return false;
    timerAttachInterrupt(s_timer, &onStepTimer);
    timerAlarm(s_timer, 1, true, 0);

    digitalWrite(PIN_STEPPER_EN, LOW);           /* energise */
    s_ready = true;
    return true;
}

void stepper_enable(bool on) {
    digitalWrite(PIN_STEPPER_EN, on ? LOW : HIGH);   /* active low */
}

void stepper_set_rate(float steps_per_sec) {
    if (!s_ready) return;

    bool  forward = (steps_per_sec >= 0.0f);
    float mag     = fabsf(steps_per_sec);
    if (mag > STEP_MAX_SPS) mag = STEP_MAX_SPS;

    /* Below one step per second the increment rounds to zero anyway; make
     * that explicit so the pulse train stops cleanly instead of emitting a
     * pulse every few seconds. */
    uint32_t inc = (mag < 1.0f)
                 ? 0u
                 : (uint32_t)((mag / (float)STEP_TIMER_HZ) * 4294967296.0f);

    portENTER_CRITICAL(&s_mux);
    if (forward != s_dir_forward) {
        s_dir_forward = forward;
        digitalWrite(PIN_STEPPER_DIR, forward ? HIGH : LOW);
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
    digitalWrite(PIN_STEPPER_EN, HIGH);          /* release the coils */
}
