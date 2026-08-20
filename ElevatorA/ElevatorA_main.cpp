/*
 * ElevatorA_main_fixed.cpp
 *
 * Drop-in replacement for ElevatorA/ElevatorA_main.cpp on branch Kevin_bi.
 *
 * Key fixes:
 *  1) An authorized RFID card now produces a floor request.
 *     In this two-floor build, an authorized card toggles between floor 0 and 1.
 *
 *  2) A recovered system can move again after a transient LINK timeout.
 *     stepper_emergency_stop() disables TMC2209 EN, while stepper_set_rate()
 *     does not re-enable it.  Every NEW safe motion request now explicitly
 *     re-enables the driver.
 *
 *  3) Startup position recovery:
 *     - wait for valid ultrasonic position AND a healthy Board-B heartbeat;
 *     - if already at floor 0 or floor 1, stay there;
 *     - if between floor 0 and floor 1, automatically go to floor 0;
 *     - ignore passenger requests until startup recovery is complete.
 *
 * Existing project files are otherwise unchanged.
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "board_config.h"
#include "ultrasonic_control_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "link.h"
#include "stepper.h"
#include "hcsr04.h"
#include "rc522.h"
#include "lcd_ui.h"

/* -------------------------------------------------------------------------- */
/* Shared objects expected by the rest of the Board-A project                  */
/* -------------------------------------------------------------------------- */

QueueHandle_t     g_q_input;
QueueHandle_t     g_q_ui;
QueueHandle_t     g_q_log;
SemaphoreHandle_t g_i2c_mutex;

static SemaphoreHandle_t s_control_tick = nullptr;
static hw_timer_t        *s_control_timer = nullptr;

static volatile uint32_t s_drop_input = 0;
static volatile uint32_t s_drop_log   = 0;

/* Defined in tasks_ui.cpp. */
extern void lcdTask(void *arg);
extern void logTask(void *arg);
extern void ui_note_access(bool granted);

/* Defined in cmd.cpp. */
extern void cmdTask(void *arg);

/* -------------------------------------------------------------------------- */
/* Local configuration                                                         */
/* -------------------------------------------------------------------------- */

/* Current physical build has two usable floors.
 * Internal indices:
 *   floor 0 -> CTRL_FLOOR0_MM
 *   floor 1 -> CTRL_FLOOR1_MM
 * CTRL_FLOOR2_MM is disabled in ultrasonic_control_config.h.
 */
#define ACTIVE_FLOORS 2u

/* Authorized RC522 UID: 23 60 FB 27 */
static const uint8_t AUTHORIZED_RFID_UID[] = {
    0x23, 0x60, 0xFB, 0x27
};

/* Extra diagnostic log codes. */
#define LOG_CODE_LINK_FIRST          0x100u
#define LOG_CODE_LINK_REJECTED       0x101u
#define LOG_CODE_LINK_DROPPED        0x102u
#define LOG_CODE_LINK_MALFORMED      0x103u

#define LOG_CODE_REQUEST_ACCEPTED    0x110u
#define LOG_CODE_REQUEST_REJECTED    0x111u
#define LOG_CODE_BOOT_HOME_START     0x112u
#define LOG_CODE_BOOT_READY          0x113u
#define LOG_CODE_RFID_GRANTED        0x114u
#define LOG_CODE_RFID_DENIED         0x115u

#define ULTRASONIC_LOG_DELTA_MM      5
#define ULTRASONIC_NEAR_ECHO_WINDOW  3

/* -------------------------------------------------------------------------- */
/* Startup state                                                               */
/* -------------------------------------------------------------------------- */

typedef enum {
    BOOT_WAIT_FOR_POSITION = 0,
    BOOT_HOMING_TO_FLOOR0,
    BOOT_READY
} boot_phase_t;

/* Owned only by controlTask. */
static boot_phase_t s_boot_phase = BOOT_WAIT_FOR_POSITION;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static void log_post(task_id_t who, uint32_t code, int32_t value) {
    if (g_q_log == nullptr) return;

    log_msg_t m;
    m.timestamp_ms   = millis();
    m.source_task_id = (uint8_t)who;
    m.code           = code;
    m.value          = value;

    if (xQueueSend(g_q_log, &m, 0) != pdTRUE) {
        s_drop_log++;
    }
}

static bool rfid_is_authorized(const input_event_t &ev) {
    if (ev.uid_len != sizeof(AUTHORIZED_RFID_UID)) {
        return false;
    }

    return memcmp(ev.uid,
                  AUTHORIZED_RFID_UID,
                  sizeof(AUTHORIZED_RFID_UID)) == 0;
}

/*
 * Motion-blocking faults.
 *
 * RFID/LCD failures do NOT need to prevent the car from moving.
 * These faults do:
 *  - ultrasonic lost: no trustworthy position feedback
 *  - link timeout: Board B safety monitor is silent
 *  - fall detected: latched emergency
 *  - stepper stall: mechanical/drive problem
 *  - position limit: unsafe measured position
 */
static uint32_t motion_blocking_faults(void) {
    return ((uint32_t)FAULT_ULTRASONIC_LOST |
            (uint32_t)FAULT_LINK_TIMEOUT |
            (uint32_t)FAULT_FALL_DETECTED |
            (uint32_t)FAULT_STEPPER_STALL |
            (uint32_t)FAULT_POSITION_LIMIT);
}

/*
 * True only when a NEW motion is allowed to start.
 *
 * In addition to checking the fault flags, require a genuinely fresh
 * Board-B heartbeat.  This avoids allowing motion during the small startup
 * window before linkTask has had a chance to assert FAULT_LINK_TIMEOUT.
 */
static bool motion_can_start(void) {
    system_state_t st;
    shared_state_get(&st);

    if ((st.fault_flags & motion_blocking_faults()) != 0u) {
        return false;
    }

    if (st.last_heartbeat_ms == 0u) {
        return false;
    }

    if ((uint32_t)(millis() - st.last_heartbeat_ms) > LINK_TIMEOUT_MS) {
        return false;
    }

    return true;
}

/*
 * Submit a floor request AND explicitly re-arm the TMC2209.
 *
 * This fixes an important integration bug:
 * stepper_emergency_stop() sets EN HIGH, but stepper_set_rate() only changes
 * STEP/DIR timing and never lowers EN again.  A transient link timeout could
 * therefore leave the system looking healthy while the motor stayed disabled.
 */
static bool request_floor_safely(uint8_t floor, req_source_t source) {
    if (!motion_can_start()) {
        return false;
    }

    /*
     * Force the controller and STEP generator back to a zero-rate starting
     * point before installing a new target.  This matters after an emergency
     * stop: EN may have been HIGH while control_step() continued calculating
     * a non-zero rate internally.
     */
    if (!control_hold_here()) {
        return false;
    }

    if (!control_request_floor(floor, source)) {
        return false;
    }

    /* Re-arm after a previously recovered transient timeout. */
    stepper_set_rate(0.0f);
    stepper_enable(true);

    /*
     * Re-check after enabling in case the other core declared a safety fault
     * during the small check/request/enable window.
     */
    if (!motion_can_start()) {
        stepper_emergency_stop();
        return false;
    }

    return true;
}

static bool near_mm(float a, float b, float tol) {
    return fabsf(a - b) <= tol;
}

static bool between_floor0_and_floor1(float pos_mm) {
    const float f0 = CTRL_FLOOR0_MM;
    const float f1 = CTRL_FLOOR1_MM;

    const float lo = (f0 < f1) ? f0 : f1;
    const float hi = (f0 < f1) ? f1 : f0;

    return pos_mm > (lo + CTRL_ULTRA_DEADBAND_MM) &&
           pos_mm < (hi - CTRL_ULTRA_DEADBAND_MM);
}

/*
 * Startup recovery policy requested for this build.
 *
 * This is called AFTER control_step(), so the first valid ultrasonic sample
 * has already been published and is available through control_get_debug().
 */
static void service_startup_homing(void) {
    if (s_boot_phase == BOOT_READY) {
        return;
    }

    float target_mm = 0.0f;
    float pos_mm    = 0.0f;
    float rate_sps  = 0.0f;

    if (!control_get_debug(&target_mm, &pos_mm, &rate_sps)) {
        return; /* no valid ultrasonic position yet */
    }

    (void)target_mm;
    (void)rate_sps;

    /* Do not begin an automatic move until Board B is alive and safety is OK. */
    if (!motion_can_start()) {
        return;
    }

    const bool at_floor0 =
        near_mm(pos_mm, CTRL_FLOOR0_MM, CTRL_ULTRA_DEADBAND_MM);

    const bool at_floor1 =
        near_mm(pos_mm, CTRL_FLOOR1_MM, CTRL_ULTRA_DEADBAND_MM);

    if (s_boot_phase == BOOT_WAIT_FOR_POSITION) {
        /*
         * If already sitting at a known floor, accept that floor and do not
         * force an unnecessary startup movement.
         */
        if (at_floor0 || at_floor1) {
            s_boot_phase = BOOT_READY;
            log_post(TASK_ID_CONTROL,
                     LOG_CODE_BOOT_READY,
                     at_floor0 ? 0 : 1);
            return;
        }

        /*
         * Requested policy:
         * if power comes up while the car is in the uncertain region BETWEEN
         * floor 0 and floor 1, automatically recover to floor 0.
         */
        if (between_floor0_and_floor1(pos_mm)) {
            if (request_floor_safely(0u, REQ_SRC_CAR_BUTTON)) {
                /*
                 * current_floor defaults to 0 at boot, so control_request_floor
                 * would otherwise label this as IDLE even though the physical
                 * car is between floors.  Correct the UI state here.  We are
                 * inside controlTask, the state owner.
                 */
                system_state_t st;
                shared_state_get(&st);
                st.target_floor = 0u;
                st.direction    = ELEV_DIR_DOWN;
                st.mode         = ELEV_MODE_MOVING;
                shared_state_set(&st);

                s_boot_phase = BOOT_HOMING_TO_FLOOR0;
                log_post(TASK_ID_CONTROL,
                         LOG_CODE_BOOT_HOME_START,
                         (int32_t)lroundf(pos_mm));
            }
            return;
        }

        /*
         * Outside the 0<->1 corridor we do NOT guess a direction.
         * Leave the controller holding its first measured position and allow
         * normal requests.  This keeps the automatic behavior limited to the
         * exact uncertain zone requested.
         */
        s_boot_phase = BOOT_READY;
        log_post(TASK_ID_CONTROL,
                 LOG_CODE_BOOT_READY,
                 (int32_t)lroundf(pos_mm));
        return;
    }

    if (s_boot_phase == BOOT_HOMING_TO_FLOOR0) {
        if (at_floor0) {
            s_boot_phase = BOOT_READY;
            log_post(TASK_ID_CONTROL, LOG_CODE_BOOT_READY, 0);
        }
    }
}

/*
 * In the two-floor demo, an authorized card acts as a one-touch "other floor"
 * request:
 *   at floor 0 -> floor 1
 *   at floor 1 -> floor 0
 *
 * If you want the card to ALWAYS go to a fixed floor, replace this function
 * with "return 0;" or "return 1;".
 */
static uint8_t rfid_destination_floor(void) {
    system_state_t st;
    shared_state_get(&st);

    return (st.current_floor == 0u) ? 1u : 0u;
}

/* -------------------------------------------------------------------------- */
/* Hardware control timer                                                      */
/* -------------------------------------------------------------------------- */

static void IRAM_ATTR onControlTimer(void) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_control_tick, &woken);

    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* -------------------------------------------------------------------------- */
/* Core 1 tasks                                                                */
/* -------------------------------------------------------------------------- */

static void controlTask(void *arg) {
    (void)arg;

    input_event_t  ev;
    system_state_t st;
    uint32_t       last_ms = millis();

    for (;;) {
        if (xSemaphoreTake(s_control_tick,
                           pdMS_TO_TICKS(PERIOD_MS_CONTROL * 3)) != pdTRUE) {
            log_post(TASK_ID_CONTROL, FAULT_POSITION_LIMIT, -1);
            continue;
        }

        const uint32_t now = millis();
        const uint32_t dt  = now - last_ms;
        last_ms = now;

        /*
         * Startup homing has priority over passenger commands.
         * We still drain the queue so it cannot fill while homing.
         */
        while (xQueueReceive(g_q_input, &ev, 0) == pdTRUE) {
            if (s_boot_phase != BOOT_READY) {
                continue;
            }

            switch (ev.type) {
            case INPUT_EVT_CAR_BUTTON: {
                const bool ok =
                    request_floor_safely(ev.floor, REQ_SRC_CAR_BUTTON);

                log_post(TASK_ID_CONTROL,
                         ok ? LOG_CODE_REQUEST_ACCEPTED
                            : LOG_CODE_REQUEST_REJECTED,
                         (int32_t)ev.floor);
                break;
            }

            case INPUT_EVT_RFID_CARD: {
                const bool granted = rfid_is_authorized(ev);
                ui_note_access(granted);

                log_post(TASK_ID_RFID,
                         granted ? LOG_CODE_RFID_GRANTED
                                 : LOG_CODE_RFID_DENIED,
                         (int32_t)ev.uid_len);

                if (granted) {
                    const uint8_t destination = rfid_destination_floor();

                    const bool ok =
                        request_floor_safely(destination, REQ_SRC_RFID);

                    log_post(TASK_ID_CONTROL,
                             ok ? LOG_CODE_REQUEST_ACCEPTED
                                : LOG_CODE_REQUEST_REJECTED,
                             (int32_t)destination);
                }
                break;
            }

            default:
                break;
            }
        }

        control_step(dt);

        /*
         * After control_step() the latest valid ultrasonic sample has been
         * published.  This is therefore the right place to decide whether
         * startup homing is necessary.
         */
        service_startup_homing();

        shared_state_get(&st);

        ui_msg_t ui;
        ui.mode          = st.mode;
        ui.direction     = st.direction;
        ui.current_floor = st.current_floor;
        ui.target_floor  = st.target_floor;
        ui.fault_flags   = st.fault_flags;

        xQueueOverwrite(g_q_ui, &ui);
    }
}

static void ultrasonicTask(void *arg) {
    (void)arg;

    TickType_t last = xTaskGetTickCount();
    hcsr04_reading_t r;

    uint32_t consecutive_bad  = 0;
    uint32_t consecutive_good = 0;
    uint32_t last_timeout_count = hcsr04_get_timeout_count();

    /*
     * Preserve the Kevin_bi branch's near-echo suppressor.
     * The observed false echo is SHORTER than the real echo, so use the
     * maximum of a 3-sample rolling window.
     */
    uint16_t range_window[ULTRASONIC_NEAR_ECHO_WINDOW] = {0};
    uint8_t  range_index = 0;
    uint8_t  range_count = 0;

    uint16_t last_reported_mm = UINT16_MAX;

    for (;;) {
        /*
         * hcsr04_collect() has two different "false" cases in the current
         * driver:
         *   1) no completed ping is ready yet;
         *   2) a completed reading was outside the valid range.
         *
         * The old code counted BOTH as a missed echo.  That can create a
         * false RANGE SENSOR fault even while the sensor is basically fine.
         *
         * Use sentinels plus the driver's timeout counter so "nothing ready
         * this iteration" is not treated as a sensor failure.
         */
        r.valid       = false;
        r.distance_mm = UINT32_MAX;
        r.echo_us     = UINT32_MAX;

        const bool fresh_valid = hcsr04_collect(&r);

        const uint32_t timeout_count = hcsr04_get_timeout_count();
        const bool timed_out = (timeout_count != last_timeout_count);
        last_timeout_count = timeout_count;

        /*
         * If hcsr04_collect() consumed an out-of-range echo, the current
         * driver returns false but has already overwritten distance_mm.
         */
        const bool consumed_invalid =
            (!fresh_valid && r.distance_mm != UINT32_MAX);

        if (fresh_valid && r.valid) {
            consecutive_bad = 0;
            consecutive_good++;

            range_window[range_index] = r.distance_mm;
            range_index =
                (uint8_t)((range_index + 1u) % ULTRASONIC_NEAR_ECHO_WINDOW);

            if (range_count < ULTRASONIC_NEAR_ECHO_WINDOW) {
                range_count++;
            }

            if (range_count == ULTRASONIC_NEAR_ECHO_WINDOW) {
                uint16_t accepted_mm = range_window[0];

                for (uint8_t i = 1u;
                     i < ULTRASONIC_NEAR_ECHO_WINDOW;
                     ++i) {
                    if (range_window[i] > accepted_mm) {
                        accepted_mm = range_window[i];
                    }
                }

                control_update_ultrasonic(accepted_mm, millis());

                /*
                 * RANGE SENSOR is a recoverable fault.  Three consecutive
                 * valid echoes prove that the sensor is alive again.
                 */
                if (consecutive_good >= 3u) {
                    shared_state_clear_fault(FAULT_ULTRASONIC_LOST);
                }

                int32_t delta =
                    (int32_t)accepted_mm - (int32_t)last_reported_mm;
                if (delta < 0) delta = -delta;

                if (last_reported_mm == UINT16_MAX ||
                    delta >= ULTRASONIC_LOG_DELTA_MM) {
                    last_reported_mm = accepted_mm;
                    log_post(TASK_ID_ULTRASONIC,
                             0,
                             (int32_t)accepted_mm);
                }
            }
        } else if (timed_out || consumed_invalid) {
            /*
             * Count only an actual timeout or a completed-but-invalid
             * measurement.  Merely finding no result ready is harmless.
             */
            consecutive_good = 0;
            consecutive_bad++;

            if (consecutive_bad >= HCSR04_FAULT_AFTER_MISSES) {
                range_count = 0;
                range_index = 0;
                shared_state_raise_fault(FAULT_ULTRASONIC_LOST);
            }
        }

        hcsr04_trigger();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_ULTRASONIC));
    }
}

static void loadTask(void *arg) {
    (void)arg;

    uint32_t candidate   = 2;
    uint32_t found       = 0;
    uint32_t last_logged = 0xFFFFFFFFu;

    for (;;) {
        bool prime = (candidate >= 2u);

        for (uint32_t d = 2u;
             (uint64_t)d * (uint64_t)d <= candidate;
             ++d) {
            if ((candidate % d) == 0u) {
                prime = false;
                break;
            }
        }

        if (prime) found++;

        candidate++;
        if (candidate == 0u) candidate = 2u;

        taskYIELD();

        if (found != last_logged && (found & 0xFFFFu) == 0u) {
            last_logged = found;
            log_post(TASK_ID_LOAD, 0, (int32_t)found);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Core 0 tasks                                                                */
/* -------------------------------------------------------------------------- */

static void linkTask(void *arg) {
    (void)arg;

    TickType_t last = xTaskGetTickCount();
    link_frame_t f;
    system_state_t st;

    uint32_t accepted       = 0;
    uint32_t rejected       = 0;
    bool     seen_any       = false;
    bool     timeout_active = false;

    const uint32_t task_start_ms = millis();
    uint32_t next_summary_ms = 0;

    for (;;) {
        while (link_receive(&f)) {
            if (!link_frame_is_valid(&f)) {
                rejected++;
                continue;
            }

            accepted++;

            if (!seen_any) {
                seen_any = true;
                log_post(TASK_ID_LINK,
                         LOG_CODE_LINK_FIRST,
                         (int32_t)f.seq);
            }

            /*
             * Any valid frame is proof that Board B is alive.
             * shared_state_note_heartbeat() also clears FAULT_LINK_TIMEOUT.
             */
            shared_state_note_heartbeat(f.seq, millis());
            timeout_active = false;

            if (f.type == LINK_FRAME_STOP) {
                control_emergency_stop(FAULT_FALL_DETECTED);
                log_post(TASK_ID_LINK,
                         FAULT_FALL_DETECTED,
                         (int32_t)f.payload[0]);
            }
        }

        const uint32_t now = millis();
        shared_state_get(&st);

        bool timed_out;

        if (!seen_any) {
            /* Grace interval starting when linkTask itself begins. */
            timed_out =
                ((uint32_t)(now - task_start_ms) > LINK_TIMEOUT_MS);
        } else {
            timed_out =
                ((uint32_t)(now - st.last_heartbeat_ms) > LINK_TIMEOUT_MS);
        }

        if (timed_out) {
            /*
             * Stop only once per silence episode.
             * The previous code called emergency_stop every 20 ms.
             */
            if (!timeout_active) {
                timeout_active = true;
                control_emergency_stop(FAULT_LINK_TIMEOUT);
            }

            if ((int32_t)(now - next_summary_ms) >= 0) {
                next_summary_ms = now + 1000u;

                log_post(TASK_ID_LINK,
                         FAULT_LINK_TIMEOUT,
                         (int32_t)accepted);

                log_post(TASK_ID_LINK,
                         LOG_CODE_LINK_REJECTED,
                         (int32_t)rejected);

                log_post(TASK_ID_LINK,
                         LOG_CODE_LINK_DROPPED,
                         (int32_t)link_get_dropped_count());

                log_post(TASK_ID_LINK,
                         LOG_CODE_LINK_MALFORMED,
                         (int32_t)link_get_malformed_count());
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_LINK));
    }
}

static void inputTask(void *arg) {
    (void)arg;

    TickType_t last = xTaskGetTickCount();

    static const uint8_t pins[ACTIVE_FLOORS] = {
        PIN_BTN_CAR_0,
        PIN_BTN_CAR_1
    };

    static uint8_t stable[ACTIVE_FLOORS] = {0};

    for (;;) {
        for (uint8_t i = 0; i < ACTIVE_FLOORS; ++i) {
            const bool down = (digitalRead(pins[i]) == LOW);

            if (!down) {
                stable[i] = 0;
                continue;
            }

            if (stable[i] > BTN_DEBOUNCE_SAMPLES) {
                continue; /* already posted this held press */
            }

            stable[i]++;

            if (stable[i] == BTN_DEBOUNCE_SAMPLES) {
                input_event_t ev;
                memset(&ev, 0, sizeof(ev));

                ev.type         = INPUT_EVT_CAR_BUTTON;
                ev.floor        = i;
                ev.timestamp_ms = millis();

                if (xQueueSend(g_q_input, &ev, 0) != pdTRUE) {
                    s_drop_input++;
                }

                /* Latch until release. */
                stable[i]++;
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_INPUT));
    }
}

static void rfidTask(void *arg) {
    (void)arg;

    TickType_t last = xTaskGetTickCount();
    rc522_card_t card;

    for (;;) {
        if (rc522_poll(&card)) {
            input_event_t ev;
            memset(&ev, 0, sizeof(ev));

            ev.type         = INPUT_EVT_RFID_CARD;
            ev.uid_len      = card.uid_len;
            ev.timestamp_ms = millis();

            const uint8_t copy_len =
                (card.uid_len <= sizeof(ev.uid))
                    ? card.uid_len
                    : (uint8_t)sizeof(ev.uid);

            ev.uid_len = copy_len;
            memcpy(ev.uid, card.uid, copy_len);

            if (xQueueSend(g_q_input, &ev, 0) != pdTRUE) {
                s_drop_input++;
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_RFID));
    }
}

/* -------------------------------------------------------------------------- */
/* Setup / loop                                                                */
/* -------------------------------------------------------------------------- */

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(2000);

    Serial.println(F("Board A: elevator controller (fixed integration build)"));

    shared_state_init();

    g_i2c_mutex    = xSemaphoreCreateMutex();
    s_control_tick = xSemaphoreCreateBinary();

    if (g_i2c_mutex == nullptr || s_control_tick == nullptr) {
        Serial.println(F("FATAL: synchronization allocation failed."));
        for (;;) delay(1000);
    }

    pinMode(PIN_BTN_CAR_0, INPUT_PULLUP);
    pinMode(PIN_BTN_CAR_1, INPUT_PULLUP);
    /* PIN_BTN_CAR_2 is intentionally unused in this two-floor build. */

    /*
     * HC-SR04 interrupt setup BEFORE the high-frequency step timer.
     * Preserve the ordering already documented in the Kevin_bi branch.
     */
    Serial.println(F("[INIT] HC-SR04 begin"));
    const bool ok_hcsr04 = hcsr04_init();
    Serial.printf("[INIT] HC-SR04 done: %s\n",
                  ok_hcsr04 ? "OK" : "FAIL");

    Serial.println(F("[INIT] stepper begin"));
    const bool ok_stepper = stepper_init();
    Serial.printf("[INIT] stepper done: %s\n",
                  ok_stepper ? "OK" : "FAIL");

    Serial.println(F("[INIT] RC522 begin"));
    const bool ok_rc522 = rc522_init();
    Serial.printf("[INIT] RC522 done: %s\n",
                  ok_rc522 ? "OK" : "FAIL");

    Serial.println(F("[INIT] LCD begin"));
    const bool ok_lcd = lcd_ui_init();
    Serial.printf("[INIT] LCD done: %s\n",
                  ok_lcd ? "OK" : "FAIL");

    if (!ok_stepper) {
        shared_state_raise_fault(FAULT_STEPPER_STALL);
    }

    if (!ok_hcsr04) {
        shared_state_raise_fault(FAULT_ULTRASONIC_LOST);
    }

    if (!ok_rc522) {
        shared_state_raise_fault(FAULT_RFID_FAILURE);
    }

    if (!ok_lcd) {
        shared_state_raise_fault(FAULT_LCD_FAILURE);
    }

    Serial.println(F("--- floor map ---"));
    Serial.printf("  floor 0 = %.1f mm\n", (double)CTRL_FLOOR0_MM);
    Serial.printf("  floor 1 = %.1f mm\n", (double)CTRL_FLOOR1_MM);

    if (!link_init()) {
        Serial.println(F("FATAL: ESP-NOW would not start. Halting."));
        for (;;) delay(1000);
    }

    uint8_t own_mac[6];

    if (link_get_own_mac(own_mac)) {
        Serial.printf("--- link ---\n"
                      "  Board A STA MAC     "
                      "%02X:%02X:%02X:%02X:%02X:%02X\n",
                      own_mac[0], own_mac[1], own_mac[2],
                      own_mac[3], own_mac[4], own_mac[5]);

        Serial.printf("  wifi channel        %d\n",
                      LINK_WIFI_CHANNEL);
    }

    g_q_input = xQueueCreate(QDEPTH_INPUT_EVENT,
                             sizeof(input_event_t));

    g_q_ui = xQueueCreate(QDEPTH_UI_MSG,
                          sizeof(ui_msg_t));

    g_q_log = xQueueCreate(QDEPTH_LOG_MSG,
                           sizeof(log_msg_t));

    if (g_q_input == nullptr ||
        g_q_ui    == nullptr ||
        g_q_log   == nullptr) {
        Serial.println(F("FATAL: queue allocation failed."));
        for (;;) delay(1000);
    }

    if (!control_init()) {
        Serial.println(F("FATAL: control_init failed."));
        for (;;) delay(1000);
    }

    /* Core 1: deterministic/control side. */
    xTaskCreatePinnedToCore(controlTask,
                            "control",
                            STACK_CONTROL,
                            nullptr,
                            PRIO_CONTROL,
                            nullptr,
                            CORE_CONTROL);

    xTaskCreatePinnedToCore(ultrasonicTask,
                            "ultra",
                            STACK_ULTRASONIC,
                            nullptr,
                            PRIO_ULTRASONIC,
                            nullptr,
                            CORE_ULTRASONIC);

    xTaskCreatePinnedToCore(loadTask,
                            "load",
                            STACK_LOAD,
                            nullptr,
                            PRIO_LOAD,
                            nullptr,
                            CORE_LOAD);

    /* Core 0: radio / UI / input side. */
    xTaskCreatePinnedToCore(linkTask,
                            "link",
                            STACK_LINK,
                            nullptr,
                            PRIO_LINK,
                            nullptr,
                            CORE_LINK);

    xTaskCreatePinnedToCore(inputTask,
                            "input",
                            STACK_INPUT,
                            nullptr,
                            PRIO_INPUT,
                            nullptr,
                            CORE_INPUT);

    xTaskCreatePinnedToCore(rfidTask,
                            "rfid",
                            STACK_RFID,
                            nullptr,
                            PRIO_RFID,
                            nullptr,
                            CORE_RFID);

    xTaskCreatePinnedToCore(lcdTask,
                            "lcd",
                            STACK_LCD,
                            nullptr,
                            PRIO_LCD,
                            nullptr,
                            CORE_LCD);

    xTaskCreatePinnedToCore(logTask,
                            "log",
                            STACK_LOG,
                            nullptr,
                            PRIO_LOG,
                            nullptr,
                            CORE_LOG);

    xTaskCreatePinnedToCore(cmdTask,
                            "cmd",
                            STACK_CMD,
                            nullptr,
                            PRIO_CMD,
                            nullptr,
                            CORE_CMD);

    /*
     * Start the control timer LAST so controlTask exists before its first
     * semaphore release.
     */
#if ESP_ARDUINO_VERSION_MAJOR >= 3

    s_control_timer = timerBegin(1000000); /* 1 MHz */

    if (s_control_timer == nullptr) {
        Serial.println(F("FATAL: control timer allocation failed."));
        for (;;) delay(1000);
    }

    timerAttachInterrupt(s_control_timer, &onControlTimer);
    timerAlarm(s_control_timer,
               PERIOD_MS_CONTROL * 1000,
               true,
               0);

#else

    s_control_timer = timerBegin(0, 80, true);

    if (s_control_timer == nullptr) {
        Serial.println(F("FATAL: control timer allocation failed."));
        for (;;) delay(1000);
    }

    timerAttachInterrupt(s_control_timer,
                         &onControlTimer,
                         true);

    timerAlarmWrite(s_control_timer,
                    PERIOD_MS_CONTROL * 1000,
                    true);

    timerAlarmEnable(s_control_timer);

#endif

    Serial.println(F("Board A running."));
    Serial.println(F("Startup policy: between floor 0/1 -> auto-home to floor 0."));
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}