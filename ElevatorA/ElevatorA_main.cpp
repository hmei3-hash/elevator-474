/*
 * ============================================================================
 * FILE: ElevatorA_main.cpp
 *
 * PURPOSE:
 *    Board A entry point. Assembly only: initialise drivers, create queues
 *    and synchronisation objects, create tasks with their configured
 *    priorities and core affinities, then hand control to the scheduler.
 *    No policy and no device access here.
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
 * BUILD:
 *    PlatformIO, environment boardA. Renamed from ElevatorA.ino because PlatformIO
 *    compiles .cpp directly; the Arduino IDE's .ino preprocessing (which
 *    auto-generates forward declarations) is not involved, so every
 *    function here is defined before it is used.
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *    - board_config.h, app_types.h
 *    - shared_state.h, control.h, link.h
 *    - stepper.h, hcsr04.h, rc522.h, lcd_ui.h
 *
 * NOTES:
 *    Core split is load-bearing. The Arduino-ESP32 WiFi task is pinned to
 *    Core 0 at priority 23, so controlTask must stay on Core 1 to keep its
 *    100 Hz period deterministic. Do not move tasks between cores without
 *    re-measuring jitter.
 *
 *    ARDUINO CORE VERSION
 *    The hardware timer API changed incompatibly between arduino-esp32 2.x
 *    and 3.x. Both spellings are kept behind ESP_ARDUINO_VERSION_MAJOR so
 *    the same source builds under either, because the Arduino IDE and
 *    PlatformIO on this project resolved to different cores.
 *
 *    THE CONTROL LOOP IS RELEASED BY A HARDWARE TIMER, NOT BY THE TICK.
 *    A hardware timer ISR gives a binary semaphore at 100 Hz and
 *    controlTask blocks on it. vTaskDelayUntil would quantise the period to
 *    the FreeRTOS tick instead, which puts jitter directly into the
 *    controller's dt -- and dt divides the derivative term.
 *
 *    The AS5600 encoder has been removed from the control architecture.
 *    HC-SR04 distance is the primary closed-loop position feedback.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "control.h"
#include "link.h"
#include "stepper.h"
#include "hcsr04.h"
#include "rc522.h"
#include "lcd_ui.h"

/* ========================================================================== */
/*                        SECTION: SHARED OBJECTS                             */
/*   Defined here, declared extern wherever else they are needed.             */
/* ========================================================================== */

QueueHandle_t     g_q_input;
QueueHandle_t     g_q_ui;
QueueHandle_t     g_q_log;
SemaphoreHandle_t g_i2c_mutex;

/* Released by the control timer ISR, taken by controlTask. */
static SemaphoreHandle_t s_control_tick;

/* The timer that releases it. */
static hw_timer_t *s_control_timer;

/* Messages dropped because a queue was full. Non-zero means a queue is
 * undersized for the observed burst, not that anything is broken. */
static volatile uint32_t s_drop_input;
static volatile uint32_t s_drop_log;

/* -------------------------------------------------------------------------- */
/* RFID ACCESS POLICY                                                         */
/* -------------------------------------------------------------------------- */

/*
 * Authorized RC522 card UID.
 *
 * Card ID supplied:
 *     0x2360FB27
 *
 * Compare byte-by-byte in the same order returned by the MFRC522:
 *     23 60 FB 27
 */
static const uint8_t AUTHORIZED_RFID_UID[] = {
    0x23, 0x60, 0xFB, 0x27
};

static bool rfid_is_authorized(const input_event_t &ev) {
    if (ev.uid_len != sizeof(AUTHORIZED_RFID_UID)) {
        return false;
    }

    return memcmp(
        ev.uid,
        AUTHORIZED_RFID_UID,
        sizeof(AUTHORIZED_RFID_UID)
    ) == 0;
}

/* -------------------------------------------------------------------------- */
/* AUTHORIZATION WINDOW                                                       */
/* -------------------------------------------------------------------------- */

/*
 * How long an accepted card keeps the car buttons live.
 *
 * Before this existed, rfid_is_authorized() only drove the display: every
 * car button was acted on whether a card had been presented or not, so the
 * reader was decorative. The window is what makes the access control real.
 */
#define ACCESS_WINDOW_MS   20000u

/* millis() at which the current authorization expires. Written and read only
 * by controlTask, so no lock is needed -- keep it that way. */
static uint32_t s_access_until_ms = 0;

/* True once a card has ever been accepted. Distinguishes "expired" from
 * "never authorised" for the log, which are different operator mistakes. */
static bool s_access_ever_granted = false;

/*
 * PURPOSE:  Report whether the authorization window is still open.
 * PARAMS:   now_ms - millis() sampled by the caller this iteration
 * RETURN:   true while car buttons must be honoured
 *
 * Subtract-then-compare against zero rather than comparing the two values
 * directly: that stays correct across the millis() rollover at 49.7 days.
 */
static bool access_is_open(uint32_t now_ms) {
    if (!s_access_ever_granted) return false;
    return (int32_t)(now_ms - s_access_until_ms) < 0;
}

/* Defined in tasks_ui.cpp. */
extern void lcdTask(void *arg);
extern void logTask(void *arg);
extern void ui_note_access(bool granted);

/* Defined in cmd.cpp. Bench facility: remove that file and this line and
 * nothing else changes. */
extern void cmdTask(void *arg);

/* Log codes above the fault-bit range, so a log line's code field is
 * unambiguous: below 0x100 it is a fault_code_t, above it is one of these. */
#define LOG_CODE_LINK_FIRST      0x100   /* first accepted frame, value = seq */
#define LOG_CODE_LINK_REJECTED   0x101   /* wrong version or unknown type     */
#define LOG_CODE_LINK_DROPPED    0x102   /* RX queue was full                 */
#define LOG_CODE_LINK_MALFORMED  0x103   /* wrong length, rejected in the ISR */
#define LOG_CODE_ACCESS_GRANTED  0x110   /* card accepted, value = window (s)  */
#define LOG_CODE_ACCESS_DENIED   0x111   /* card presented, UID not on the list*/
#define LOG_CODE_ACCESS_REFUSED  0x112   /* button ignored, value = floor      */

/* How far the ranged distance must move before ultrasonicTask logs it again.
 * A diagnostics choice, not a control constant: nothing reads it, so it can
 * be changed freely. 5 mm sits above the sensor's observed 1-2 mm of
 * standstill jitter (log boardA_20260818_142344: 41/42/43 mm at rest) and
 * far below the smallest car movement worth seeing. */
#define ULTRASONIC_LOG_DELTA_MM  5

#define ULTRASONIC_NEAR_ECHO_WINDOW 3
#define ACTIVE_FLOORS                 2u

/*
 * HC-SR04 near-echo suppressor.
 *
 * Bench observation:
 *   physical distance ~= 39 mm
 *   raw sensor stream alternates ~= 29 mm, 39 mm, 29 mm, 39 mm...
 *
 * The false echo is consistently SHORTER than the real target echo.
 * Therefore we keep a rolling 3-sample window and use its MAXIMUM.
 *
 * At 60 ms/sample this adds at most ~120 ms of lag while moving toward
 * the sensor, but it suppresses the repeatable near-field false echo without
 * hard-coding "39 mm" or any particular floor distance.
 *
 * The controller is not fed until the window has been primed with three
 * valid measurements. This prevents a false 29 mm first sample from becoming
 * the power-up hold target.
 */

/*
 * ============================================================================
 * FUNCTION: log_post
 *
 * PURPOSE:
 *    Queue one log line. Never blocks and never fails loudly.
 *
 * PARAMETERS:
 *    who (task_id_t) - which task is speaking
 *    code (uint32_t) - event or fault code
 *    value (int32_t) - event-specific payload
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    Any task, either core.
 * ============================================================================
 */
static void log_post(task_id_t who, uint32_t code, int32_t value) {
    log_msg_t m;
    m.timestamp_ms   = millis();
    m.source_task_id = (uint8_t)who;
    m.code           = code;
    m.value          = value;
    if (xQueueSend(g_q_log, &m, 0) != pdTRUE) s_drop_log++;
}

/*
 * ============================================================================
 * FUNCTION: onControlTimer
 *
 * PURPOSE:
 *    Release controlTask once per control period.
 *
 * PARAMETERS:
 *    None
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    Hardware timer interrupt at 100 Hz. Not a task. It gives a semaphore
 *    and does nothing else.
 * ============================================================================
 */
static void IRAM_ATTR onControlTimer(void) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_control_tick, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}

/* ========================================================================== */
/*                        SECTION: CORE 1 TASKS                               */
/* ========================================================================== */

/*
 * ============================================================================
 * FUNCTION: controlTask
 *
 * PURPOSE:
 *    Drain the input queue, run one control iteration, and publish the UI
 *    mailbox. Sole writer of the shared state.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 1, priority 6, released at 100 Hz by
 *    onControlTimer.
 * ============================================================================
 */
static void controlTask(void *arg) {
    (void)arg;
    input_event_t  ev;
    system_state_t st;
    uint32_t       last_ms = millis();

    for (;;) {
        /* Block on the timer, not on the tick. If the semaphore does not
         * arrive within a period and a half, the timer has stopped, which
         * is a fault worth reporting rather than a reason to spin. */
        if (xSemaphoreTake(s_control_tick,
                           pdMS_TO_TICKS(PERIOD_MS_CONTROL * 3)) != pdTRUE) {
            log_post(TASK_ID_CONTROL, FAULT_POSITION_LIMIT, -1);
            continue;
        }

        uint32_t now = millis();
        uint32_t dt  = now - last_ms;
        last_ms = now;

        /* Drain fully. Handling one event per iteration would let a burst
         * of presses trail the control loop by an unbounded amount. */
        while (xQueueReceive(g_q_input, &ev, 0) == pdTRUE) {
            switch (ev.type) {
            case INPUT_EVT_CAR_BUTTON:
                /* A press outside the authorization window is rejected, not
                 * queued. Queuing it would make the car move later, when the
                 * window happens to reopen, with nobody having asked. */
                if (!access_is_open(now)) {
                    log_post(TASK_ID_CONTROL, LOG_CODE_ACCESS_REFUSED,
                             (int32_t)ev.floor);
                    break;
                }
                control_request_floor(ev.floor, REQ_SRC_CAR_BUTTON);
                log_post(TASK_ID_CONTROL, INPUT_EVT_CAR_BUTTON, ev.floor);
                break;

            case INPUT_EVT_RFID_CARD: {
                const bool granted = rfid_is_authorized(ev);
                ui_note_access(granted);

                if (granted) {
                    /* Re-presenting a card restarts the full window rather
                     * than extending it, so the rider can always get another
                     * clean 20 s without waiting for the old one to lapse. */
                    s_access_ever_granted = true;
                    s_access_until_ms     = now + ACCESS_WINDOW_MS;
                    log_post(TASK_ID_CONTROL, LOG_CODE_ACCESS_GRANTED,
                             (int32_t)(ACCESS_WINDOW_MS / 1000u));
                } else {
                    log_post(TASK_ID_CONTROL, LOG_CODE_ACCESS_DENIED, 0);
                }
                break;
            }

            default:
                break;
            }
        }

        control_step(dt);

        /* Overwrite, not send: controlTask produces at 100 Hz and lcdTask
         * consumes at 5 Hz, so the only frame worth keeping is the newest. */
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

/*
 * ============================================================================
 * FUNCTION: ultrasonicTask
 *
 * PURPOSE:
 *    Range the shaft periodically. Each valid reading is filtered and fed
 *    to the primary ultrasonic closed-loop controller.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 1, priority 5, ~16 Hz.
 * ============================================================================
 */
static void ultrasonicTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    hcsr04_reading_t r;
    uint32_t consecutive_bad = 0;

    /*
     * Rolling upper-envelope filter.
     *
     * Our real target is ~39 mm while the HC-SR04 repeatedly produces a
     * false near echo at ~29 mm. Since that failure mode is "too short",
     * taking the maximum of a short rolling window rejects it.
     */
    uint16_t range_window[ULTRASONIC_NEAR_ECHO_WINDOW] = {0};
    uint8_t  range_index = 0;
    uint8_t  range_count = 0;

    /* Last ACCEPTED distance written to the log. */
    uint16_t last_reported_mm = UINT16_MAX;

    for (;;) {
        /*
         * Collect before triggering: the result belongs to the ping started
         * one period ago. The driver is interrupt driven and never blocks.
         */
        if (hcsr04_collect(&r) && r.valid) {
            consecutive_bad = 0;

            /* Insert this raw measurement into the rolling window. */
            range_window[range_index] = r.distance_mm;
            range_index =
                (uint8_t)((range_index + 1) % ULTRASONIC_NEAR_ECHO_WINDOW);

            if (range_count < ULTRASONIC_NEAR_ECHO_WINDOW) {
                range_count++;
            }

            /*
             * Do not feed the controller until the filter is fully primed.
             * This avoids booting on one false 29 mm near echo.
             */
            if (range_count == ULTRASONIC_NEAR_ECHO_WINDOW) {
                uint16_t accepted_mm = range_window[0];

                for (uint8_t i = 1;
                     i < ULTRASONIC_NEAR_ECHO_WINDOW;
                     ++i) {
                    if (range_window[i] > accepted_mm) {
                        accepted_mm = range_window[i];
                    }
                }

                /*
                 * HC-SR04 is the PRIMARY closed-loop position feedback.
                 * Only the anti-near-echo filtered measurement reaches PID.
                 */
                control_update_ultrasonic(accepted_mm, millis());

                /*
                 * Log the accepted distance, not the raw false echo.
                 * With the observed 29/39 alternating stream this should
                 * become approximately 38/39/40 instead of 29/39/29/39.
                 */
                if (last_reported_mm == UINT16_MAX ||
                    (uint16_t)abs((int32_t)accepted_mm -
                                  (int32_t)last_reported_mm)
                        >= ULTRASONIC_LOG_DELTA_MM) {
                    last_reported_mm = accepted_mm;
                    log_post(TASK_ID_ULTRASONIC, 0, (int32_t)accepted_mm);
                }
            }

        } else {
            consecutive_bad++;

            /*
             * One missed echo is ordinary. A run means the primary position
             * sensor is lost. Re-prime the filter after a real loss so stale
             * pre-fault distances cannot influence the recovered position.
             */
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

/*
 * ============================================================================
 * FUNCTION: loadTask
 *
 * PURPOSE:
 *    Background compute load, used to demonstrate that the control loop
 *    keeps its deadline under CPU pressure.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 1, priority 1, free-running.
 * ============================================================================
 */
static void loadTask(void *arg) {
    (void)arg;
    uint32_t candidate   = 2;
    uint32_t found       = 0;
    uint32_t last_logged = 0xFFFFFFFF;

    for (;;) {
        bool prime = (candidate >= 2);
        for (uint32_t d = 2; (uint64_t)d * d <= candidate; d++) {
            if (candidate % d == 0) { prime = false; break; }
        }
        if (prime) found++;
        candidate++;
        if (candidate == 0) candidate = 2;          /* wrap, keep running */

        /* Priority 1 and free-running: without a yield the idle task never
         * runs and the watchdog is never fed. This is the whole reason the
         * load is a real computation rather than a busy spin. */
        taskYIELD();

        /* Report only when the count CROSSES a multiple, not while it sits
         * on one. The obvious spelling -- (found & 0x3FF) == 0 -- is true on
         * every iteration between two primes, because found does not change
         * in between. That logged thousands of identical lines per second,
         * swamped the log queue, and truncated the serial output. */

        /* The mask was 0x3FF, which reported every 1024 primes -- several
         * lines per second, sharing the wire with the ultrasonic stream and
         * helping overrun the CDC buffer. 0xFFFF reports every 65536, which
         * is still often enough to prove the load task is progressing and
         * rare enough to stay out of the way of real events. */
        if (found != last_logged && (found & 0xFFFF) == 0) {
            last_logged = found;
            log_post(TASK_ID_LOAD, 0, (int32_t)found);
        }
    }
}

/* ========================================================================== */
/*                        SECTION: CORE 0 TASKS                               */
/* ========================================================================== */

/*
 * ============================================================================
 * FUNCTION: linkTask
 *
 * PURPOSE:
 *    Drain received frames, validate them, refresh the heartbeat deadline,
 *    and trigger emergency stop on timeout or on a STOP frame.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 5, 50 Hz.
 *
 * NOTE:
 *    The link is safe-on-silence. The car is not stopped by a message; it
 *    is kept running by the continued arrival of heartbeats. A dead Board B
 *    cannot send STOP, so waiting for one would fail open.
 * ============================================================================
 */
static void linkTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    link_frame_t f;
    system_state_t st;

    /* Diagnostics. "No heartbeat" has several causes that look identical
     * from the outside: nothing arriving at all, frames arriving and being
     * rejected for a version mismatch, or frames arriving faster than this
     * task drains them. Counting each separately turns one symptom into
     * three distinguishable faults. */
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    bool     seen_any = false;
    uint32_t next_summary_ms = 0;

    for (;;) {
        while (link_receive(&f)) {
            if (!link_frame_is_valid(&f)) {
                rejected++;
                continue;
            }
            accepted++;

            /* The first accepted frame is the moment the link proves it
             * works. Say so once, loudly, rather than leaving it implicit
             * in the absence of a fault. */
            if (!seen_any) {
                seen_any = true;
                log_post(TASK_ID_LINK, LOG_CODE_LINK_FIRST, (int32_t)f.seq);
            }

            shared_state_note_heartbeat(f.seq, millis());

            if (f.type == LINK_FRAME_STOP) {
                control_emergency_stop(FAULT_FALL_DETECTED);
                log_post(TASK_ID_LINK, FAULT_FALL_DETECTED, (int32_t)f.payload[0]);
            }
        }

        uint32_t now = millis();

        shared_state_get(&st);
        if ((now - st.last_heartbeat_ms) > LINK_TIMEOUT_MS) {
            control_emergency_stop(FAULT_LINK_TIMEOUT);

            /* Once a second, not every 20 ms. A fault that repeats at the
             * task rate drowns out everything else on the wire, including
             * whatever would explain it. */
            if ((int32_t)(now - next_summary_ms) >= 0) {
                next_summary_ms = now + 1000;
                log_post(TASK_ID_LINK, FAULT_LINK_TIMEOUT, (int32_t)accepted);
                log_post(TASK_ID_LINK, LOG_CODE_LINK_REJECTED, (int32_t)rejected);
                log_post(TASK_ID_LINK, LOG_CODE_LINK_DROPPED,
                         (int32_t)link_get_dropped_count());
                log_post(TASK_ID_LINK, LOG_CODE_LINK_MALFORMED,
                         (int32_t)link_get_malformed_count());
            }
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_LINK));
    }
}

/*
 * ============================================================================
 * FUNCTION: inputTask
 *
 * PURPOSE:
 *    Sample the car-panel buttons, debounce them, and post accepted presses
 *    to the input queue.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 4, 200 Hz.
 * ============================================================================
 */
static void inputTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();

    /* Two-floor build: only Floor 1 and Floor 2 buttons are active.
     * Internal floor indices are 0 and 1 respectively. */
    static const uint8_t pins[ACTIVE_FLOORS] = {
        PIN_BTN_CAR_0, PIN_BTN_CAR_1
    };
    /* Consecutive samples each button has read pressed. A press is accepted
     * once, on the sample where the count crosses the threshold, so holding
     * a button does not queue an event every 5 ms. */
    static uint8_t stable[ACTIVE_FLOORS];

    for (;;) {
        for (uint8_t i = 0; i < ACTIVE_FLOORS; i++) {
            bool down = (digitalRead(pins[i]) == LOW);   /* INPUT_PULLUP */

            if (!down) { stable[i] = 0; continue; }
            if (stable[i] > BTN_DEBOUNCE_SAMPLES) continue;   /* already sent */

            stable[i]++;
            if (stable[i] == BTN_DEBOUNCE_SAMPLES) {
                input_event_t ev;
                memset(&ev, 0, sizeof(ev));
                ev.type         = INPUT_EVT_CAR_BUTTON;
                ev.floor        = i;
                ev.timestamp_ms = millis();
                if (xQueueSend(g_q_input, &ev, 0) != pdTRUE) s_drop_input++;
                stable[i]++;                     /* latch until released */
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_INPUT));
    }
}

/*
 * ============================================================================
 * FUNCTION: rfidTask
 *
 * PURPOSE:
 *    Poll the card reader and post detected cards to the input queue.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 3, ~10 Hz.
 * ============================================================================
 */
static void rfidTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    rc522_card_t card;

    for (;;) {
        /* The driver suppresses repeats while a card stays in the field, so
         * one presentation yields one event. */
        if (rc522_poll(&card)) {
            input_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type         = INPUT_EVT_RFID_CARD;
            ev.uid_len      = card.uid_len;
            memcpy(ev.uid, card.uid, card.uid_len);
            ev.timestamp_ms = millis();
            if (xQueueSend(g_q_input, &ev, 0) != pdTRUE) s_drop_input++;
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_RFID));
    }
}

/* ========================================================================== */
/*                        SECTION: SETUP                                      */
/* ========================================================================== */

/*
 * ============================================================================
 * FUNCTION: setup
 *
 * PURPOSE:
 *    One-time initialisation. Order matters: shared state, then
 *    synchronisation objects, then drivers, then queues, then tasks.
 *
 * PARAMETERS:
 *    None
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    Arduino core, Core 1, before the application tasks exist.
 * ============================================================================
 */
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(2000);                        /* let USB CDC enumerate */
    Serial.println(F("Board A: elevator controller"));

    shared_state_init();

    /* Synchronisation objects before anything that might use them. */
    g_i2c_mutex    = xSemaphoreCreateMutex();
    s_control_tick = xSemaphoreCreateBinary();
    if (g_i2c_mutex == NULL || s_control_tick == NULL) {
        Serial.println(F("FATAL: could not create synchronisation objects."));
        for (;;) delay(1000);
    }

    for (uint8_t i = 0; i < NUM_FLOORS; i++) {
        /* INPUT_PULLUP, so a button shorts to GND and needs no resistor. */
    }
    pinMode(PIN_BTN_CAR_0, INPUT_PULLUP);  /* Floor 1 */
    pinMode(PIN_BTN_CAR_1, INPUT_PULLUP);  /* Floor 2 */
    /* PIN_BTN_CAR_2 intentionally unused in this two-floor build. */

    /* Drivers. A failure here latches a fault rather than halting: the
     * elevator can still run degraded without a card reader or a display,
     * and refusing to boot would hide which one failed.
     *
     * Each result is also PRINTED. Latching a fault bit and saying nothing
     * made a dead driver indistinguishable from a working one on the wire:
     * the panel shows only the highest-priority fault, so a link timeout
     * masks a failed display, and the log carries no init record at all.
     * One line per driver at boot costs nothing and turns "the LCD is
     * blank" from a guess into a reading. */
    /*
     * IMPORTANT INIT ORDER:
     * Install the HC-SR04 GPIO interrupt service BEFORE starting the
     * stepper's high-frequency hardware timer ISR.
     *
     * Arduino-ESP32 installs the shared GPIO ISR service on the first
     * attachInterrupt() call. Starting the 40 kHz STEP timer first can
     * interfere with that IPC-backed initialization on ESP32-S3.
     */
    Serial.println(F("[INIT] HC-SR04 begin"));
    const bool ok_hcsr04 = hcsr04_init();
    Serial.printf("[INIT] HC-SR04 done: %s\n", ok_hcsr04 ? "OK" : "FAIL");

    Serial.println(F("[INIT] stepper begin"));
    const bool ok_stepper = stepper_init();
    Serial.printf("[INIT] stepper done: %s\n", ok_stepper ? "OK" : "FAIL");

    Serial.println(F("[INIT] RC522 begin"));
    const bool ok_rc522 = rc522_init();
    Serial.printf("[INIT] RC522 done: %s\n", ok_rc522 ? "OK" : "FAIL");

    Serial.println(F("[INIT] LCD begin"));
    const bool ok_lcd = lcd_ui_init();
    Serial.printf("[INIT] LCD done: %s\n", ok_lcd ? "OK" : "FAIL");

    if (!ok_stepper) shared_state_raise_fault(FAULT_STEPPER_STALL);
    if (!ok_hcsr04)  shared_state_raise_fault(FAULT_ULTRASONIC_LOST);
    if (!ok_rc522)   shared_state_raise_fault(FAULT_RFID_FAILURE);
    if (!ok_lcd)     shared_state_raise_fault(FAULT_LCD_FAILURE);

    Serial.println(F("--- driver init ---"));
    Serial.printf("  stepper (TMC2209)   %s\n", ok_stepper ? "OK" : "FAIL");
    Serial.printf("  ultrasonic (HC-SR04)%s\n", ok_hcsr04  ? " OK" : " FAIL");
    Serial.printf("  rfid (RC522, SPI)   %s\n", ok_rc522   ? "OK" : "FAIL");
    Serial.printf("  lcd (I2C)           %s\n", ok_lcd     ? "OK" : "FAIL");
    Serial.println(F("--- floor map ---"));
    Serial.println(F("  Floor 1 = 30 mm"));
    Serial.println(F("  Floor 2 = 153 mm"));

    if (!link_init()) {
        Serial.println(F("FATAL: ESP-NOW would not start. Halting."));
        /* Without the link there is no fall detection and no heartbeat, so
         * the safety case does not hold. Refusing to run is the safe
         * failure. */
        for (;;) delay(1000);
    }

    /* The address Board B must be transmitting to. Printing it makes the one
     * link failure that leaves every counter at zero -- a peer MAC pointing
     * at the wrong board -- visible in two seconds instead of an afternoon.
     * Compare against PEER_MAC_BYTES in ElevatorB/board_config.h. */
    uint8_t own_mac[6];
    if (link_get_own_mac(own_mac)) {
        Serial.printf("--- link ---\n  Board A STA MAC     "
                      "%02X:%02X:%02X:%02X:%02X:%02X\n",
                      own_mac[0], own_mac[1], own_mac[2],
                      own_mac[3], own_mac[4], own_mac[5]);
        Serial.printf("  wifi channel        %d\n", LINK_WIFI_CHANNEL);
        Serial.println(F("  ^ this must equal PEER_MAC_BYTES on Board B"));
    } else {
        Serial.println(F("--- link ---\n  could not read own MAC"));
    }

    g_q_input = xQueueCreate(QDEPTH_INPUT_EVENT, sizeof(input_event_t));
    g_q_ui    = xQueueCreate(QDEPTH_UI_MSG,      sizeof(ui_msg_t));
    g_q_log   = xQueueCreate(QDEPTH_LOG_MSG,     sizeof(log_msg_t));
    if (g_q_input == NULL || g_q_ui == NULL || g_q_log == NULL) {
        Serial.println(F("FATAL: queue allocation failed."));
        for (;;) delay(1000);
    }

    control_init();

    /* Core 1: the deterministic side. */
    xTaskCreatePinnedToCore(controlTask,    "control", STACK_CONTROL, NULL,
                            PRIO_CONTROL,    NULL, CORE_CONTROL);
    xTaskCreatePinnedToCore(ultrasonicTask, "ultra",   STACK_ULTRASONIC, NULL,
                            PRIO_ULTRASONIC, NULL, CORE_ULTRASONIC);
    xTaskCreatePinnedToCore(loadTask,       "load",    STACK_LOAD, NULL,
                            PRIO_LOAD,       NULL, CORE_LOAD);

    /* Core 0: everything that shares the core with the radio. */
    xTaskCreatePinnedToCore(linkTask,  "link",  STACK_LINK,  NULL,
                            PRIO_LINK,  NULL, CORE_LINK);
    xTaskCreatePinnedToCore(inputTask, "input", STACK_INPUT, NULL,
                            PRIO_INPUT, NULL, CORE_INPUT);
    xTaskCreatePinnedToCore(rfidTask,  "rfid",  STACK_RFID,  NULL,
                            PRIO_RFID,  NULL, CORE_RFID);
    xTaskCreatePinnedToCore(lcdTask,   "lcd",   STACK_LCD,   NULL,
                            PRIO_LCD,   NULL, CORE_LCD);
    xTaskCreatePinnedToCore(logTask,   "log",   STACK_LOG,   NULL,
                            PRIO_LOG,   NULL, CORE_LOG);
    xTaskCreatePinnedToCore(cmdTask,   "cmd",   STACK_CMD,   NULL,
                            PRIO_CMD,   NULL, CORE_CMD);

    /* The control timer starts LAST, so the first tick cannot arrive before
     * the task that consumes it exists. */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    /* Core 3.x: timerBegin takes the tick frequency directly. */
    s_control_timer = timerBegin(1000000);              /* 1 MHz time base */
    timerAttachInterrupt(s_control_timer, &onControlTimer);
    timerAlarm(s_control_timer, PERIOD_MS_CONTROL * 1000, true, 0);
#else
    /* Core 2.x: timer index and a prescaler off the 80 MHz APB clock.
     * 80 MHz / 80 = 1 MHz, the same time base. Timer 0 here, timer 1 in
     * stepper.cpp -- on 2.x the indices are ours to allocate and must not
     * collide. */
    s_control_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(s_control_timer, &onControlTimer, true);
    timerAlarmWrite(s_control_timer, PERIOD_MS_CONTROL * 1000, true);
    timerAlarmEnable(s_control_timer);
#endif

    Serial.println(F("Board A running."));
}

/*
 * ============================================================================
 * FUNCTION: loop
 *
 * PURPOSE:
 *    Left empty on purpose. All work happens in the tasks created above.
 *    The Arduino loopTask still exists on Core 1 at priority 1, so anything
 *    placed here competes with loadTask.
 *
 * PARAMETERS:
 *    None
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    Arduino core loopTask, Core 1, priority 1.
 * ============================================================================
 */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
