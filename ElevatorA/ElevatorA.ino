/*
 * ============================================================================
 * FILE: ElevatorA.ino
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
 * DEPENDENCIES:
 *    - Arduino.h
 *    - board_config.h, app_types.h
 *    - shared_state.h, control.h, link.h
 *    - stepper.h, as5600.h, hcsr04.h, rc522.h, lcd_ui.h
 *
 * NOTES:
 *    Core split is load-bearing. The Arduino-ESP32 WiFi task is pinned to
 *    Core 0 at priority 23, so controlTask must stay on Core 1 to keep its
 *    100 Hz period deterministic. Do not move tasks between cores without
 *    re-measuring jitter.
 *
 *    THE CONTROL LOOP IS RELEASED BY A HARDWARE TIMER, NOT BY THE TICK.
 *    A hardware timer ISR gives a binary semaphore at 100 Hz and
 *    controlTask blocks on it. vTaskDelayUntil would quantise the period to
 *    the FreeRTOS tick instead, which puts jitter directly into the
 *    controller's dt -- and dt divides the derivative term.
 *
 *    THE I2C BUS IS SHARED ACROSS CORES. The display (Core 0, 5 Hz) and the
 *    encoder (Core 1, 100 Hz) sit on one bus. Every transaction takes
 *    g_i2c_mutex. Losing the mutex is reported as a fault; it never becomes
 *    a missed deadline.
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
#include "as5600.h"
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

/* Defined in tasks_ui.cpp. */
extern void lcdTask(void *arg);
extern void logTask(void *arg);
extern void ui_note_access(bool granted);

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
                control_request_floor(ev.floor, REQ_SRC_CAR_BUTTON);
                log_post(TASK_ID_CONTROL, INPUT_EVT_CAR_BUTTON, ev.floor);
                break;

            case INPUT_EVT_RFID_CARD:
                /* TODO: compare ev.uid against the authorised card list and
                 *       decide what a valid card is allowed to do. The list
                 *       itself is application policy and needs the real card
                 *       UIDs, which are read with bringup_a's 'c' command. */
                ui_note_access(false);
                break;

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
 *    Range the shaft periodically and publish the reading as an independent
 *    check on the encoder.
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

    for (;;) {
        /* Collect before triggering: the result belongs to the ping started
         * one period ago. The driver is interrupt driven and never blocks. */
        if (hcsr04_collect(&r) && r.valid) {
            consecutive_bad = 0;
            log_post(TASK_ID_ULTRASONIC, 0, (int32_t)r.distance_mm);
        } else {
            consecutive_bad++;
            /* One missed echo is ordinary -- a bad angle, a soft target.
             * A run of them means the sensor is not seeing the car. */
            if (consecutive_bad >= HCSR04_FAULT_AFTER_MISSES) {
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
    uint32_t candidate = 2;
    uint32_t found     = 0;

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

        if ((found & 0x3FF) == 0) {
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

    for (;;) {
        while (link_receive(&f)) {
            if (!link_frame_is_valid(&f)) continue;

            shared_state_note_heartbeat(f.seq, millis());

            if (f.type == LINK_FRAME_STOP) {
                control_emergency_stop(FAULT_FALL_DETECTED);
                log_post(TASK_ID_LINK, FAULT_FALL_DETECTED, (int32_t)f.payload[0]);
            }
        }

        shared_state_get(&st);
        if ((millis() - st.last_heartbeat_ms) > LINK_TIMEOUT_MS) {
            control_emergency_stop(FAULT_LINK_TIMEOUT);
            log_post(TASK_ID_LINK, FAULT_LINK_TIMEOUT,
                     (int32_t)link_get_dropped_count());
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

    static const uint8_t pins[NUM_FLOORS] = {
        PIN_BTN_CAR_0, PIN_BTN_CAR_1, PIN_BTN_CAR_2
    };
    /* Consecutive samples each button has read pressed. A press is accepted
     * once, on the sample where the count crosses the threshold, so holding
     * a button does not queue an event every 5 ms. */
    static uint8_t stable[NUM_FLOORS];

    for (;;) {
        for (uint8_t i = 0; i < NUM_FLOORS; i++) {
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
    pinMode(PIN_BTN_CAR_0, INPUT_PULLUP);
    pinMode(PIN_BTN_CAR_1, INPUT_PULLUP);
    pinMode(PIN_BTN_CAR_2, INPUT_PULLUP);

    /* Drivers. A failure here latches a fault rather than halting: the
     * elevator can still run degraded without a card reader or a display,
     * and refusing to boot would hide which one failed. */
    if (!stepper_init())  shared_state_raise_fault(FAULT_STEPPER_STALL);
    if (!hcsr04_init())   shared_state_raise_fault(FAULT_ULTRASONIC_LOST);
    if (!rc522_init())    shared_state_raise_fault(FAULT_RFID_FAILURE);

    /* Both of these share the I2C bus; no task exists yet, so no mutex is
     * needed, but the order is kept deliberate. */
    if (!lcd_ui_init())   shared_state_raise_fault(FAULT_LCD_FAILURE);
    if (!as5600_init())   shared_state_raise_fault(FAULT_POSITION_LIMIT);

    if (!link_init()) {
        Serial.println(F("FATAL: ESP-NOW would not start. Halting."));
        /* Without the link there is no fall detection and no heartbeat, so
         * the safety case does not hold. Refusing to run is the safe
         * failure. */
        for (;;) delay(1000);
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

    /* The control timer starts LAST, so the first tick cannot arrive before
     * the task that consumes it exists. */
    s_control_timer = timerBegin(1000000);              /* 1 MHz time base */
    timerAttachInterrupt(s_control_timer, &onControlTimer);
    timerAlarm(s_control_timer, PERIOD_MS_CONTROL * 1000, true, 0);

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
