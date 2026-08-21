/*
 * ============================================================================
 * FILE: tasks_ui.cpp
 *
 * PURPOSE:
 *    Presentation tasks: composes the text shown on the display and formats
 *    the serial log. Turns elevator state into human-readable output.
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
 *    - shared_state.h
 *    - lcd_ui.h: rendering reached only through the driver
 *
 * NOTES:
 *    App layer. Owns all string formatting so no driver has to know what a
 *    floor is. logTask is the only task permitted to write to Serial.
 *
 *    PANEL LAYOUT, 16 x 2
 *      row 0:  floor and direction        e.g.  "Floor 2    UP  "
 *      row 1:  access / fault state       e.g.  "ACCESS GRANTED "
 *    A fault outranks everything on row 1: a passenger needs to know the
 *    car has stopped more than they need to know their card was read.
 *
 *    Both tasks take the I2C mutex around every display call. The bus is
 *    shared with the encoder, which controlTask reads on the other core.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "app_types.h"
#include "shared_state.h"
#include "lcd_ui.h"

/* Queues and the bus mutex are defined in ElevatorA.ino. */
extern QueueHandle_t g_q_ui;
extern QueueHandle_t g_q_log;
extern SemaphoreHandle_t g_i2c_mutex;

/* Composition buffers, one per row. File-local by design. */
static char s_line[LCD_ROWS][LCD_COLS + 1];

/* ========================================================================== */
/*                     SECTION: LOG OUTPUT CONTROL                            */
/* ========================================================================== */

/*
 * Two independent problems are solved here, and they are separate on purpose.
 *
 * MUTE is a request: the bench operator wants the wire to themselves. The
 * telemetry stream in cmd.cpp is a 100 Hz CSV that gets pasted into a
 * spreadsheet and plotted, and log lines interleaved into it corrupt exactly
 * the artifact the tuning session exists to produce.
 *
 * THE RATE CAP is a guarantee: no source, present or future, can flood the
 * port. A flood cannot be prevented at the source without knowing in advance
 * which source will flood, so it is bounded at the single point every line
 * passes through instead.
 *
 * Neither one is allowed to lose information silently. Suppressed lines are
 * counted and the count is reported, because "12000 lines were dropped" and
 * "nothing happened" must never look the same on the wire.
 */

/* Defined here, declared extern in cmd.cpp. Written by cmdTask (Core 0),
 * read by logTask (Core 0). Both are on the same core and the value is a
 * single aligned word, so no lock is needed. */
volatile bool g_log_muted = false;

/* Ceiling on log lines emitted per second. At 115200 baud the wire carries
 * roughly 1150 characters per second above which output backs up; a log line
 * is about 25 characters, so 40 lines/s uses well under half the link and
 * leaves headroom for the telemetry stream and command replies. */
#define LOG_MAX_LINES_PER_SEC   40

/* Lines dropped by mute or by the rate cap since the last report. */
static uint32_t s_log_suppressed;

/* Access result most recently decided by the application, for row 1. */
static bool s_access_shown;
static uint32_t s_access_until_ms;

/* How long an access verdict stays on the panel before row 1 reverts.
 *
 * Kept equal to ACCESS_WINDOW_MS in ElevatorA_main.cpp so the panel stops
 * showing ACCESS GRANTED at the same instant the buttons stop being
 * accepted. A display that outlives the permission it reports is worse than
 * no display: the rider reads GRANTED, presses a floor, and nothing happens.
 *
 * A DENIED verdict is not a permission window, so it is held only briefly --
 * see ui_note_access(). */
#define UI_ACCESS_HOLD_MS   20000

/* How long a DENIED verdict stays up. Short: nothing is unlocked by it. */
#define UI_DENIED_HOLD_MS   3000

/*
 * ============================================================================
 * FUNCTION: ui_dir_text
 *
 * PURPOSE:
 *    Short label for a direction of travel.
 *
 * PARAMETERS:
 *    d (elev_dir_t) - direction
 *
 * RETURN VALUE:
 *    const char* - "UP", "DOWN", or "--"
 *
 * CALLED FROM:
 *    lcdTask, Core 0, 5 Hz.
 * ============================================================================
 */
static const char *ui_dir_text(elev_dir_t d) {
    switch (d) {
        case ELEV_DIR_UP:   return "UP";
        case ELEV_DIR_DOWN: return "DOWN";
        default:            return "--";
    }
}

/*
 * ============================================================================
 * FUNCTION: ui_fault_text
 *
 * PURPOSE:
 *    Label for the highest-priority fault currently latched.
 *
 * PARAMETERS:
 *    flags (uint32_t) - bitwise OR of fault_code_t values
 *
 * RETURN VALUE:
 *    const char* - a label, or NULL when no fault is set
 *
 * CALLED FROM:
 *    lcdTask, Core 0, 5 Hz.
 *
 * NOTE:
 *    Order is by consequence to the passenger, not by bit position. A fall
 *    and a dead link both stop the car, and the panel must say which.
 * ============================================================================
 */
static const char *ui_fault_text(uint32_t flags) {
    if (flags & FAULT_FALL_DETECTED)   return "FALL - STOPPED";
    if (flags & FAULT_LINK_TIMEOUT)    return "LINK LOST-STOP";
    if (flags & FAULT_STEPPER_STALL)   return "MOTOR STALLED";
    if (flags & FAULT_POSITION_LIMIT)  return "POSITION FAULT";
    if (flags & FAULT_ULTRASONIC_LOST) return "RANGE SENSOR";
    if (flags & FAULT_RFID_FAILURE)    return "READER FAULT";
    if (flags & FAULT_LCD_FAILURE)     return "DISPLAY FAULT";
    return NULL;
}

/*
 * ============================================================================
 * FUNCTION: ui_format_status_line
 *
 * PURPOSE:
 *    Render row 0: current floor and direction of travel.
 *
 * PARAMETERS:
 *    st (const system_state_t*) - state snapshot to render
 *    out (char*) - destination buffer
 *    out_len (size_t) - capacity of out in bytes, including the terminator
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    lcdTask, Core 0, 5 Hz.
 * ============================================================================
 */
static void ui_format_status_line(const system_state_t *st, char *out, size_t out_len) {
    if (st == NULL || out == NULL || out_len == 0) return;

    if (st->mode == ELEV_MODE_INIT) {
        snprintf(out, out_len, "Starting up...");
        return;
    }
    snprintf(out, out_len, "Floor %u %s",
             (unsigned)st->current_floor, ui_dir_text(st->direction));
}

/*
 * ============================================================================
 * FUNCTION: ui_format_access_line
 *
 * PURPOSE:
 *    Render row 1: a latched fault if there is one, otherwise the most
 *    recent access verdict, otherwise the idle prompt.
 *
 * PARAMETERS:
 *    st (const system_state_t*) - state snapshot
 *    now_ms (uint32_t) - millis() at composition time
 *    out (char*) - destination buffer
 *    out_len (size_t) - capacity of out in bytes
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    lcdTask, Core 0, 5 Hz.
 * ============================================================================
 */
static void ui_format_access_line(const system_state_t *st, uint32_t now_ms,
                                  char *out, size_t out_len) {
    if (st == NULL || out == NULL || out_len == 0) return;

    const char *fault = ui_fault_text(st->fault_flags);
    if (fault != NULL) {
        snprintf(out, out_len, "%s", fault);
        return;
    }

    if ((int32_t)(now_ms - s_access_until_ms) < 0) {
        snprintf(out, out_len, "%s",
                 s_access_shown ? "ACCESS GRANTED" : "ACCESS DENIED");
        return;
    }

    snprintf(out, out_len, "Scan card");
}

/*
 * ============================================================================
 * FUNCTION: ui_note_access
 *
 * PURPOSE:
 *    Record an access verdict so row 1 can show it briefly.
 *
 * PARAMETERS:
 *    granted (bool) - true if the presented card was authorised
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    controlTask (Core 1) when it consumes an RFID event.
 * ============================================================================
 */
void ui_note_access(bool granted) {
    s_access_shown    = granted;
    s_access_until_ms = millis() +
                        (granted ? UI_ACCESS_HOLD_MS : UI_DENIED_HOLD_MS);
}

/*
 * ============================================================================
 * FUNCTION: lcdTask
 *
 * PURPOSE:
 *    Periodically snapshot the state and repaint the display.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 2, 5 Hz.
 * ============================================================================
 */
void lcdTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    system_state_t st;

    for (;;) {
        /* The UI mailbox is depth 1 and written with xQueueOverwrite, so a
         * read here always yields the newest frame or nothing. Falling back
         * to a direct snapshot keeps the panel live even before controlTask
         * has published anything. */
        ui_msg_t msg;
        if (xQueueReceive(g_q_ui, &msg, 0) == pdTRUE) {
            shared_state_get(&st);
            st.mode          = msg.mode;
            st.direction     = msg.direction;
            st.current_floor = msg.current_floor;
            st.fault_flags  |= msg.fault_flags;
        } else {
            shared_state_get(&st);
        }

        uint32_t now = millis();
        ui_format_status_line(&st, s_line[0], sizeof(s_line[0]));
        ui_format_access_line(&st, now, s_line[1], sizeof(s_line[1]));

        /* Hold the bus for both rows together: releasing between them would
         * let the encoder read interleave and double the arbitration cost
         * for no benefit. The two writes are microseconds apart. */
        if (xSemaphoreTake(g_i2c_mutex,
                           pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
            lcd_ui_write_at(0, 0, s_line[0]);
            lcd_ui_write_at(1, 0, s_line[1]);
            xSemaphoreGive(g_i2c_mutex);
        } else {
            /* Losing the bus is a bus fault, not a reason to miss the next
             * deadline. Report it and carry on. */
            shared_state_raise_fault(FAULT_LCD_FAILURE);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_LCD));
    }
}

/*
 * ============================================================================
 * FUNCTION: logTask
 *
 * PURPOSE:
 *    Drain the log queue and emit one serial line per message. The only
 *    task permitted to write to Serial.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 1, 10 Hz.
 * ============================================================================
 */
void logTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    log_msg_t m;

    /* Start of the current one-second accounting window, and how many lines
     * have been emitted inside it. */
    uint32_t window_start_ms = millis();
    uint32_t emitted_in_window = 0;

    for (;;) {
        uint32_t now = millis();

        /* Roll the window. Reporting the suppressed count here, rather than
         * when a line is dropped, keeps the report itself from becoming the
         * flood it is describing. */
        if ((uint32_t)(now - window_start_ms) >= 1000u) {
            window_start_ms   = now;
            emitted_in_window = 0;

            if (s_log_suppressed > 0 && !g_log_muted) {
                Serial.printf("# %lu log lines suppressed\n",
                              (unsigned long)s_log_suppressed);
                s_log_suppressed = 0;
            }
        }

        /* Drain fully, ALWAYS -- including while muted. Draining one per tick
         * would let a burst outlive the event that caused it and arrive out
         * of context. Not draining while muted would be worse still: the
         * queue would fill, log_post would start dropping at the producers,
         * and the drop counters would blame the wrong thing. Mute suppresses
         * printing, never collection. */
        while (xQueueReceive(g_q_log, &m, 0) == pdTRUE) {
            if (g_log_muted || emitted_in_window >= LOG_MAX_LINES_PER_SEC) {
                s_log_suppressed++;
                continue;
            }
            emitted_in_window++;

            /* Integer formatting only. %f pulls in the float formatter,
             * which costs far more stack than the integer path -- and this
             * task's stack is sized from measurements of what it actually
             * does. */
            Serial.printf("%lu,%u,%lu,%ld\n",
                          (unsigned long)m.timestamp_ms,
                          (unsigned)m.source_task_id,
                          (unsigned long)m.code,
                          (long)m.value);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_LOG));
    }
}
