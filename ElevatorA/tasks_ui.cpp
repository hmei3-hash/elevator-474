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
 *    08/15/2026
 *
 * DEPENDENCIES:
 *    - Arduino.h
 *    - app_types.h
 *    - shared_state.h
 *    - lcd_ui.h: rendering reached only through the driver
 *
 * NOTES:
 *    App layer. Owns all string formatting so no driver has to know what a
 *    floor is. logTask is the only task permitted to write to Serial.
 * ============================================================================
 */

#include <Arduino.h>
#include "app_types.h"
#include "shared_state.h"
#include "lcd_ui.h"

/* Composition buffer for one display row. File-local by design. */
static char s_line[2][32];

/*
 * ============================================================================
 * FUNCTION: ui_format_status_line
 *
 * PURPOSE:
 *    Render the current mode, direction, and floor into a display row.
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
    // TODO: compose the status text; the exact wording and abbreviations
    //       depend on the 16-column width and are decided at the bench
    (void)st;
    (void)out;
    (void)out_len;
    return;
}

/*
 * ============================================================================
 * FUNCTION: ui_format_fault_line
 *
 * PURPOSE:
 *    Render the latched fault flags into a display row.
 *
 * PARAMETERS:
 *    fault_flags (uint32_t) - bitwise OR of fault_code_t values
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
static void ui_format_fault_line(uint32_t fault_flags, char *out, size_t out_len) {
    // TODO: map the highest-priority set fault bit to a short label
    (void)fault_flags;
    (void)out;
    (void)out_len;
    return;
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
    // TODO: loop on vTaskDelayUntil at PERIOD_MS_LCD, take a state snapshot,
    //       format both rows, and write only the rows that changed
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
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
    // TODO: loop on vTaskDelayUntil at PERIOD_MS_LOG, drain the log queue,
    //       and print each entry
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
