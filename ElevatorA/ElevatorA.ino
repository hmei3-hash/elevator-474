/*
 * ============================================================================
 * FILE: ElevatorA.ino
 *
 * PURPOSE:
 *    Board A entry point. Assembly only: initialise drivers, create queues,
 *    create tasks with their configured priorities and core affinities, then
 *    hand control to the scheduler. No policy and no device access here.
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
 *    - board_config.h, app_types.h
 *    - shared_state.h, control.h, link.h
 *    - stepper.h, hcsr04.h, rc522.h, lcd_ui.h
 *
 * NOTES:
 *    Core split is load-bearing. The Arduino-ESP32 WiFi task is pinned to
 *    Core 0 at priority 23, so controlTask must stay on Core 1 to keep its
 *    100 Hz period deterministic. Do not move tasks between cores without
 *    re-measuring jitter.
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
/*                        SECTION: QUEUE HANDLES                              */
/*   Defined here, declared extern wherever else they are needed.             */
/* ========================================================================== */

QueueHandle_t g_q_input;
QueueHandle_t g_q_ui;
QueueHandle_t g_q_log;

/* ========================================================================== */
/*                        SECTION: TASK ENTRY POINTS                          */
/*   lcdTask and logTask are defined in tasks_ui.cpp.                         */
/* ========================================================================== */

extern void lcdTask(void *arg);
extern void logTask(void *arg);

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
 *    FreeRTOS scheduler. Core 1, priority 6, 100 Hz.
 * ============================================================================
 */
static void controlTask(void *arg) {
    // TODO: vTaskDelayUntil at PERIOD_MS_CONTROL; drain g_q_input fully each
    //       iteration, call control_step(), then xQueueOverwrite g_q_ui
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: ultrasonicTask
 *
 * PURPOSE:
 *    Range the shaft periodically and publish the raw reading.
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
    // TODO: vTaskDelayUntil at PERIOD_MS_ULTRASONIC; call hcsr04_read() and
    //       publish the result; raise FAULT_ULTRASONIC_LOST after repeated
    //       invalid readings
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: loadTask
 *
 * PURPOSE:
 *    Background compute load used to demonstrate that the control loop keeps
 *    its deadline under CPU pressure.
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
    // TODO: search for primes indefinitely, yielding often enough that the
    //       idle task still runs and the watchdog is fed
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

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
 * ============================================================================
 */
static void linkTask(void *arg) {
    // TODO: vTaskDelayUntil at PERIOD_MS_LINK; drain link_receive(), reject
    //       invalid frames, and raise FAULT_LINK_TIMEOUT when the heartbeat
    //       deadline passes
    // TODO: the heartbeat timeout is unknown until Board B's transmit period
    //       and the observed packet loss rate are measured on the bench
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: inputTask
 *
 * PURPOSE:
 *    Sample the car-panel buttons and post accepted presses to the input
 *    queue.
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
    // TODO: vTaskDelayUntil at PERIOD_MS_INPUT; sample the three car buttons
    //       and post INPUT_EVT_CAR_BUTTON events with a zero send timeout
    // TODO: the debounce strategy and its interval depend on the measured
    //       bounce duration of the actual switches
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
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
    // TODO: vTaskDelayUntil at PERIOD_MS_RFID; call rc522_poll() and post
    //       INPUT_EVT_RFID_CARD events
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: setup
 *
 * PURPOSE:
 *    One-time initialisation. Order matters: shared state, then drivers,
 *    then queues, then tasks.
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
    // TODO: Serial.begin(SERIAL_BAUD) and wait for USB CDC enumeration

    // TODO: shared_state_init()

    // TODO: initialise each driver and latch a fault for any that fails:
    //       stepper_init(), hcsr04_init(), rc522_init(), lcd_ui_init(),
    //       link_init()

    // TODO: create g_q_input, g_q_ui, g_q_log with the configured depths;
    //       g_q_ui has depth 1 and is written with xQueueOverwrite

    // TODO: control_init()

    // TODO: create Core 1 tasks: controlTask, ultrasonicTask, loadTask

    // TODO: create Core 0 tasks: linkTask, inputTask, rfidTask, lcdTask,
    //       logTask

    return;
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
