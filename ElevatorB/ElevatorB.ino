/*
 * ============================================================================
 * FILE: ElevatorB.ino
 *
 * PURPOSE:
 *    Board B entry point. Assembly only: initialise the sensor and the radio,
 *    create the queue and tasks, then hand control to the scheduler.
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
 *    - board_config.h, link_protocol.h
 *    - mpu6050.h, fall_detect.h
 *
 * NOTES:
 *    Board B transmits only. It never receives, so no ESP-NOW receive
 *    callback is registered.
 *    Sampling runs on Core 1 because the WiFi task occupies Core 0 at
 *    priority 23 and would otherwise stretch the sample interval.
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "link_protocol.h"
#include "mpu6050.h"
#include "fall_detect.h"

/* Queue from sampleTask to detectTask. */
QueueHandle_t g_q_sample;

/* Monotonic frame counter placed in link_frame_t.seq. */
static uint32_t s_tx_seq;

/*
 * ============================================================================
 * FUNCTION: sampleTask
 *
 * PURPOSE:
 *    Read the inertial sensor at a fixed rate and queue each sample.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 1, priority 6.
 * ============================================================================
 */
static void sampleTask(void *arg) {
    // TODO: vTaskDelayUntil at PERIOD_MS_SAMPLE; call mpu6050_read() and
    //       post the sample with a zero send timeout
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: detectTask
 *
 * PURPOSE:
 *    Feed queued samples to the detector and request a STOP transmission
 *    when a fall is confirmed.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 1, priority 5.
 * ============================================================================
 */
static void detectTask(void *arg) {
    // TODO: drain g_q_sample, call fall_detect_update(), and on
    //       FALL_STATE_CONFIRMED send STOP_FRAME_REPEATS stop frames
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: txTask
 *
 * PURPOSE:
 *    Emit the periodic heartbeat frame that keeps Board A alive.
 *
 * PARAMETERS:
 *    arg (void*) - unused
 *
 * RETURN VALUE:
 *    void - never returns
 *
 * CALLED FROM:
 *    FreeRTOS scheduler. Core 0, priority 4.
 * ============================================================================
 */
static void txTask(void *arg) {
    // TODO: vTaskDelayUntil at PERIOD_MS_HEARTBEAT; build a
    //       LINK_FRAME_HEARTBEAT frame with the next seq and send it
    (void)arg;
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/*
 * ============================================================================
 * FUNCTION: link_send_frame
 *
 * PURPOSE:
 *    Fill in the protocol header and transmit one frame to Board A.
 *
 * PARAMETERS:
 *    type (uint8_t) - a link_frame_type_t value
 *    payload (const uint8_t*) - payload bytes, may be NULL
 *    payload_len (uint8_t) - number of payload bytes, at most
 *                            LINK_PAYLOAD_BYTES
 *
 * RETURN VALUE:
 *    bool - true if the radio accepted the frame for transmission. This is
 *           NOT delivery confirmation; ESP-NOW does not guarantee arrival.
 *
 * CALLED FROM:
 *    txTask (Core 0), detectTask (Core 1).
 * ============================================================================
 */
static bool link_send_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
    // TODO: zero a link_frame_t, set ver/type/seq, copy the payload, and
    //       call esp_now_send() to the registered peer
    (void)type;
    (void)payload;
    (void)payload_len;
    return false;
}

/*
 * ============================================================================
 * FUNCTION: setup
 *
 * PURPOSE:
 *    One-time initialisation for Board B.
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

    // TODO: mpu6050_init(), fall_detect_init()

    // TODO: WiFi.mode(WIFI_STA), do not connect to any AP, esp_now_init(),
    //       and register Board A as a peer using PEER_MAC_BYTES

    // TODO: create g_q_sample with QDEPTH_SAMPLE

    // TODO: create sampleTask and detectTask on Core 1, txTask on Core 0

    return;
}

/*
 * ============================================================================
 * FUNCTION: loop
 *
 * PURPOSE:
 *    Left empty on purpose. All work happens in the tasks created above.
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
