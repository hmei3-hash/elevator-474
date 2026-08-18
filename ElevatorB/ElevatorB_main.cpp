/*
 * ============================================================================
 * FILE: ElevatorB_main.cpp
 *
 * PURPOSE:
 *    Board B entry point. Assembly only: initialise the sensor and the
 *    radio, create the queue and tasks, then hand control to the scheduler.
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
 *    PlatformIO, environment boardB. Renamed from ElevatorB.ino because PlatformIO
 *    compiles .cpp directly; the Arduino IDE's .ino preprocessing (which
 *    auto-generates forward declarations) is not involved, so every
 *    function here is defined before it is used.
 *
 * DEPENDENCIES:
 *    - Arduino.h, WiFi.h, esp_now.h, esp_wifi.h
 *    - board_config.h, link_protocol.h
 *    - mpu6050.h, fall_detect.h
 *
 * NOTES:
 *    Board B transmits only. It never receives, so no ESP-NOW receive
 *    callback is registered.
 *
 *    Sampling runs on Core 1 because the WiFi task occupies Core 0 at
 *    priority 23 and would otherwise stretch the sample interval. The
 *    detector's timing arithmetic uses the sample timestamps, so a
 *    stretched interval would move the thresholds without anyone noticing.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "board_config.h"
#include "link_protocol.h"
#include "mpu6050.h"
#include "fall_detect.h"

/* Queue from sampleTask to detectTask. */
QueueHandle_t g_q_sample;

/* Monotonic frame counter placed in link_frame_t.seq. */
static uint32_t s_tx_seq;

/* Board A's hardware address. */
static const uint8_t s_peer_mac[6] = PEER_MAC_BYTES;

/* Frames the radio refused to accept for transmission. */
static uint32_t s_tx_errors;

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
    link_frame_t f;
    memset(&f, 0, sizeof(f));

    f.ver  = LINK_PROTO_VER;
    f.type = type;
    f.seq  = s_tx_seq++;

    if (payload != NULL && payload_len > 0) {
        if (payload_len > LINK_PAYLOAD_BYTES) payload_len = LINK_PAYLOAD_BYTES;
        memcpy(f.payload, payload, payload_len);
    }

    esp_err_t r = esp_now_send(s_peer_mac, (const uint8_t *)&f, sizeof(f));
    if (r != ESP_OK) {
        s_tx_errors++;
        return false;
    }
    return true;
}

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
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    imu_sample_t s;

    for (;;) {
        if (mpu6050_read(&s)) {
            /* Zero timeout. A sampling task that blocks on a full queue
             * stops sampling, which is the one thing it must never do. */
            xQueueSend(g_q_sample, &s, 0);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_SAMPLE));
    }
}

/*
 * ============================================================================
 * FUNCTION: detectTask
 *
 * PURPOSE:
 *    Feed queued samples to the detector and transmit STOP when a fall is
 *    confirmed.
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
    (void)arg;
    imu_sample_t s;

    for (;;) {
        /* Block on the queue rather than polling: the sample rate sets the
         * pace, and blocking here leaves the core to the sampler. */
        if (xQueueReceive(g_q_sample, &s, portMAX_DELAY) != pdTRUE) continue;

        if (fall_detect_update(&s) != FALL_STATE_CONFIRMED) continue;

        /* ESP-NOW does not retry. A single STOP frame lost to one collision
         * is a fall that never reaches the elevator, so it is repeated.
         * This trades a handful of frames for a much lower miss
         * probability, which is the correct trade for a safety message. */
        uint8_t reason = LINK_STOP_REASON_FALL;
        for (uint8_t i = 0; i < STOP_FRAME_REPEATS; i++) {
            link_send_frame(LINK_FRAME_STOP, &reason, 1);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        fall_detect_clear();
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
 *
 * NOTE:
 *    This is the safety mechanism, not a status report. Board A stops the
 *    car when these stop arriving, so anything that silently kills this
 *    task also stops the elevator -- which is the intended behaviour.
 * ============================================================================
 */
static void txTask(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        link_send_frame(LINK_FRAME_HEARTBEAT, NULL, 0);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS_HEARTBEAT));
    }
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
    Serial.begin(SERIAL_BAUD);
    delay(2000);                       /* let USB CDC enumerate */

    Serial.println(F("Board B: fall detector"));

    if (!mpu6050_init()) {
        Serial.println(F("FATAL: IMU did not answer. Halting."));
        /* Halting is correct here. A fall detector that cannot sense is
         * worse than one that is obviously dead: Board A stops the car
         * when the heartbeat stops, so refusing to start is the safe
         * failure. */
        for (;;) delay(1000);
    }
    fall_detect_init();

    /* Station mode, never associated.
     *
     * WiFi.disconnect() used to be called here to guarantee that. On core
     * 3.x it logs "STA not started! You must call begin first" -- the
     * station is configured but not brought up until begin(), which we
     * never call, so there is nothing to disconnect FROM. The call was
     * defending against a condition that cannot arise: without begin() the
     * radio never associates.
     *
     * persistent(false) replaces the part that was doing real work. It
     * stops the core writing credentials to NVS, so a build that once
     * joined an access point cannot silently auto-reconnect on a later
     * boot and drag the radio onto a different channel. */
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(LINK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_ps(WIFI_PS_NONE);     /* no napping through a heartbeat */

    if (esp_now_init() != ESP_OK) {
        Serial.println(F("FATAL: esp_now_init failed. Halting."));
        for (;;) delay(1000);
    }

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, s_peer_mac, 6);
    peer.channel = LINK_WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println(F("FATAL: could not register Board A as a peer."));
        for (;;) delay(1000);
    }

    g_q_sample = xQueueCreate(QDEPTH_SAMPLE, sizeof(imu_sample_t));
    if (g_q_sample == NULL) {
        Serial.println(F("FATAL: queue allocation failed."));
        for (;;) delay(1000);
    }

    xTaskCreatePinnedToCore(sampleTask, "sample", STACK_SAMPLE, NULL,
                            PRIO_SAMPLE, NULL, CORE_SAMPLE);
    xTaskCreatePinnedToCore(detectTask, "detect", STACK_DETECT, NULL,
                            PRIO_DETECT, NULL, CORE_DETECT);
    xTaskCreatePinnedToCore(txTask,     "tx",     STACK_TX,     NULL,
                            PRIO_TX,     NULL, CORE_TX);

    Serial.println(F("Board B running."));
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
