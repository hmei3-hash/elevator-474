/*
 * ============================================================================
 * FILE: link.cpp
 *
 * PURPOSE:
 *    ESP-NOW link implementation for Board A. Brings up the radio, registers
 *    the receive callback, and owns the queue that decouples that callback
 *    from the application.
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
 *    - WiFi.h, esp_now.h, esp_wifi.h
 *    - board_config.h
 *    - link.h, link_protocol.h
 *
 * NOTES:
 *    THE CALLBACK IS NOT A TASK.
 *    esp_now_register_recv_cb() installs link_rx_callback into the WiFi
 *    task, which the Arduino-ESP32 core pins to Core 0 at priority 23 --
 *    above every application task on this board. Work done there cannot be
 *    preempted by anything that matters. It copies bytes into a queue and
 *    returns; that is the whole contract. No printing, no parsing, no
 *    blocking, no locks.
 *
 *    linkTask exists to be the buffer on the other side of that queue.
 *
 *    Board A only receives. It registers no peer and never transmits, so
 *    there is no send path here at all.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "board_config.h"
#include "link.h"

/* Queue between the receive callback and linkTask. */
static QueueHandle_t s_rx_queue;

/* Frames discarded because s_rx_queue was full. A non-zero value means the
 * queue is undersized for the observed burst, not that anything is broken. */
static volatile uint32_t s_dropped;

/* Frames rejected for the wrong length before they ever reached the queue. */
static volatile uint32_t s_malformed;

/*
 * ============================================================================
 * FUNCTION: link_rx_callback
 *
 * PURPOSE:
 *    ESP-NOW receive callback. Copies the incoming bytes into the queue and
 *    returns. Nothing else may happen here.
 *
 * PARAMETERS:
 *    info / mac - sender metadata, unused. ESP-NOW changed this parameter
 *                 from a bare MAC pointer to a struct between arduino-esp32
 *                 2.x and 3.x, so both signatures are kept behind
 *                 ESP_ARDUINO_VERSION_MAJOR.
 *    data (const uint8_t*) - received bytes
 *    len (int) - number of received bytes
 *
 * RETURN VALUE:
 *    void
 *
 * CALLED FROM:
 *    WiFi task, Core 0, priority 23. NOT an application task.
 * ============================================================================
 */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void link_rx_callback(const esp_now_recv_info_t *info,
                             const uint8_t *data, int len) {
    (void)info;
#else
static void link_rx_callback(const uint8_t *mac,
                             const uint8_t *data, int len) {
    (void)mac;
#endif

    /* A frame of the wrong size cannot be our protocol. Rejecting it here
     * costs one comparison and keeps malformed bytes out of the queue
     * entirely, so linkTask never has to reason about length. */
    if (len != (int)sizeof(link_frame_t)) {
        s_malformed++;
        return;
    }

    link_frame_t f;
    memcpy(&f, data, sizeof(f));

    /* Zero timeout: this context must never block. A full queue means the
     * frame is dropped and counted, which is the correct behaviour for a
     * heartbeat -- the next one is 20 ms away.
     *
     * xQueueSend, not xQueueSendFromISR. This callback runs in the WiFi
     * TASK, not in an interrupt: it is high priority, but it is still a
     * task. The FromISR variants assume interrupt context, and
     * portYIELD_FROM_ISR() from a task is not something to rely on. */
    if (xQueueSend(s_rx_queue, &f, 0) != pdTRUE) {
        s_dropped++;
    }
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool link_init(void) {
    s_dropped   = 0;
    s_malformed = 0;

    s_rx_queue = xQueueCreate(QDEPTH_LINK_RX, sizeof(link_frame_t));
    if (s_rx_queue == NULL) return false;

    /* Station mode with no association. ESP-NOW needs the interface up; it
     * does not need, and must not have, an access point connection. */
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

    /* Both boards must sit on the same channel. Unassociated stations
     * default to channel 1, but setting it explicitly turns a silent
     * "sends succeed, nothing arrives" failure into an impossibility. */
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(LINK_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    /* Modem sleep would let the radio nap between beacons and quietly miss
     * heartbeats, which the application would then read as a link fault. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK) return false;
    if (esp_now_register_recv_cb(link_rx_callback) != ESP_OK) return false;

    return true;
}

bool link_receive(link_frame_t *out) {
    if (out == NULL || s_rx_queue == NULL) return false;
    return (xQueueReceive(s_rx_queue, out, 0) == pdTRUE);
}

bool link_frame_is_valid(const link_frame_t *f) {
    if (f == NULL) return false;

    /* Version first. A frame from a board flashed with a different build of
     * link_protocol.h would otherwise be parsed into plausible-looking
     * garbage, which is the worst failure mode available here. */
    if (f->ver != LINK_PROTO_VER) return false;

    if (f->type != LINK_FRAME_HEARTBEAT && f->type != LINK_FRAME_STOP) {
        return false;
    }
    return true;
}

uint32_t link_get_dropped_count(void) {
    return s_dropped;
}

uint32_t link_get_malformed_count(void) {
    return s_malformed;
}

bool link_get_own_mac(uint8_t out[6]) {
    if (out == NULL) return false;
    /* WIFI_IF_STA, not the soft-AP interface: the station address is the one
     * ESP-NOW delivers to, and the two differ by one in the last byte. */
    return (esp_wifi_get_mac(WIFI_IF_STA, out) == ESP_OK);
}
