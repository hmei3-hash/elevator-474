/*
 * ============================================================================
 * FILE: link.cpp
 *
 * PURPOSE:
 *    ESP-NOW link implementation for Board A.
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
 *    - board_config.h
 *    - link.h
 *    - link_protocol.h
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "link.h"

/* Queue between the ESP-NOW receive callback and linkTask. */
static QueueHandle_t s_rx_queue;

/* Frames discarded because s_rx_queue was full. */
static volatile uint32_t s_dropped;

/*
 * ============================================================================
 * FUNCTION: link_rx_callback
 *
 * PURPOSE:
 *    ESP-NOW receive callback. Copies the incoming bytes into the queue and
 *    returns. Nothing else may happen here.
 *
 * PARAMETERS:
 *    mac (const uint8_t*) - sender hardware address, unused
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
static void link_rx_callback(const uint8_t *mac, const uint8_t *data, int len) {
    // TODO: reject len != sizeof(link_frame_t), then xQueueSend with a zero
    //       timeout and increment s_dropped on failure. No printing, no
    //       parsing, no blocking calls in this function.
    (void)mac;
    (void)data;
    (void)len;
    return;
}

bool link_init(void) {
    // TODO: WiFi.mode(WIFI_STA), do not connect to any AP, esp_now_init(),
    //       register link_rx_callback, create s_rx_queue
    return false;
}

bool link_receive(link_frame_t *out) {
    // TODO: xQueueReceive from s_rx_queue with a zero timeout
    (void)out;
    return false;
}

bool link_frame_is_valid(const link_frame_t *f) {
    // TODO: check f->ver == LINK_PROTO_VER and f->type is a known enum value
    (void)f;
    return false;
}

uint32_t link_get_dropped_count(void) {
    // TODO: return s_dropped
    return 0;
}
