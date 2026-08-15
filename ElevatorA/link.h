/*
 * ============================================================================
 * FILE: link.h
 *
 * PURPOSE:
 *    ESP-NOW link management for Board A. Owns radio bring-up, the receive
 *    callback, and the frame queue that decouples the callback from the
 *    application.
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
 *    - link_protocol.h: wire format shared with Board B
 *
 * NOTES:
 *    The ESP-NOW receive callback runs in the WiFi task context, which is
 *    pinned to Core 0 at priority 23. It must copy and enqueue only. Any
 *    parsing, logging, or blocking work there breaks system timing.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>
#include "link_protocol.h"

/**
 * Bring up the radio in station mode, start ESP-NOW, register the receive
 * callback, and create the internal frame queue.
 *
 * @return true if ESP-NOW started and the callback was registered
 * Called from: setup(), before tasks start.
 */
bool link_init(void);

/**
 * Pop the oldest received frame, if any. Non-blocking.
 *
 * @param out  destination for the frame, must not be NULL
 * @return true if a frame was dequeued
 * Called from: linkTask (Core 0, 50 Hz).
 */
bool link_receive(link_frame_t *out);

/**
 * Test a frame for acceptability: correct version and known type.
 *
 * @param f  frame to check, must not be NULL
 * @return true if the frame should be acted on
 * Called from: linkTask (Core 0, 50 Hz).
 */
bool link_frame_is_valid(const link_frame_t *f);

/**
 * Report how many frames were dropped because the internal queue was full.
 * A non-zero value means the queue is undersized for the observed burst.
 *
 * @return cumulative dropped-frame count since init
 * Called from: logTask (Core 0, 10 Hz).
 */
uint32_t link_get_dropped_count(void);
