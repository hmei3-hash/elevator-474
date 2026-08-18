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

/**
 * Report how many frames were rejected in the receive callback for having
 * the wrong length. A rising count means something else is transmitting on
 * this channel, not that the peer is misbehaving.
 *
 * @return cumulative malformed-frame count since init
 * Called from: logTask (Core 0, 10 Hz).
 */
uint32_t link_get_malformed_count(void);

/**
 * Read this board's own station MAC address.
 *
 * This is the address Board B must have in PEER_MAC_BYTES. Reading it from
 * the running controller, rather than from a note taken during bring-up,
 * removes the one link failure that produces no counter anywhere: a peer
 * address that is well-formed but belongs to the wrong board. Board A sees
 * nothing arrive, Board B sees every send succeed, and every diagnostic
 * counter on both sides stays at zero.
 *
 * @param out  six-byte buffer to receive the address, must not be NULL
 * @return true if the address was read
 * Called from: setup(), Core 1, before the tasks exist. Valid only after
 *              link_init() has brought the station interface up.
 */
bool link_get_own_mac(uint8_t out[6]);
