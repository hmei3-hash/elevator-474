/*
 * ============================================================================
 * FILE: link_protocol.h
 *
 * PURPOSE:
 *    Wire format contract for the ESP-NOW link between Board A (elevator
 *    controller) and Board B (fall detector). This file defines the exact
 *    byte layout both boards agree on. It contains no functions and no
 *    behavior.
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
 *    - stdint.h: fixed-width integer types
 *
 * CRITICAL:
 *    ElevatorA/link_protocol.h and ElevatorB/link_protocol.h MUST be
 *    byte-identical. Arduino IDE compiles each sketch folder separately, so
 *    the file cannot be shared; it is duplicated instead. After editing
 *    either copy, run:
 *        diff ElevatorA/link_protocol.h ElevatorB/link_protocol.h
 *    and confirm it reports no difference. A silent divergence produces
 *    garbage field values with no error message.
 *
 * NOTES:
 *    - Direction is one-way: B transmits, A receives. A does not ACK.
 *    - ESP-NOW already CRCs the frame at the MAC layer, so no application
 *      checksum field is defined here.
 *    - Bump LINK_PROTO_VER on ANY change to this file, then reflash BOTH
 *      boards. The receiver drops frames whose version does not match.
 *
 * ============================================================================
 */

#pragma once

#include <stdint.h>

/* ========================================================================== */
/*                          SECTION: PROTOCOL VERSION                         */
/* ========================================================================== */

/** Wire format version. Increment on any change to this file. */
#define LINK_PROTO_VER      1

/** Payload size in bytes. Fixed so the frame length never changes; unused
 *  bytes are zero. Chosen small: ESP-NOW allows up to 250 bytes total. */
#define LINK_PAYLOAD_BYTES  8

/* ========================================================================== */
/*                          SECTION: FRAME TYPES                              */
/* ========================================================================== */

/**
 * Frame type discriminator, carried in link_frame_t.type.
 */
typedef enum {
    LINK_FRAME_HEARTBEAT = 0,   // periodic liveness beacon, payload unused
    LINK_FRAME_STOP      = 1    // emergency stop request, payload[0] = reason
} link_frame_type_t;

/**
 * Reason codes for LINK_FRAME_STOP, carried in payload[0].
 * Present so a future stop cause can be added without a version bump.
 */
typedef enum {
    LINK_STOP_REASON_FALL      = 0,   // fall detected by the motion sensor
    LINK_STOP_REASON_SELF_TEST = 1    // sender declared itself unhealthy
} link_stop_reason_t;

/* ========================================================================== */
/*                          SECTION: FRAME LAYOUT                             */
/* ========================================================================== */

/**
 * The single frame exchanged over the link. Sent as raw bytes with
 * esp_now_send() and received into an identical struct.
 *
 * Layout, 14 bytes total, no padding:
 *    offset 0   uint8_t  ver
 *    offset 1   uint8_t  type
 *    offset 2   uint32_t seq
 *    offset 6   uint8_t  payload[8]
 *
 * Sent by:     Board B, linkTask equivalent
 * Received by: Board A, ESP-NOW receive callback (WiFi task, Core 0, prio 23)
 *              which must copy and enqueue only. Parsing happens in linkTask.
 */
typedef struct __attribute__((packed)) {
    uint8_t  ver;                          // must equal LINK_PROTO_VER
    uint8_t  type;                         // a link_frame_type_t value
    uint32_t seq;                          // monotonic, increments per frame
    uint8_t  payload[LINK_PAYLOAD_BYTES];  // type-specific, zero when unused
} link_frame_t;

/* Compile-time assertion, spelled for whichever language is compiling this.
 * _Static_assert is C11; C++ spells it static_assert. This header is
 * included from .cpp on both boards and from C on the host, so it must
 * work in both. */
#if defined(__cplusplus)
  #define LINK_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
  #define LINK_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

/** Compile-time guard: the layout above must not have grown padding. */
LINK_STATIC_ASSERT(sizeof(link_frame_t) == 6 + LINK_PAYLOAD_BYTES,
                   "link_frame_t has unexpected padding; check packed");
