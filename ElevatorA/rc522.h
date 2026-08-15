/*
 * ============================================================================
 * FILE: rc522.h
 *
 * PURPOSE:
 *    Driver for an SPI contactless card reader. Reports the presence of a
 *    card and its unique identifier. It does not decide what any card means.
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
 *
 * NOTES:
 *    Driver layer. Must not reference application concepts.
 *    Card-to-permission mapping belongs in the application layer.
 * ============================================================================
 */

#pragma once

#include <Arduino.h>

/** Maximum UID length this reader reports, in bytes. */
#define RC522_UID_MAX_BYTES  10

/** One detected card. */
typedef struct {
    uint8_t uid[RC522_UID_MAX_BYTES];
    uint8_t uid_len;        // number of valid bytes in uid
} rc522_card_t;

/**
 * Bring up the SPI bus and reset the reader.
 *
 * @return true if the reader answered with its expected version register
 * Called from: setup(), before tasks start.
 */
bool rc522_init(void);

/**
 * Poll once for a card in the field.
 *
 * @param out  destination for the card UID, must not be NULL
 * @return true if a card was present and its UID was read
 * Called from: rfidTask (Core 0, ~10 Hz).
 */
bool rc522_poll(rc522_card_t *out);

/**
 * Check that the reader is still responding on the bus.
 *
 * @return true if the reader is healthy
 * Called from: rfidTask (Core 0, ~10 Hz), periodically.
 */
bool rc522_is_alive(void);
