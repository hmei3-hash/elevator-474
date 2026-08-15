/*
 * ============================================================================
 * FILE: rc522.cpp
 *
 * PURPOSE:
 *    SPI contactless card reader driver implementation.
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
 *    - board_config.h: pin assignments
 *    - rc522.h
 *
 * ============================================================================
 */

#include <Arduino.h>
#include "board_config.h"
#include "rc522.h"

/* Last UID seen, used to suppress repeat reports while a card is held in
 * the field. Local to this driver. */
static rc522_card_t s_last_card;

bool rc522_init(void) {
    // TODO: begin SPI on the configured pins, assert and release the reset
    //       line, then read the version register to confirm presence
    return false;
}

bool rc522_poll(rc522_card_t *out) {
    // TODO: request a card, run anti-collision, copy the UID into *out
    (void)out;
    return false;
}

bool rc522_is_alive(void) {
    // TODO: read a known register and compare against its reset value
    return false;
}
