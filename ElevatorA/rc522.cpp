/*
 * ============================================================================
 * FILE: rc522.cpp
 *
 * PURPOSE:
 *    SPI contactless card reader driver implementation.
 *
 *    Integrated from the bench prototype sketch_aug12b, 08/16/2026. The SPI
 *    bring-up, the version read-back and the poll sequence are carried over;
 *    only the pins moved, to match board_config.h, which is authoritative.
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
 *    - SPI.h
 *    - MFRC522.h: "MFRC522 by GithubCommunity", library manager
 *    - board_config.h: pin assignments
 *    - rc522.h
 *
 * DIFFERENCES FROM THE PROTOTYPE
 *    - The authorised-card comparison is GONE from this file. Deciding which
 *      card may do what is application policy; a driver that knows about
 *      authorisation is a driver that has to change when the policy does.
 *      This file reports a UID and nothing more.
 *    - The prototype compared bytes up to rfid.uid.size against a 4-byte
 *      array. A 7-byte UID -- which plenty of cards have -- would read past
 *      the end of that array. The length is now carried explicitly so the
 *      caller can compare lengths before contents.
 *    - Nothing here prints. Only logTask may touch Serial.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include "board_config.h"
#include "rc522.h"

/* Reader handle. */
static MFRC522 s_reader(PIN_RC522_CS, PIN_RC522_RST);

/* Set once init has succeeded. */
static bool s_ready;

/* Last UID reported, used to suppress repeats while a card is held in the
 * field. Local to this driver: the application should see one event per
 * presentation, not one per poll. */
static rc522_card_t s_last;

/* Polls since the last successful read. Once a card leaves the field the
 * suppression must lapse, or presenting the same card twice in a row would
 * be reported once. */
static uint32_t s_absent_polls;

/* How many consecutive empty polls count as "the card has left".
 * At the rfidTask rate this is a fraction of a second. */
#define RC522_ABSENT_POLLS   3

/*
 * ============================================================================
 * FUNCTION: same_card
 *
 * PURPOSE:
 *    Compare two UIDs by length first, then by content.
 *
 * PARAMETERS:
 *    a (const rc522_card_t*) - first card
 *    b (const rc522_card_t*) - second card
 *
 * RETURN VALUE:
 *    bool - true if both length and every byte match
 *
 * CALLED FROM:
 *    rc522_poll(), rfidTask, Core 0.
 * ============================================================================
 */
static bool same_card(const rc522_card_t *a, const rc522_card_t *b) {
    if (a->uid_len != b->uid_len) return false;
    for (uint8_t i = 0; i < a->uid_len; i++) {
        if (a->uid[i] != b->uid[i]) return false;
    }
    return true;
}

/* ========================================================================== */
/*                          SECTION: PUBLIC INTERFACE                         */
/* ========================================================================== */

bool rc522_init(void) {
    s_ready        = false;
    s_last.uid_len = 0;
    s_absent_polls = RC522_ABSENT_POLLS;

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RC522_CS);
    s_reader.PCD_Init();

    /* Reading the version register back is the only proof the SPI wiring
     * works. Every write before this point is fire-and-forget, so a swapped
     * MOSI/MISO pair looks exactly like a successful init. */
    uint8_t version = s_reader.PCD_ReadRegister(MFRC522::VersionReg);

    /* 0x00 and 0xFF are the two failure signatures: nothing driving the bus,
     * or the bus stuck. Genuine parts report 0x91 or 0x92, and clones report
     * other values, so anything else is accepted. */
    if (version == 0x00 || version == 0xFF) return false;

    s_ready = true;
    return true;
}

bool rc522_poll(rc522_card_t *out) {
    if (out == NULL || !s_ready) return false;

    if (!s_reader.PICC_IsNewCardPresent()) {
        if (s_absent_polls < RC522_ABSENT_POLLS) s_absent_polls++;
        return false;
    }
    if (!s_reader.PICC_ReadCardSerial()) {
        return false;
    }

    rc522_card_t card;
    card.uid_len = s_reader.uid.size;
    if (card.uid_len > RC522_UID_MAX_BYTES) {
        card.uid_len = RC522_UID_MAX_BYTES;
    }
    for (uint8_t i = 0; i < card.uid_len; i++) {
        card.uid[i] = s_reader.uid.uidByte[i];
    }

    /* Stop the card talking, so the next poll sees a fresh presentation
     * rather than the same card still selected. */
    s_reader.PICC_HaltA();

    /* Suppress a repeat only while the card has stayed in the field. */
    bool repeat = (s_absent_polls < RC522_ABSENT_POLLS) &&
                  same_card(&card, &s_last);
    s_last = card;
    s_absent_polls = 0;

    if (repeat) return false;

    *out = card;
    return true;
}

bool rc522_is_alive(void) {
    if (!s_ready) return false;
    uint8_t v = s_reader.PCD_ReadRegister(MFRC522::VersionReg);
    return (v != 0x00 && v != 0xFF);
}
