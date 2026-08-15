/*
 * ============================================================================
 * FILE: app_types.h
 *
 * PURPOSE:
 *    Application-layer type vocabulary for Board A: inter-task queue message
 *    types, fault codes, and the aggregate system state record. This is an
 *    App-layer file: it names floors, doors, and elevator policy concepts.
 *    Driver and HAL files must NOT include it.
 *
 * AUTHOR:
 *    Hongyi Mei / Kevin Bi
 *
 * DATE CREATED:
 *    08/14/2026
 *
 * LAST MODIFIED:
 *    08/14/2026
 *
 * DEPENDENCIES:
 *    - stdint.h: fixed-width integer types
 *    - stdbool.h: bool
 *
 * NOTES:
 *    - Types only. No storage is defined here; see shared_state.h/.cpp.
 *    - No thresholds, gains, or timeouts appear in this file.
 *
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                        SECTION: ENUMERATED VOCABULARY                      */
/* ========================================================================== */

/**
 * Top-level operating mode of the elevator application.
 * Owned by controlTask (Core 1). Read by lcdTask and logTask.
 */
typedef enum {
    ELEV_MODE_INIT = 0,   // powered up, not yet homed
    ELEV_MODE_IDLE,       // parked at a floor, no pending request
    ELEV_MODE_MOVING,     // travelling toward target_floor
    ELEV_MODE_DOOR,       // stopped at a floor, door dwell in progress
    ELEV_MODE_ESTOP,      // emergency stop asserted (fall frame or fault)
    ELEV_MODE_FAULT       // latched fault, requires operator clear
} elev_mode_t;

/**
 * Direction of travel. Distinct from mode so a stopped car can still
 * remember which way it was heading.
 */
typedef enum {
    ELEV_DIR_NONE = 0,
    ELEV_DIR_UP,
    ELEV_DIR_DOWN
} elev_dir_t;

/**
 * Source of a floor request. Recorded so logTask can attribute requests.
 * All requests originate inside the car: there are no hall-call buttons.
 */
typedef enum {
    REQ_SRC_CAR_BUTTON = 0,   // car-panel pushbutton
    REQ_SRC_RFID              // request implied by an authorised card
} req_source_t;

/**
 * Fault codes. Bit positions so multiple faults can be latched at once
 * in system_state_t.fault_flags.
 */
typedef enum {
    FAULT_NONE            = 0,
    FAULT_ULTRASONIC_LOST = 1u << 0,   // no valid echo within the allowed window
    FAULT_LINK_TIMEOUT    = 1u << 1,   // heartbeat from Board B overdue
    FAULT_FALL_DETECTED   = 1u << 2,   // STOP frame received from Board B
    FAULT_STEPPER_STALL   = 1u << 3,   // commanded motion not observed
    FAULT_RFID_FAILURE    = 1u << 4,   // RC522 not responding on SPI
    FAULT_LCD_FAILURE     = 1u << 5,   // LCD not acknowledging on I2C
    FAULT_POSITION_LIMIT  = 1u << 6    // measured position outside travel range
} fault_code_t;

/* ========================================================================== */
/*                        SECTION: QUEUE MESSAGE TYPES                        */
/* ========================================================================== */

/**
 * Discriminator for input_event_t.
 */
typedef enum {
    INPUT_EVT_CAR_BUTTON = 0,
    INPUT_EVT_RFID_CARD
} input_evt_type_t;

/**
 * Message posted by inputTask (Core 0, 200 Hz, car buttons) and rfidTask
 * (Core 0, ~10 Hz) onto the input queue. Consumed by controlTask (Core 1, 100 Hz).
 */
typedef struct {
    input_evt_type_t type;
    uint8_t          floor;        // valid when type == INPUT_EVT_CAR_BUTTON
    uint8_t          uid[10];      // valid when type == INPUT_EVT_RFID_CARD
    uint8_t          uid_len;      // number of valid bytes in uid
    uint32_t         timestamp_ms; // millis() at capture
} input_event_t;

/**
 * Message posted by controlTask onto the UI queue.
 * Consumed by lcdTask (Core 0, 5 Hz).
 */
typedef struct {
    elev_mode_t mode;
    elev_dir_t  direction;
    uint8_t     current_floor;
    uint8_t     target_floor;
    uint32_t    fault_flags;      // bitwise OR of fault_code_t values
} ui_msg_t;

/**
 * Message posted by any task onto the log queue.
 * Consumed by logTask (Core 0, 10 Hz), which owns Serial.
 */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  source_task_id;      // TODO: define the task id enumeration
    uint32_t code;                // event or fault code
    int32_t  value;               // event-specific payload
} log_msg_t;

/* ========================================================================== */
/*                        SECTION: AGGREGATE SYSTEM STATE                     */
/* ========================================================================== */

/**
 * Complete elevator state. Single instance lives in shared_state.cpp and is
 * accessed only through the snapshot accessors declared in shared_state.h,
 * which take a portMUX spinlock. Do not reference the instance directly.
 *
 * Written by: controlTask (Core 1). Read by: lcdTask, logTask, linkTask.
 */
typedef struct {
    elev_mode_t mode;
    elev_dir_t  direction;

    uint8_t     current_floor;
    uint8_t     target_floor;
    bool        request_pending[8];   // TODO: size from NUM_FLOORS once fixed

    int32_t     position_mm;          // filtered car position, millimetres
    int32_t     position_raw_mm;      // last ultrasonic reading, millimetres
    int32_t     stepper_steps;        // signed accumulated step count

    uint32_t    fault_flags;          // bitwise OR of fault_code_t values
    bool        estop_active;

    uint32_t    last_heartbeat_ms;    // millis() of last frame from Board B
    uint32_t    link_seq;             // last accepted sequence number

    uint32_t    control_ticks;        // controlTask iteration counter
    uint32_t    load_primes_found;    // loadTask progress counter
} system_state_t;
